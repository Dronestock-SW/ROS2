"""Sequential-beam synthetic sensor and measured-only deskew/odometry adapter.

SpinningLidar is the ONLY class here receiving simulator trajectory truth.
SpinningScanOdometry receives ranges, clock exchanges, FC IMU and floor range.
No truth positions/velocities are passed across that boundary.
"""
from collections import deque
from dataclasses import replace

import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation, Slerp

from .lidar_scanmatch import Cloud, Config, ordered_cloud
from .lidar_sitl_observation import PlanarScanOdometry
from .lidar_timing import ClockAlignment, ImuTimeline, TimedScan, TimingError, deskew_scan


class PoseTimeline:
    """Sensor-backend trajectory only; never supply this to the estimator."""
    def __init__(self):
        self.rows = deque(maxlen=600)

    def add(self, stamp_s, position_enu_m, rotation_enu_flu, velocity_enu_m_s):
        if self.rows and stamp_s<=self.rows[-1][0]:
            return False
        values = (float(stamp_s), np.asarray(position_enu_m, dtype=float),
            np.asarray(rotation_enu_flu, dtype=float), np.asarray(velocity_enu_m_s, dtype=float))
        if any(not np.isfinite(v).all() for v in values):
            raise ValueError('nonfinite_simulation_truth')
        self.rows.append(values)
        return True

    def sample(self, times):
        times = np.asarray(times)
        stamps = np.array([r[0] for r in self.rows])
        if len(stamps)<2 or times.min()<stamps[0] or times.max()>stamps[-1]:
            raise TimingError('simulation_trajectory_not_bracketed')
        left = max(0, np.searchsorted(stamps, times.min(), side='right')-1)
        right = min(len(stamps), np.searchsorted(stamps, times.max())+1)
        rows = list(self.rows)[left:right]
        t = stamps[left:right]
        if len(t)<2 or np.max(np.diff(t))>.04:
            raise TimingError('simulation_trajectory_gap')
        positions = CubicHermiteSpline(t, np.array([r[1] for r in rows]), np.array([r[3] for r in rows]))(times)
        rotations = Slerp(t, Rotation.from_matrix(np.array([r[2] for r in rows])))(times).as_matrix()
        return positions, rotations


class SpinningLidar:
    """10 Hz acquisition with one trajectory sample per sequential ray.

    Pillars and moving cylinders are ray cast per beam (occlusion included).
    Raw LiDAR clock has a declared simulation offset/drift; the estimator is
    given two-way exchanges, not these oracle calibration values.
    """
    def __init__(self, seed, *, beams=430, hz=10., range_noise_m=.01,
                 scan_bias_noise_m=.003, clock_offset_s=.037, clock_drift_ppm=35.):
        self.rng = np.random.default_rng(seed)
        self.clock_rng = np.random.default_rng(seed+50000)
        self.beams = beams
        self.period = 1/hz
        self.noise = range_noise_m
        self.bias = scan_bias_noise_m
        self.offset = clock_offset_s
        self.scale = 1+clock_drift_ppm*1e-6

    def local_time(self, fc_time):
        return (fc_time-self.offset)/self.scale

    def exchange(self, fc_reply_s):
        # Simulated request/reply with independent asymmetric positive delays.
        inbound, outbound = self.clock_rng.uniform(.00015, .0007, 2)
        local_reply = self.local_time(fc_reply_s)
        return (local_reply-inbound, fc_reply_s, local_reply+outbound)

    def observe(self, timeline, *, first_ray_fc_s, wall_x_m=5., pillars=(),
                person=None, missing_sector=None):
        times = first_ray_fc_s+np.arange(self.beams)*self.period/self.beams
        positions, rotations = timeline.sample(times)
        angles = -np.pi+np.arange(self.beams)*2*np.pi/self.beams
        body = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(self.beams)))
        directions = np.einsum('nij,nj->ni', rotations, body)
        distances = np.full_like(directions, np.inf)
        np.divide(np.array([wall_x_m, 5., 3.])-positions, directions, out=distances, where=directions>1e-10)
        negative = np.full_like(directions, np.inf)
        np.divide(-positions, directions, out=negative, where=directions<-1e-10)
        ranges = np.minimum(distances, negative).min(axis=1)
        for center_x, center_y, radius in pillars:
            ranges = self._cylinder(ranges, positions, directions,
                np.tile([center_x, center_y], (self.beams, 1)), radius)
        if person is not None:
            centers, radius = person(times)
            ranges = self._cylinder(ranges, positions, directions, np.asarray(centers), radius)
        ranges += self.rng.normal(0, self.noise, self.beams)+self.rng.normal(0, self.bias)
        if missing_sector is not None:
            begin, end = missing_sector
            ranges[(angles>=begin)&(angles<=end)] = np.nan
        return TimedScan(ranges, self.local_time(first_ray_fc_s),
            self.period/self.beams/self.scale, self.period/self.scale, -np.pi, 2*np.pi/self.beams)

    @staticmethod
    def _cylinder(ranges, positions, directions, centers, radius):
        delta = positions[:, :2]-centers
        a = np.sum(directions[:, :2]**2, axis=1)
        b = np.sum(delta*directions[:, :2], axis=1)
        c = np.sum(delta**2, axis=1)-radius**2
        determinant = b*b-a*c
        root = np.where(determinant>=0, (-b-np.sqrt(np.maximum(0, determinant)))/np.maximum(a, 1e-12), np.inf)
        return np.minimum(ranges, np.where(root>0, root, np.inf))


class SpinningScanOdometry(PlanarScanOdometry):
    def __init__(self, *, deskew=True, spatial_support=True, **kwargs):
        config = kwargs.pop('config', Config())
        config = replace(config, spatial_support=spatial_support, normal_half_window=4,
            estimate_range_bias=True,convergence_translation_m=.0005,convergence_yaw_rad=.0002)
        super().__init__(config=config, **kwargs)
        self.matcher.temporal_check = spatial_support
        self.clock = ClockAlignment()
        self.imu = ImuTimeline()
        self.deskew = deskew
        self.last_velocity = None
        self.last_velocity_stamp = None
        self.last_epoch = None
        self.last_obstacle_cloud = None

    def observe_scan(self, scan, *, now_local_s, height_above_floor_m):
        self.last_obstacle_cloud = None
        try:
            if not .75<=height_above_floor_m<=2.25:
                raise TimingError('floor_ceiling_clearance_unavailable')
            points, guard_valid, attitude, timing, bearings = deskew_scan(scan, self.clock, self.imu,
                now_local_s=now_local_s, velocity_enu_m_s=self.last_velocity,
                velocity_stamp_s=self.last_velocity_stamp, range_max_m=12.,
                enabled=self.deskew)
            valid = guard_valid & (scan.ranges<=self.range_max)
            if self.last_epoch is not None and timing['clock_epoch']!=self.last_epoch:
                self.matcher.reference = self.matcher.previous = None
                self.history.clear()
                self.last_velocity = self.last_velocity_stamp = None
            self.last_epoch = timing['clock_epoch']
            yaw = np.arctan2(attitude[1, 0], attitude[0, 0])
            stamp = timing['reference_fc_s']
            c,s = np.cos(yaw),np.sin(yaw)
            self.last_obstacle_cloud = dict(relative_enu_xy=points@np.array([[c,s],[-s,c]]),
                valid=guard_valid, bearings_enu_rad=bearings, first_ray_fc_s=timing['first_ray_fc_s'],
                reference_fc_s=stamp, acquisition_motion_bound_m=.12,
                clock_uncertainty_s=timing['clock_uncertainty_s'])
            record = self.observe_cloud(ordered_cloud(points, valid, self.config), stamp, yaw)
            record['timing'] = timing
            record['valid_beams'] = int(valid.sum())
            if record['accepted']:
                self.last_velocity = np.asarray(record['velocity_ne_m_s'])[::-1]
                self.last_velocity_stamp = record['sample_midpoint_s']
                # Gyro/time/prior correlation is not independent pose evidence.
                # Add a declared model margin to the existing conservative C.
                extra = (timing['clock_uncertainty_s']*.8/max(record['interval_s'], .05))**2
                extra += (.01 if timing['translation_prior_available'] else .04)**2
                covariance = np.asarray(record['covariance_ne_m2_s2'])+np.eye(2)*extra
                record['covariance_ne_m2_s2'] = covariance.tolist()
                record['source_lineage'] = ['rotating_ranges', 'measured_FC_gyro',
                    'FC_attitude_anchor', 'two_way_clock_alignment', 'previous_scan_velocity']
            else:
                self.last_velocity = self.last_velocity_stamp = None
            return record
        except TimingError as error:
            # Break continuity without substituting arrival time for capture time.
            self.matcher.previous = None
            self.history.clear()
            self.last_velocity = self.last_velocity_stamp = None
            return dict(accepted=False, reasons=[str(error)], simulation_only=True,
                physical_aiding_ready=False, reset_counter=max(0, self.matcher.segment)%256)
