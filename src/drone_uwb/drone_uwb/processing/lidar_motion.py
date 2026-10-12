"""Measured SDK rays -> IMU-origin horizontal interval velocity.

No UWB or FC position/velocity enters this estimator. FC capture-time attitude
and gyro are used only for scan compensation and heading. The uncertainty
budget is an explicit provisional engineering model, not accuracy evidence.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .lidar_imu_timing import ImuTimeline, TimingError
from .lidar_matching import Config, KeyframeMatcher, ordered_cloud, rotation


@dataclass(frozen=True)
class LidarMotionSettings:
    enabled: bool = False
    tag_id: str = '6'
    source_frame: str = 'laser_frame'
    imu_frame: str = 'base_link'
    sensor_to_body_flu_m: tuple | None = None
    sensor_to_body_yaw_rad: float | None = None
    mount_position_ref: str = ''
    mount_yaw_ref: str = ''
    scan_plane_parallel_confirmed: bool = False
    timing_ref: str = ''
    max_observation_age_s: float = .18
    max_receipt_age_s: float = .15
    sensor_time_offset_s: float = 0.
    timing_uncertainty_s: float = .01
    range_min_m: float = .20
    range_max_m: float = 4.
    min_sensor_height_m: float = .10
    max_sensor_height_m: float = 2.25
    max_speed_m_s: float = .8
    max_tilt_deg: float = 10.
    interval_count: int = 1
    velocity_stddev_m_s: float = .15
    displacement_stddev_m: float = .02
    heading_stddev_rad: float = .15
    mount_stddev_m: float = .02
    uncertainty_model: str = 'provisional_engineering_bound'
    expected_avel_delay_ms: float = 5.

    def __post_init__(self):
        if self.tag_id not in ('5', '6') or type(self.enabled) is not bool:
            raise ValueError('invalid_lidar_identity_or_enable')
        if self.sensor_to_body_flu_m is not None:
            arm = np.asarray(self.sensor_to_body_flu_m, dtype=float)
            if arm.shape != (3,) or not np.isfinite(arm).all() or np.linalg.norm(arm) > 1.:
                raise ValueError('invalid_lidar_mount')
        if self.sensor_to_body_yaw_rad is not None and not math.isfinite(self.sensor_to_body_yaw_rad):
            raise ValueError('invalid_mount_yaw')
        if self.interval_count not in (1, 2):
            raise ValueError('invalid_velocity_interval_count')
        positive = ('max_observation_age_s', 'max_receipt_age_s', 'timing_uncertainty_s',
            'range_min_m', 'range_max_m', 'min_sensor_height_m', 'max_sensor_height_m',
            'max_speed_m_s', 'max_tilt_deg', 'velocity_stddev_m_s',
            'displacement_stddev_m', 'heading_stddev_rad', 'mount_stddev_m')
        if any(type(getattr(self, key)) not in (int, float) or not math.isfinite(getattr(self,key))
               or getattr(self,key) <= 0 for key in positive):
            raise ValueError('invalid_lidar_limit_or_uncertainty')
        if (self.max_observation_age_s > .2 or self.max_receipt_age_s > .2
                or self.max_tilt_deg > 15. or self.max_speed_m_s > 1.
                or self.range_min_m >= self.range_max_m
                or self.min_sensor_height_m >= self.max_sensor_height_m
                or not math.isfinite(self.sensor_time_offset_s) or abs(self.sensor_time_offset_s) > .05):
            raise ValueError('lidar_limit_outside_supported_contract')
        if self.uncertainty_model != 'provisional_engineering_bound':
            raise ValueError('unsupported_physical_uncertainty_model')

    def geometry_gate(self):
        if self.sensor_to_body_flu_m is None or not self.mount_position_ref:
            return 'mount_position_unconfirmed'
        if self.sensor_to_body_yaw_rad is None or not self.mount_yaw_ref:
            return 'mount_yaw_unconfirmed'
        if self.scan_plane_parallel_confirmed is not True:
            return 'scan_plane_unconfirmed'
        if not self.timing_ref:
            return 'acquisition_timing_unconfirmed'
        return 'ready'


def sdk_points(points, offsets):
    """Acquisition order from SDK PointCloud.stamps, never LaserScan bins."""
    p, t = np.asarray(points, dtype=float), np.asarray(offsets, dtype=float)
    if p.ndim != 2 or p.shape[1:] != (3,) or not 40 <= len(p) <= 2048 or t.shape != (len(p),):
        raise TimingError('sdk_points_or_times_shape')
    if not np.isfinite(p).all() or not np.isfinite(t).all() or np.max(np.abs(p[:,2])) > .001:
        raise TimingError('sdk_points_nonfinite_or_nonplanar')
    if t[0] < 0 or t[-1] > .15 or np.any(np.diff(t) <= 0):
        raise TimingError('sdk_times_invalid')
    keep = np.r_[True, np.any(np.diff(p, axis=0) != 0, axis=1)]
    # Keep zero range slots: they break neighborhood normals at missing rays.
    keep |= np.linalg.norm(p[:,:2], axis=1) < .001
    if keep.sum() < 40:
        raise TimingError('sdk_distinct_points_missing')
    return p[keep], t[keep], int((~keep).sum())


class MeasuredLidarMotion:
    def __init__(self, settings):
        self.settings = settings
        self.imu = ImuTimeline()
        self.config = Config(spatial_support=True, estimate_range_bias=True,
            normal_half_window=4, convergence_translation_m=.0005,
            convergence_yaw_rad=.0002, range_min_m=settings.range_min_m)
        self.matcher = KeyframeMatcher(self.config, require_imu=True)
        self.matcher.temporal_check = True
        self.history = deque(maxlen=settings.interval_count+1)
        self.heading = None
        self.segment = None
        self.last_velocity = self.last_velocity_stamp = None
        self.last_first_ns = None

    def break_continuity(self):
        self.matcher.previous = None
        self.history.clear()
        self.last_velocity = self.last_velocity_stamp = None

    def reset_clock(self):
        self.imu = ImuTimeline()
        self.matcher.reference = self.matcher.previous = None
        self.matcher.last_seen = None
        self.last_first_ns = None
        self.break_continuity()

    def observe(self, *, points, offsets, first_stamp_ns, now_ns, receipt_age_s,
                frame_id, floor_range_m, floor_stamp_ns):
        cfg = self.settings
        base = dict(accepted=False, observed_axes=['x','y'], z_observed=False,
            reference_point='fc_imu', physical_accuracy_verified=False,
            uncertainty_model=cfg.uncertainty_model)
        try:
            reason = cfg.geometry_gate()
            if reason != 'ready':
                raise TimingError(reason)
            if frame_id != cfg.source_frame:
                raise TimingError('lidar_frame_mismatch')
            p, offset, dropped = sdk_points(points, offsets)
            if (type(first_stamp_ns) is not int or first_stamp_ns <= 0
                    or type(now_ns) is not int or self.last_first_ns is not None and first_stamp_ns <= self.last_first_ns):
                raise TimingError('lidar_capture_time_nonmonotonic')
            times = first_stamp_ns/1e9+cfg.sensor_time_offset_s+offset
            mid = float((times[0]+times[-1])/2)
            if (not 0 <= receipt_age_s <= cfg.max_receipt_age_s
                    or not 0 <= now_ns/1e9-times[-1] <= cfg.max_receipt_age_s):
                raise TimingError('lidar_scan_stale_or_future')
            if (not math.isfinite(floor_range_m) or not isinstance(floor_stamp_ns, int)
                    or not 0 <= (now_ns-floor_stamp_ns)/1e9 <= .15):
                raise TimingError('measured_floor_range_unavailable')
            rotations, diagnostic = self.imu.rotations(times)
            middle_rotation = self.imu.rotations(np.array([mid]))[0][0]
            if rotations[:,2,2].min() < math.cos(math.radians(cfg.max_tilt_deg)):
                raise TimingError('scan_tilt_limit')
            arm = np.asarray(cfg.sensor_to_body_flu_m)
            sensor_height = floor_range_m*middle_rotation[2,2]+float((middle_rotation@arm)[2])
            if not cfg.min_sensor_height_m <= sensor_height <= cfg.max_sensor_height_m:
                raise TimingError('sensor_floor_clearance_unavailable')
            sensor_rotation = Rotation.from_euler('z', cfg.sensor_to_body_yaw_rad).as_matrix()
            body = p@sensor_rotation.T+arm
            # Including the measured lever arm here makes cloud translation an
            # IMU-origin displacement, also during roll/pitch/yaw rotation.
            world = np.einsum('nij,nj->ni', rotations, body)
            has_prior = (self.last_velocity is not None and self.last_velocity_stamp is not None
                and 0 < mid-self.last_velocity_stamp <= .25)
            if has_prior:
                world[:,:2] += (times-mid)[:,None]*self.last_velocity
            yaw = math.atan2(middle_rotation[1,0], middle_rotation[0,0])
            level = world[:,:2]@rotation(yaw)
            ranges = np.linalg.norm(p[:,:2], axis=1)
            valid = (ranges >= cfg.range_min_m)&(ranges <= cfg.range_max_m)
            self.last_first_ns = first_stamp_ns
            obs = self.matcher.observe(ordered_cloud(level, valid, self.config), mid, imu_yaw_rad=yaw)
            base.update(reasons=obs.reasons, reset_counter=max(0,obs.segment)%256,
                segment=obs.segment, valid_beams=int(valid.sum()), duplicate_points_dropped=dropped,
                scan_midpoint_ns=round(mid*1e9), timing=dict(diagnostic,
                    scan_span_s=float(times[-1]-times[0]), translation_prior_available=has_prior,
                    sensor_time_offset_s=cfg.sensor_time_offset_s,
                    timing_uncertainty_s=cfg.timing_uncertainty_s), sensor_height_m=sensor_height)
            if obs.match is not None:
                base['match'] = {k:float(getattr(obs.match,k)) for k in
                    ('overlap','residual_p90_m','condition','correspondences','spatial_cells',
                     'range_bias_m','translation_information_ratio')}
                base['match'] = {k:v if math.isfinite(v) else None for k,v in base['match'].items()}
            if obs.segment != self.segment:
                self.segment, self.heading = obs.segment, yaw
                self.history.clear()
            if obs.pose is None:
                self.break_continuity()
                return base
            self.history.append((mid, obs.pose.copy()))
            if not obs.accepted or len(self.history) < self.history.maxlen:
                base['reasons'] = obs.reasons or ['interval_history_warmup']
                self.last_velocity = self.last_velocity_stamp = None
                return base
            start, previous = self.history[0]
            duration = mid-start
            velocity = rotation(self.heading)@(obs.pose[:2]-previous[:2])/duration
            midpoint_ns = round((start+mid)*.5e9)
            if np.linalg.norm(velocity) > cfg.max_speed_m_s:
                raise TimingError('horizontal_speed_limit')
            if not 0 <= (now_ns-midpoint_ns)/1e9 <= cfg.max_observation_age_s:
                raise TimingError('interval_observation_stale')
            # Endpoint displacement, capture time, translation prior, mounting
            # and correlated FC heading uncertainty remain explicit margins.
            var = cfg.velocity_stddev_m_s**2+2*cfg.displacement_stddev_m**2/duration**2
            var += (cfg.max_speed_m_s*cfg.timing_uncertainty_s/duration)**2
            var += (.03 if has_prior else .10)**2+(2*cfg.mount_stddev_m/duration)**2
            jac = np.array([-velocity[1], velocity[0]])
            covariance = np.eye(2)*var+cfg.heading_stddev_rad**2*np.outer(jac,jac)
            self.last_velocity, self.last_velocity_stamp = velocity.copy(), (start+mid)/2
            base.update(accepted=True, reasons=[], velocity_ne_m_s=velocity[::-1].tolist(),
                covariance_ne_m2_s2=covariance[::-1,::-1].tolist(),
                interval_start_ns=round(start*1e9), interval_end_ns=round(mid*1e9),
                representative_stamp_ns=midpoint_ns, interval_s=duration,
                measurement_kind='interval_average_velocity',
                source_lineage=['sdk_pointcloud_stamps','FC_capture_time_attitude_and_gyro',
                    'confirmed_sensor_to_IMU_mount','previous_accepted_lidar_velocity'],
                frame_id='local_ned', quality=1)
            return base
        except (TimingError, ValueError, ArithmeticError) as error:
            self.break_continuity()
            base['reasons'] = [str(error)]
            return base
