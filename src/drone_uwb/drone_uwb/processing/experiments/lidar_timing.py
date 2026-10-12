"""Measured-time scan preprocessing; no position state or flight controller.

Clock exchanges carry sensor capture time, not callback arrival time. Gyro
integration only deskews a scan; PX4 remains the vehicle attitude estimator.
The affine map and all bounds are experimental, not a hardware calibration.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class TimingError(ValueError):
    pass


class ClockAlignment:
    """Bounded affine sensor -> FC clock fit from two-way exchanges.

    Unknown link asymmetry is included as half the round-trip time. Reject
    stale fits, clock steps, implausible drift and insufficient exchanges.
    """
    def __init__(self, *, max_uncertainty_s=.002, max_age_s=.5):
        self.exchanges = deque(maxlen=64)
        self.max_uncertainty = max_uncertainty_s
        self.max_age = max_age_s
        self.epoch = 0

    def add_exchange(self, local_send_s, fc_reply_s, local_receive_s):
        values = np.array([local_send_s, fc_reply_s, local_receive_s])
        if not np.isfinite(values).all() or not 0 <= local_receive_s-local_send_s <= .02:
            return False
        midpoint = (local_send_s+local_receive_s)/2
        if self.exchanges and (midpoint <= self.exchanges[-1][0]
                or midpoint-self.exchanges[-1][0]>self.max_age):
            self.exchanges.clear()
            self.epoch += 1
        if len(self.exchanges) >= 8:
            try:
                predicted, _, _ = self.convert(midpoint, now_local_s=midpoint)
                if abs(predicted-fc_reply_s) > .005:
                    self.exchanges.clear()
                    self.epoch += 1
            except TimingError:
                pass
        self.exchanges.append((midpoint, fc_reply_s, local_receive_s-local_send_s))
        return True

    def convert(self, local_stamp_s, *, now_local_s):
        if not np.isfinite([local_stamp_s,now_local_s]).all():
            raise TimingError('nonfinite_clock_query')
        if len(self.exchanges) < 8:
            raise TimingError('clock_warmup')
        data = np.array(self.exchanges)
        if now_local_s-data[-1, 0] > self.max_age or now_local_s < data[-1, 0]-.02:
            raise TimingError('clock_stale')
        # Low RTT reduces queueing sensitivity without assuming symmetry.
        selected = data[np.argsort(data[:, 2])[:max(8, len(data)//2)]]
        origin = selected[:, 0].mean()
        x = selected[:, 0]-origin
        if np.ptp(x) < .15:
            raise TimingError('clock_baseline_short')
        slope, intercept = np.linalg.lstsq(np.column_stack((x, np.ones(len(x)))), selected[:, 1], rcond=None)[0]
        if abs(slope-1.) > .001:
            raise TimingError('clock_drift_limit')
        residual = selected[:, 1]-(slope*x+intercept)
        extrapolation = max(0., float(local_stamp_s)-selected[:, 0].max())
        uncertainty = np.max(np.abs(residual))+selected[:, 2].max()/2+extrapolation*.001
        if uncertainty > self.max_uncertainty:
            raise TimingError('clock_uncertainty')
        return slope*(np.asarray(local_stamp_s)-origin)+intercept, float(uncertainty), float(slope)


@dataclass(frozen=True)
class TimedScan:
    ranges: np.ndarray
    first_ray_s: float
    time_increment_s: float
    scan_time_s: float
    angle_min: float
    angle_increment: float
    clock_id: str = 'lidar'

    def times(self):
        n = len(self.ranges)
        if (n < 5 or not np.isfinite([self.first_ray_s, self.time_increment_s,
                self.scan_time_s, self.angle_min, self.angle_increment]).all()
                or self.time_increment_s <= 0 or self.scan_time_s <= 0
                or self.angle_increment == 0
                or (n-1)*self.time_increment_s > self.scan_time_s*1.01):
            raise TimingError('invalid_beam_timing')
        return self.first_ray_s+np.arange(n)*self.time_increment_s


class ImuTimeline:
    """FC capture-time gyro (FLU) plus FC attitude anchors (ENU <- FLU).

    No extrapolation across missing IMU samples. Gyros are the measured,
    bias-corrected HIGHRES_IMU values in SITL, never simulator truth rates.
    """
    def __init__(self, *, max_gap_s=.025, max_anchor_age_s=.04):
        self.gyros = deque(maxlen=1200)
        self.attitudes = deque(maxlen=600)
        self.max_gap = max_gap_s
        self.max_anchor_age = max_anchor_age_s

    def add_gyro(self, stamp_s, angular_rate_flu):
        angular = np.asarray(angular_rate_flu, dtype=float)
        if not np.isfinite(stamp_s) or angular.shape != (3,) or not np.isfinite(angular).all():
            raise TimingError('invalid_imu')
        if self.gyros and stamp_s <= self.gyros[-1][0]:
            return False
        self.gyros.append((float(stamp_s), angular.copy()))
        return True

    def add_attitude(self, stamp_s, rotation_enu_flu):
        r = np.asarray(rotation_enu_flu, dtype=float)
        if (r.shape != (3, 3) or not np.isfinite(r).all()
                or not np.allclose(r.T@r, np.eye(3), atol=1e-4) or np.linalg.det(r)<.999):
            raise TimingError('invalid_attitude_anchor')
        if self.attitudes and stamp_s <= self.attitudes[-1][0]:
            return False
        self.attitudes.append((float(stamp_s), r.copy()))
        return True

    def rotations(self, times):
        times = np.asarray(times, dtype=float)
        if times.ndim != 1 or len(times)<1 or not np.isfinite(times).all() or np.any(np.diff(times)<0):
            raise TimingError('invalid_query_times')
        anchors = [a for a in self.attitudes if a[0] <= times[0]+1e-9]
        if not anchors or times[0]-anchors[-1][0]>self.max_anchor_age:
            raise TimingError('attitude_anchor_missing')
        stamp, initial = anchors[-1]
        gyro_times = np.array([g[0] for g in self.gyros])
        if len(gyro_times)<2 or gyro_times[0]>stamp or gyro_times[-1]<times[-1]:
            raise TimingError('imu_not_bracketed')
        left = max(0, np.searchsorted(gyro_times, stamp, side='right')-1)
        right = min(len(gyro_times), np.searchsorted(gyro_times, times[-1])+1)
        gt = gyro_times[left:right]
        if len(gt)<2 or np.max(np.diff(gt))>self.max_gap:
            raise TimingError('imu_gap')
        gv = np.array([g[1] for g in self.gyros])[left:right]
        knots = np.unique(np.r_[stamp, gt[(gt>stamp)&(gt<times[-1])], times[-1]])
        if len(knots)==1:
            return np.repeat(initial[None,:,:], len(times), axis=0), dict(
                imu_max_gap_s=float(np.max(np.diff(gt))), attitude_anchor_age_s=0., imu_samples=len(gt))
        rotations = [Rotation.from_matrix(initial)]
        for start, end in zip(knots[:-1], knots[1:]):
            middle = (start+end)/2
            omega = np.array([np.interp(middle, gt, gv[:, axis]) for axis in range(3)])
            rotations.append(rotations[-1]*Rotation.from_rotvec(omega*(end-start)))
        output = Slerp(knots, Rotation.concatenate(rotations))(times).as_matrix()
        return output, dict(imu_max_gap_s=float(np.max(np.diff(gt))),
            attitude_anchor_age_s=float(times[0]-stamp), imu_samples=len(gt))


def deskew_scan(scan, alignment, imu, *, now_local_s, velocity_enu_m_s=None,
                velocity_stamp_s=None, range_min_m=.2, range_max_m=4., enabled=True):
    """Return ordered level points at the middle ray, retaining invalid slots.

    Translation uses only a recent previously accepted scan velocity. It never
    receives UWB/FC velocity or truth. An unavailable prior is zero with an
    explicit bounded extra uncertainty, not a claim of observed zero motion.
    """
    local_times = scan.times()
    first, uncertainty, scale = alignment.convert(local_times[0], now_local_s=now_local_s)
    times = first+(local_times-local_times[0])*scale
    rotations, diagnostics = imu.rotations(times)
    mid = (times[0]+times[-1])/2
    mid_rotation, _ = imu.rotations(np.array([mid]))
    mid_rotation = mid_rotation[0]
    if np.min(rotations[:, 2, 2]) < np.cos(np.deg2rad(10.)):
        raise TimingError('scan_tilt_limit')
    angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
    valid = np.isfinite(scan.ranges)&(scan.ranges>=range_min_m)&(scan.ranges<=range_max_m)
    ranges = np.where(valid, scan.ranges, 0.)
    body = np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles), np.zeros(len(ranges))))
    velocity = np.zeros(2)
    has_prior = velocity_enu_m_s is not None and velocity_stamp_s is not None and 0<mid-velocity_stamp_s<=.25
    if has_prior:
        velocity = np.asarray(velocity_enu_m_s, dtype=float)
        if velocity.shape != (2,) or not np.isfinite(velocity).all() or np.linalg.norm(velocity)>.8:
            raise TimingError('invalid_deskew_velocity_prior')
    world = np.einsum('nij,nj->ni', rotations, body) if enabled else body@mid_rotation.T
    if enabled:
        world[:, :2] += (times-mid)[:, None]*velocity
    yaw = np.arctan2(mid_rotation[1, 0], mid_rotation[0, 0])
    c, s = np.cos(yaw), np.sin(yaw)
    level = world[:, :2]@np.array([[c, -s], [s, c]])
    unit_body = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    unit_world = np.einsum('nij,nj->ni', rotations, unit_body)
    bearings = np.arctan2(unit_world[:,1], unit_world[:,0])
    diagnostics.update(first_ray_fc_s=float(times[0]), last_ray_fc_s=float(times[-1]),
        reference_fc_s=float(mid), clock_uncertainty_s=uncertainty, clock_scale=scale,
        clock_epoch=alignment.epoch, deskew_applied=bool(enabled),
        translation_prior_available=bool(has_prior), rotation_span_rad=float(
            (Rotation.from_matrix(rotations[0]).inv()*Rotation.from_matrix(rotations[-1])).magnitude()),
        motion_uncertainty_m=(.06 if has_prior else .8)*(times[-1]-times[0])/2)
    return level, valid, mid_rotation, diagnostics, bearings
