"""Capture-time FC gyro integration for LiDAR compensation only."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

class TimingError(ValueError):
    pass

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
