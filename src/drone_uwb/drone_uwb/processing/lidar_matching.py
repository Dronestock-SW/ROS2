"""File-only planar scan matching experiment. No ROS, FC output or flight state.

All poses map CURRENT sensor coordinates into REFERENCE sensor coordinates.
Yaw is counter-clockwise about the scan +Z axis. IMU rotation is only used for
initialization and consistency checks, never fused as an independent LiDAR yaw.
The diagnostics are NOT calibrated measurement covariance or flight approval.
"""
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.spatial import cKDTree


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def compose(a, b):
    return np.r_[a[:2] + rotation(a[2]) @ b[:2], wrap(a[2] + b[2])]


def inverse(a):
    return np.r_[-rotation(-a[2]) @ a[:2], -a[2]]


@dataclass(frozen=True)
class Config:
    range_min_m: float = .20
    edge_m: float = .30
    eigen_ratio: float = .12
    correspondence_m: float = .20
    huber_m: float = .03
    trim_quantile: float = .90
    iterations: int = 30
    convergence_translation_m: float = 1e-4
    convergence_yaw_rad: float = 5e-5
    min_correspondences: int = 60
    min_overlap: float = .50
    residual_p90_m: float = .05
    max_condition: float = 1000.
    max_pair_translation_m: float = .20
    max_pair_yaw_rad: float = math.radians(10.)
    max_gap_s: float = .25
    reverse_translation_m: float = .02
    reverse_yaw_rad: float = math.radians(.30)
    imu_yaw_tolerance_rad: float = math.radians(1.)
    keyframe_translation_m: float = .35
    keyframe_yaw_rad: float = math.radians(12.)
    spatial_support: bool = False
    min_translation_information_ratio: float = .06
    spatial_cell_m: float = .50
    max_cell_weight: float = 8.
    normal_half_window: int = 2
    estimate_range_bias: bool = False

    def __post_init__(self):
        values = vars(self)
        if any(not np.isfinite(v) or v <= 0 for k,v in values.items()
               if k not in ('spatial_support','estimate_range_bias')):
            raise ValueError('matcher parameters must be positive and finite')
        if not 0 < self.trim_quantile <= 1 or not 0 < self.min_overlap <= 1:
            raise ValueError('invalid matching fraction')
        if self.min_correspondences < 3 or self.iterations < 1:
            raise ValueError('invalid matcher count')
        if type(self.normal_half_window) is not int or not 2<=self.normal_half_window<=8:
            raise ValueError('invalid_normal_window')


@dataclass
class Cloud:
    points: np.ndarray
    normals: np.ndarray
    tree: object = field(init=False, repr=False)

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float)
        self.normals = np.asarray(self.normals, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 2 or self.normals.shape != self.points.shape:
            raise ValueError('cloud shape must be N by 2')
        if not np.isfinite(self.points).all() or not np.isfinite(self.normals).all():
            raise ValueError('nonfinite cloud')
        self.tree = cKDTree(self.points)


def scan_cloud(ranges, angle_min, angle_increment, range_min, range_max, config=Config()):
    """Use five consecutive valid beams; do not join across invalid returns."""
    ranges = np.array([float(r) if r is not None else np.nan for r in ranges])
    n = len(ranges)
    if n < 5 or not np.isfinite([angle_min, angle_increment, range_min, range_max]).all():
        raise ValueError('invalid scan geometry')
    if angle_increment == 0 or range_max <= max(0, range_min):
        raise ValueError('invalid scan bounds')
    angles = angle_min + np.arange(n) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= max(range_min, config.range_min_m)) & (ranges <= range_max)
    finite_ranges = np.where(valid, ranges, 0.)
    points = np.column_stack((finite_ranges * np.cos(angles), finite_ranges * np.sin(angles)))
    return ordered_cloud(points,valid,config)


def ordered_cloud(points, valid, config=Config()):
    """Build normals after a declared geometric projection of ordered beams.

    Keep invalid beam slots so five-point windows never cross missing returns.
    Projection timing, attitude and covariance are the caller's responsibility.
    """
    points=np.asarray(points,dtype=float);valid=np.asarray(valid,dtype=bool)
    if points.ndim!=2 or points.shape[1]!=2 or valid.shape!=(len(points),):
        raise ValueError('ordered_points_shape')
    valid=valid & np.isfinite(points).all(axis=1)
    n=len(points)
    half = config.normal_half_window
    indices = np.arange(half, n - half)
    windows = indices[:, None] + np.arange(-half, half+1)
    good = valid[windows].all(axis=1)
    # Select finite windows before eigendecomposition; NaN is never free space.
    windows, indices = windows[good], indices[good]
    local = points[windows]
    if len(local) == 0:
        return Cloud(np.empty((0, 2)), np.empty((0, 2)))
    good = (np.linalg.norm(np.diff(local, axis=1), axis=2) <= config.edge_m).all(axis=1)
    local, indices = local[good], indices[good]
    centered = local - local.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered) / (2*half+1)
    eig, vectors = np.linalg.eigh(covariance)
    good = (eig[:, 1] > 1e-10) & (eig[:, 0] <= config.eigen_ratio * eig[:, 1])
    return Cloud(points[indices[good]], vectors[good, :, 0])


@dataclass
class Match:
    pose: np.ndarray
    accepted: bool
    reasons: list
    correspondences: int = 0
    overlap: float = 0.
    residual_p90_m: float = float('inf')
    condition: float = float('inf')
    iterations: int = 0
    spatial_cells: int = 0
    translation_information_ratio: float = 0.
    range_bias_m: float = 0.


def match(reference, current, initial=None, config=Config()):
    """Robust Gauss-Newton point-to-line ICP, not Censi's exact PLICP solver.

    Translation and yaw updates are additive in the reference frame. Thus the
    yaw Jacobian uses ROTATED source points, without the current translation.
    Final diagnostics are recomputed after the final update.
    """
    pose = np.zeros(3) if initial is None else np.array(initial, dtype=float, copy=True)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError('invalid initial transform')
    if min(len(reference.points), len(current.points)) < config.min_correspondences:
        return Match(pose, False, ['insufficient_points'])
    bias = 0.
    unit = current.points/np.maximum(np.linalg.norm(current.points,axis=1)[:,None],.1)

    def linearize():
        rotated = (current.points+bias*unit) @ rotation(pose[2]).T
        transformed = rotated + pose[:2]
        distances, nearest = reference.tree.query(transformed)
        keep = distances <= config.correspondence_m
        normals = reference.normals[nearest]
        residual = np.einsum('ij,ij->i', normals, transformed - reference.points[nearest])
        if keep.any():
            keep &= np.abs(residual) <= np.quantile(np.abs(residual[keep]), config.trim_quantile)
        r = residual[keep]
        normal, rotated_kept = normals[keep], rotated[keep]
        jacobian = np.column_stack((normal, normal[:, 1] * rotated_kept[:, 0] - normal[:, 0] * rotated_kept[:, 1]))
        if config.estimate_range_bias:
            # Relative per-scan range offset is a nuisance, not motion. A
            # partial view otherwise turns common range error into XY motion.
            rotated_unit = unit[keep]@rotation(pose[2]).T
            jacobian = np.column_stack((jacobian,np.sum(normal*rotated_unit,axis=1)))
        weights = np.minimum(1., config.huber_m / np.maximum(np.abs(r), 1e-12))
        cells = np.floor(reference.points[nearest[keep]]/config.spatial_cell_m).astype(int)
        _, indices, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
        if config.spatial_support and len(r):
            # Dense returns from one nearby moving surface cannot dominate.
            weights *= np.minimum(1., config.max_cell_weight/counts[indices])
        radius = max(float(np.sqrt(np.mean(np.sum(rotated_kept ** 2, axis=1)))) if len(r) else 0., .1)
        scale = np.array([1., 1., 1. / radius]+([1.] if config.estimate_range_bias else []))
        scaled = jacobian * scale
        information = scaled.T @ (weights[:, None] * scaled)
        eigenvalues = np.linalg.eigvalsh(information)
        condition = eigenvalues[-1] / eigenvalues[0] if eigenvalues[0] > 1e-10 else float('inf')
        # Translation observability after marginalizing the yaw nuisance.
        marginal = information[:2,:2].copy()
        marginal -= information[:2,2:]@np.linalg.pinv(information[2:,2:])@information[2:,:2]
        translation_eig = np.linalg.eigvalsh(marginal)
        ratio = max(0.,float(translation_eig[0]))/max(float(np.trace(information[:2,:2])),1e-10)
        return r, jacobian, weights, condition, (len(counts), ratio)

    converged = False
    iterations = 0
    for iterations in range(1, config.iterations + 1):
        residual, jacobian, weights, condition, _ = linearize()
        if len(residual) < config.min_correspondences or condition > config.max_condition:
            break
        root_weights = np.sqrt(weights)
        update = np.linalg.lstsq(jacobian * root_weights[:, None], -residual * root_weights, rcond=None)[0]
        pose += update[:3]
        if config.estimate_range_bias:
            bias += update[3]
        pose[2] = wrap(pose[2])
        # Discrete nearest-neighbour assignments need a finite numerical
        # tolerance. This is below the simulated 3 mm range noise, not an
        # empirically established sensor accuracy or mission acceptance bound.
        if (np.linalg.norm(update[:2]) < config.convergence_translation_m
                and abs(update[2]) < config.convergence_yaw_rad
                and (not config.estimate_range_bias or abs(update[3])<config.convergence_translation_m)):
            converged = True
            break
    residual, _, _, condition, support = linearize()
    count = len(residual)
    overlap = count / len(current.points)
    p90 = float(np.quantile(np.abs(residual), .90)) if count else float('inf')
    reasons = []
    if count < config.min_correspondences:
        reasons.append('insufficient_correspondences')
    if overlap < config.min_overlap:
        reasons.append('low_overlap')
    if p90 > config.residual_p90_m:
        reasons.append('large_residual')
    if condition > config.max_condition:
        reasons.append('degenerate_geometry')
    if config.spatial_support and support[1]<config.min_translation_information_ratio:
        reasons.append('weak_translation_direction')
    if not converged:
        reasons.append('not_converged')
    if abs(bias)>.03:
        reasons.append('range_bias_limit')
    return Match(pose, not reasons, reasons, count, overlap, p90, float(condition), iterations, *support,bias)


@dataclass
class Observation:
    accepted: bool
    reasons: list
    stamp_s: float
    segment: int
    keyframe: int
    pose: object = None
    delta: object = None
    interval_s: object = None
    sample_midpoint_s: object = None
    velocity_scan_midpoint_mps: object = None
    match: object = None
    reverse_error: object = None
    imu_error_rad: object = None


class KeyframeMatcher:
    """Diagnostic relative displacement only; no global localization feedback.

    A rejected frame invalidates velocity continuity. Reacquisition returns a
    pose-only sample before fresh adjacent displacement can be emitted again.
    Long gaps start a new segment; accumulated poses never bridge that gap.
    """
    def __init__(self, config=Config(), *, require_imu=True):
        self.config = config
        self.require_imu = require_imu
        self.reference = None
        self.last_seen = None
        self.previous = None
        self.previous_stamp = None
        self.key_pose = np.zeros(3)
        self.key_imu_yaw = None
        self.reference_stamp = None
        self.segment = -1
        self.keyframe = -1
        self.initial = np.zeros(3)
        self.temporal_check = False
        self.previous_cloud = None

    def observe(self, cloud, stamp_s, *, imu_yaw_rad=None):
        if not np.isfinite(stamp_s) or (imu_yaw_rad is not None and not np.isfinite(imu_yaw_rad)):
            raise ValueError('nonfinite sensor time or IMU yaw')
        if self.last_seen is not None and stamp_s <= self.last_seen:
            self.previous = None
            return Observation(False, ['nonmonotonic_time'], stamp_s, self.segment, self.keyframe)
        gap = self.last_seen is not None and stamp_s - self.last_seen >= self.config.max_gap_s
        self.last_seen = stamp_s
        if gap:
            self.reference = self.previous = None
        if self.require_imu and imu_yaw_rad is None:
            self.previous = None
            return Observation(False, ['imu_unavailable'], stamp_s, self.segment, self.keyframe)
        if len(cloud.points) < self.config.min_correspondences:
            self.previous = None
            return Observation(False, ['insufficient_points'], stamp_s, self.segment, self.keyframe)
        if self.reference is None:
            # Do not seed a reference whose geometry itself cannot constrain SE(2).
            check = match(cloud, cloud, config=self.config)
            if not check.accepted:
                return Observation(False, check.reasons, stamp_s, self.segment, self.keyframe, match=check)
            self.reference = cloud
            self.reference_stamp = stamp_s
            self.key_imu_yaw = imu_yaw_rad
            self.key_pose = self.initial = np.zeros(3)
            self.previous, self.previous_stamp = np.zeros(3), stamp_s
            self.previous_cloud = cloud
            self.segment += 1
            self.keyframe += 1
            return Observation(False, ['new_segment'], stamp_s, self.segment, self.keyframe, pose=np.zeros(3))
        initial = self.initial.copy()
        imu_delta = None
        if imu_yaw_rad is not None and self.key_imu_yaw is not None:
            imu_delta = wrap(imu_yaw_rad - self.key_imu_yaw)
            initial[2] = imu_delta
        forward = match(self.reference, cloud, initial, self.config)
        # Independent reverse optimization, not the algebraic inverse of forward.
        backward = match(cloud, self.reference, inverse(initial), self.config)
        reverse_error = compose(forward.pose, backward.pose)
        reasons = list(forward.reasons)
        if not backward.accepted:
            reasons.append('reverse_rejected')
        if np.linalg.norm(reverse_error[:2]) > self.config.reverse_translation_m or abs(reverse_error[2]) > self.config.reverse_yaw_rad:
            reasons.append('bidirectional_inconsistent')
        imu_error = wrap(forward.pose[2] - imu_delta) if imu_delta is not None else None
        if imu_error is not None and abs(imu_error) > self.config.imu_yaw_tolerance_rad:
            reasons.append('imu_rotation_inconsistent')
        pose = compose(self.key_pose, forward.pose)
        delta = None if self.previous is None else compose(inverse(self.previous), pose)
        dt = None if self.previous is None else stamp_s - self.previous_stamp
        if delta is not None and (np.linalg.norm(delta[:2]) > self.config.max_pair_translation_m or abs(delta[2]) > self.config.max_pair_yaw_rad):
            reasons.append('pair_motion_limit')
        if self.temporal_check and delta is not None and self.previous_cloud is not None:
            adjacent = match(self.previous_cloud, cloud, delta, self.config)
            discrepancy = compose(inverse(delta), adjacent.pose)
            if (not adjacent.accepted or np.linalg.norm(discrepancy[:2])>.025
                    or abs(discrepancy[2])>math.radians(.5)):
                reasons.append('short_window_inconsistent')
        if reasons:
            self.previous = None
            return Observation(False, reasons, stamp_s, self.segment, self.keyframe,
                               match=forward, reverse_error=reverse_error, imu_error_rad=imu_error)
        # Δt and midpoint retain the two original observation times. Do not label
        # this interval-average velocity as an instantaneous end-of-scan velocity.
        velocity = None if delta is None else rotation(-delta[2] / 2.) @ delta[:2] / dt
        midpoint = None if dt is None else (stamp_s + self.previous_stamp) / 2.
        result = Observation(delta is not None, [] if delta is not None else ['reacquired_pose_only'],
                             stamp_s, self.segment, self.keyframe, pose, delta, dt, midpoint,
                             velocity, forward, reverse_error, imu_error)
        self.previous, self.previous_stamp = pose, stamp_s
        self.previous_cloud = cloud
        self.initial = forward.pose.copy()
        if np.linalg.norm(forward.pose[:2]) >= self.config.keyframe_translation_m or abs(forward.pose[2]) >= self.config.keyframe_yaw_rad:
            self.reference, self.key_pose, self.key_imu_yaw = cloud, pose, imu_yaw_rad
            self.reference_stamp = stamp_s
            self.initial = np.zeros(3)
            self.keyframe += 1
        return result


def uwb_velocity_from_lidar(velocity_lidar_frd, angular_rate_frd, uwb_to_lidar_frd):
    """Same body axes: v_UWB = v_LiDAR - omega cross r_UWB_to_LiDAR.

    This pure sensitivity calculation does not choose or confirm a real mount.
    It does not perform PX4's separate UWB-to-IMU correction.
    """
    vectors = [np.asarray(v, dtype=float) for v in (velocity_lidar_frd, angular_rate_frd, uwb_to_lidar_frd)]
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in vectors):
        raise ValueError('expected three finite FRD vectors')
    return vectors[0] - np.cross(vectors[1], vectors[2])
