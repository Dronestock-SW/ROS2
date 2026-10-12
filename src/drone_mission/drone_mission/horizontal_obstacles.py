"""Two-dimensional obstacle observations and a horizontal command limiter.

This file has no ROS/FC side effects. A missing return is unknown. The guard
uses a swept circular footprint, measured closing speed, observation age and
an explicit braking model. It cannot observe obstacles outside the scan plane.
The model parameters require measurement before any real vehicle deployment.
"""
from dataclasses import dataclass
import math

import numpy as np


def _finite(*values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


@dataclass(frozen=True)
class GuardConfig:
    radius_m: float
    clearance_m: float
    brake_accel_m_s2: float
    command_latency_s: float
    max_age_s: float
    max_speed_m_s: float
    range_error_m: float
    pose_error_m: float
    obstacle_speed_bound_m_s: float
    velocity_error_m_s: float
    bins: int = 72

    def __post_init__(self):
        positive = ('radius_m', 'brake_accel_m_s2', 'max_age_s', 'max_speed_m_s')
        nonnegative = ('clearance_m', 'command_latency_s', 'range_error_m',
                       'pose_error_m', 'obstacle_speed_bound_m_s', 'velocity_error_m_s')
        for name in positive + nonnegative:
            value = getattr(self, name)
            if not _finite(value) or value < 0 or (name in positive and value == 0):
                raise ValueError('invalid_guard_parameter:' + name)
        if type(self.bins) is not int or not 8 <= self.bins <= 720:
            raise ValueError('invalid_angular_bin_count')


@dataclass(frozen=True)
class ObstacleFrame:
    """Points and angular coverage in the fixed horizontal planning frame.

    source_origin_xy is the sensor origin at acquisition. Acquisition motion
    error must be bounded separately; an arbitrary 2D tilt projection is not
    accepted. pose_error_m in GuardConfig includes transformation uncertainty.
    """
    points_xy_m: tuple
    source_origin_xy_m: tuple
    known_bins: tuple
    stamp_ns: int
    sequence: int
    clock_id: str
    source_id: str
    epoch: str
    acquisition_motion_bound_m: float
    usable: bool
    reason: str


def scan_obstacles(scan, *, stamp_ns, sequence, clock_id, source_id, epoch,
                   sensor_origin_xy_m, sensor_yaw_rad, bins,
                   mount_confirmed, horizontal_plane_confirmed,
                   oldest_stamp_confirmed, acquisition_motion_bound_m):
    """Transform a scan using an explicitly supplied, same-time surveyed frame.

    The caller must supply the source pose at acquisition, not the latest pose.
    Pose used for collision checks is not exported as an EKF observation.
    """
    if (type(bins) is not int or not 8 <= bins <= 720
            or len(sensor_origin_xy_m) != 2
            or not _finite(*sensor_origin_xy_m, sensor_yaw_rad, acquisition_motion_bound_m)
            or acquisition_motion_bound_m < 0):
        raise ValueError('invalid_obstacle_transform_or_bounds')
    identity_ok = (type(stamp_ns) is int and stamp_ns > 0
                   and type(sequence) is int and sequence >= 0
                   and all(isinstance(v, str) and v for v in (clock_id, source_id, epoch)))
    confirmed = (mount_confirmed is True and horizontal_plane_confirmed is True
                 and oldest_stamp_confirmed is True)
    fields = [scan.get(k) for k in ('angle_min', 'angle_max', 'angle_increment',
                                   'range_min', 'range_max')]
    ranges = scan.get('ranges', ())
    geometry = len(ranges) >= 2 and _finite(*fields)
    if geometry:
        lo, hi, step, rlo, rhi = fields
        geometry = (step != 0 and rlo >= 0 and rhi > rlo
                    and abs(lo + step*(len(ranges)-1) - hi) <= abs(step)*.51
                    and abs(step) <= 2*math.pi/bins)
    seen = np.zeros(bins, dtype=int)
    invalid = np.zeros(bins, dtype=int)
    points = []
    if geometry:
        for i, distance in enumerate(ranges):
            angle = lo + step*i + sensor_yaw_rad
            index = int(math.floor((angle % (2*math.pi))/(2*math.pi)*bins)) % bins
            seen[index] += 1
            if not _finite(distance) or not rlo <= distance <= rhi:
                invalid[index] += 1
                continue
            points.append((sensor_origin_xy_m[0] + distance*math.cos(angle),
                           sensor_origin_xy_m[1] + distance*math.sin(angle)))
    reason = ('identity_invalid' if not identity_ok else
              'mount_plane_or_acquisition_time_unverified' if not confirmed else
              'scan_geometry_invalid' if not geometry else 'ready')
    return ObstacleFrame(tuple(points), tuple(sensor_origin_xy_m),
                         tuple(bool(v) for v in ((seen > 0) & (invalid == 0))),
                         stamp_ns, sequence, clock_id, source_id, epoch,
                         acquisition_motion_bound_m, reason == 'ready', reason)


def stopping_distance(speed_m_s, config, age_s):
    """Static braking distance plus bounded obstacle approach during braking."""
    tau = config.command_latency_s + age_s
    other = config.obstacle_speed_bound_m_s
    return ((speed_m_s + other)*tau + speed_m_s**2/(2*config.brake_accel_m_s2)
            + other*speed_m_s/config.brake_accel_m_s2)


def ordered_obstacles(*, relative_enu_xy, valid, bearings_enu_rad,
                      sensor_origin_xy_m, stamp_ns, sequence, clock_id,
                      source_id, epoch, bins, acquisition_motion_bound_m):
    """Deskewed beams in ENU, same-time FC origin, explicit oldest ray stamp.

    Used only by the experimental spinning SITL caller. Invalid beam bins
    remain unknown. Projected floor/ceiling hits are conservatively obstacles.
    This method never produces a vehicle state or an EKF input.
    """
    points = np.asarray(relative_enu_xy, dtype=float)
    valid = np.asarray(valid, dtype=bool)
    bearings = np.asarray(bearings_enu_rad, dtype=float)
    origin = np.asarray(sensor_origin_xy_m, dtype=float)
    if (points.ndim!=2 or points.shape[1]!=2 or valid.shape!=(len(points),)
            or bearings.shape!=valid.shape or origin.shape!=(2,)
            or not np.isfinite(points).all() or not np.isfinite(bearings).all()
            or not np.isfinite(origin).all() or not 8<=bins<=720
            or not _finite(acquisition_motion_bound_m) or acquisition_motion_bound_m<0
            or type(stamp_ns) is not int or stamp_ns<=0 or type(sequence) is not int or sequence<0
            or not all(isinstance(v,str) and v for v in (clock_id,source_id,epoch))):
        raise ValueError('invalid_deskewed_obstacle_contract')
    indices = np.floor((bearings%(2*np.pi))*bins/(2*np.pi)).astype(int)%bins
    seen = np.bincount(indices, minlength=bins)
    unknown = np.bincount(indices[~valid], minlength=bins)
    return ObstacleFrame(tuple(map(tuple, points[valid]+origin)), tuple(origin),
        tuple(bool(v) for v in ((seen>0)&(unknown==0))), stamp_ns, sequence,
        clock_id, source_id, epoch, acquisition_motion_bound_m, True, 'ready')


def speed_for_clearance(distance_m, config, age_s):
    tau = config.command_latency_s + age_s
    a, other = config.brake_accel_m_s2, config.obstacle_speed_bound_m_s
    available = distance_m - other*tau
    if available <= 0:
        return 0.
    b = a*tau + other
    return min(config.max_speed_m_s, max(0., math.sqrt(b*b + 2*a*available) - b))


class HorizontalGuard:
    """A single observation source, with explicit epoch and clock identity."""
    def __init__(self, config, *, source_id, clock_id, epoch):
        if not source_id or not clock_id or not epoch:
            raise ValueError('guard_identity_required')
        self.config = config
        self.identity = (source_id, clock_id, epoch)
        self.frame = None
        self.input_reason = 'scan_missing'
        self.last_sequence = -1
        self.last_stamp_ns = 0

    def observe(self, frame):
        # Invalid or reordered input invalidates the active observation. A
        # prior clear scan must not survive a detected clock/source failure.
        self.frame = None
        if (frame.source_id, frame.clock_id, frame.epoch) != self.identity:
            self.input_reason = 'scan_source_or_epoch_changed'
        elif not frame.usable:
            self.input_reason = frame.reason
        elif len(frame.known_bins) != self.config.bins:
            self.input_reason = 'angular_bin_contract_mismatch'
        elif frame.sequence <= self.last_sequence or frame.stamp_ns <= self.last_stamp_ns:
            self.input_reason = 'scan_duplicate_or_out_of_order'
        else:
            self.frame = frame
            self.last_sequence, self.last_stamp_ns = frame.sequence, frame.stamp_ns
            self.input_reason = 'ready'
        return self.input_reason

    def _clearance(self, direction, current_xy, age_s, velocity_error_margin_m=0.):
        frame, cfg = self.frame, self.config
        bin_width = 2*math.pi/cfg.bins
        centres = (np.arange(cfg.bins) + .5)*bin_width
        directions = np.column_stack((np.cos(centres), np.sin(centres)))
        # A bin touching the forward half-plane is required, including boundary
        # bins. This conservative policy is explicit; missing bins are not rays
        # of infinite free range. It is not a full 3D visibility certificate.
        relevant = directions @ direction >= -math.sin(bin_width/2)
        if not np.asarray(frame.known_bins)[relevant].all():
            return 0., 'direction_has_unknown_returns'
        points = np.asarray(frame.points_xy_m, dtype=float).reshape(-1, 2)
        delta = points - current_xy
        margin = (cfg.radius_m + cfg.clearance_m + cfg.range_error_m
                  + cfg.pose_error_m + frame.acquisition_motion_bound_m + velocity_error_margin_m)
        # Inflate each angular cell by its farthest transverse uncertainty.
        range_from_sensor = np.linalg.norm(points - frame.source_origin_xy_m, axis=1)
        radii = margin + range_from_sensor*math.sin(bin_width/2)
        along = delta @ direction
        lateral = np.abs(delta[:, 0]*direction[1] - delta[:, 1]*direction[0])
        in_corridor = lateral <= radii
        entry = along[in_corridor] - np.sqrt(np.maximum(
            0., radii[in_corridor]**2 - lateral[in_corridor]**2))
        exit_ = along[in_corridor] + np.sqrt(np.maximum(
            0., radii[in_corridor]**2 - lateral[in_corridor]**2))
        forward = entry[exit_ >= 0]
        if not len(forward):
            # Finite observed ranges bound the available free corridor. Do not
            # extrapolate beyond the sensor's actually observed endpoints.
            return 0., 'forward_corridor_not_bounded_by_returns'
        return max(0., float(forward.min())), 'observed_corridor'

    def limit(self, *, desired_xy_m_s, measured_xy_m_s, current_xy_m, now_ns):
        cfg = self.config
        requested = np.asarray(desired_xy_m_s, dtype=float)
        actual = np.asarray(measured_xy_m_s, dtype=float)
        current = np.asarray(current_xy_m, dtype=float)
        if any(v.shape != (2,) or not np.isfinite(v).all() for v in (requested, actual, current)):
            raise ValueError('invalid_horizontal_command_or_state')

        def result(velocity, reason, distance=None, age=None):
            return dict(velocity_xy_m_s=tuple(float(v) for v in velocity),
                        reason=reason, clearance_m=distance, scan_age_s=age,
                        source_sequence=self.frame.sequence if self.frame else None,
                        scope='observed_horizontal_plane_only')

        if self.frame is None:
            return result((0., 0.), self.input_reason)
        if type(now_ns) is not int or now_ns <= 0:
            return result((0., 0.), 'invalid_current_clock')
        age = (now_ns-self.frame.stamp_ns)/1e9
        if not 0 <= age <= cfg.max_age_s:
            return result((0., 0.), 'scan_stale_or_future', age=age)
        # First consider current momentum even when the requested direction is
        # away from a nearby obstacle. Zero is a braking request, not proof that
        # the aircraft has stopped; the controller response is tested in SITL.
        moving = float(np.linalg.norm(actual))
        # Bound displacement from an unknown velocity error in ANY direction
        # over the declared delay plus braking horizon. The bound must be
        # calibrated on the vehicle; the SITL value is an experimental assumption.
        error_horizon = (cfg.command_latency_s + age
                         + (max(moving, cfg.max_speed_m_s) + cfg.velocity_error_m_s)
                         / cfg.brake_accel_m_s2)
        error_margin = cfg.velocity_error_m_s * error_horizon
        if moving > 1e-9:
            distance, reason = self._clearance(actual/moving, current, age, error_margin)
            if reason != 'observed_corridor' or stopping_distance(moving, cfg, age) >= distance:
                return result((0., 0.), 'brake:' + reason, distance, age)
        speed = float(np.linalg.norm(requested))
        if speed <= 1e-9:
            return result((0., 0.), 'zero_requested', age=age)
        direction = requested/speed
        distance, reason = self._clearance(direction, current, age, error_margin)
        if reason != 'observed_corridor':
            return result((0., 0.), reason, distance, age)
        allowed = min(speed, speed_for_clearance(distance, cfg, age))
        if allowed <= 1e-9:
            return result((0., 0.), 'brake:clearance_exhausted', distance, age)
        return result(direction*allowed,
                      'limited' if allowed < speed else 'clear', distance, age)
