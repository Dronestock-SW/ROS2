"""Horizontal LiDAR observation contracts. No ROS publisher or FC command.

An angular LaserScan bin is not necessarily an acquisition-order sample. The
YDLIDAR fixed-resolution path needs independent provenance before deskew. Shape
validity and temporal validity are deliberately separate. Unknown covariance is
never replaced by zero, and no unobserved vertical velocity is constructed.
"""
from dataclasses import dataclass
import math

import numpy as np


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _stamp(value):
    return type(value) is int and value > 0


def covariance(value, size, *, positive=False):
    """Validate rather than silently symmetrize or repair caller uncertainty."""
    c = np.asarray(value, dtype=float)
    if c.shape != (size, size) or not np.isfinite(c).all():
        raise ValueError('covariance_shape_or_nonfinite')
    if not np.allclose(c, c.T, atol=1e-12, rtol=1e-9):
        raise ValueError('covariance_not_symmetric')
    smallest = float(np.linalg.eigvalsh(c).min())
    if smallest < -1e-12 or (positive and smallest <= 0):
        raise ValueError('covariance_not_positive')
    return c


@dataclass(frozen=True)
class TimingProvenance:
    clock_id: str = ''
    header_is_first_acquired_ray: bool = False
    bins_follow_acquisition_order: bool = False
    per_ray_increment_verified: bool = False
    evidence: str = ''

    @property
    def verified(self):
        return (bool(self.clock_id) and bool(self.evidence)
                and self.header_is_first_acquired_ray is True
                and self.bins_follow_acquisition_order is True
                and self.per_ray_increment_verified is True)


@dataclass(frozen=True)
class ScanAudit:
    geometry_valid: bool
    fresh: bool
    declared_timing_consistent: bool
    deskew_ready: bool
    valid_returns: int
    beam_count: int
    declared_span_s: float | None
    span_period_ratio: float | None
    oldest_age_s: float | None
    reasons: tuple


def audit_scan(scan, *, header_ns, received_ns, now_ns,
               provenance=TimingProvenance(), max_age_s=.25):
    if not _finite(max_age_s) or max_age_s <= 0:
        raise ValueError('invalid_maximum_age')
    reasons = []
    ranges = scan.get('ranges', ())
    count = len(ranges)
    names = ('angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max')
    values = [scan.get(name) for name in names]
    geometry = 2 <= count <= 65536 and all(_finite(v) for v in values)
    valid = 0
    if geometry:
        lo, hi, step, rlo, rhi = values
        geometry = (step != 0 and rlo >= 0 and rhi > rlo
                    and abs(lo + step * (count - 1) - hi) <= abs(step) * .51)
        if geometry:
            for value in ranges:
                # The JSON capture uses strings for nonfinite sensor returns.
                if _finite(value) and rlo <= value <= rhi:
                    valid += 1
            geometry = valid >= 2
    if not geometry:
        reasons.append('scan_geometry_or_returns_invalid')

    age = None
    fresh = all(_stamp(t) for t in (header_ns, received_ns, now_ns))
    if fresh:
        age = (now_ns - header_ns) / 1e9
        fresh = header_ns <= received_ns <= now_ns and 0 <= age <= max_age_s
    if not fresh:
        reasons.append('clock_order_or_freshness_invalid')

    dt, period = scan.get('time_increment'), scan.get('scan_time')
    consistent = (count >= 2 and _finite(dt) and _finite(period)
                  and dt > 0 and period > 0)
    span = (count - 1) * dt if count >= 2 and _finite(dt) and dt >= 0 else None
    ratio = span / period if span is not None and _finite(period) and period > 0 else None
    if consistent:
        consistent = span <= period * 1.01
        if fresh:
            consistent = consistent and header_ns + round(span * 1e9) <= received_ns
    if not consistent:
        reasons.append('declared_beam_timing_invalid')
    if not provenance.verified:
        reasons.append('acquisition_order_or_clock_unverified')
    ready = geometry and fresh and consistent and provenance.verified
    return ScanAudit(geometry, fresh, consistent, ready, valid, count,
                     span, ratio, age, tuple(reasons))


def beam_timestamps(scan, *, header_ns, received_ns, provenance):
    report = audit_scan(scan, header_ns=header_ns, received_ns=received_ns,
                        now_ns=received_ns, provenance=provenance)
    if not report.deskew_ready:
        raise ValueError('deskew_contract_unverified:' + ','.join(report.reasons))
    return np.array([header_ns + round(i * scan['time_increment'] * 1e9)
                     for i in range(report.beam_count)], dtype=np.int64)


def interval_velocity(*, displacement_xy_m, displacement_covariance_m2,
                      start_ns, end_ns, additional_covariance_m2_s2,
                      frame_id, reference_point, source_lineage):
    """Return a two-axis interval average, not endpoint instantaneous velocity.

    Input displacement and covariance must be in one fixed reference frame.
    The caller supplies model/calibration uncertainty separately. Reusing IMU
    or UWB information must remain visible in source_lineage for downstream
    correlation handling; this helper does not establish fusion readiness.
    """
    if not (_stamp(start_ns) and _stamp(end_ns) and end_ns > start_ns):
        raise ValueError('invalid_observation_interval')
    d = np.asarray(displacement_xy_m, dtype=float)
    if d.shape != (2,) or not np.isfinite(d).all():
        raise ValueError('invalid_horizontal_displacement')
    if not frame_id or not reference_point or not source_lineage:
        raise ValueError('observation_provenance_required')
    c = covariance(displacement_covariance_m2, 2, positive=True)
    extra = covariance(additional_covariance_m2_s2, 2)
    duration = (end_ns - start_ns) / 1e9
    return dict(velocity_xy_m_s=(d / duration).tolist(),
                covariance_xy_m2_s2=(c / duration**2 + extra).tolist(),
                interval_start_ns=start_ns, interval_end_ns=end_ns,
                representative_stamp_ns=start_ns + (end_ns-start_ns)//2,
                measurement_kind='interval_average_velocity',
                frame_id=frame_id, reference_point=reference_point,
                source_lineage=list(source_lineage),
                observed_axes=['x', 'y'], z_observed=False,
                px4_aiding_ready=False)


def _skew(a):
    x, y, z = a
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def horizontal_lever_arm(*, lidar_velocity_body_xy_m_s, omega_body_rad_s,
                         uwb_to_lidar_body_m, joint_covariance):
    """Instantaneous v_UWB.xy = v_LiDAR.xy - (omega cross r_UL).xy.

    All arguments use the SAME body basis and SAME time. This is not a tilt
    correction or a conversion from interval-average to instantaneous speed.
    Joint covariance order is [vLx,vLy, wx,wy,wz, rx,ry,rz], including any cross
    terms. The caller must establish horizontal scan geometry independently.
    """
    v = np.asarray(lidar_velocity_body_xy_m_s, dtype=float)
    w = np.asarray(omega_body_rad_s, dtype=float)
    r = np.asarray(uwb_to_lidar_body_m, dtype=float)
    if v.shape != (2,) or w.shape != (3,) or r.shape != (3,):
        raise ValueError('lever_arm_shape')
    if not all(np.isfinite(a).all() for a in (v, w, r)):
        raise ValueError('lever_arm_nonfinite')
    c = covariance(joint_covariance, 8)
    jacobian = np.column_stack((np.eye(2), _skew(r)[:2], -_skew(w)[:2]))
    return v - np.cross(w, r)[:2], jacobian @ c @ jacobian.T


class SingleUseObservation:
    """Reject duplicate/out-of-order timestamps; epochs require explicit reset."""
    def __init__(self, source_id, clock_id, epoch):
        if not source_id or not clock_id or not epoch:
            raise ValueError('observation_identity_required')
        self.identity = (source_id, clock_id, epoch)
        self.last_ns = None

    def accept(self, *, source_id, clock_id, epoch, stamp_ns, now_ns, max_age_s):
        if (source_id, clock_id, epoch) != self.identity:
            return 'epoch_or_source_changed'
        if not all(_stamp(t) for t in (stamp_ns, now_ns)):
            return 'timestamp_invalid'
        if not _finite(max_age_s) or max_age_s <= 0:
            return 'maximum_age_invalid'
        if not 0 <= (now_ns - stamp_ns) / 1e9 <= max_age_s:
            return 'observation_stale_or_future'
        if self.last_ns is not None and stamp_ns <= self.last_ns:
            return 'duplicate_or_out_of_order'
        self.last_ns = stamp_ns
        return 'accepted'
