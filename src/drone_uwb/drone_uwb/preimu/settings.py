"""Pre-filter settings. Values marked CANDIDATE are not in the handoff or are unmeasured."""
from dataclasses import asdict, dataclass, field, fields
import math

import numpy as np


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass
class PreimuSettings:
    tag_id: str = '5'
    # Range calibration. All zeros with calibrated=False means "not calibrated", not "zero bias".
    range_bias_m: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    range_bias_calibrated: bool = False
    # Height is an assumption (no ToF/attitude input). It never reaches PX4.
    fixed_z_m: float = 1.2  # CANDIDATE: altitude_policy demo default, not measured
    uwb_lever_arm_body_m: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    lever_arm_confirmed: bool = False  # R_WB is assumed identity while False
    # Input checks (same values the live node already uses).
    max_range_m: float = 80.0
    max_cycle_s: float = 0.10
    max_report_span_s: float = 0.04
    max_queue_s: float = 0.15
    status_timeout_s: float = 10.0
    min_anchors: int = 3
    # Clock mapping. CANDIDATE: none of these are in the handoff.
    clock_window_s: float = 10.0
    clock_min_samples: int = 30
    clock_alpha_min_span_s: float = 5.0
    clock_scale_tolerance: float = 1e-3
    clock_residual_p95_max_s: float = 0.05
    # Raw XY diagnostic. CANDIDATE.
    raw_xy_max_condition: float = 30.0
    raw_xy_max_rms_m: float = 0.15
    raw_xy_region_margin_m: float = 1.0
    # H80 (handoff section 10).
    window_s: float = 0.8
    sigma_r_m: float = 0.08
    huber_m: float = 0.12
    min_samples_per_anchor: int = 3
    min_samples_total: int = 12
    max_scaled_condition: float = 1e6  # handoff: initial candidate, not confirmed
    fit_period_s: float = 0.05  # handoff: 20 Hz start candidate
    max_extrapolation_s: float = 0.04  # handoff: start candidate
    h80_irls_iterations: int = 8
    # Q_S10 (handoff section 11).
    recovery_gap_s: float = 0.15
    q_min_span_s: float = 0.40
    q_max_source_age_s: float = 0.10
    q_iterations: int = 40
    q_step_halvings: int = 14
    q_bias_max_m: float = 1.0
    q_bias_prior_m: float = 0.10
    q_velocity_weight: float = 1.0  # lambda_v: value not in the handoff, CANDIDATE
    q_init_last_valid_max_age_s: float = 2.0  # CANDIDATE
    warmup_s: float = 0.10
    bias_detect_m: float = 0.08
    bias_hold_s: float = 0.80
    # Range gate before H80 (handoff v1.2 section 10A): |r[k] - r_accepted| <= margin + speed * dt.
    range_gate_margin_m: float = 0.20
    range_gate_speed_m_s: float = 1.20
    range_reacquire_cluster_m: float = 0.15
    range_reacquire_confirm: int = 3
    # Position gate after H80/Q candidates (section 10A): |p_cand - p_accepted| <= margin + speed * dt.
    position_gate_margin_m: float = 0.08
    position_gate_speed_m_s: float = 0.80
    position_reacquire_cluster_m: float = 0.12
    position_reacquire_confirm: int = 3
    # Validity guards. residual_rms_max_m is the section 10A max fit RMS; residual_max_max_m is a CANDIDATE.
    residual_rms_max_m: float = 0.10
    residual_max_max_m: float = 0.30
    # Safety default: hold time counts as "no valid coordinate" (handoff section 13).
    failed_after_s: float = 0.5

    def __post_init__(self):
        if not isinstance(self.tag_id, str) or not self.tag_id:
            raise ValueError('tag_id must be nonempty')
        for name in ('range_bias_calibrated', 'lever_arm_confirmed'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(name + ' must be boolean')
        for name, size in (('range_bias_m', 4), ('uwb_lever_arm_body_m', 3)):
            values = getattr(self, name)
            if (not isinstance(values, (list, tuple)) or len(values) != size
                    or not all(_finite(v) for v in values)):
                raise ValueError(name + ' must be %d finite numbers' % size)
            setattr(self, name, [float(v) for v in values])
        if self.min_anchors not in (3, 4):
            raise ValueError('min_anchors must be 3 or 4')
        for name, value in asdict(self).items():
            if name in ('tag_id', 'range_bias_m', 'uwb_lever_arm_body_m', 'range_bias_calibrated',
                        'lever_arm_confirmed', 'fixed_z_m'):
                continue
            if not _finite(value) or value <= 0:
                raise ValueError('invalid setting: ' + name)
        if not _finite(self.fixed_z_m):
            raise ValueError('fixed_z_m must be finite')
        if self.min_samples_per_anchor < 1 or self.min_samples_total < 3 * self.min_samples_per_anchor:
            raise ValueError('sample minimums are inconsistent')

    @property
    def lever_arm(self):
        return np.array(self.uwb_lever_arm_body_m, dtype=float)


def settings_from_mapping(values):
    known = {f.name for f in fields(PreimuSettings)}
    unknown = sorted(set(values) - known)
    if unknown:
        raise ValueError('unknown preimu settings: ' + ', '.join(unknown))
    return PreimuSettings(**values)
