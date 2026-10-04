"""Parse ranges, generate XY observations, and reject inconsistent samples.

No IMU integration, height estimate, flight EKF, hold, or interpolation.
Timing is an approximate host mapping of Report-read times, not HW sync.
"""
from collections import deque
from dataclasses import dataclass, field
import json
import math

import numpy as np


from drone_uwb.contracts.protocol import InvalidSample, finite, integer, decode_line
from drone_uwb.acquisition.framing import LineFramer


@dataclass
class Settings:
    tag_id: str = '5'
    source_mode: str = 'raw_ranges'
    min_anchors: int = 4
    max_range_m: float = 1000.0
    status_timeout_s: float = 10.0
    max_cycle_s: float = 0.10
    max_report_span_s: float = 0.04
    max_queue_s: float = 0.15
    max_pair_residual_m: float = 0.15
    range_step_margin_m: float = 0.20  # handoff v1.2 section 10A range gate: 0.20 m + 1.20 m/s * dt
    max_speed_m_s: float = 1.20
    history_timeout_s: float = 0.5
    recovery_samples: int = 3  # handoff v1.2: 3 confirmations before a new range is trusted
    clock_warmup_samples: int = 30
    clock_window_s: float = 5.0
    xy_stddev_m: float = 0.30
    geometry_tolerance_m: float = 0.03
    report_delay_s: float = 0.0

    def __post_init__(self):
        if self.source_mode not in ('raw_ranges', 'tag_xy'):
            raise ValueError('invalid source_mode')
        if self.min_anchors not in (3, 4):
            raise ValueError('min_anchors must be 3 or 4')
        if (not integer(self.recovery_samples) or not integer(self.clock_warmup_samples)
                or self.recovery_samples < 1 or self.clock_warmup_samples < 2):
            raise ValueError('invalid warmup count')
        for name, value in vars(self).items():
            if name not in ('tag_id', 'source_mode') and (not finite(value) or value < 0):
                raise ValueError('invalid setting: ' + name)
        if min(self.xy_stddev_m, self.clock_window_s, self.status_timeout_s, self.max_range_m) <= 0:
            raise ValueError('uncertainty and time windows must be positive')


@dataclass
class Observation:
    seq: int
    stamp_ns: int
    x: float
    y: float
    variance: float
    anchor_mask: int
    pair_residual_m: float
    report_span_s: float
    source_mode: str


@dataclass
class Decision:
    reason: str
    observation: Observation = None
    details: dict = field(default_factory=dict)


def solve_xy(anchors, ranges, indices):
    """Equal anchor height removes the common vertical term by subtraction."""
    base = indices[0]
    others = indices[1:]
    offsets = anchors[others, :2] - anchors[base, :2]
    matrix = 2.0 * offsets
    rhs = (ranges[base] ** 2 - ranges[others] ** 2
           + np.sum(anchors[others, :2] ** 2, axis=1)
           - np.sum(anchors[base, :2] ** 2))
    xy, _, rank, singular = np.linalg.lstsq(matrix, rhs, rcond=None)
    if rank != 2 or singular[0] / singular[-1] > 30:
        raise InvalidSample('degenerate_geometry')
    residual = float(np.max(np.abs(rhs - matrix @ xy) / (2 * np.linalg.norm(offsets, axis=1))))
    return xy, residual


class Processor:
    """One source cycle produces at most one new observation."""

    def __init__(self, layout, settings=None):
        self.settings = settings or Settings()
        self.layout = layout
        self.anchors = np.asarray(layout['anchors_xyz_m'], dtype=float)
        if layout['anchor_order'] != ['A1', 'A2', 'A3', 'A4']:
            raise ValueError('anchor order must match the wire arrays')
        if self.anchors.shape != (4, 3) or not np.isfinite(self.anchors).all():
            raise ValueError('invalid anchor coordinates')
        if np.ptp(self.anchors[:, 2]) > 0.001:
            raise ValueError('range-difference XY requires equal anchor heights')
        solve_xy(self.anchors, np.ones(4), list(range(4)))
        self.status = None
        self.status_ns = None
        self.last_seq = None
        self.last_end_us = None
        self.last_ros_offset = None
        self.reset_temporal()

    def reset_temporal(self):
        self.offsets = deque()
        self.history = None
        self.stable = 0
        self.last_stamp_ns = None

    def disconnect(self):
        self.status = None
        self.status_ns = None
        self.last_seq = None
        self.last_end_us = None
        self.reset_temporal()

    def process(self, msg, mono_ns, ros_ns):
        try:
            return self._process(msg, mono_ns, ros_ns)
        except (InvalidSample, KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError) as exc:
            self.stable = 0
            return Decision(str(exc) if isinstance(exc, InvalidSample) else 'invalid_schema')

    def _process(self, msg, mono_ns, ros_ns):
        s = self.settings
        if not integer(msg.get('schema')) or msg.get('schema') != 1 or msg.get('tag_id') != s.tag_id:
            raise InvalidSample('schema_or_tag_mismatch')
        kind = msg.get('type')
        if kind == 'uwb_raw_status':
            valid = (msg.get('uwb_ready') is True
                     and msg.get('anchor_order') == self.layout['anchor_order']
                     and msg.get('anchor_count') == 4
                     and msg.get('clock_domain') == 'esp32_monotonic_boot_us'
                     and msg.get('temporal_filter_applied') is False)
            if not valid:
                self.disconnect()
                raise InvalidSample('unsupported_status')
            self.status = dict(msg)
            self.status_ns = mono_ns
            return Decision('status')
        if kind != 'uwb_raw_cycle':
            raise InvalidSample('unsupported_type')
        if self.status_ns is None or not 0 <= (mono_ns - self.status_ns) / 1e9 <= s.status_timeout_s:
            raise InvalidSample('status_unavailable')
        seq, start, end, mask = (msg[k] for k in ('seq', 'cycle_start_us', 'cycle_end_us', 'valid_mask'))
        if (not all(integer(v) for v in (seq, start, end, mask))
                or not 0 <= seq <= 0xffffffff or not 0 <= mask <= 15
                or start < 0 or end < start):
            raise InvalidSample('invalid_cycle')
        if self.last_seq is not None:
            delta = (seq - self.last_seq) & 0xffffffff
            if end < self.last_end_us - 1_000_000 and seq < self.last_seq:
                self.disconnect()
                raise InvalidSample('source_restart_wait_status')
            if delta == 0 or delta >= 0x80000000 or end <= self.last_end_us:
                raise InvalidSample('duplicate_or_out_of_order')
        self.last_seq, self.last_end_us = seq, end
        if (end - start) / 1e6 > s.max_cycle_s:
            raise InvalidSample('cycle_too_long')
        values, failures, sample_times = (msg[k] for k in ('raw_slant_m', 'failure', 'sample_time_us'))
        if any(not isinstance(a, list) or len(a) != 4 for a in (values, failures, sample_times)):
            raise InvalidSample('invalid_anchor_arrays')
        indices = []
        for i in range(4):
            if mask & (1 << i):
                if not finite(values[i]) or not 0 < values[i] <= s.max_range_m or failures[i] != 'ok':
                    raise InvalidSample('valid_mask_range_conflict')
                if not integer(sample_times[i]) or not start <= sample_times[i] <= end:
                    raise InvalidSample('invalid_report_time')
                indices.append(i)
        if len(indices) < s.min_anchors:
            raise InvalidSample('insufficient_anchors')
        span = (max(sample_times[i] for i in indices) - min(sample_times[i] for i in indices)) / 1e6
        if span > s.max_report_span_s:
            raise InvalidSample('report_span_too_long')
        # Map source intervals to host time using the least observed receive delay.
        # Unknown constant transport/Report delays remain and require bench validation.
        ros_offset = ros_ns - mono_ns
        if self.last_ros_offset is not None and abs(ros_offset - self.last_ros_offset) > 250_000_000:
            self.reset_temporal()
        self.last_ros_offset = ros_offset
        self.offsets.append((mono_ns, mono_ns - end * 1000))
        while self.offsets and mono_ns - self.offsets[0][0] > s.clock_window_s * 1e9:
            self.offsets.popleft()
        offset = min(v for _, v in self.offsets)
        queue_s = (mono_ns - (end * 1000 + offset)) / 1e9
        if queue_s > s.max_queue_s:
            raise InvalidSample('queued_sample')
        representative_us = sum(sample_times[i] for i in indices) / len(indices)
        stamp = int(representative_us * 1000 + offset + ros_offset - s.report_delay_s * 1e9)
        if stamp > ros_ns or ros_ns - stamp > (s.max_queue_s + s.max_cycle_s + s.report_delay_s) * 1e9:
            raise InvalidSample('invalid_host_stamp')
        ranges = np.array([v if finite(v) else np.nan for v in values], dtype=float)
        xy, residual = solve_xy(self.anchors, ranges, indices)
        details = {'seq': seq, 'candidate_xy_m': xy.tolist(), 'pair_residual_m': residual,
                   'queue_s': queue_s, 'report_span_s': span,
                   'tag_layout_id': self.status.get('anchor_layout_id'),
                   'layout_id': self.layout['layout_id'],
                   'stamp_kind': 'approximate_mean_report_read_time'}
        if residual > s.max_pair_residual_m:
            self.stable = 0
            return Decision('inconsistent_ranges', details=details)
        if s.source_mode == 'tag_xy':
            if (msg.get('raw_xy_valid') is not True or not finite(msg.get('raw_x_m'))
                    or not finite(msg.get('raw_y_m')) or msg.get('raw_xy_anchor_mask') != mask):
                raise InvalidSample('invalid_tag_xy')
            tag_xy = np.array([msg['raw_x_m'], msg['raw_y_m']])
            if np.max(np.abs(tag_xy - xy)) > s.geometry_tolerance_m:
                self.stable = 0
                return Decision('tag_layout_mismatch', details=details)
            xy = tag_xy
        if self.history is not None:
            previous_us, previous_ranges = self.history
            dt = (end - previous_us) / 1e6
            if 0 < dt <= s.history_timeout_s:
                jump_limit = s.range_step_margin_m + s.max_speed_m_s * dt
                if any(finite(previous_ranges[i]) and abs(ranges[i] - previous_ranges[i]) > jump_limit for i in indices):
                    self.stable = 0
                    return Decision('range_jump', details=details)
        self.history = (end, ranges.copy())
        self.stable += 1
        if len(self.offsets) < s.clock_warmup_samples or self.stable < s.recovery_samples:
            return Decision('warming_up', details=details)
        if self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            return Decision('non_increasing_stamp', details=details)
        self.last_stamp_ns = stamp
        variance = s.xy_stddev_m ** 2 + residual ** 2 + (s.max_speed_m_s * span / 2) ** 2
        if len(indices) == 3:
            variance *= 4  # No fourth-anchor consistency redundancy.
        obs = Observation(seq, stamp, float(xy[0]), float(xy[1]), variance,
                          mask, residual, span, s.source_mode)
        return Decision('accepted', obs, details)
