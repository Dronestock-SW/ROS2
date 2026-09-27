"""Protocol validation for uwb_raw_status / uwb_raw_cycle frames. Raw values are never modified."""
from dataclasses import dataclass, field

import numpy as np

from drone_uwb.contracts.protocol import finite, integer

ANCHOR_ORDER = ['A1', 'A2', 'A3', 'A4']
CONSUMED_FIELDS = frozenset(('type', 'schema', 'tag_id', 'seq', 'cycle_start_us', 'cycle_end_us', 'valid_mask',
                             'raw_slant_m', 'sample_time_us', 'failure'))


class InvalidInput(ValueError):
    """A frame that cannot be used. `reason` is the counter name; `reset` drops all temporal state."""

    def __init__(self, reason, reset=False):
        super().__init__(reason)
        self.reason = reason
        self.reset = reset


@dataclass
class Cycle:
    seq: int
    start_us: int
    end_us: int
    mask: int
    raw: np.ndarray            # raw_slant_m as sent; nan where missing
    sample_us: list            # per-anchor Report read time, None where missing
    failure: list
    indices: list              # anchors usable for position (mask bit + failure ok + physical range)
    excluded: dict = field(default_factory=dict)   # anchor index -> why a set mask bit was not used
    seq_gap: int = 0           # cycles missing since the previous accepted cycle
    host_mono_ns: int = 0
    ros_ns: int = 0
    extra: dict = field(default_factory=dict)


class InputValidator:
    def __init__(self, settings, require_recent_status=True):
        self.settings = settings
        self.require_recent_status = require_recent_status
        self.status = None
        self.status_ns = None
        self.reset()

    def reset(self):
        self.last_seq = None
        self.last_end_us = None

    def disconnect(self):
        self.status = None
        self.status_ns = None
        self.reset()

    def on_status(self, msg, mono_ns):
        # The supplied RAW-only source omits clock_domain. Its firmware identifier defines it.
        domain = msg.get('clock_domain')
        clock_known = (domain == 'esp32_monotonic_boot_us'
                       or (domain is None and msg.get('firmware') == 'uwb-tag-jetson-raw-v1'))
        valid = (msg.get('uwb_ready') is True
                 and msg.get('anchor_order') == ANCHOR_ORDER
                 and msg.get('anchor_count') == 4
                 and clock_known
                 and msg.get('temporal_filter_applied') is False)
        if not valid:
            self.disconnect()
            raise InvalidInput('unsupported_status', reset=True)
        self.status = dict(msg)
        self.status_ns = mono_ns

    def check_common(self, msg):
        if not integer(msg.get('schema')) or msg.get('schema') != 1 or msg.get('tag_id') != self.settings.tag_id:
            raise InvalidInput('schema_or_tag_mismatch')

    def on_cycle(self, msg, mono_ns, ros_ns):
        s = self.settings
        # A recorded boot status can be latched for a file session; live callers retain timeout behavior.
        if self.status_ns is None or (self.require_recent_status
                and not 0 <= (mono_ns - self.status_ns) / 1e9 <= s.status_timeout_s):
            raise InvalidInput('status_unavailable')
        try:
            seq, start, end, mask = (msg[k] for k in ('seq', 'cycle_start_us', 'cycle_end_us', 'valid_mask'))
        except KeyError:
            raise InvalidInput('invalid_cycle')
        if (not all(integer(v) for v in (seq, start, end, mask))
                or not 0 <= seq <= 0xffffffff or not 0 <= mask <= 15 or start < 0 or end < start):
            raise InvalidInput('invalid_cycle')
        gap = 0
        if self.last_seq is not None:
            delta = (seq - self.last_seq) & 0xffffffff
            if end < self.last_end_us - 1_000_000 and seq < self.last_seq:
                # ESP32 rebooted: the clock mapping and every history are meaningless now.
                self.disconnect()
                raise InvalidInput('source_restart_wait_status', reset=True)
            if delta == 0 or delta >= 0x80000000 or end <= self.last_end_us:
                raise InvalidInput('duplicate_or_out_of_order')
            gap = delta - 1
        self.last_seq, self.last_end_us = seq, end
        if (end - start) / 1e6 > s.max_cycle_s:
            raise InvalidInput('cycle_too_long')
        if 'cycle_duration_us' in msg and (not integer(msg['cycle_duration_us'])
                                         or msg['cycle_duration_us'] != end - start):
            raise InvalidInput('cycle_duration_mismatch')
        try:
            values, failures, samples = (msg[k] for k in ('raw_slant_m', 'failure', 'sample_time_us'))
        except KeyError:
            raise InvalidInput('invalid_anchor_arrays')
        if any(not isinstance(a, list) or len(a) != 4 for a in (values, failures, samples)):
            raise InvalidInput('invalid_anchor_arrays')
        raw = np.array([v if finite(v) else np.nan for v in values], dtype=float)
        indices, excluded = [], {}
        for i in range(4):
            if not mask & (1 << i):
                continue
            if failures[i] != 'ok':
                excluded[i] = 'failure_not_ok'
            elif not finite(values[i]) or not 0 < values[i] <= s.max_range_m:
                excluded[i] = 'range_invalid'
            elif not integer(samples[i]) or not start <= samples[i] <= end:
                raise InvalidInput('invalid_report_time')
            else:
                indices.append(i)
        cycle = Cycle(seq, start, end, mask, raw, [v if integer(v) else None for v in samples],
                      list(failures), indices, excluded, gap, mono_ns, ros_ns)
        if indices:
            times = [samples[i] for i in indices]
            if (max(times) - min(times)) / 1e6 > s.max_report_span_s:
                raise InvalidInput('report_span_too_long')
        # Everything the pre-filter does not consume (RF diagnostics, twr_seq, tag raw XY) is kept as-is.
        cycle.extra = {k: v for k, v in msg.items() if k not in CONSUMED_FIELDS}
        return cycle
