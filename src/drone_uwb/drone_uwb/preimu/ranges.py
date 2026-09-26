"""Bias subtraction and the reference per-anchor gate, independent of any Z/XY solver."""
from collections import deque
from dataclasses import dataclass

from ..core import finite


def subtract_bias(raw, bias):
    """RAW - bias; invalid/missing RAW remains null, never a previous measurement."""
    return [float(r - b) if finite(r) else None for r, b in zip(raw, bias)]


@dataclass
class RangeDecision:
    accepted: bool
    value_m: float = None
    reason: str = 'unavailable'
    pending_count: int = 0
    allowed_change_m: float = None


class RangeGate:
    """One anchor's gate. dt caps and pending mean follow the supplied ESP32 source."""

    def __init__(self, settings):
        self.settings = settings
        self.accepted_m = None
        self.last_observed_us = None
        self.pending_m = None
        self.pending_count = 0

    def update(self, source_us, value_m):
        s = self.settings
        if not finite(value_m) or not 0 < value_m <= s.max_range_m:
            return RangeDecision(False, reason='calibrated_range_invalid')
        if self.last_observed_us is not None and source_us <= self.last_observed_us:
            return RangeDecision(False, reason='non_increasing_source_time')
        allowed = None
        reason = 'initialized'
        if self.accepted_m is not None:
            dt = min((source_us - self.last_observed_us) / 1e6, 0.50)
            allowed = s.range_gate_margin_m + s.range_gate_speed_m_s * dt
            self.last_observed_us = source_us
            reason = 'accepted'
            if abs(value_m - self.accepted_m) > allowed:
                if (self.pending_m is not None
                        and abs(value_m - self.pending_m) <= s.range_reacquire_cluster_m):
                    self.pending_m = (self.pending_m * self.pending_count + value_m) / (self.pending_count + 1)
                    self.pending_count += 1
                else:
                    self.pending_m, self.pending_count = value_m, 1
                if self.pending_count < s.range_reacquire_confirm:
                    return RangeDecision(False, reason='range_gate_pending',
                                         pending_count=self.pending_count, allowed_change_m=allowed)
                value_m, reason = self.pending_m, 'reacquired'
        self.accepted_m, self.last_observed_us = float(value_m), source_us
        self.pending_m, self.pending_count = None, 0
        return RangeDecision(True, self.accepted_m, reason, allowed_change_m=allowed)


class RangeHistory:
    """Only accepted observations; original ESP32 timestamps survive clock refitting."""

    def __init__(self, window_s, max_samples=192):
        self.window_us = round(window_s * 1e6)
        self.rows = deque(maxlen=max_samples)

    def append(self, anchor, source_us, value_m):
        self.rows.append((anchor, source_us, value_m))

    def prune(self, now_us):
        while self.rows and now_us - self.rows[0][1] > self.window_us:
            self.rows.popleft()

    def counts(self):
        return [sum(row[0] == i for row in self.rows) for i in range(4)]
