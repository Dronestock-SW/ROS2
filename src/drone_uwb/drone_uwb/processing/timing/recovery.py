"""Revalidate a retained ESP/host map while normal acquisition continues.

Only the owner of a validated complete TDMA pair may call update/qualified.
No stale input, orphan epoch or geometric fit can renew the retention deadline.
The limits below are trial design values, not flight-validated accuracy bounds.
"""
from dataclasses import dataclass
import statistics
import numpy as np

from .clock import ClockMap


@dataclass(frozen=True)
class QualifiedMap:
    clock: ClockMap
    esp_us: int
    source_ns: int
    offset_ns: int
    seq: int
    identity: tuple
    steady_ns: int


class ClockRecovery:
    hold_ns = 2_000_000_000
    confirm_window_ns = 100_000_000
    offset_limit_ns = 5_000_000
    residual_min_ns = -2_000_000
    residual_max_ns = 50_000_000
    residual_median_ns = 20_000_000
    confirmations = 3

    def __init__(self):
        self.reset()

    def reset(self):
        self.trusted = self.held = None
        self.checks = []
        self.reacquiring = False
        self.reason = 'normal_acquisition'
        self.residual_ns = self.offset_delta_ns = None

    def reject_pair(self):
        self.checks.clear()
        if self.held is not None:
            self.reason = 'pair_rejected'

    def qualified(self, clock, esp_us, source_ns, offset_ns, seq, identity, steady_ns):
        # Called only after clock, mapped age and timestamp checks succeeded.
        self.trusted = (QualifiedMap(clock, esp_us, source_ns, offset_ns, seq,
                                     identity, steady_ns) if identity is not None else None)

    def expire(self, clock, steady_ns):
        """Discard stale work; an existing reacquisition must not start over."""
        self.reject_pair()
        if self.reacquiring:
            if self.held is not None and not 0 <= steady_ns-self.held.steady_ns <= self.hold_ns:
                self.held = None
                self.reason = 'retention_expired'
            return clock
        prior = self.trusted
        self.held = (prior if prior is not None and prior.clock is clock and clock.ready
                     and clock.points[-1] == (prior.esp_us, prior.source_ns)
                     and 0 <= steady_ns-prior.steady_ns <= self.hold_ns else None)
        self.trusted = None
        self.reacquiring = True
        self.reason = 'awaiting_revalidation' if self.held is not None else 'normal_reacquisition'
        # Keep the old object immutable. Fresh verified pairs immediately fit
        # this independent map; there is no extra thread or TTL wait.
        return ClockMap(clock.settings)

    def update(self, clock, esp_us, source_ns, offset_ns, seq, identity, steady_ns):
        held = self.held
        if held is not None:
            source_age = source_ns-held.source_ns
            local_age = steady_ns-held.steady_ns
            delta = (seq-held.seq) & 0xffffffff
            if (identity != held.identity or esp_us <= held.esp_us or source_age <= 0
                    or not 0 < delta < 0x80000000 or local_age < 0):
                self.held = None
                self.checks.clear()
                self.reason = 'identity_or_time_discontinuity'
            elif source_age > self.hold_ns or local_age > self.hold_ns:
                self.held = None
                self.checks.clear()
                self.reason = 'retention_expired'
            else:
                # Subtract integer host origins first, avoiding ROS-epoch
                # float precision loss. Positive residual includes transport.
                c = held.clock
                self.residual_ns = (source_ns-c.base[1]
                                    - c.slope*(esp_us-c.base[0])-c.beta)
                self.offset_delta_ns = offset_ns-held.offset_ns
                if (not self.residual_min_ns <= self.residual_ns <= self.residual_max_ns
                        or abs(self.offset_delta_ns) > self.offset_limit_ns):
                    self.checks.clear()
                    self.reason = 'revalidation_point_rejected'
                else:
                    if self.checks and (source_ns-self.checks[0][1] > self.confirm_window_ns
                                       or steady_ns-self.checks[0][2] > self.confirm_window_ns):
                        self.checks.clear()
                    self.checks.append((esp_us, source_ns, steady_ns, self.residual_ns))
                    self.reason = 'confirming_retained_map'
                    if len(self.checks) == self.confirmations:
                        if statistics.median(p[3] for p in self.checks) <= self.residual_median_ns:
                            try:
                                candidate = c.updated_copy([(p[0], p[1]) for p in self.checks])
                                age = (source_ns/1e9-candidate.host_s(esp_us)
                                       if candidate.ready else float('inf'))
                            except (ValueError, OverflowError, np.linalg.LinAlgError):
                                candidate, age = None, float('inf')
                            if candidate is not None and candidate.ready and 0 <= age <= candidate.settings.max_queue_s:
                                self.held = None
                                self.checks.clear()
                                self.reacquiring = False
                                self.reason = 'retained_map_requalified'
                                return candidate
                            self.reason = 'retained_refit_rejected'
                        else:
                            self.reason = 'revalidation_median_rejected'
                        self.checks.clear()
        # Also runs on every fast-path failure: acquisition does not wait for
        # retention to expire. On fast-path success the duplicate fit is skipped.
        clock.update(esp_us, source_ns)
        if self.reacquiring and clock.ready:
            self.held = None
            self.checks.clear()
            self.reacquiring = False
            self.reason = 'new_map_qualified'
        return clock

    def diagnostics(self):
        return dict(reason=self.reason, retained=self.held is not None,
                    reacquiring=self.reacquiring, confirmations=len(self.checks),
                    residual_ms=None if self.residual_ns is None else self.residual_ns/1e6,
                    ros_offset_delta_ms=None if self.offset_delta_ns is None else self.offset_delta_ns/1e6)
