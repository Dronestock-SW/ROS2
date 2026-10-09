"""Clock qualification survives transport silence, but freshness never does."""
from dataclasses import dataclass
import math


@dataclass
class ClockReadiness:
    stable_samples: int = 0
    last_valid_s: float = float('-inf')
    offset_ns: int | None = None
    remote_ns: int = 0
    qualified: bool = False

    def reset(self):
        self.stable_samples = 0
        self.last_valid_s = float('-inf')
        self.offset_ns = None
        self.remote_ns = 0
        self.qualified = False

    def ready(self, now):
        return self.qualified and 0 <= now-self.last_valid_s < .5

    def observe(self, *, now, remote_ns, offset_ns, rtt_ms):
        """Return (accepted, clock_changed). Never refresh from a late reply.

        Thirty good probes qualify one clock epoch. After a transport gap, a
        fresh low-RTT probe matching that epoch can resume immediately. Offset
        changes or disconnect/reset require a new thirty-probe acquisition.
        """
        if not math.isfinite(now):
            raise ValueError('invalid_monotonic_clock')
        if remote_ns <= self.remote_ns or remote_ns <= 0:
            return False, False
        jump = self.offset_ns is not None and abs(offset_ns-self.offset_ns) > 5_000_000
        if jump:
            self.qualified = False
            self.stable_samples = 0
        if not math.isfinite(rtt_ms) or not 0 <= rtt_ms <= 20:
            return False, jump
        continuous = (self.offset_ns is not None and not jump
                      and 0 <= now-self.last_valid_s < .5)
        if self.qualified:
            self.stable_samples = 30
        else:
            self.stable_samples = min(30, self.stable_samples+1) if continuous else 1
        self.qualified = self.stable_samples >= 30
        self.last_valid_s, self.offset_ns, self.remote_ns = now, offset_ns, remote_ns
        return True, jump
