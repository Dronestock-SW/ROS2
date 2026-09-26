"""Latch the demo off when a real range measurement becomes available."""
import math


class DemoZGate:
    def __init__(self, enabled=True):
        self.blocked = not enabled
        self.reason = 'disabled_by_config' if self.blocked else ''

    def observe(self, distance, minimum, maximum):
        if (all(math.isfinite(v) for v in (distance, minimum, maximum))
                and minimum <= distance <= maximum and distance > 0):
            self.blocked = True
            self.reason = 'real_tof_detected'
        return self.blocked
