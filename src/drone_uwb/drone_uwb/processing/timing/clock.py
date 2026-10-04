"""ESP32 -> Jetson monotonic clock mapping: t_host ~ alpha * t_esp32 + beta.

Receive times are the true event time plus a non-negative queueing delay, so the mapping follows
the lower envelope of (esp32, host) pairs. The unknown constant transport delay stays in beta and
is not corrected here.
"""
from collections import deque

import numpy as np


class ClockMap:
    def __init__(self, settings):
        self.settings = settings
        self.reset()

    def reset(self):
        self.points = deque()   # (esp32 us, host ns)
        self.base = None
        self.slope = 1000.0     # host ns per esp32 us (alpha = slope / 1000)
        self.beta = 0.0
        self.residual_p95_s = None
        self.state = 'clock_unsynced'

    @property
    def alpha(self):
        return self.slope / 1000.0

    def update(self, esp_us, host_ns):
        s = self.settings
        self.points.append((esp_us, host_ns))
        while len(self.points) > 1 and (host_ns - self.points[0][1]) / 1e9 > s.clock_window_s:
            self.points.popleft()
        self.base = self.points[0]
        x = np.array([p[0] - self.base[0] for p in self.points], dtype=float)
        y = np.array([p[1] - self.base[1] for p in self.points], dtype=float)
        span_s = (host_ns - self.points[0][1]) / 1e9
        if len(x) < s.clock_min_samples:
            self.state = 'clock_unsynced'
            return
        if span_s >= s.clock_alpha_min_span_s:
            design = np.column_stack([x, np.ones_like(x)])
            coef = np.linalg.lstsq(design, y, rcond=None)[0]
            low = (y - design @ coef) <= np.median(y - design @ coef)
            self.slope = float(np.linalg.lstsq(design[low], y[low], rcond=None)[0][0])
        else:
            self.slope = 1000.0
        offset = y - self.slope * x
        self.beta = float(np.min(offset))
        delay = offset - self.beta
        self.residual_p95_s = float(np.percentile(delay, 95)) / 1e9
        scale_ok = abs(self.alpha - 1.0) <= s.clock_scale_tolerance
        self.state = 'ok' if scale_ok and self.residual_p95_s <= s.clock_residual_p95_max_s else 'clock_unsynced'

    @property
    def ready(self):
        return self.state == 'ok'

    def host_s(self, esp_us):
        """Host monotonic seconds for an ESP32 timestamp."""
        return (self.base[1] + self.slope * (esp_us - self.base[0]) + self.beta) / 1e9
