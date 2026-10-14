"""ESP32 -> Jetson monotonic clock mapping: t_host ~ alpha * t_esp32 + beta.

Receive times are the true event time plus a non-negative queueing delay, so the mapping follows
the lower envelope of (esp32, host) pairs. The unknown constant transport delay stays in beta and
is not corrected here.
"""
from collections import deque
import copy
import math

import numpy as np


def _sorted_finite(values):
    # A few hundred clock samples: sorting Python floats avoids NumPy's small
    # quantile dispatch/partition overhead and repeated GIL handoffs to ingress.
    ordered = sorted(float(v) for v in values)
    if not ordered or any(not math.isfinite(v) for v in ordered):
        return None
    return ordered


def _median(values):
    ordered = _sorted_finite(values)
    if ordered is None:
        return float(np.median(values))
    middle = len(ordered)//2
    return (ordered[middle] if len(ordered)%2 else
            (ordered[middle-1]+ordered[middle])/2.)


def _percentile95(values):
    ordered = _sorted_finite(values)
    if ordered is None:
        return float(np.percentile(values, 95))
    index = (len(ordered)-1)*.95
    lower = int(index)
    upper = min(lower+1, len(ordered)-1)
    weight = index-lower
    a, b = ordered[lower], ordered[upper]
    # Match NumPy's default linear interpolation, including its upper-end
    # arithmetic for weight >= .5. This is not an approximate quantile.
    return b-(b-a)*(1.-weight) if weight >= .5 else a+(b-a)*weight


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
        self.reason = 'insufficient_samples'

    @property
    def alpha(self):
        return self.slope / 1000.0

    def update(self, esp_us, host_ns):
        self.update_many(((esp_us, host_ns),))

    def updated_copy(self, points):
        """Fit a candidate once without mutating a retained, qualified map."""
        candidate = copy.copy(self)
        candidate.points = deque(self.points)
        candidate.update_many(points)
        return candidate

    def update_many(self, points):
        """Append already validated points and perform one fit."""
        s = self.settings
        self.points.extend(points)
        host_ns = self.points[-1][1]
        while len(self.points) > 1 and (host_ns - self.points[0][1]) / 1e9 > s.clock_window_s:
            self.points.popleft()
        self.base = self.points[0]
        x = np.array([p[0] - self.base[0] for p in self.points], dtype=float)
        y = np.array([p[1] - self.base[1] for p in self.points], dtype=float)
        span_s = (host_ns - self.points[0][1]) / 1e9
        if len(x) < s.clock_min_samples:
            self.state = 'clock_unsynced'
            self.reason = 'insufficient_samples'
            return
        if span_s >= s.clock_alpha_min_span_s:
            design = np.column_stack([x, np.ones_like(x)])
            coef = np.linalg.lstsq(design, y, rcond=None)[0]
            residual = y - design @ coef
            low = residual <= _median(residual)
            self.slope = float(np.linalg.lstsq(design[low], y[low], rcond=None)[0][0])
        else:
            self.slope = 1000.0
        offset = y - self.slope * x
        self.beta = float(np.min(offset))
        delay = offset - self.beta
        self.residual_p95_s = _percentile95(delay) / 1e9
        scale_ok = abs(self.alpha - 1.0) <= s.clock_scale_tolerance
        self.state = 'ok' if scale_ok and self.residual_p95_s <= s.clock_residual_p95_max_s else 'clock_unsynced'
        self.reason = ('scale_out_of_bounds' if not scale_ok else
                       'residual_out_of_bounds' if self.state != 'ok' else 'ok')

    def diagnostics(self):
        """Explain a rejection without changing limits or implying delay calibration."""
        return dict(state=self.state, reason=self.reason, alpha=self.alpha,
                    residual_p95_s=self.residual_p95_s, samples=len(self.points),
                    span_s=(self.points[-1][1]-self.points[0][1])/1e9 if self.points else 0.,
                    scale_tolerance=self.settings.clock_scale_tolerance,
                    residual_p95_max_s=self.settings.clock_residual_p95_max_s,
                    fixed_delay_calibrated=False)

    @property
    def ready(self):
        return self.state == 'ok'

    def host_s(self, esp_us):
        """Host monotonic seconds for an ESP32 timestamp."""
        return (self.base[1] + self.slope * (esp_us - self.base[0]) + self.beta) / 1e9
