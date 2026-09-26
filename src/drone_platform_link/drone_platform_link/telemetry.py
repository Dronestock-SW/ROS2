"""Fresh, read-only observations for Platform telemetry."""

import math
import time


class Observations:
    def __init__(self):
        self.pose = None
        self.battery = None

    def receive_pose(self, x, y, frame, stamp_ns, now_ns=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        if (frame != 'uwb_map' or not all(map(math.isfinite, (x, y)))
                or stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= 500_000_000):
            return
        self.pose = (float(x), float(y), stamp_ns, time.monotonic())

    def receive_battery(self, percentage, voltage, connected=True):
        if not connected:
            self.battery = None
            return
        percent = percentage * 100 if math.isfinite(percentage) and 0 <= percentage <= 1 else None
        volts = voltage if math.isfinite(voltage) and voltage > 0 else None
        self.battery = (percent, volts, time.monotonic()) if percent is not None or volts is not None else None

    def fields(self):
        result = {'fix': False, 'telemetry_verified': False, 'x': None, 'y': None,
                  'uwb_age_ms': None, 'battery': None, 'battery_voltage': None}
        now = time.monotonic()
        if self.pose is not None:
            x, y, _stamp, received = self.pose
            age_ms = round((now - received) * 1000)
            if 0 <= age_ms <= 500:
                result.update(fix=True, telemetry_verified=True, x=x, y=y,
                              uwb_age_ms=age_ms)
        if self.battery is not None:
            percent, volts, received = self.battery
            if 0 <= now - received <= 3:
                result.update(battery=percent, battery_voltage=volts)
        return result
