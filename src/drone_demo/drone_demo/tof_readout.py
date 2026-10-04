"""Display a timestamped measured range; do not estimate or control altitude."""
import math


class TofReadout:
    def __init__(self, timeout_s=0.2):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('tof_timeout_s must be positive and finite')
        self.timeout_s = timeout_s
        self.sample = None
        self.last_stamp_ns = 0
        self.reason = 'tof_missing'

    def observe(self, distance, minimum, maximum, stamp_ns, frame_id, mono_ns, ros_ns):
        age_s = (ros_ns - stamp_ns) / 1e9
        valid = (all(math.isfinite(v) for v in (distance, minimum, maximum))
                 and 0 <= minimum < maximum and minimum <= distance <= maximum
                 and distance > 0 and bool(frame_id) and stamp_ns > 0
                 and 0 <= age_s <= self.timeout_s)
        if not valid:
            self.sample = None
            self.reason = 'tof_invalid_or_stale'
            return False
        if stamp_ns <= self.last_stamp_ns:
            return False
        self.last_stamp_ns = stamp_ns
        self.sample = dict(range_m=float(distance), stamp_ns=stamp_ns,
                           frame_id=frame_id, received_mono_ns=mono_ns,
                           age_at_receipt_s=age_s)
        self.reason = 'tof_received'
        return True

    def snapshot(self, mono_ns, ros_ns):
        age_s = None
        if self.sample:
            elapsed = (mono_ns - self.sample['received_mono_ns']) / 1e9
            ros_age = (ros_ns - self.sample['stamp_ns']) / 1e9
            age_s = max(self.sample['age_at_receipt_s'] + elapsed, ros_age)
            if elapsed < 0 or ros_age < 0 or age_s > self.timeout_s:
                self.sample = None
                self.reason = 'tof_stale'
        return dict(available=self.sample is not None, reason=self.reason,
                    range_m=self.sample['range_m'] if self.sample else None,
                    stamp_ns=self.sample['stamp_ns'] if self.sample else None,
                    frame_id=self.sample['frame_id'] if self.sample else None,
                    age_s=age_s, quantity='downward_range', source='sensor_msgs/Range')
