"""Demo-only mission assessment. Produces decisions, never flight commands."""
from dataclasses import dataclass, fields
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class MissionConfig:
    frame_id: str
    approach_enter_m: float
    approach_exit_m: float
    arrival_enter_m: float
    arrival_exit_m: float
    arrival_speed_m_s: float
    settle_s: float
    pose_timeout_s: float
    uwb_timeout_s: float
    recovery_s: float
    recovery_samples: int

    def __post_init__(self):
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError('frame_id must be nonempty')
        for field in fields(self):
            if field.name == 'frame_id':
                continue
            value = getattr(self, field.name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(field.name + ' must be positive and finite')
        if type(self.recovery_samples) is not int or self.recovery_samples < 2:
            raise ValueError('recovery_samples must be an integer >= 2')
        if not (self.arrival_enter_m < self.arrival_exit_m
                <= self.approach_enter_m < self.approach_exit_m):
            raise ValueError('arrival/approach radii must have separate entry and exit limits')


def load_mission_config(path=''):
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('drone_demo')) / 'config/mission.json'
    return MissionConfig(**json.loads(Path(path).read_text(encoding='utf-8')))


@dataclass
class Sample:
    x: float
    y: float
    stamp_ns: int
    received_s: float
    age_at_receipt_s: float

    def age(self, now):
        return now - self.received_s + self.age_at_receipt_s


def finite_xy(x, y):
    return all(type(v) in (int, float) and math.isfinite(v) for v in (x, y))


class MissionMonitor:
    """Uses demo position for progress, UWB freshness only for input health."""

    def __init__(self, config):
        self.config = config
        self.target = None
        self.target_generation = 0
        self.target_reason = 'target_missing'
        self.pose = self.uwb = None
        self.last_stamp = {'pose': 0, 'uwb': 0}
        self.received_count = {'pose': 0, 'uwb': 0}
        self.seen = {'pose': False, 'uwb': False}
        self.speed = None
        self.state = 'WAITING_TARGET'
        self.last_evaluation_s = None
        self.reset_health()

    def reset_arrival(self):
        self.settle_started_ns = None
        self.near = False

    def reset_health(self):
        self.healthy_since = None
        self.recovery_counts = dict(self.received_count)
        self.reset_arrival()

    def set_target(self, x, y, frame):
        if frame != self.config.frame_id or not finite_xy(x, y):
            self.target = None
            self.target_reason = 'invalid_target'
            self.reset_health()
            return False
        target = (float(x), float(y))
        if target != self.target:
            self.target_generation += 1
            self.target = target
            self.reset_health()
        self.target_reason = 'target_accepted'
        return True

    def update(self, kind, x, y, stamp_ns, received_s, age_at_receipt_s, frame):
        if kind not in ('pose', 'uwb'):
            raise ValueError('unsupported input')
        self.seen[kind] = True
        timeout = getattr(self.config, kind + '_timeout_s')
        valid = (frame == self.config.frame_id and finite_xy(x, y)
                 and type(stamp_ns) is int and stamp_ns > 0
                 and type(received_s) in (int, float) and math.isfinite(received_s)
                 and type(age_at_receipt_s) in (int, float)
                 and math.isfinite(age_at_receipt_s)
                 and 0 <= age_at_receipt_s <= timeout)
        if not valid:
            setattr(self, kind, None)
            if kind == 'pose':
                self.speed = None
            self.reset_health()
            return False
        # A repeated/older packet must not renew freshness or settling time.
        if stamp_ns <= self.last_stamp[kind]:
            return False
        previous = getattr(self, kind)
        if previous is None or previous.age(received_s) > timeout:
            self.reset_health()
        sample = Sample(float(x), float(y), stamp_ns, received_s, age_at_receipt_s)
        if kind == 'pose':
            dt = (stamp_ns - previous.stamp_ns)/1e9 if previous else 0.0
            self.speed = (math.hypot(x-previous.x, y-previous.y)/dt
                          if previous and 0 < dt <= self.config.pose_timeout_s else None)
            if self.speed is not None and not math.isfinite(self.speed):
                self.pose = None
                self.speed = None
                self.reset_health()
                return False
            # Do not miss a brief departure between two assessment timer ticks.
            if self.target and self.settle_started_ns is not None:
                distance = math.hypot(x-self.target[0], y-self.target[1])
                if (distance > self.config.arrival_exit_m or self.speed is None
                        or self.speed > self.config.arrival_speed_m_s):
                    self.settle_started_ns = None
        setattr(self, kind, sample)
        self.last_stamp[kind] = stamp_ns
        self.received_count[kind] += 1
        return True

    def evaluate(self, now):
        if not isinstance(now, (int, float)) or not math.isfinite(now):
            raise ValueError('evaluation time must be finite')
        if self.last_evaluation_s is not None and now < self.last_evaluation_s:
            self.pose = self.uwb = None
            self.speed = None
            self.last_stamp = {'pose': 0, 'uwb': 0}
            self.reset_health()
        self.last_evaluation_s = now
        ages = {kind: getattr(self, kind).age(now) if getattr(self, kind) else None
                for kind in ('pose', 'uwb')}
        fresh = {kind: ages[kind] is not None
                 and 0 <= ages[kind] <= getattr(self.config, kind + '_timeout_s')
                 for kind in ages}
        distance = (math.hypot(self.pose.x-self.target[0], self.pose.y-self.target[1])
                    if self.pose and self.target and fresh['pose'] else None)
        if distance is not None and not math.isfinite(distance):
            distance = None
            fresh['pose'] = False
        settle_elapsed = 0.0
        recovery_elapsed = 0.0
        if self.target is None:
            state, reason = 'WAITING_TARGET', self.target_reason
            self.reset_health()
        elif not all(fresh.values()) or self.speed is None:
            if not all(self.seen.values()):
                state, reason = 'WAITING_DATA', 'inputs_missing'
            else:
                state = 'DEGRADED'
                reason = ('pose_unavailable' if not fresh['pose'] else
                          'uwb_unavailable' if not fresh['uwb'] else 'speed_unavailable')
            self.reset_health()
        else:
            if self.healthy_since is None:
                self.healthy_since = now
                self.recovery_counts = {k: v-1 for k, v in self.received_count.items()}
            recovery_elapsed = now - self.healthy_since
            enough_samples = all(self.received_count[k] - self.recovery_counts[k]
                                 >= self.config.recovery_samples for k in self.received_count)
            if recovery_elapsed < self.config.recovery_s or not enough_samples:
                state, reason = 'RECOVERING', 'waiting_continuous_fresh_inputs'
                self.reset_arrival()
            else:
                c = self.config
                self.near = distance <= (c.approach_exit_m if self.near else c.approach_enter_m)
                radius = c.arrival_exit_m if self.settle_started_ns is not None else c.arrival_enter_m
                if distance <= radius and self.speed <= c.arrival_speed_m_s:
                    if self.settle_started_ns is None:
                        self.settle_started_ns = self.pose.stamp_ns
                    settle_elapsed = (self.pose.stamp_ns - self.settle_started_ns)/1e9
                    state = 'ARRIVED' if settle_elapsed >= c.settle_s else 'SETTLING'
                    reason = 'arrival_conditions_met' if state == 'ARRIVED' else 'waiting_settle_time'
                else:
                    self.settle_started_ns = None
                    state = 'APPROACHING' if self.near else 'MOVING'
                    reason = 'near_target' if self.near else 'target_far'
        self.state = state
        return {'schema': 1, 'demo': True, 'state': state, 'reason': reason,
                'target_generation': self.target_generation, 'target_valid': self.target is not None,
                'target_xy_m': list(self.target) if self.target else None,
                'arrival_valid': state == 'ARRIVED', 'distance_m': distance,
                'speed_m_s': self.speed if fresh['pose'] else None,
                'pose_age_s': ages['pose'], 'uwb_age_s': ages['uwb'],
                'settle_elapsed_s': settle_elapsed, 'recovery_elapsed_s': recovery_elapsed,
                'frame_id': self.config.frame_id, 'position_source': 'demo_pose',
                'uwb_role': 'freshness_check_only', 'flight_output': False}
