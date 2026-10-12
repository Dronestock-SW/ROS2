"""Causal UWB integrity. Never publishes extrapolated or repeated poses.

No FC position, IMU integration, flow integration or truth input is permitted.
The accepted positions remain measurements; PX4 owns sensor fusion.
XY displacement is diagnostic only; it never quarantines an observation.
"""
from dataclasses import dataclass
import math
from drone_uwb.processing.motion_reacquisition import MotionReacquisition


@dataclass(frozen=True)
class GuardConfig:
    max_speed_m_s: float = 1.0
    jump_margin_m: float = .10
    acquire_s: float = .25
    recover_s: float = .15
    max_gap_s: float = .25

    def __post_init__(self):
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
               for v in vars(self).values()):
            raise ValueError('positive_integrity_limits_required')


class ObservationGuard:
    def __init__(self, config=GuardConfig(), *, motion_recovery=False):
        self.config = config
        self.last_seen = self.prior_stamp = self.prior_xy = None
        self.stable_since = None
        self.ever_accepted = False
        self.motion = MotionReacquisition() if motion_recovery else None
        self.motion_evidence = None

    def check(self, xy, stamp_ns, *, motion_eligible=True, reject_jumps=False):
        # Legacy keyword remains readable by old launchers, but cannot restore
        # the removed displacement gate. Value/time validation is unchanged.
        if type(reject_jumps) is not bool:
            raise ValueError('boolean_jump_policy_required')
        c = self.config
        self.motion_evidence = None
        if (type(stamp_ns) is not int or stamp_ns <= 0 or len(xy) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in xy)):
            self.stable_since = None
            if self.motion is not None:self.motion.freeze('invalid_observation')
            return 'invalid_observation'
        if self.last_seen is not None and stamp_ns <= self.last_seen:
            if self.motion is not None:self.motion.freeze('duplicate_or_backward_observation')
            return 'duplicate_or_backward_observation'
        gap = None if self.last_seen is None else (stamp_ns-self.last_seen)/1e9
        self.last_seen = stamp_ns
        if gap is not None and gap > c.max_gap_s:
            self.stable_since = None
        self.prior_xy, self.prior_stamp = tuple(xy), stamp_ns
        if self.motion is not None:
            if motion_eligible:self.motion.observe(stamp_ns, xy)
            else:self.motion.clear()
        if self.stable_since is None:
            self.stable_since = stamp_ns
        duration = c.recover_s if self.ever_accepted else c.acquire_s
        if (stamp_ns-self.stable_since)/1e9 < duration:
            return 'observation_recovering' if self.ever_accepted else 'observation_acquiring'
        self.ever_accepted = True
        return 'ready'
