"""Experimental Offboard XY velocity path, for isolated SITL only.

The operational FlightNode does not instantiate this class. XY command
generation and obstacle limiting share the LocalXYMission state machine;
PX4 keeps the existing fixed vertical position goal and all attitude control.
No claim about physical braking is made by this command generator.
"""
import math

from .contracts import finite
from .local_xy import LocalXYMission


class GuardedLocalXYMission(LocalXYMission):
    VELOCITY_PHASES = {'MOVING', 'RETURNING', 'DWELL'}

    def __init__(self, settings, *, guard, proportional_gain_s_inv,
                 resume_clear_s, max_hold_s, hover_position_hold=False,
                 native_position_gain_bound_s_inv=None, native_speed_limit_m_s=None, **kwargs):
        if (not settings.local_xy or not finite(proportional_gain_s_inv,
                resume_clear_s, max_hold_s) or proportional_gain_s_inv <= 0
                or resume_clear_s <= 0 or max_hold_s <= resume_clear_s):
            raise ValueError('explicit_experimental_horizontal_contract_required')
        if hover_position_hold and (native_position_gain_bound_s_inv is None
                or not finite(native_position_gain_bound_s_inv)
                or native_position_gain_bound_s_inv <= 0):
            raise ValueError('native_position_gain_bound_required')
        super().__init__(settings, **kwargs)
        self.guard = guard
        self.xy_gain = proportional_gain_s_inv
        self.resume_clear_s, self.max_hold_s = resume_clear_s, max_hold_s
        self.velocity_xy = None
        self.guard_decision = None
        self.hold_since = self.clear_since = None
        self.last_clear_sequence = None
        self.brake_hold_xy = None
        self.hover_position_hold = bool(hover_position_hold)
        self.native_position_gain_bound = native_position_gain_bound_s_inv
        if native_speed_limit_m_s is not None and (not finite(native_speed_limit_m_s)
                or not 0 < native_speed_limit_m_s <= settings.speed_m_s):
            raise ValueError('verified_native_speed_limit_required')
        # Only callers that read back PX4's speed cap may supply this bound.
        self.native_speed_limit = native_speed_limit_m_s

    def _guard_velocity(self, s, now, ros_ns):
        error = tuple(b-a for a,b in zip(s.xy, self.goal))
        distance = math.hypot(*error)
        speed = min(self.settings.speed_m_s, self.xy_gain*distance)
        if self.hover_position_hold and self.phase == 'DWELL':
            # Guard the maximum position-loop demand, including recovery
            # after drift. Only an entirely clear decision permits handoff;
            # any speed reduction remains an explicitly limited velocity.
            speed = max(self.xy_gain, self.native_position_gain_bound)*distance
            if self.native_speed_limit is not None:
                speed = min(speed,self.native_speed_limit)
        desired = tuple(e*speed/distance for e in error) if distance > 1e-9 else (0., 0.)
        decision = self.guard.limit(desired_xy_m_s=desired,
            measured_xy_m_s=s.velocity_xy, current_xy_m=s.xy, now_ns=ros_ns)
        allowed = decision['velocity_xy_m_s']
        allowed_speed = math.hypot(*allowed)
        if allowed_speed > self.settings.speed_m_s:
            allowed = tuple(v*self.settings.speed_m_s/allowed_speed for v in allowed)
            decision = dict(decision,velocity_xy_m_s=allowed,reason='limited')
        safe = decision['reason'] in ('clear', 'limited', 'zero_requested')
        if not safe:
            if self.hold_since is None:
                self.hold_since = now
                # Zero velocity alone can retain physical drift when the EKF
                # velocity is biased. Latch one PX4 position hold target;
                # never chase the drifting estimate by relatching each tick.
                self.brake_hold_xy = tuple(s.xy)
            self.clear_since = self.last_clear_sequence = None
        elif self.hold_since is not None:
            # Resume requires time with multiple NEW clear scans. Repeated
            # control ticks over one scan do not count as repeated evidence.
            if self.clear_since is None:
                self.clear_since = now
                self.last_clear_sequence = decision['source_sequence']
            elif (decision['source_sequence'] != self.last_clear_sequence
                    and now-self.clear_since >= self.resume_clear_s
                    and math.hypot(*s.velocity_xy) <= .1):
                self.hold_since = self.clear_since = None
                self.last_clear_sequence = None
                self.brake_hold_xy = None
        if self.hold_since is not None:
            allowed = (0., 0.)
            decision = dict(decision, resume_pending=True)
        self.guard_decision = decision
        self.velocity_xy = tuple(allowed)
        return distance

    def advance_horizontal_target(self, s, dt, ros_ns):
        distance = self._guard_velocity(s, self.last_tick, ros_ns)
        # This local XY field is bookkeeping, not the commanded position.
        # The velocity mask ignores it. Home and the route goal stay immutable.
        self.setpoint_xy = tuple(s.xy)
        return distance

    def tick(self, s, now, wall_s, ros_ns):
        actions = super().tick(s, now, wall_s, ros_ns)
        if self.phase in self.VELOCITY_PHASES:
            # Covers DWELL and the exact transition out of TAKEOFF.
            self._guard_velocity(s, now, ros_ns)
            if self.hold_since is not None and now-self.hold_since > self.max_hold_s:
                self.abort('horizontal_obstacle_hold_timeout', s, now)
                actions = self.actions
        else:
            self.velocity_xy = None
        return actions

    def horizontal_target_ready(self, command_distance):
        # There is no position ramp in the velocity path. The superclass still
        # checks measured XY within 15 cm, UWB within 25 cm, and stable speed.
        return self.hold_since is None

    def horizontal_hover_ready(self, s, now, ros_ns):
        # The superclass has already checked state validity and pilot priority.
        self._guard_velocity(s, now, ros_ns)
        return (self.hold_since is None and self.guard_decision is not None
                and self.guard_decision['reason'] in ('clear', 'zero_requested'))

    def local_setpoint(self):
        goal = super().local_setpoint()
        if goal is not None and self.phase in self.VELOCITY_PHASES:
            if self.brake_hold_xy is not None:
                goal['xy'] = self.brake_hold_xy
                goal['horizontal_control'] = 'position'
                goal['obstacle_braking_hold'] = True
            elif self._hover_position_active():
                # Arrival has already verified the immutable route goal.
                # Let PX4's native position loop maintain it during dwell.
                # Recompute the obstacle decision every tick; a limited or
                # blocked corridor takes priority over position recovery.
                goal['xy'] = self.goal
                goal['horizontal_control'] = 'position'
                goal['hover_position_hold'] = True
            else:
                goal['velocity_xy_m_s'] = self.velocity_xy or (0., 0.)
                goal['horizontal_control'] = 'velocity'
        return goal

    def _hover_position_active(self):
        return (self.hover_position_hold and self.phase == 'DWELL' and self.hold_since is None
                and self.guard_decision is not None
                and self.guard_decision['reason'] in ('clear', 'zero_requested'))

    def applied_local_velocity(self, velocity_xy, height, stamp_ns):
        if (self.phase not in self.VELOCITY_PHASES or self.velocity_xy is None
                or self.brake_hold_xy is not None or self._hover_position_active()
                or self.started_ns is None or stamp_ns < self.started_ns
                or len(velocity_xy) != 2 or not finite(*velocity_xy, height)
                or math.dist(velocity_xy, self.velocity_xy) > .08
                or abs(height-self.height_target) > .02):
            return False
        if self.feedback_ns is None or stamp_ns > self.feedback_ns:
            self.feedback_ns = stamp_ns
            self.target_applied = True
        return True

    def applied_local_target(self, xy, height, stamp_ns):
        if self.phase in self.VELOCITY_PHASES:
            expected = self.brake_hold_xy
            if expected is None and self._hover_position_active():
                expected = self.goal
            if (expected is None or self.started_ns is None
                    or stamp_ns < self.started_ns or len(xy) != 2
                    or not finite(*xy, height)
                    or math.dist(xy, expected) > .08
                    or abs(height-self.height_target) > .02):
                return False
            if self.feedback_ns is None or stamp_ns > self.feedback_ns:
                self.feedback_ns = stamp_ns
                self.target_applied = True
            return True
        return super().applied_local_target(xy, height, stamp_ns)

    def status(self):
        value = super().status()
        value['horizontal_obstacle_guard'] = self.guard_decision
        value['experimental_sitl_only'] = True
        return value
