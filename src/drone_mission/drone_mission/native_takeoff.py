"""Explicit web TAKEOFF -> PX4 HOLD. No local setpoints or automatic mission START."""
import math
from .contracts import finite
from .local_takeoff import LocalTakeoff
from .observation_policy import observation_status


def native_speed_ready(s):
    # PX4 d6f12ad1: Auto requires unsmoothed vz < -0.3 and starts at +0.7.
    return (0 <= s.takeoff_param_age_s <= 5 and len(s.native_takeoff_speeds) == 4
            and finite(*s.native_takeoff_speeds)
            and min(s.native_takeoff_speeds[:3]) > .3
            and s.native_takeoff_speeds[3] * 1.3 >= .7)


class NativeTakeoff(LocalTakeoff):
    NATIVE_PHASES = {'NATIVE_MODE', 'NATIVE_ARMING', 'NATIVE_WAIT_ASCENT', 'NATIVE_CLIMB', 'HOLD_READY', 'NATIVE_LANDING'} | LocalTakeoff.LOCAL_TAKEOFF_STREAMING

    @property
    def waiting_mode(self):
        return 'POSCTL' if self.settings.takeoff_backend == 'local_position' else 'AUTO.LOITER'

    def init_native(self):
        self.native_request = None
        self.native_ground = self.native_home = None
        self.native_hold_since = None
        self.native_ready = False
        self.native_armed_s = None

    def takeoff_blocker(self, s, now):
        if self.settings.start_mode != 'takeoff_hold': return 'separate_takeoff_disabled'
        if self.phase != 'IDLE' or self.intent or self.pending: return 'session_not_idle'
        if not self.settings.execute: return 'execution_disabled'
        if self.settings.takeoff_backend == 'native' and not native_speed_ready(s): return 'native_takeoff_speed_incompatible'
        if self.settings.takeoff_backend == 'local_position' and s.rc_required and not s.rc_sticks_centered:
            return 'position_takeoff_requires_centered_rc_sticks'
        if not s.connected or not 0 <= s.state_age_s <= self.settings.state_timeout_s or not 0 <= s.landed_age_s <= self.settings.state_timeout_s:
            return 'fresh_fc_required'
        if s.armed or s.landed != 1 or s.mode != 'POSCTL': return 'disarmed_position_ground_required'
        if self.ground_since is None or now-self.ground_since < 3: return 'stationary_ground_required'
        return ''

    def begin_native(self, request, s, now):
        reason = self.takeoff_blocker(s, now)
        if reason: raise ValueError(reason)
        self.consume(request)
        self.native_request = request
        self.native_ground, self.native_home = s.altitude_enu_m, tuple(s.xy)
        self.rc_switch_reference = {ch:s.rc_channels[ch-1] for ch in s.rc_switch_channels}
        if self.settings.takeoff_backend == 'local_position':self.begin_local_takeoff(s,now)
        else:self.enter('NATIVE_MODE', 'web_takeoff_requested', now)

    def hold_blocker(self, s, now):
        if self.settings.takeoff_backend == 'local_position' and s.rc_required and not s.rc_sticks_centered:
            return 'position_requires_centered_rc_sticks'
        if not self.native_ready or self.native_ground is None: return 'px4_takeoff_hold_required'
        if (not s.connected or not 0 <= s.state_age_s <= self.settings.state_timeout_s or not 0 <= s.landed_age_s <= self.settings.state_timeout_s
                or not s.armed or s.landed != 2 or s.mode != self.waiting_mode):
            return 'fresh_airborne_hold_required'
        if (not finite(s.altitude_enu_m, s.vertical_speed_m_s) or len(s.velocity_xy) != 2
                or not finite(*s.velocity_xy) or not 0 <= s.pose_age_s <= self.settings.pose_timeout_s
                or abs(s.altitude_enu_m-self.native_ground-self.settings.expected_mis_takeoff_alt_m) > .15
                or abs(s.vertical_speed_m_s) > .1 or math.hypot(*s.velocity_xy) > .1
                or math.dist(s.xy, self.native_home) > .15):
            return 'hold_position_height_not_stable'
        return ''

    def native_land(self, reason, s, now):
        self.native_ready = False
        if self.pending is not None:
            self.end('UNCONFIRMED', reason, now)
        elif not s.connected or not 0 <= s.state_age_s <= self.settings.state_timeout_s:
            self.end('UNCONFIRMED', reason, now)
        elif not s.armed:
            self.end('FAILED', reason, now)
        else:
            self.failure = reason
            self.enter('NATIVE_LANDING', reason, now)
            self.issue('land', now)

    def tick_native(self, s, now, ros_ns=None):
        if self.settings.takeoff_backend == 'local_position' and self.phase != 'NATIVE_LANDING':
            return self.tick_local_takeoff(s,now,ros_ns)
        self.actions, self.scan_requests = [], []
        fresh = s.connected and 0 <= s.state_age_s <= self.settings.state_timeout_s
        if fresh:
            expected = {'NATIVE_MODE': ('POSCTL','AUTO.TAKEOFF'),
                        'NATIVE_ARMING': ('AUTO.TAKEOFF',),
                        'NATIVE_WAIT_ASCENT': ('AUTO.TAKEOFF','AUTO.LOITER'),
                        'NATIVE_CLIMB': ('AUTO.TAKEOFF','AUTO.LOITER'),
                        'HOLD_READY': ('AUTO.LOITER',),
                        'NATIVE_LANDING': ('AUTO.TAKEOFF','AUTO.LOITER','AUTO.LAND','OFFBOARD','POSCTL')}
            changed = (self.rc_switch_reference and s.rc_valid and 0 <= s.rc_age_s <= 1 and
                (tuple(self.rc_switch_reference) != s.rc_switch_channels or any(
                    ch > len(s.rc_channels) or abs(s.rc_channels[ch-1]-value) >= 100
                    for ch,value in self.rc_switch_reference.items())))
            if changed or s.mode not in expected[self.phase]:
                self.native_ready = False
                self.end('PILOT_OVERRIDE', 'manual_or_px4_failsafe_has_priority', now)
                return []
        if self.pending and now-self.pending['sent_s'] > self.settings.command_timeout_s:
            self.end('UNCONFIRMED', 'command_response_timeout_effect_unknown', now)
            return []
        if self.failure and self.phase != 'NATIVE_LANDING' and self.pending is None:
            self.native_land(self.failure, s, now)
            return self.actions
        if self.phase == 'NATIVE_LANDING':
            if (self.pending is None and self.accepted_command and self.command_confirmed_s is not None
                    and fresh and not s.armed and s.landed == 1 and 0 <= s.landed_age_s <= self.settings.state_timeout_s
                    and now-s.state_age_s >= self.command_confirmed_s
                    and now-s.landed_age_s >= self.command_confirmed_s):
                self.landing_verified = True
                self.end('END' if self.failure == 'operator_land_before_mission' else 'FAILED', self.failure, now)
            elif now-self.entered_s > self.settings.landing_timeout_s:
                self.end('UNCONFIRMED', 'landing_completion_unverified', now)
            return self.actions
        # Validate observations throughout ascent and the operator's HOLD wait.
        reason = self.ready(s, now)
        if reason != 'ready':
            self.native_land(reason, s, now)
            return self.actions
        if not fresh or not 0 <= s.landed_age_s <= self.settings.state_timeout_s:
            self.native_land('fresh_fc_required', s, now)
            return self.actions
        if s.rc_required and (not s.rc_valid or not 0 <= s.rc_age_s <= 1):
            self.native_land('fresh_rc_required', s, now)
            return self.actions
        if self.phase == 'NATIVE_MODE':
            if self.pending is None and not self.accepted_command:
                self.issue('takeoff_mode', now)
            elif self.pending is None and self.accepted_command and s.mode == 'AUTO.TAKEOFF':
                self.enter('NATIVE_ARMING', 'px4_takeoff_mode_confirmed', now)
                self.issue('arm', now)
        elif self.phase == 'NATIVE_ARMING':
            if self.pending is None and self.accepted_command and s.armed:
                self.native_armed_s = now
                self.enter('NATIVE_WAIT_ASCENT', 'armed_waiting_for_liftoff', now)
        else:
            if not s.armed:
                self.end('UNCONFIRMED', 'disarmed_before_liftoff' if self.phase == 'NATIVE_WAIT_ASCENT' else 'unexpected_disarm', now)
                return []
            if self.phase == 'NATIVE_WAIT_ASCENT' and s.landed == 2:
                self.enter('NATIVE_CLIMB', 'px4_airborne_confirmed', now)
            if math.dist(s.xy,self.native_home) > .6:
                self.native_land('native_takeoff_horizontal_drift', s, now)
                return self.actions
            self.observation_status = observation_status(self.settings,s)
            stable = (not self.observation_status['progress_held'] and s.mode == 'AUTO.LOITER' and s.landed == 2 and
                abs(s.altitude_enu_m-self.native_ground-self.settings.expected_mis_takeoff_alt_m) <= .15
                and math.hypot(*s.velocity_xy) <= .1 and abs(s.vertical_speed_m_s) <= .1
                and math.dist(s.xy,self.native_home) <= .15)
            if not stable:
                self.native_hold_since = None
                self.native_ready = False
            elif self.native_hold_since is None:
                self.native_hold_since = now
            elif now-self.native_hold_since >= 3:
                self.native_ready = True
                if self.phase != 'HOLD_READY': self.enter('HOLD_READY', 'waiting_for_separate_web_start', now)
            if self.phase in ('NATIVE_WAIT_ASCENT','NATIVE_CLIMB') and now-self.native_armed_s > self.settings.takeoff_timeout_s:
                self.native_land('native_takeoff_timeout', s, now)
        if self.phase in ('NATIVE_MODE','NATIVE_ARMING') and now-self.entered_s > 6:
            self.end('UNCONFIRMED', 'native_mode_or_arm_not_confirmed', now)
        if s.battery < .2 and self.phase in ('NATIVE_WAIT_ASCENT','NATIVE_CLIMB','HOLD_READY'):
            self.native_land('battery_below_hold_reserve', s, now)
        return self.actions
