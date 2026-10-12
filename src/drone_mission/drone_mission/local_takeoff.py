"""Separate web local-position takeoff; PX4 closes all control loops."""
import math


def sticks_centered(channels, parameters):
    try:
        for axis in ('ROLL','PITCH','YAW','THROTTLE'):
            ch=parameters['RC_MAP_'+axis]
            if type(ch) is not int or not 1 <= ch <= len(channels):return False
            lo,hi,trim=(parameters[f'RC{ch}_{k}'] for k in ('MIN','MAX','TRIM'))
            if not all(math.isfinite(v) for v in (lo,hi,trim)) or not 400 <= hi-lo <= 1600:return False
            center=(lo+hi)/2 if axis=='THROTTLE' else trim
            if abs(channels[ch-1]-center) > .07*(hi-lo):return False
        return True
    except (KeyError,TypeError,ValueError):return False


class LocalTakeoff:
    LOCAL_TAKEOFF_STREAMING = {'LOCAL_TAKEOFF_PREPARING', 'LOCAL_TAKEOFF_MODE',
        'LOCAL_TAKEOFF_ARMING', 'LOCAL_TAKEOFF_CLIMB', 'LOCAL_POSITION_HANDOFF'}

    def begin_local_takeoff(self, s, now):
        self.setpoint_xy = self.native_home
        self.ground_height = self.height_target = self.native_ground
        self.held_yaw = s.yaw_deg
        self.feedback_ns = None
        self.started_ns = s.pose_stamp_ns
        self.native_armed_s = None
        self.native_hold_since = None
        self.enter('LOCAL_TAKEOFF_PREPARING', 'local_takeoff_setpoint_warmup', now)

    def tick_local_takeoff(self, s, now, ros_ns):
        self.actions, self.scan_requests = [], []
        fresh = s.connected and 0 <= s.state_age_s <= self.settings.state_timeout_s
        if fresh:
            modes = {'LOCAL_TAKEOFF_PREPARING': ('POSCTL',),
                'LOCAL_TAKEOFF_MODE': ('POSCTL','OFFBOARD'),
                'LOCAL_TAKEOFF_ARMING': ('OFFBOARD',), 'LOCAL_TAKEOFF_CLIMB': ('OFFBOARD',),
                'LOCAL_POSITION_HANDOFF': ('OFFBOARD','POSCTL'), 'HOLD_READY': ('POSCTL',)}
            changed = (self.rc_switch_reference and s.rc_valid and 0 <= s.rc_age_s <= 1 and
                (tuple(self.rc_switch_reference) != s.rc_switch_channels or any(
                    ch > len(s.rc_channels) or abs(s.rc_channels[ch-1]-value) >= 100
                    for ch,value in self.rc_switch_reference.items())))
            if changed or s.mode not in modes[self.phase]:
                self.native_ready = False
                self.end('PILOT_OVERRIDE','manual_or_px4_failsafe_has_priority',now)
                return []
            if self.phase == 'HOLD_READY' and s.rc_required and not s.rc_sticks_centered:
                self.native_ready=False
                self.end('PILOT_OVERRIDE','position_sticks_moved',now)
                return []
        if self.pending and now-self.pending['sent_s'] > self.settings.command_timeout_s:
            self.native_ready = False
            self.end('UNCONFIRMED','command_response_timeout_effect_unknown',now)
            return []
        if self.failure and self.pending is None:
            self.native_land(self.failure,s,now)
            return self.actions
        reason = self.ready(s,now)
        if reason != 'ready':
            self.native_land(reason,s,now)
            return self.actions
        if not fresh or not 0 <= s.landed_age_s <= self.settings.state_timeout_s:
            self.native_land('fresh_fc_required',s,now)
            return self.actions
        if s.rc_required and (not s.rc_valid or not 0 <= s.rc_age_s <= 1):
            self.native_land('fresh_rc_required',s,now)
            return self.actions
        if math.dist(s.xy,self.native_home) > .6:
            self.native_land('local_takeoff_horizontal_drift',s,now)
            return self.actions
        if self.phase in ('LOCAL_TAKEOFF_CLIMB','LOCAL_POSITION_HANDOFF','HOLD_READY') and not s.armed:
            self.native_ready = False
            self.end('UNCONFIRMED','unexpected_disarm',now)
            return []
        feedback = self.feedback_ns is not None and 0 <= (ros_ns-self.feedback_ns)/1e9 <= .5
        if self.phase == 'LOCAL_TAKEOFF_PREPARING':
            if now-self.entered_s >= 1. and feedback:
                self.enter('LOCAL_TAKEOFF_MODE','request_offboard_for_local_takeoff',now)
                self.issue('offboard_mode',now)
            elif now-self.entered_s > 6:
                self.native_land('local_takeoff_feedback_missing',s,now)
        elif self.phase == 'LOCAL_TAKEOFF_MODE':
            if self.pending is None and self.accepted_command and s.mode == 'OFFBOARD':
                self.enter('LOCAL_TAKEOFF_ARMING','offboard_confirmed',now)
                self.issue('arm',now)
        elif self.phase == 'LOCAL_TAKEOFF_ARMING':
            if self.pending is None and self.accepted_command and s.armed:
                self.native_armed_s = now
                self.height_target = self.native_ground+self.settings.expected_mis_takeoff_alt_m
                self.feedback_ns = None
                self.enter('LOCAL_TAKEOFF_CLIMB','armed_waiting_for_local_target_ascent',now)
        else:
            if self.phase == 'LOCAL_TAKEOFF_CLIMB' and not feedback and now-self.entered_s > 1.:
                self.native_land('local_takeoff_feedback_stale',s,now)
                return self.actions
            stable = (s.landed == 2 and abs(s.altitude_enu_m-self.native_ground-self.settings.expected_mis_takeoff_alt_m) <= .15
                and math.hypot(*s.velocity_xy) <= .1 and abs(s.vertical_speed_m_s) <= .1
                and math.dist(s.xy,self.native_home) <= .15)
            if self.phase == 'LOCAL_POSITION_HANDOFF':
                if self.pending is None and self.accepted_command and s.mode == 'POSCTL':
                    self.setpoint_xy = None
                    self.native_hold_since = None
                    self.enter('HOLD_READY','position_mode_confirmed_waiting_for_stability',now)
            elif stable:
                if self.native_hold_since is None:self.native_hold_since=now
                elif now-self.native_hold_since >= 3:
                    if self.phase == 'LOCAL_TAKEOFF_CLIMB' and (not s.rc_required or s.rc_sticks_centered):
                        self.enter('LOCAL_POSITION_HANDOFF','height_and_xy_stable_request_position',now)
                        self.issue('position_mode',now)
                    elif self.phase == 'HOLD_READY':self.native_ready=True
            else:
                self.native_hold_since = None
                self.native_ready = False
            if self.phase == 'LOCAL_TAKEOFF_CLIMB' and now-self.native_armed_s > self.settings.takeoff_timeout_s:
                self.native_land('local_takeoff_timeout',s,now)
        if self.phase in ('LOCAL_TAKEOFF_MODE','LOCAL_TAKEOFF_ARMING','LOCAL_POSITION_HANDOFF') and now-self.entered_s > 6:
            self.native_land('local_takeoff_mode_not_confirmed',s,now)
        if s.battery < .2 and self.native_armed_s is not None:
            self.native_land('battery_below_hold_reserve',s,now)
        return self.actions
