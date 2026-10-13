"""Anchor-frame mission targets; PX4 owns position, altitude and attitude control.

No geographic origin, GPS measurements or motor/attitude commands are produced.
The vertical reference is a fixed PX4-relative height goal, not a height controller.
"""
import math
from .contracts import finite
from .mission_chain import MissionChain
from .native_takeoff import NativeTakeoff
from .observation_policy import observation_status, recent


class LocalXYMission(NativeTakeoff, MissionChain):
    ACTIVE = {'PREPARING', 'ARMING', 'TAKING_OFF', 'HANDOFF', 'MOVING', 'DWELL',
              'RETURNING', 'RECOVERING', 'LANDING', 'GROUND_RECOVERY'}
    STREAMING = (ACTIVE - {'LANDING', 'GROUND_RECOVERY'}) | NativeTakeoff.LOCAL_TAKEOFF_STREAMING
    AUTO_MODES = {'OFFBOARD', 'AUTO.LAND'}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setpoint_xy = None
        self.height_target = self.ground_height = self.held_yaw = None
        self.feedback_ns = self.started_ns = None
        self.last_tick = None
        self.visited = []
        self.goal = None
        self.gap_since = None
        self.last_pose = None
        self.last_pose_source_s = None
        self.mission_ai = None
        self.cpp_ceiling = None
        self.cpp_wait_since = None
        self.cpp_reply = None
        self.cpp_leg_sequence = 0
        self.init_native()

    def submit(self, payload, now, wall_s):
        if self.settings.start_mode == 'takeoff_hold':
            if (self.native_request and payload.get('control_request_id') == self.native_request['request_id']
                    and payload.get('control_action') in (None, 'takeoff')
                    and str(payload.get('drone_id')) == self.settings.drone_id):
                self.last_web_s = now
                return True
            if (payload.get('control_action') == 'start' and not self.assignment and not self.intent
                    and self.phase != 'HOLD_READY'):
                self.validation = dict(accepted=False, reason='separate_takeoff_hold_required')
                return False
        accepted = super().submit(payload, now, wall_s)
        if accepted and self.intent and self.intent['action'] == 'start':
            self.cpp_ceiling = payload.get('ceiling_height_m')
        return accepted

    def ready(self, s, now, *, ground=False):
        support = (not ground and self.settings.lidar_gap.get('enabled') is True
            and self.phase in ('TAKING_OFF','MOVING','DWELL','RETURNING')
            and self.cpp_reply is not None and 0 <= now-self.cpp_reply['sample_time_s'] <= .15
            and self.cpp_reply.get('lidar_support',{}).get('approved') is True
            and isinstance(s.lidar_scan,dict) and 0 <= s.lidar_scan.get('age_s',1e9) <= .15
            and 0 <= s.uwb_age_s <= .8 and 0 <= s.bridge_observation_age_s <= .8)
        reason = super().ready(s, now, ground=ground, horizontal_support=support)
        if reason != 'ready':
            return reason
        if self.settings.mission_ai == 'cpp_bt' and (ground or self.phase in ('IDLE','READY','HOLD_READY')):
            if self.mission_ai is None or not self.mission_ai.status(now)['connected']:
                return 'cpp_mission_ai_unavailable'
        if not finite(s.altitude_enu_m) and (ground or not s.armed):
            return 'px4_local_height_unavailable'
        if not s.local_setpoint_ready and (ground or not s.armed):
            return 'local_setpoint_transport_unavailable'
        if type(s.rc_override) is not int or not s.rc_override & 2:
            return 'px4_offboard_rc_override_required'
        return 'ready'

    def end(self, phase, reason, now):
        super().end(phase, reason, now)
        self.setpoint_xy = None
        if self.mission_ai is not None and phase in ('PILOT_OVERRIDE','UNCONFIRMED','FAILED'):
            self.mission_ai.release(phase, reason)

    def abort(self, reason, s, now):
        self.failure = reason
        self.setpoint_xy = None
        if self.settings.start_mode == 'takeoff_hold' and self.phase == 'PREPARING' and s.mode == self.waiting_mode:
            self.end('UNCONFIRMED' if self.pending else 'FAILED', reason, now)
            return
        # A late ARM acknowledgement has an unknown effect: never resume it.
        if self.pending is not None:
            self.end('UNCONFIRMED', reason, now)
        elif s.connected and not s.armed and s.landed == 1 and s.state_age_s <= 1:
            self.enter('GROUND_RECOVERY', reason, now)
            self.issue('position_mode', now)
        else:
            self.enter('LANDING', reason, now)
            self.issue('land', now)

    def applied_local_target(self, xy, height, stamp_ns):
        if (self.phase not in self.STREAMING or self.setpoint_xy is None
                or self.started_ns is None or stamp_ns < self.started_ns
                or not finite(*xy, height) or len(xy) != 2):
            return False
        if (math.dist(xy, self.setpoint_xy) > .08
                or abs(height-self.height_target) > .02):
            return False
        if self.feedback_ns is None or stamp_ns > self.feedback_ns:
            self.feedback_ns = stamp_ns
            self.target_applied = True
        return True

    def local_setpoint(self):
        if self.phase not in self.STREAMING or self.setpoint_xy is None:
            return None
        return dict(xy=self.setpoint_xy, altitude_enu_m=self.height_target,
                    yaw_deg=self.held_yaw)

    def choose_goal(self, xy, now):
        self.goal = tuple(xy)
        self.stable_since = None
        self.monitor.set_target(*xy, 'uwb_map')
        self.enter('RETURNING' if self.returning else 'MOVING', 'local_xy_target', now)
        self.targets_sent += 1

    def begin_return(self, s, now, ros_ns, reason='route_complete_return'):
        if self.mission_ai is not None:
            self.cpp_command = 'return_to_home'
            return
        self.returning = True
        # Saved trial routes already include the homeward legs. Do not fly
        # the complete round trip a second time after reaching home.
        self.return_path = ([self.home] if math.dist(self.visited[-1], self.home) < .01
                            else list(reversed(self.visited)))
        while len(self.return_path) > 1 and math.dist(self.return_path[0], s.xy) < .15:
            self.return_path.pop(0)
        self.return_index = 0
        self.choose_goal(self.return_path[0], now)

    def tick(self, s, now, wall_s, ros_ns):
        if self.phase in self.NATIVE_PHASES:
            return self.tick_native(s, now, ros_ns)
        self.actions, self.scan_requests = [], []
        dt = 0. if self.last_tick is None else now-self.last_tick
        self.last_tick = now
        if self.phase in self.TERMINAL:
            return []
        if self.phase in ('IDLE', 'READY'):
            stationary = (s.connected and not s.armed and s.landed == 1
                and s.mode in ('POSCTL', 'AUTO.LOITER') and len(s.xy) == 2
                and len(s.velocity_xy) == 2 and s.pose_age_s <= self.settings.pose_timeout_s
                and math.hypot(*s.velocity_xy) <= .1)
            if not stationary or self.ground_xy and math.dist(s.xy, self.ground_xy) > .1:
                self.ground_since = self.ground_xy = None
            elif self.ground_since is None:
                self.ground_since, self.ground_xy = now, tuple(s.xy)
        if self.phase in self.ACTIVE:
            if s.connected and s.state_age_s <= 1:
                if self.rc_switch_reference and s.rc_valid and s.rc_age_s <= 1:
                    if tuple(self.rc_switch_reference) != s.rc_switch_channels or any(
                        abs(s.rc_channels[ch-1]-v) >= 100
                        for ch, v in self.rc_switch_reference.items() if ch <= len(s.rc_channels)):
                        self.end('PILOT_OVERRIDE', 'mapped_rc_switch_changed', now)
                        return []
                expected = ('POSCTL', 'AUTO.LOITER', 'OFFBOARD') if self.phase == 'PREPARING' else (
                    ('OFFBOARD', 'AUTO.LAND') if self.phase == 'LANDING' else
                    ('POSCTL', 'OFFBOARD', 'AUTO.LOITER') if self.phase == 'GROUND_RECOVERY' else ('OFFBOARD',))
                if s.mode not in expected:
                    self.end('PILOT_OVERRIDE', 'manual_or_px4_failsafe_has_priority', now)
                    return []
            if self.pending and now-self.pending['sent_s'] > self.settings.command_timeout_s:
                self.end('UNCONFIRMED', 'command_response_timeout_effect_unknown', now)
                return []
            if self.failure and self.pending is None and self.phase not in ('LANDING', 'GROUND_RECOVERY'):
                self.abort(self.failure, s, now)
                return self.actions
        if self.intent:
            r = self.intent
            if r['action'] == 'start' and self.phase == 'READY':
                self.reason = self.ready(s, now, ground=self.settings.start_mode != 'takeoff_hold')
                if self.settings.start_mode == 'takeoff_hold' and self.reason == 'ready':
                    self.reason = self.hold_blocker(s, now) or 'ready'
                if not 0 <= wall_s-r['requested_s'] <= self.settings.request_ttl_s:
                    self.end('FAILED', 'control_request_expired', now)
                elif not self.settings.execute:
                    self.reason = 'execution_disabled'
                elif self.reason == 'ready' and (self.settings.start_mode == 'takeoff_hold'
                        or self.ground_since is not None and now-self.ground_since >= 3):
                    points = [s.xy]+[p['xy'] for p in r['waypoints']]
                    if any(math.dist(a,b)>self.settings.max_leg_m for a,b in zip(points,points[1:])):
                        self.end('FAILED', 'route_leg_exceeds_trial_limit', now)
                        return []
                    if self.settings.mission_ai == 'cpp_bt':
                        try:
                            if not self.mission_ai.prepare(r, self.settings, s, self.cpp_ceiling):
                                self.reason = 'waiting_for_cpp_mission_plan'
                                return []
                            if math.dist(s.xy, self.mission_ai.prepare_home) > .01:
                                raise ValueError('cpp_ai_launch_changed_regenerate_mission')
                        except (ValueError, OSError) as exc:
                            self.end('FAILED', str(exc), now)
                            return []
                    self.consume(r)
                    self.assignment, self.intent = r, None
                    self.home = self.mission_ai.prepare_home if self.mission_ai else tuple(s.xy)
                    self.visited = [self.home]
                    self.setpoint_xy = self.home
                    self.ground_height = self.height_target = (self.mission_ai.prepare_ground_height
                        if self.mission_ai else s.altitude_enu_m)
                    self.held_yaw = s.yaw_deg
                    self.started_s, self.started_ns = now, ros_ns
                    self.rc_switch_reference = {ch:s.rc_channels[ch-1] for ch in s.rc_switch_channels}
                    self.accepted_command = False
                    self.command_confirmed_s = None
                    self.enter('PREPARING', 'local_setpoint_warmup', now)
                    self.cpp_wait_since = now
                return []
            elif r['action'] in ('land', 'return_to_home') and self.phase in self.ACTIVE:
                self.consume(r); self.intent = None
                if r['action'] == 'return_to_home' and self.phase in ('MOVING','DWELL','RETURNING'):
                    self.begin_return(s, now, ros_ns)
                elif self.phase not in ('LANDING','GROUND_RECOVERY'):
                    self.abort('operator_cancelled', s, now)
                return self.actions
        if self.phase not in self.ACTIVE:
            return []
        if self.phase in ('LANDING', 'GROUND_RECOVERY'):
            if self.mission_ai is not None and not self.failure:
                try:
                    self.cpp_step(s, now, ros_ns)
                except (ValueError, OSError) as exc:
                    self.end('UNCONFIRMED', str(exc), now)
                    return []
            if (self.pending is None and self.accepted_command and self.command_confirmed_s is not None
                    and s.connected and not s.armed and s.landed == 1
                    and 0 <= s.state_age_s <= 1 and 0 <= s.landed_age_s <= 1
                    and now-s.state_age_s >= self.command_confirmed_s
                    and now-s.landed_age_s >= self.command_confirmed_s):
                if (self.phase == 'LANDING' and not self.failure and self.mission_ai is not None
                        and (not self.cpp_reply or self.cpp_reply['phase'] != 'END')):
                    if now-self.entered_s > self.settings.landing_timeout_s:
                        self.end('UNCONFIRMED', 'cpp_ai_landing_completion_unverified', now)
                    return []
                self.landing_verified = self.phase == 'LANDING'
                self.end('FAILED' if self.failure else 'END', self.failure or 'home_landed_disarmed', now)
            elif now-self.entered_s > self.settings.landing_timeout_s:
                self.end('UNCONFIRMED', 'landing_or_mode_completion_unverified', now)
            return []
        reason = self.ready(s, now)
        if reason != 'ready':
            self.abort(reason, s, now)
            return self.actions
        if dt <= 0:
            self.abort('local_setpoint_clock_invalid', s, now)
            return self.actions
        if dt > .2:
            self.stable_since = None
        if now-self.started_s > 300 or s.battery < .15:
            self.abort('mission_time_or_battery_limit', s, now)
            return self.actions
        feedback_age = (ros_ns-self.feedback_ns)/1e9 if self.feedback_ns is not None else math.inf
        self.observation_status = observation_status(self.settings, s, feedback_age)
        if self.phase in ('PREPARING','ARMING') and not s.armed and not recent(s.uwb_age_s,.25):
            self.abort('prearm_uwb_unavailable', s, now)
            return self.actions
        pose_current=recent(s.pose_age_s,self.settings.pose_timeout_s) and len(s.xy)==2 and finite(s.altitude_enu_m)
        if self.phase in ('PREPARING','ARMING') and not s.armed and not pose_current:
            self.abort('prearm_position_feedback_missing',s,now)
            return self.actions
        pose_source_s=now-s.pose_age_s
        pose_dt=max(dt,pose_source_s-self.last_pose_source_s) if pose_current and self.last_pose_source_s is not None else dt
        if pose_current and self.last_pose and (math.dist(s.xy,self.last_pose[:2]) > .3+pose_dt
                              or abs(s.altitude_enu_m-self.last_pose[2]) > .3+pose_dt):
            self.abort('px4_local_frame_or_pose_jump', s, now)
            return self.actions
        if pose_current:self.last_pose = (*s.xy, s.altitude_enu_m);self.last_pose_source_s=pose_source_s
        if self.phase in ('TAKING_OFF','HANDOFF','MOVING','DWELL','RETURNING') and not s.armed:
            self.abort('airborne_state_lost', s, now)
            return self.actions
        if pose_current and self.phase in ('TAKING_OFF','HANDOFF') and math.dist(s.xy,self.home)>.6:
            self.abort('takeoff_horizontal_drift',s,now)
            return self.actions
        if pose_current and self.phase in ('MOVING','RETURNING','DWELL') and math.dist(s.xy,self.setpoint_xy)>.5:
            self.abort('hover_horizontal_drift' if self.phase=='DWELL' else 'local_path_tracking_error',s,now)
            return self.actions
        if self.mission_ai is not None:
            try:
                reply = self.cpp_step(s, now, ros_ns)
            except (ValueError, OSError) as exc:
                self.reason = 'cpp_ai_progress_held:' + str(exc)
                return self.actions
            if reply is None:
                self.reason = 'cpp_ai_progress_held'
                return self.actions
            self.cpp_reply = reply
            if reply['phase'] in ('LANDING', 'FAILED'):
                if (reply['phase'] == 'LANDING' and reply['home_verified']
                        and reply['completed_waypoints'] == len(self.assignment['waypoints'])):
                    self.completed_waypoints = reply['completed_waypoints']
                    self.home_verified = True
                    self.setpoint_xy = None
                    self.enter('LANDING', 'cpp_bt_home_arrival_verified', now)
                    self.issue('land', now)
                else:
                    self.abort('cpp_ai:' + reply['reason'], s, now)
                return self.actions
            if reply['phase'] == 'PILOT_OVERRIDE':
                self.end('PILOT_OVERRIDE', 'cpp_ai_released_control', now)
                return []
        if self.phase == 'PREPARING':
            if self.settings.start_mode == 'takeoff_hold' and (not s.armed or s.landed != 2
                    or math.dist(s.xy,self.home) > .15 or abs(s.altitude_enu_m-self.height_target) > .15):
                self.abort('hold_handoff_pose_changed', s, now)
                return self.actions
            if self.pending is None and not self.accepted_command and now-self.entered_s >= 1.2:
                self.issue('offboard_mode', now)
            elif self.pending is None and self.accepted_command and s.mode == 'OFFBOARD':
                # PX4 only publishes controller position targets after spool-up.
                # OFFBOARD confirmation proves the warmup stream was accepted.
                if self.settings.start_mode == 'takeoff_hold':
                    self.enter('HANDOFF', 'px4_hold_to_offboard_confirmed', now)
                    self.feedback_ns = None
                else:
                    self.enter('ARMING', 'local_mode_confirmed', now)
                    self.issue('arm', now)
            elif now-self.entered_s > 6:
                self.abort('local_mode_or_target_not_confirmed', s, now)
            return self.actions
        if self.phase == 'ARMING':
            if self.pending is None and self.accepted_command and s.armed:
                self.height_target = self.ground_height+self.settings.expected_mis_takeoff_alt_m
                self.enter('TAKING_OFF', 'px4_local_height_goal', now)
                self.feedback_ns = None
            elif now-self.entered_s > self.settings.command_timeout_s:
                self.abort('arm_state_not_confirmed', s, now)
            return self.actions
        if self.mission_ai is None and self.observation_status['progress_held']:
            self.stable_since = None
            self.entered_s = now
            self.reason = 'fc_hold_wait_for_observation_or_target_report'
            return self.actions
        if self.phase in ('TAKING_OFF','HANDOFF'):
            if self.mission_ai is not None:
                try:
                    if self.cpp_reply['phase'] == 'MOVING': self.apply_cpp_navigation(now)
                except (ValueError, TypeError, KeyError) as exc:
                    self.abort('cpp_ai_invalid_decision:' + str(exc), s, now)
                if now-self.entered_s > self.settings.takeoff_timeout_s:
                    self.abort('takeoff_timeout', s, now)
                return self.actions
            if (s.armed and s.landed == 2
                  and abs(s.altitude_enu_m-self.height_target) <= .15 and abs(s.vertical_speed_m_s) <= .1):
                if self.stable_since is None:self.stable_since = now
                if now-self.stable_since >= self.settings.takeoff_settle_s:
                    self.choose_goal(self.waypoint()['xy'], now)
            else:self.stable_since = None
            if now-self.entered_s > self.settings.takeoff_timeout_s:self.abort('takeoff_timeout',s,now)
            return self.actions
        if not s.armed or s.landed != 2:
            self.abort('airborne_state_lost', s, now)
            return self.actions
        if self.mission_ai is not None:
            try:
                self.apply_cpp_navigation(now)
            except (ValueError, TypeError, KeyError) as exc:
                self.abort('cpp_ai_invalid_decision:' + str(exc), s, now)
            return self.actions
        if self.phase in ('MOVING','RETURNING'):
            distance = math.dist(self.setpoint_xy,self.goal)
            step = min(distance,self.settings.speed_m_s*min(dt,.1))
            if distance > 0:self.setpoint_xy = tuple(a+(b-a)*step/distance for a,b in zip(self.setpoint_xy,self.goal))
            if (distance <= .01 and math.dist(s.xy,self.goal) <= .15
                  and len(s.uwb_xy)==2 and math.dist(s.uwb_xy,self.goal)<=.25
                  and math.hypot(*s.velocity_xy)<=.1):
                if self.stable_since is None:self.stable_since = now
                if now-self.stable_since >= 1.:
                    if self.returning:
                        self.return_index += 1
                        if self.return_index == len(self.return_path):
                            self.home_verified = True;self.setpoint_xy = None
                            self.enter('LANDING','home_arrival_verified',now);self.issue('land',now)
                        else:self.choose_goal(self.return_path[self.return_index],now)
                    else:
                        self.visited.append(self.goal)
                        self.enter('DWELL','local_arrival_verified',now)
                        self.stable_since = None
            else:self.stable_since = None
            if self.phase in ('MOVING','RETURNING') and now-self.entered_s > self.settings.leg_timeout_s:
                self.abort('local_leg_timeout',s,now)
        elif self.phase == 'DWELL':
            # Count continuous stable hover, never redefine home after drift.
            stable = (math.dist(s.xy,self.goal) <= .15 and len(s.uwb_xy)==2
                      and math.dist(s.uwb_xy,self.goal) <= .25
                      and math.hypot(*s.velocity_xy) <= .1
                      and abs(s.altitude_enu_m-self.height_target) <= .15
                      and abs(s.vertical_speed_m_s) <= .1)
            if stable:
                if self.stable_since is None:self.stable_since = now
                if now-self.stable_since >= self.waypoint()['dwell_s']:
                    self.completed_waypoints += 1;self.index += 1
                    if self.index == len(self.assignment['waypoints']):self.begin_return(s,now,ros_ns)
                    else:self.choose_goal(self.waypoint()['xy'],now)
            else:self.stable_since = None
            if self.phase == 'DWELL' and now-self.entered_s > self.settings.leg_timeout_s+self.waypoint()['dwell_s']:
                self.abort('hover_stability_timeout',s,now)
        return self.actions

    def cpp_step(self, s, now, ros_ns):
        feedback_age = (ros_ns-self.feedback_ns)/1e9 if self.feedback_ns is not None else math.inf
        confirmed = (self.pending is None and self.accepted_command
            and self.command_confirmed_s is not None
            and s.connected and not s.armed and s.landed == 1
            and 0 <= s.state_age_s <= 1 and 0 <= s.landed_age_s <= 1
            and now-s.state_age_s >= self.command_confirmed_s
            and now-s.landed_age_s >= self.command_confirmed_s)
        command = getattr(self, 'cpp_command', '')
        reply = self.mission_ai.step(s, now, executor_phase=self.phase,
            feedback_age_s=feedback_age, landing_confirmed=confirmed,
            executor_failure=self.failure or '', command=command)
        # The command remains in coalesced snapshots until the C++ return latch
        # is observed. Cancel/LAND still uses the independent executor guard.
        if reply and reply.get('returning'): self.cpp_command = ''
        if reply: self.cpp_reply = reply
        return reply

    def apply_cpp_navigation(self, now):
        reply = self.cpp_reply
        if reply['phase'] not in ('MOVING', 'DWELL', 'RETURNING'):
            raise ValueError('cpp_ai_invalid_navigation_phase')
        goal, stream = tuple(reply['goal_xy']), tuple(reply['stream_xy'])
        if not self.settings.inside(goal) or not self.settings.inside(stream):
            raise ValueError('cpp_ai_goal_outside_bounds')
        if reply['leg_sequence'] != self.cpp_leg_sequence:
            self.targets_sent += 1
            self.cpp_leg_sequence = reply['leg_sequence']
        self.index = reply['waypoint_index']
        self.completed_waypoints = reply['completed_waypoints']
        self.goal, self.setpoint_xy = goal, stream
        self.returning, self.home_verified = reply['returning'], reply['home_verified']
        if self.phase != reply['phase']:
            self.enter(reply['phase'], 'cpp_bt_mission_decision', now)
        self.arrival = dict(source='sangwon_cpp_bt', stable_s=reply['stable_s'],
                            decision_sequence=reply['_sequence'])

    def status(self):
        out = super().status()
        if self.settings.start_mode == 'takeoff_hold' and self.native_home is not None:
            out['home_xy_m'] = list(self.native_home)
        out['observation_status'] = getattr(self,'observation_status',{})
        out['native_takeoff'] = dict(enabled=self.settings.start_mode == 'takeoff_hold',
            hold_ready=self.native_ready, phase=self.phase,
            flight_commands_enabled=self.phase in self.NATIVE_PHASES)
        out['flight_control_enabled'] = out['flight_control_enabled'] or self.phase in self.NATIVE_PHASES
        if self.mission_ai is not None:
            out['mission_ai'] = self.mission_ai.status(self.last_tick)
            if self.cpp_reply:
                out['horizontal_observation'] = dict(self.cpp_reply.get('lidar_support',{}),
                    gap_supported=self.cpp_reply.get('gap_supported',False),
                    observation_gap=self.cpp_reply.get('observation_gap',False))
        else:
            out['mission_ai'] = dict(backend='python_test', connected=False, fc_transport=False)
        return out
