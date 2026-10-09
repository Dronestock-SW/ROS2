"""Native PX4 mission execution: XY/yaw requests and scan action contracts.

PX4 is the sole flight estimator and controller. No vertical setpoints, attitude
controller, integrated flow or duplicate UWB observations are produced here.
"""
import math
import secrets
from dataclasses import replace
from .contracts import finite
from .session import FlightSession


class MissionChain(FlightSession):
    TARGET_PHASES = {'MOVING', 'RETURNING', 'RECOVERING', 'DWELL', 'ALIGNING', 'SCANNING', 'EGRESS'}
    ACTIVE = {'PREPARING', 'ARMING', 'TAKING_OFF', 'STABILIZING', 'MOVING',
              'DWELL', 'ALIGNING', 'SCANNING', 'EGRESS', 'RETURNING', 'RECOVERING', 'LANDING'}
    TERMINAL = {'END', 'FAILED', 'PILOT_OVERRIDE', 'UNCONFIRMED'}

    def __init__(self, settings, *args, **kwargs):
        super().__init__(settings, *args, **kwargs)
        self.stable_since = self.recovery_since = None
        self.resume_phase = None
        self.return_path = []
        self.return_index = 0
        self.home_verified = False
        self.scan_window = self.marker = self.scan_result = None
        self.scan_requests = []
        self.scan_results = []
        self.target_yaw_deg = None
        self.cancel_requested = False
        self.last_scan_stamp = 0
        self.last_result_stamp = 0
        self.scan_deadline = None
        self.ground_since = self.ground_xy = None
        self.started_s = None
        self.rc_switch_reference = None

    def submit(self, payload, now, wall_s):
        if self.phase == 'PILOT_OVERRIDE':
            self.validation = {'accepted': False, 'reason': 'pilot_authority_latched_new_ground_session_required'}
            return False
        if (self.assignment and isinstance(payload, dict) and payload.get('control_action') == 'start'
                and payload.get('control_request_id') not in self.consumed):
            self.validation = {'accepted': False, 'reason': 'execution_requires_new_ground_session'}
            return False
        return super().submit(payload, now, wall_s)

    def ready(self, s, now, *, ground=False):
        reason = super().ready(s, now, ground=ground)
        if reason != 'ready':
            return reason
        if not self.settings.heading_control_confirmed:
            return 'native_heading_control_not_confirmed'
        if s.mag_type != self.settings.expected_ekf2_mag_type:
            return 'px4_heading_policy_mismatch_or_unavailable'
        if ground and s.mag_type in (0, 1) and self.settings.expected_mis_takeoff_alt_m < 1.6:
            return 'native_mag_final_alignment_requires_verified_height'
        if ground and (type(s.rc_override) is not int or not s.rc_override & 1):
            return 'px4_auto_rc_override_required'
        if ground and s.rc_required and (not s.rc_valid or not 0 <= s.rc_age_s <= 1.):
            return 'live_rc_input_required'
        if ground and s.rc_required and s.rc_mode != 0:
            return 'physical_rc_receiver_mode_required'
        if ground and s.rc_required and not s.rc_mapping_valid:
            return 'physical_rc_switch_mapping_required'
        if (not finite(s.battery) or not 0 <= s.battery <= 1
                or not 0 <= s.battery_age_s <= 3):
            return 'battery_unavailable'
        if ground and s.battery < .3:
            return 'battery_below_start_reserve'
        if ground and s.takeoff_action != 0:
            return 'px4_takeoff_action_must_hold'
        if len(s.velocity_xy) != 2 or not finite(*s.velocity_xy):
            return 'px4_velocity_unavailable'
        if not finite(s.vertical_speed_m_s):
            return 'px4_native_climb_state_unavailable'
        if not finite(s.yaw_deg):
            return 'px4_yaw_unavailable'
        return 'ready'

    def end(self, phase, reason, now):
        if phase == 'PILOT_OVERRIDE' and self.scan_window is not None:
            self.scan_requests.append(self.scan_request('CANCEL'))
            self.scan_window = self.marker = self.scan_result = None
        self.pending = self.intent = None
        self.enter(phase, reason, now)

    def expected_mode(self, mode):
        # AUTO.LAND requested by a pilot/failsafe is also an external takeover
        # during movement. Do not send HOLD or a new LAND over that selection.
        if self.phase == 'PREPARING':
            return mode in ('POSCTL', 'AUTO.LOITER', 'AUTO.TAKEOFF')
        if self.phase == 'ARMING':
            return mode == 'AUTO.TAKEOFF'
        if self.phase == 'TAKING_OFF':
            return mode in ('AUTO.TAKEOFF', 'AUTO.LOITER')
        if self.phase == 'LANDING':
            return mode in self.AUTO_MODES
        return mode == 'AUTO.LOITER'

    def abort(self, reason, s, now):
        self.failure = reason
        if self.pending is not None:
            # An unknown in-flight service effect cannot be superseded safely.
            self.end('UNCONFIRMED', reason+'_command_effect_unknown', now)
            return
        if s.mode not in self.AUTO_MODES and s.armed:
            self.end('PILOT_OVERRIDE', 'external_mode_has_priority', now)
            return
        self.enter('LANDING', reason, now)
        self.issue('disarm' if s.landed == 1 else 'land', now)

    def move(self, target, s, now, ros_ns, yaw_deg=None, speed_m_s=None):
        super().move(target, s, now, ros_ns)
        if self.phase not in ('MOVING','RETURNING'):
            return False
        self.stable_since = None
        self.target_yaw_deg = yaw_deg
        if self.actions and speed_m_s is not None:
            self.actions[-1]['speed'] = min(speed_m_s, self.settings.speed_m_s)
        if self.actions and self.actions[-1]['kind'] == 'reposition' and yaw_deg is not None:
            # PX4 d6f12ad navigator consumes DO_REPOSITION yaw in NED radians.
            self.actions[-1]['yaw_rad'] = math.radians(90-yaw_deg-self.settings.enu_yaw_deg)
        return True

    def waypoint(self):
        return self.assignment['waypoints'][self.index]

    def begin_return(self, s, now, ros_ns, reason='tasks_finished'):
        self.returning = True
        self.reason = reason
        # Retrace visited segment endpoints. Validate every homeward leg too.
        self.return_path = list(reversed([self.home]+[
            p['xy'] for p in self.assignment['waypoints'][:self.index]]))
        while self.return_path and math.dist(self.return_path[0], s.xy) < .15:
            self.return_path.pop(0)
        if not self.return_path:
            self.return_path = [self.home]
        self.return_index = 0
        self.move(self.return_path[0], s, now, ros_ns)

    def next_task(self, s, now, ros_ns):
        self.completed_waypoints += 1
        self.index += 1
        self.scan_window = self.marker = self.scan_result = None
        self.scan_deadline = None
        if self.index == len(self.assignment['waypoints']):
            self.begin_return(s, now, ros_ns)
        else:
            p = self.waypoint()
            self.move(p['xy'], s, now, ros_ns, p['yaw_deg'])

    def finish_scan(self, result, s, now, ros_ns):
        # Complete the current handoff before requesting staging egress. An
        # expired scan window must never overwrite an unconfirmed XY command.
        if self.pending is not None:
            return False
        self.scan_results.append(result)
        self.scan_requests.append(self.scan_request('CANCEL'))
        self.scan_window = self.marker = self.scan_result = None
        p = self.waypoint()
        if self.move(p['xy'],s,now,ros_ns,p['yaw_deg']):
            self.enter('EGRESS','scan_result_recorded_return_to_staging',now)
            return True
        return False

    def observe_marker(self, value, now_ns):
        if self.phase not in ('ALIGNING', 'SCANNING') or self.scan_window is None or not isinstance(value, dict):
            return False
        p = self.waypoint()
        stamp = value.get('stamp_ns')
        if (value.get('window_id') != self.scan_window
                or value.get('execution_id') != self.assignment['request_id']
                or value.get('task_id') != p['id'] or value.get('marker_id') != p['marker_id']
                or value.get('calibration_ref') != p['calibration_ref']
                or value.get('mounting_ref') != p['mounting_ref']
                or value.get('frame_id') != 'uwb_map' or value.get('valid') is not True
                or type(stamp) is not int or not 0 <= (now_ns-stamp)/1e9 <= .1
                or stamp <= self.last_scan_stamp):
            return False
        xy, yaw = value.get('xy_m'), value.get('yaw_deg')
        if (not isinstance(xy, list) or len(xy) != 2 or not finite(*xy, yaw)
                or math.dist(xy, p['xy']) > .1
                or abs(math.remainder(yaw-p['yaw_deg'], 360)) > 10
                or not self.settings.inside(xy)):
            return False
        self.last_scan_stamp = stamp
        self.marker = dict(value)
        return True

    def observe_scan_result(self, value, now_ns):
        if self.phase != 'SCANNING' or not isinstance(value, dict):
            return False
        p = self.waypoint()
        stamp = value.get('stamp_ns')
        if (value.get('window_id') != self.scan_window
                or value.get('execution_id') != self.assignment['request_id']
                or value.get('task_id') != p['id'] or value.get('label_id') != p['label_id']
                or value.get('marker_id') != p['marker_id']
                or value.get('calibration_ref') != p['calibration_ref']
                or value.get('mounting_ref') != p['mounting_ref']
                or value.get('frame_id') != 'uwb_map'
                or value.get('outcome') not in ('SUCCEEDED', 'FAILED')
                or type(stamp) is not int or not 0 <= (now_ns-stamp)/1e9 <= .25
                or stamp <= self.last_result_stamp
                or (value['outcome'] == 'SUCCEEDED' and
                    (value.get('stored') is not True or not isinstance(value.get('result_id'), str)
                     or not value['result_id']))):
            return False
        self.last_result_stamp = stamp
        self.scan_result = dict(value)
        return True

    def arrived(self, s, now, ros_ns):
        if self.target_last_stamp_ns is None or (ros_ns-self.target_last_stamp_ns)/1e9 > .5:
            return False
        self.monitor.update('pose', *s.xy, s.pose_stamp_ns, now, s.pose_age_s,
                            'uwb_map', velocity_xy=s.velocity_xy)
        self.monitor.update('uwb', *s.uwb_xy, s.uwb_stamp_ns, now, s.uwb_age_s, 'uwb_map')
        self.arrival = self.monitor.evaluate(now)
        yaw_ok = (self.target_yaw_deg is None or
                  abs(math.remainder(s.yaw_deg-self.target_yaw_deg, 360)) <= 3)
        return self.accepted_command and self.target_applied and self.arrival['arrival_valid'] and yaw_ok

    def external_ready(self, s, now):
        return super().ready(s, now) == 'ready'

    def tick(self, s, now, wall_s, ros_ns):
        self.actions, self.scan_requests = [], []
        if self.phase in ('IDLE', 'READY'):
            stationary = (s.connected and not s.armed and s.landed == 1
                          and s.mode in ('POSCTL','AUTO.LOITER')
                          and s.xy and s.velocity_xy and 0 <= s.pose_age_s <= self.settings.pose_timeout_s
                          and math.hypot(*s.velocity_xy) <= .1)
            if not stationary or self.ground_xy is not None and math.dist(s.xy,self.ground_xy) > .1:
                self.ground_since = self.ground_xy = None
            elif self.ground_since is None:
                self.ground_since, self.ground_xy = now,tuple(s.xy)
        if self.phase in self.TERMINAL:
            self.intent = None
            return []
        if (self.phase in ('IDLE','READY') and s.connected and s.armed and s.landed == 2
                and 0 <= s.state_age_s <= self.settings.state_timeout_s):
            self.end('UNCONFIRMED', 'airborne_without_active_session_no_authority', now)
            return []
        if self.phase in self.ACTIVE:
            if (s.rc_required and self.rc_switch_reference and s.rc_valid and s.rc_mapping_valid
                    and 0 <= s.rc_age_s <= 1.):
                # Only mapped flight switches, not accessory controls. These
                # receiver values are microseconds. PX4 still owns RC handling.
                changed = (tuple(self.rc_switch_reference) != s.rc_switch_channels
                           or any(abs(s.rc_channels[ch-1]-value) >= 100
                                  for ch,value in self.rc_switch_reference.items()
                                  if ch <= len(s.rc_channels)))
                if changed:
                    self.end('PILOT_OVERRIDE', 'mapped_rc_switch_changed', now)
                    return []
            if (s.connected and 0 <= s.state_age_s <= self.settings.state_timeout_s
                    and (s.armed or self.phase in ('PREPARING', 'ARMING'))
                    and not self.expected_mode(s.mode)):
                self.end('PILOT_OVERRIDE', 'manual_or_px4_failsafe_has_priority', now)
                return []
            if self.pending and now-self.pending['sent_s'] > self.settings.command_timeout_s:
                self.end('UNCONFIRMED', 'command_response_timeout_effect_unknown', now)
                return []
            if self.failure and self.pending is None and self.phase != 'LANDING':
                self.abort(self.failure, s, now)
                return self.actions
        if self.intent:
            request = self.intent
            if request['action'] == 'start' and self.phase == 'READY':
                self.reason = self.ready(s, now, ground=True)
                if not self.settings.execute:
                    self.reason = 'execution_disabled'
                elif not 0 <= wall_s-request['requested_s'] <= self.settings.request_ttl_s:
                    self.end('FAILED', 'control_request_expired', now)
                elif self.reason == 'ready':
                    if self.ground_since is None or now-self.ground_since < 3.:
                        self.reason = 'three_seconds_stationary_ground_required'
                        return []
                    points = [s.xy]+[p['xy'] for p in request['waypoints']]
                    if any(math.dist(a,b)>self.settings.max_leg_m for a,b in zip(points,points[1:])):
                        self.end('FAILED', 'route_leg_exceeds_trial_limit', now)
                        return []
                    self.consume(request)
                    self.assignment, self.intent = request, None
                    self.home, self.origin = tuple(s.xy), s.origin
                    self.rc_switch_reference = {ch:s.rc_channels[ch-1] for ch in s.rc_switch_channels}
                    self.started_s = now
                    self.enter('PREPARING', 'native_takeoff_mode_before_arm', now)
                    self.issue('takeoff_mode', now)
                    return self.actions
            elif request['action'] in ('land', 'return_to_home') and self.phase in self.ACTIVE:
                self.consume(request)
                self.intent = None
                self.cancel_requested = request['action'] == 'land'
                if self.pending:
                    # Await a possible late ARM before issuing a ground disarm.
                    self.failure = 'operator_cancelled'
                elif request['action'] == 'return_to_home' and self.phase not in ('PREPARING','ARMING','TAKING_OFF','LANDING'):
                    self.begin_return(s, now, ros_ns, 'operator_return')
                else:
                    self.abort('operator_cancelled', s, now)
                return self.actions
        if self.phase not in self.ACTIVE:
            return self.actions
        if self.phase == 'LANDING':
            if self.pending is None and self.accepted_command and self.command_confirmed_s is not None:
                fresh = (s.connected and not s.armed and s.landed == 1
                         and 0 <= s.state_age_s <= self.settings.state_timeout_s
                         and 0 <= s.landed_age_s <= self.settings.state_timeout_s
                         and now-s.state_age_s >= self.command_confirmed_s
                         and now-s.landed_age_s >= self.command_confirmed_s)
                if fresh:
                    self.landing_verified = True
                    self.end('FAILED' if self.failure else 'END',
                             self.failure or 'home_landed_disarmed', now)
                    return []
            if self.failure and self.pending is None and not self.accepted_command:
                self.end('UNCONFIRMED', self.failure, now)
            elif now-self.entered_s > self.settings.landing_timeout_s:
                self.end('UNCONFIRMED', 'landing_completion_unverified', now)
            return []
        if self.started_s is not None and now-self.started_s > 300.:
            self.abort('mission_time_budget_exceeded', s, now)
            return self.actions
        # Immediate failures of FC state/estimation are never hidden by UWB grace.
        relaxed = replace(s, uwb_age_s=0., height_age_s=0., bridge_observation_age_s=0.)
        if s.bridge_gate == 'timesync_unavailable_or_unstable':
            relaxed = replace(relaxed,bridge_ready=True)
        reason = self.ready(relaxed, now)
        if reason != 'ready':
            self.abort(reason, s, now)
            return self.actions
        if s.battery < .15:
            self.abort('critical_battery', s, now)
            return self.actions
        if self.phase in ('PREPARING', 'ARMING', 'TAKING_OFF'):
            if not self.external_ready(s, now):
                self.abort('takeoff_observations_unavailable', s, now)
            elif self.phase == 'PREPARING' and self.pending is None and self.accepted_command and s.mode == 'AUTO.TAKEOFF':
                self.enter('ARMING', 'takeoff_mode_confirmed', now)
                self.issue('arm', now)
            elif self.phase == 'ARMING' and self.pending is None and self.accepted_command and s.armed:
                self.enter('TAKING_OFF', 'arm_confirmed_native_climb', now)
            elif self.phase == 'TAKING_OFF' and s.armed and s.landed == 2 and s.mode == 'AUTO.LOITER':
                self.enter('STABILIZING', 'takeoff_complete_waiting_stable_xy', now)
                self.stable_since = None
            elif now-self.entered_s > (self.settings.takeoff_timeout_s if self.phase == 'TAKING_OFF' else self.settings.command_timeout_s):
                self.abort('native_takeoff_state_timeout', s, now)
            return self.actions
        external = self.external_ready(s, now)
        if not external and self.phase not in ('RECOVERING', 'STABILIZING'):
            if self.pending is not None:
                return []
            if self.scan_window is not None:
                self.scan_requests.append(self.scan_request('CANCEL'))
            self.resume_phase, self.recovery_since = self.phase, now
            self.scan_window = self.marker = self.scan_result = None
            if not self.move(s.xy, s, now, ros_ns, s.yaw_deg):
                return self.actions
            self.enter('RECOVERING', 'uwb_gap_hold_without_republishing_old_xy', now)
            self.stable_since = None
            return self.actions
        if s.battery < .2 and not self.returning and self.phase in ('MOVING','DWELL','ALIGNING','SCANNING','EGRESS'):
            if self.pending is None:
                if self.scan_window is not None:
                    self.scan_requests.append(self.scan_request('CANCEL'))
                    self.scan_window = self.marker = self.scan_result = None
                self.begin_return(s, now, ros_ns, 'battery_reserve_return')
            return self.actions
        if self.phase == 'RECOVERING':
            if now-self.recovery_since > self.settings.observation_recovery_s:
                self.abort('observation_recovery_timeout', s, now)
            elif external and self.pending is None and self.accepted_command and math.hypot(*s.velocity_xy) <= .1:
                if self.stable_since is None:
                    self.stable_since = now
                if now-self.stable_since >= self.settings.observation_settle_s:
                    if self.returning:
                        self.move(self.return_path[self.return_index], s, now, ros_ns)
                    else:
                        p = self.waypoint()
                        if self.move(p['xy'], s, now, ros_ns, p['yaw_deg']) and self.resume_phase == 'EGRESS':
                            self.enter('EGRESS','resume_staging_return',now)
            else:
                self.stable_since = None
            return self.actions
        if self.phase == 'STABILIZING':
            if math.dist(s.xy, self.home) > .6:
                self.abort('takeoff_horizontal_drift', s, now)
            elif external and math.hypot(*s.velocity_xy) <= .1 and abs(s.vertical_speed_m_s) <= .1:
                if self.stable_since is None:
                    self.stable_since = now
                if now-self.stable_since >= self.settings.takeoff_settle_s:
                    p = self.waypoint()
                    self.move(p['xy'], s, now, ros_ns, p['yaw_deg'])
            else:
                self.stable_since = None
            if now-self.entered_s > self.settings.takeoff_timeout_s:
                self.abort('takeoff_stabilization_timeout', s, now)
            return self.actions
        if self.phase in ('MOVING','RETURNING','EGRESS'):
            if not s.armed or s.landed != 2 or s.mode != 'AUTO.LOITER':
                self.abort('airborne_hold_state_lost', s, now)
            elif self.arrived(s, now, ros_ns):
                if self.phase == 'EGRESS':
                    self.next_task(s,now,ros_ns)
                elif self.returning:
                    self.return_index += 1
                    if self.return_index == len(self.return_path):
                        self.home_verified = True
                        self.enter('LANDING', 'home_arrival_verified', now)
                        self.issue('land', now)
                    else:
                        self.move(self.return_path[self.return_index], s, now, ros_ns)
                elif self.waypoint()['type'] == 'scan':
                    self.enter('ALIGNING', 'awaiting_context_bound_aruco_pose', now)
                    self.scan_window = secrets.token_hex(12)
                    self.scan_deadline = now+self.settings.scan_timeout_s
                    self.scan_requests.append(self.scan_request('ALIGN'))
                else:
                    self.enter('DWELL', 'arrival_verified', now)
            elif now-self.entered_s > self.settings.leg_timeout_s:
                self.abort('waypoint_or_yaw_timeout', s, now)
            elif not self.target_applied and now-self.entered_s > self.settings.command_timeout_s:
                self.abort('target_application_unverified', s, now)
        elif self.phase == 'DWELL':
            if not self.arrived(s, now, ros_ns):
                self.stable_since = None
            else:
                if self.stable_since is None:
                    self.stable_since = now
                if now-self.stable_since >= self.waypoint()['dwell_s']:
                    self.next_task(s, now, ros_ns)
            if now-self.entered_s > self.settings.leg_timeout_s:
                self.abort('dwell_timeout', s, now)
        elif self.phase == 'ALIGNING':
            if self.scan_deadline is not None and now > self.scan_deadline:
                self.finish_scan(dict(task_id=self.waypoint()['id'],outcome='FAILED',reason='marker_timeout'),s,now,ros_ns)
                return self.actions
            marker_fresh = self.marker and 0 <= (ros_ns-self.marker['stamp_ns'])/1e9 <= .1
            if marker_fresh:
                xy, yaw = self.marker['xy_m'], self.marker['yaw_deg']
                aligned = (math.dist(s.xy, xy) <= .03 and math.hypot(*s.velocity_xy) <= .03
                           and abs(math.remainder(s.yaw_deg-yaw,360)) <= 3)
                target_fresh = (self.target_last_stamp_ns is not None and
                                0 <= (ros_ns-self.target_last_stamp_ns)/1e9 <= .5)
                if aligned and self.target_applied and target_fresh and self.pending is None and self.accepted_command:
                    if self.stable_since is None:
                        self.stable_since = now
                    if now-self.stable_since >= 1.:
                        self.enter('SCANNING', 'aruco_alignment_stable', now)
                        self.scan_requests.append(self.scan_request('SCAN'))
                else:
                    self.stable_since = None
                    if self.pending is None and (self.target_yaw_deg != yaw or math.dist(self.monitor.target,xy) > .01):
                        # A bounded XY/yaw proposal. PX4 still controls attitude.
                        if self.move(xy, s, now, ros_ns, yaw, self.settings.scan_speed_m_s):
                            self.enter('ALIGNING', 'bounded_marker_adjustment', now)
            else:
                self.stable_since = None
        elif self.phase == 'SCANNING':
            if self.scan_result is None and self.scan_deadline is not None and now > self.scan_deadline:
                self.finish_scan(dict(task_id=self.waypoint()['id'],outcome='FAILED',reason='scanner_timeout'),s,now,ros_ns)
                return self.actions
            marker_fresh = self.marker and 0 <= (ros_ns-self.marker['stamp_ns'])/1e9 <= .1
            if (not marker_fresh or math.dist(s.xy, self.marker['xy_m']) > .03
                    or math.hypot(*s.velocity_xy) > .03
                    or abs(math.remainder(s.yaw_deg-self.marker['yaw_deg'],360)) > 3):
                self.scan_requests.append(self.scan_request('CANCEL'))
                self.scan_window = secrets.token_hex(12)
                self.scan_result = self.marker = None
                self.enter('ALIGNING','alignment_lost_new_scan_window',now)
                self.scan_requests.append(self.scan_request('ALIGN'))
            elif self.scan_result:
                self.finish_scan(self.scan_result,s,now,ros_ns)
            elif self.scan_deadline is not None and now > self.scan_deadline:
                self.finish_scan(dict(task_id=self.waypoint()['id'],outcome='FAILED',reason='scanner_timeout'),s,now,ros_ns)
        return self.actions

    def scan_request(self, action):
        return dict(schema=1, action=action, execution_id=self.assignment['request_id'],
                    task_id=self.waypoint()['id'], window_id=self.scan_window,
                    task=self.waypoint(), frame_id='uwb_map')

    def status(self):
        value = super().status()
        tasks = self.assignment['waypoints'] if self.assignment else []
        total = sum(p['type']=='scan' for p in tasks)
        succeeded = sum(r['outcome']=='SUCCEEDED' for r in self.scan_results)
        flight = (self.phase == 'END' and self.home_verified and self.landing_verified)
        value.update(mission_complete=flight and self.completed_waypoints == len(self.assignment['waypoints'])
                     and len(self.scan_results) == total and succeeded == total,
                     home_verified=self.home_verified, home_xy_m=list(self.home) if self.home is not None else None,
                     scan_window=self.scan_window,
                     scan_results=self.scan_results, flight_outcome='SUCCEEDED' if flight else
                     'FAILED' if self.phase in ('FAILED','UNCONFIRMED') else
                     'PILOT_OVERRIDE' if self.phase == 'PILOT_OVERRIDE' else 'RUNNING',
                     work_outcome='ALL_SUCCEEDED' if len(self.scan_results)==total and succeeded==total
                     and self.assignment and self.completed_waypoints==len(self.assignment['waypoints']) else 'INCOMPLETE')
        value.update(scan_failure_policy='continue_remaining_tasks',
                     attempted_task_ids=[p['id'] for p in tasks[:self.index]],
                     remaining_task_ids=[p['id'] for p in tasks[self.index:]],
                     failed_scan_task_ids=[r['task_id'] for r in self.scan_results if r['outcome']=='FAILED'],
                     route_complete=flight and self.completed_waypoints==len(tasks))
        return value
