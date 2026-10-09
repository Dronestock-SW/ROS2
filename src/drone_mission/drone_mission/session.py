"""Flight-trial FSM. Actions are PX4 requests, never a companion controller."""

from dataclasses import dataclass
import hashlib
import json
import math

from drone_demo.mission import MissionConfig, MissionMonitor
from drone_uwb.integration.sitl.sitl_target_contract import ned_to_global
from .contracts import finite, parse_request


def native_estimator_valid(msg):
    # Pinned PX4 v1.17 sets CONST_POS_MODE for at_rest OR fake_pos.
    # PRED_POS_HORIZ_REL proves active horizontal aiding, excluding fake-only.
    return bool(msg.attitude_status_flag and msg.velocity_horiz_status_flag
                and msg.pos_horiz_rel_status_flag
                and (not msg.const_pos_mode_status_flag or msg.pred_pos_horiz_rel_status_flag)
                and not msg.gps_glitch_status_flag and not msg.accel_error_status_flag)


@dataclass
class Snapshot:
    yaw_deg: object = None
    battery: object = None
    battery_age_s: float = math.inf
    takeoff_action: object = None
    mag_type: object = None
    rc_required: bool = True
    rc_valid: bool = False
    rc_age_s: float = math.inf
    rc_override: object = None
    rc_mode: object = None
    rc_channels: tuple = ()
    rc_switch_channels: tuple = ()
    rc_mapping_valid: bool = False
    command_services_ready: bool = False
    land_service_ready: bool = False
    connected: bool = False
    armed: bool = False
    mode: str = ''
    landed: int = 0
    state_age_s: float = math.inf
    landed_age_s: float = math.inf
    estimator_age_s: float = math.inf
    estimator_valid: bool = False
    pose_stamp_ns: int = 0
    pose_age_s: float = math.inf
    xy: tuple = ()
    velocity_xy: tuple = ()
    vertical_speed_m_s: object = None
    uwb_stamp_ns: int = 0
    uwb_age_s: float = math.inf
    uwb_xy: tuple = ()
    height_age_s: float = math.inf
    bridge_age_s: float = math.inf
    bridge_observation_age_s: float = math.inf
    bridge_ready: bool = False
    bridge_gate: str = ''
    bridge_published: int = 0
    alignment_matches: bool = False
    origin: object = None
    takeoff_param_age_s: float = math.inf
    takeoff_alt_m: object = None


def route_fingerprint(payload):
    keys = ('mission_db_id', 'mission_code', 'route_revision', 'route_tasks',
            'coordinate_frame', 'origin', 'x_axis', 'y_axis', 'z_axis', 'unit',
            'anchor_layout_id', 'ceiling_height_m', 'planned_launch_xy_m')
    body = json.dumps({k: payload.get(k) for k in keys}, sort_keys=True,
                      allow_nan=False, separators=(',', ':'))
    return hashlib.sha256(body.encode()).hexdigest()


class FlightSession:
    TARGET_PHASES = {'MOVING', 'RETURNING'}
    ACTIVE = {'ARMING', 'TAKING_OFF', 'MOVING', 'RETURNING', 'LANDING', 'ABORTING'}
    AUTO_MODES = {'AUTO.TAKEOFF', 'AUTO.LOITER', 'AUTO.LAND'}

    def __init__(self, settings, consumed=(), remember=lambda request_id: None):
        self.settings = settings
        self.consumed = set(consumed)
        self.remember = remember
        self.monitor = MissionMonitor(MissionConfig(
            'uwb_map', .5, .6, .15, .2, .1, 1., settings.pose_timeout_s, .25, .5, 3),
            position_source='px4_ekf2')
        self.phase, self.reason = 'IDLE', 'waiting_for_web_start'
        self.validation = {'accepted': False, 'reason': 'assignment_missing'}
        self.assignment = self.intent = None
        self.fingerprint = None
        self.last_web_s = None
        self.index = 0
        self.home = self.origin = None
        self.pending = None
        self.counter = 0
        self.actions = []
        self.entered_s = 0.
        self.accepted_command = False
        self.command_confirmed_s = None
        self.target_applied = False
        self.target_global = None
        self.target_sent_ros_ns = None
        self.target_last_stamp_ns = None
        self.failure = None
        self.ack = None
        self.returning = False
        self.landing_verified = False
        self.arrival = None
        self.targets_sent = self.completed_waypoints = 0

    def submit(self, payload, now, wall_s):
        try:
            fingerprint = route_fingerprint(payload)
            # The same accepted route may be polled throughout a flight. Its
            # old start timestamp must not become a new launch or expire it.
            if (self.assignment and fingerprint == self.fingerprint
                    and payload.get('ok') is True
                    and payload.get('contract_version') == '1.0'
                    and str(payload.get('drone_id')) == self.settings.drone_id
                    and payload.get('status') == 'ACTIVE'
                    and (payload.get('control_action') is None
                         or (payload.get('control_action') in ('start', 'land', 'return_to_home')
                             and payload.get('control_request_id') in self.consumed))):
                self.last_web_s = now
                self.validation = {'accepted': True, 'reason': 'same_active_route'}
                return True
            request = parse_request(payload, self.settings, wall_s)
            if request is None:
                return False
            if request['request_id'] in self.consumed:
                self.validation = {'accepted': False, 'reason': 'request_already_consumed'}
                return False
            if self.phase in self.ACTIVE and request['action'] == 'start':
                raise ValueError('active_route_replacement_rejected')
            self.last_web_s = now
            self.intent = request
            if request['action'] == 'start':
                self.fingerprint = fingerprint
                self.phase, self.reason = 'READY', 'waiting_for_flight_gates'
            self.validation = {'accepted': True, 'reason': 'request_valid',
                               'request_id': request['request_id']}
            return True
        except (TypeError, ValueError, AttributeError) as exc:
            self.validation = {'accepted': False, 'reason': str(exc)}
            return False

    def consume(self, request):
        self.remember(request['request_id'])  # Persist before handing off an action.
        self.consumed.add(request['request_id'])
        self.ack = {k: request[k] for k in ('action', 'request_id', 'requested_at')}

    def enter(self, phase, reason, now):
        self.phase, self.reason, self.entered_s = phase, reason, now

    def issue(self, kind, now, **data):
        self.counter += 1
        self.pending = dict(token=self.counter, kind=kind, sent_s=now)
        self.accepted_command = False
        self.command_confirmed_s = None
        self.actions.append(dict(self.pending, **data))

    def ready(self, s, now, *, ground=False):
        if not s.command_services_ready:
            return 'mavros_command_services_unavailable'
        for name in ('layout_confirmed', 'alignment_confirmed', 'fusion_confirmed', 'takeoff_settings_confirmed',
                     'timing_confirmed', 'sensor_mount_confirmed'):
            if not getattr(self.settings, name):
                return name + '_required'
        if not s.connected or not 0 <= s.state_age_s <= self.settings.state_timeout_s:
            return 'fc_disconnected_or_stale'
        if not 0 <= s.landed_age_s <= self.settings.state_timeout_s:
            return 'landed_state_stale'
        if not s.estimator_valid or not 0 <= s.estimator_age_s <= .5:
            return 'px4_estimator_unavailable'
        if not s.xy or not self.settings.inside(s.xy) or not 0 <= s.pose_age_s <= self.settings.pose_timeout_s:
            return 'px4_pose_unavailable_or_outside_bounds'
        if not 0 <= s.uwb_age_s <= .25:
            return 'uwb_unavailable'
        if not 0 <= s.height_age_s <= .2:
            return 'measured_height_unavailable'
        if (not s.bridge_ready or s.bridge_published <= 0 or not 0 <= s.bridge_age_s <= 2.5
                or not 0 <= s.bridge_observation_age_s <= .2):
            return 'uwb_px4_bridge_unavailable'
        if not s.alignment_matches:
            return 'observation_and_command_alignment_mismatch'
        if s.origin is None:
            return 'px4_global_origin_required'
        if self.origin is not None and s.origin != self.origin:
            return 'px4_global_origin_changed'
        if self.last_web_s is None or not 0 <= now-self.last_web_s <= self.settings.web_timeout_s:
            return 'web_assignment_stale'
        if ground:
            if s.armed or s.landed != 1:
                return 'disarmed_ground_state_required'
            if s.mode not in ('POSCTL', 'AUTO.LOITER'):
                return 'px4_position_mode_required'
            if (not finite(s.takeoff_alt_m) or not 0 <= s.takeoff_param_age_s <= 5.
                    or abs(s.takeoff_alt_m-self.settings.expected_mis_takeoff_alt_m) > .01):
                return 'px4_mis_takeoff_alt_mismatch_or_unavailable'
        return 'ready'

    def command_result(self, token, accepted, now):
        if self.pending is None or token != self.pending['token']:
            return
        kind = self.pending['kind']
        self.pending = None
        self.accepted_command = accepted is True
        self.command_confirmed_s = now if self.accepted_command else None
        if not self.accepted_command:
            self.failure = kind + '_rejected_or_transport_failed'

    def applied_target(self, latitude, longitude, stamp_ns):
        if (self.phase not in self.TARGET_PHASES or self.target_global is None
                or not finite(latitude, longitude)
                or type(stamp_ns) is not int or stamp_ns < self.target_sent_ros_ns
                or (self.target_last_stamp_ns is not None and stamp_ns <= self.target_last_stamp_ns)):
            return False
        # Integer command -> double navigator -> integer telemetry can truncate
        # by one 1e-7 degree unit (~1 cm). Larger target changes remain failures.
        match = (abs(latitude-self.target_global[0]) <= 1.5e-7
                 and abs(longitude-self.target_global[1]) <= 1.5e-7)
        if match:
            self.target_applied = True
            self.target_last_stamp_ns = stamp_ns
        elif self.target_applied:
            self.target_applied = False
            self.failure = 'px4_target_changed'
        return match

    def move(self, target, s, now, ros_ns):
        if not self.settings.inside(target) or math.dist(target, s.xy) > self.settings.max_leg_m:
            self.abort('target_exceeds_trial_limit', s, now)
            return
        angle = math.radians(self.settings.enu_yaw_deg)
        enu_x = math.cos(angle)*target[0]-math.sin(angle)*target[1]+self.settings.enu_offset_x_m
        enu_y = math.sin(angle)*target[0]+math.cos(angle)*target[1]+self.settings.enu_offset_y_m
        latitude, longitude = ned_to_global(enu_y, enu_x, s.origin)
        x, y = round(latitude*1e7), round(longitude*1e7)
        self.target_global = (x/1e7, y/1e7)
        self.target_applied = False
        self.target_last_stamp_ns = None
        self.target_sent_ros_ns = ros_ns
        self.arrival = None
        self.monitor.reset_health()
        self.monitor.set_target(*target, 'uwb_map')
        self.enter('RETURNING' if self.returning else 'MOVING', 'px4_target_handoff', now)
        self.targets_sent += 1
        self.issue('reposition', now, x=x, y=y, speed=self.settings.speed_m_s)

    def abort(self, reason, s, now):
        self.failure = reason
        self.pending = None
        if (s.landed == 1 and 0 <= s.landed_age_s <= self.settings.state_timeout_s
                and self.phase == 'ARMING'):
            self.enter('ABORTING', reason, now)
            self.issue('disarm', now)
        else:
            self.enter('LANDING', reason, now)
            self.issue('land', now)

    def tick(self, s, now, wall_s, ros_ns):
        self.actions = []
        if (self.settings.execute and self.phase in ('IDLE', 'READY')
                and s.connected and s.armed and s.landed == 2
                and 0 <= s.state_age_s <= self.settings.state_timeout_s
                and 0 <= s.landed_age_s <= self.settings.state_timeout_s
                and s.mode in self.AUTO_MODES):
            # A restarted executor must not leave an autonomous flight unowned.
            self.intent = None
            if s.land_service_ready:
                self.abort('airborne_without_active_session', s, now)
            else:
                self.reason = 'recovery_waiting_for_land_service'
            return self.actions
        if self.phase in self.ACTIVE:
            # RC/manual mode takes priority. Late service responses cannot resume.
            if (s.connected and 0 <= s.state_age_s <= self.settings.state_timeout_s
                    and self.phase != 'ABORTING'
                    and not (self.phase == 'ARMING' and s.mode == 'POSCTL' and s.landed == 1)
                    and not (self.phase == 'TAKING_OFF' and s.mode == 'POSCTL'
                             and s.landed == 1
                             and now-self.entered_s <= self.settings.command_timeout_s)
                    and s.mode not in self.AUTO_MODES
                    and not (s.landed == 1 and not s.armed)):
                self.pending = None
                self.intent = None
                self.enter('PILOT_OVERRIDE', 'manual_mode_took_priority', now)
                return self.actions
        if self.intent is not None:
            request = self.intent
            if not 0 <= wall_s-request['requested_s'] <= self.settings.request_ttl_s:
                self.intent = None
                self.validation = {'accepted': False, 'reason': 'control_request_expired'}
            elif request['action'] == 'start' and self.phase not in self.ACTIVE:
                self.reason = self.ready(s, now, ground=True)
                if not self.settings.execute:
                    self.reason = 'execution_disabled'
                elif self.reason == 'ready':
                    points = [s.xy]+[p['xy'] for p in request['waypoints']]
                    if any(math.dist(a, b) > self.settings.max_leg_m
                           for a, b in zip(points, points[1:])):
                        self.intent = None
                        self.validation = {'accepted': False, 'reason': 'route_leg_exceeds_trial_limit'}
                        return self.actions
                    self.consume(request)
                    self.assignment, self.intent = request, None
                    self.home, self.origin, self.index = tuple(s.xy), s.origin, 0
                    self.failure = None
                    self.returning = False
                    self.landing_verified = False
                    self.completed_waypoints = 0
                    self.enter('ARMING', 'web_start_accepted', now)
                    self.issue('arm', now)
                    return self.actions
            elif request['action'] in ('land', 'return_to_home'):
                if (self.settings.execute and self.phase == 'ARMING'
                        and s.connected and 0 <= s.state_age_s <= self.settings.state_timeout_s
                        and s.landed == 1 and 0 <= s.landed_age_s <= self.settings.state_timeout_s):
                    self.consume(request)
                    self.intent = None
                    self.abort('web_start_cancelled', s, now)
                    return self.actions
                if (self.settings.execute and s.connected
                        and 0 <= s.state_age_s <= self.settings.state_timeout_s
                        and s.mode in self.AUTO_MODES):
                    if self.phase in ('LANDING', 'ABORTING'):
                        if request['action'] == 'land':
                            self.consume(request)
                        else:
                            self.validation = {'accepted': False, 'reason': 'landing_already_in_progress'}
                        self.intent = None
                        return self.actions
                    self.consume(request)
                    self.intent = self.pending = None
                    if (request['action'] == 'return_to_home' and self.home is not None
                            and self.ready(s, now) == 'ready' and s.landed == 2
                            and s.mode == 'AUTO.LOITER'):
                        self.returning = True
                        self.move(self.home, s, now, ros_ns)
                    else:
                        self.enter('LANDING', 'web_land_accepted', now)
                        self.issue('land', now)
                    return self.actions
        if self.phase not in self.ACTIVE:
            return self.actions
        if self.phase in ('LANDING', 'ABORTING'):
            ground_verified = (s.connected and not s.armed and s.landed == 1
                    and 0 <= s.state_age_s <= self.settings.state_timeout_s
                    and 0 <= s.landed_age_s <= self.settings.state_timeout_s)
            if self.phase == 'ABORTING':
                # An old disarmed heartbeat cannot confirm a cancelled arm.
                ground_verified = (ground_verified and self.pending is None
                    and self.accepted_command and self.command_confirmed_s is not None
                    and now-s.state_age_s >= self.command_confirmed_s
                    and now-s.landed_age_s >= self.command_confirmed_s)
            if ground_verified:
                self.pending = None
                self.landing_verified = True
                self.enter('FAILED' if self.failure else 'LANDED',
                           self.failure or 'px4_landed_and_disarmed', now)
            elif now-self.entered_s > self.settings.landing_timeout_s:
                self.pending = None
                self.enter('FAILED', 'landing_completion_unverified', now)
            return self.actions
        if self.failure:
            self.abort(self.failure, s, now)
            return self.actions
        reason = self.ready(s, now)
        if reason != 'ready':
            self.abort(reason, s, now)
            return self.actions
        if self.pending and now-self.pending['sent_s'] > self.settings.command_timeout_s:
            self.abort('command_response_timeout', s, now)
            return self.actions
        if self.phase == 'ARMING':
            if self.accepted_command and s.armed:
                self.enter('TAKING_OFF', 'px4_arm_confirmed', now)
                self.issue('takeoff', now)
            elif now-self.entered_s > self.settings.command_timeout_s:
                self.abort('arming_state_timeout', s, now)
        elif self.phase == 'TAKING_OFF':
            if self.accepted_command and s.armed and s.landed == 2 and s.mode == 'AUTO.LOITER':
                self.move(self.assignment['waypoints'][0]['xy'], s, now, ros_ns)
            elif now-self.entered_s > self.settings.takeoff_timeout_s:
                self.abort('px4_takeoff_completion_timeout', s, now)
        elif self.phase in ('MOVING', 'RETURNING'):
            if not s.armed or s.landed != 2 or s.mode != 'AUTO.LOITER':
                self.abort('px4_airborne_hold_state_lost', s, now)
                return self.actions
            if (self.target_applied and self.target_last_stamp_ns is not None
                    and not 0 <= (ros_ns-self.target_last_stamp_ns)/1e9 <= .5):
                self.target_applied = False
                self.abort('px4_target_feedback_stale', s, now)
                return self.actions
            self.monitor.update('pose', *s.xy, s.pose_stamp_ns, now,
                                s.pose_age_s, 'uwb_map', velocity_xy=s.velocity_xy)
            self.monitor.update('uwb', *s.uwb_xy, s.uwb_stamp_ns, now,
                                s.uwb_age_s, 'uwb_map')
            self.arrival = self.monitor.evaluate(now)
            if (not self.target_applied and now-self.entered_s > self.settings.command_timeout_s):
                self.abort('px4_target_application_unverified', s, now)
            elif now-self.entered_s > self.settings.leg_timeout_s:
                self.abort('waypoint_timeout', s, now)
            elif self.accepted_command and self.target_applied and self.arrival['arrival_valid']:
                self.completed_waypoints += int(not self.returning)
                self.index += 1
                if self.returning or self.index == len(self.assignment['waypoints']):
                    self.enter('LANDING', 'route_completed', now)
                    self.issue('land', now)
                else:
                    self.move(self.assignment['waypoints'][self.index]['xy'], s, now, ros_ns)
        return self.actions

    def status(self):
        a = self.assignment
        waypoint = (a['waypoints'][self.index] if a and self.index < len(a['waypoints']) else None)
        return dict(state=self.phase, reason=self.reason,
                    failure=self.failure,
                    flight_control_enabled=self.settings.execute and self.phase in self.ACTIVE,
                    mission_db_id=a['mission_db_id'] if a else None,
                    mission_code=a['mission_code'] if a else None,
                    route_revision=a['route_revision'] if a else None,
                    active_waypoint_index=self.index if waypoint else None,
                    active_waypoint_id=waypoint['id'] if waypoint else None,
                    target_applied=self.target_applied, completed_waypoints=self.completed_waypoints,
                    targets_sent=self.targets_sent, arrival=self.arrival, validation=self.validation,
                    control_ack=self.ack, landing_verified=self.landing_verified,
                    mission_complete=(self.phase == 'LANDED' and a is not None
                                      and self.completed_waypoints == len(a['waypoints'])),
                    position_source='px4_ekf2', altitude_control='px4',
                    takeoff_altitude_policy='px4_MIS_TAKEOFF_ALT', fusion_verified=False)
