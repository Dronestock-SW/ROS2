"""MAVROS adapter for web-triggered PX4 flight trials."""

from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import sys
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, GlobalPositionTarget, State, RCIn
from mavros_msgs.srv import CommandBool, CommandInt, CommandLong, CommandTOL, SetMode
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

from drone_uwb.processing.gazebo_geometry import rotation_world_body
from drone_uwb.integration.sitl.sitl_target_contract import PX4GlobalReference
from drone_uwb.integration.planar import enu_xy_to_map, enu_vector_to_map, enu_yaw_to_map
from .contracts import Settings, finite
from .session import FlightSession, Snapshot, native_estimator_valid
from .mission_chain import MissionChain
from .writer_lock import WriterLock
from .native_ai import NativePlanner
from .preflight import field_preflight


def stamp_ns(msg):
    return msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec


class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_mission')
        descriptor = ParameterDescriptor(read_only=True)
        default = str(Path(get_package_share_directory('drone_mission'))/'config/flight.json')
        self.declare_parameter('config_file', default, descriptor)
        self.declare_parameter('execute', False, descriptor)
        self.declare_parameter('test_mode', False, descriptor)
        self.declare_parameter('record_directory', '', descriptor)
        self.declare_parameter('native_plan_binary', '', descriptor)
        self.declare_parameter('native_map_file', '', descriptor)
        self.declare_parameter('native_map_sha256', '', descriptor)
        self.declare_parameter('ledger_file', str(Path.home()/'.local/state/dronestock-flight/requests.json'), descriptor)
        values = json.loads(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))
        self.settings = replace(Settings(**values), execute=self.get_parameter('execute').value)
        test_mode = self.get_parameter('test_mode').value
        self.rc_required = not test_mode
        self.native_planner = None
        if self.settings.full_mission:
            binary = self.get_parameter('native_plan_binary').value
            map_file = self.get_parameter('native_map_file').value
            sha = self.get_parameter('native_map_sha256').value
            if binary and map_file and sha:
                self.native_planner = NativePlanner(binary, map_file, sha, allow_virtual=test_mode)
        domain = self.context.get_domain_id()
        if not (
                (test_mode and domain == 99 and os.environ.get('ROS_LOCALHOST_ONLY') == '1')
                or (not test_mode and domain == (1 if self.settings.drone_id == '5' else 2))):
            raise ValueError('tag_domain_mismatch_or_invalid_local_test_mode')
        self.ledger = Path(self.get_parameter('ledger_file').value)
        consumed = []
        if self.ledger.exists():
            consumed = json.loads(self.ledger.read_text(encoding='utf-8'))['consumed']
            if not isinstance(consumed, list) or any(not isinstance(v, str) for v in consumed):
                raise ValueError('invalid_request_ledger')
        cls = MissionChain if self.settings.full_mission else FlightSession
        self.session = cls(self.settings, consumed, self.remember)
        self.samples = {}
        self.param_value = None
        self.param_received = float('-inf')
        self.param_future = None
        self.param_requested = float('-inf')
        self.streams_requested = False
        self.origin_requested = float('-inf')
        self.origin = None
        self.log = None
        self.record_fault = False
        directory = self.get_parameter('record_directory').value
        if self.settings.full_mission and self.settings.execute and not directory:
            raise ValueError('full_mission_execution_requires_record_directory')
        if directory:
            root = Path(directory)
            root.mkdir(parents=True, exist_ok=False)
            (root/'settings.json').write_text(json.dumps(asdict(self.settings), indent=2)+'\n', encoding='utf-8')
            if self.native_planner is not None:
                (root/'native-map.json').write_bytes(self.native_planner.raw_map)
            self.log = (root/'events.jsonl').open('x', encoding='utf-8')
        self.status_pub = self.create_publisher(String, '/flight_state', 10)
        self.valid_pub = self.create_publisher(String, '/target_valid', 10)
        self.result_pub = self.create_publisher(String, '/mission_result', 10)
        self.scan_pub = self.create_publisher(String, '/mission/scan_request', 10)
        if self.settings.full_mission:
            self.create_subscription(RCIn, '/mavros/rc/in', lambda msg: self.receive('rc', msg), qos_profile_sensor_data)
            self.create_subscription(String, '/mission/marker_observation', self.marker_observation, 10)
            self.create_subscription(String, '/mission/scan_result', self.scan_result, 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/mission/assignment', self.assignment, 10)
        for name, cls, topic in (
                ('state', State, '/mavros/state'),
                ('landed', ExtendedState, '/mavros/extended_state'),
                ('estimator', EstimatorStatus, '/mavros/estimator_status'),
                ('battery', BatteryState, '/mavros/battery'),
                ('pose', Odometry, '/mavros/local_position/odom'),
                ('uwb', PoseWithCovarianceStamped, '/uwb/btf_pose'),
                ('height', PoseStamped, '/uwb/btf_xyz')):
            self.create_subscription(cls, topic, lambda msg, key=name: self.receive(key, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(String, '/uwb/bridge_status', self.bridge, 10)
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.receive_origin,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(GlobalPositionTarget, '/mavros/setpoint_raw/target_global',
                                 self.target_received, qos_profile_sensor_data)
        self.command_clients = {name: self.create_client(cls, topic) for name, cls, topic in (
            ('arm', CommandBool, '/mavros/cmd/arming'),
            ('takeoff', CommandTOL, '/mavros/cmd/takeoff'),
            ('land', CommandTOL, '/mavros/cmd/land'),
            ('reposition', CommandInt, '/mavros/cmd/command_int'))}
        self.command_clients['takeoff_mode'] = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.stream_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.previous_state = None
        self.target_feedback = None
        self.scan_renewed_s = float('-inf')
        self.writer_lock = WriterLock(domain) if self.settings.execute else None
        self.create_timer(.05, self.tick)
        self.create_timer(1., self.query)

    def remember(self, request_id):
        self.ledger.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.ledger.with_suffix('.tmp')
        consumed = sorted(self.session.consumed | {request_id})
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump({'consumed': consumed}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.ledger)
        descriptor = os.open(self.ledger.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def record(self, event):
        if self.log:
            try:
                self.log.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
                self.log.flush()
            except OSError:
                self.record_fault = True
                self.session.failure = 'event_log_write_failed'
                try:
                    self.log.close()
                except OSError:
                    pass
                self.log = None

    def receive(self, key, msg):
        if self.settings.full_mission:
            stamp = stamp_ns(msg)
            if (stamp <= 0 or not 0 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= 3.
                    or key in self.samples and stamp <= stamp_ns(self.samples[key][0])):
                return
        self.samples[key] = (msg, time.monotonic())

    def marker_observation(self, msg):
        try:
            if len(msg.data) > 65536:
                return
            self.session.observe_marker(json.loads(msg.data), self.get_clock().now().nanoseconds)
        except (TypeError, ValueError):
            pass

    def scan_result(self, msg):
        try:
            if len(msg.data) > 65536:
                return
            value = json.loads(msg.data)
            if self.session.observe_scan_result(value, self.get_clock().now().nanoseconds):
                self.record(dict(type='scan_result_accepted', value=value))
                if self.log:
                    os.fsync(self.log.fileno())
        except (TypeError, ValueError):
            pass
        except OSError:
            self.session.failure = 'scan_result_persistence_failed'

    def age(self, key):
        if key not in self.samples:
            return math.inf
        msg, received = self.samples[key]
        wall_age = (self.get_clock().now().nanoseconds-stamp_ns(msg))/1e9
        if wall_age < 0:
            return math.inf
        return max(time.monotonic()-received, wall_age)

    def assignment(self, msg):
        try:
            if len(msg.data) > 65536:
                raise ValueError('assignment_too_large')
            payload = json.loads(msg.data)
            if self.settings.full_mission and payload.get('route_tasks'):
                if self.native_planner is None:
                    if payload.get('control_action') == 'start':
                        raise ValueError('native_validated_map_required')
                elif not self.session.assignment and not self.session.intent and payload.get('control_action') == 'start':
                    sample = self.snapshot()
                    if sample.armed or sample.landed != 1 or sample.pose_age_s > self.settings.pose_timeout_s:
                        raise ValueError('native_plan_requires_fresh_ground_pose')
                    payload, proof = self.native_planner.compile(payload, sample.xy, self.settings)
                    self.record(dict(type='native_ai_plan_validated', **proof))
                    if self.log:
                        os.fsync(self.log.fileno())
                elif self.native_planner.key(payload) in self.native_planner.cache:
                    payload, proof = self.native_planner.reuse(payload)
                    if not self.session.assignment:
                        sample = self.snapshot()
                        if not sample.xy or math.dist(sample.xy, proof['launch_xy_m']) > .1:
                            raise ValueError('native_plan_launch_position_changed')
            self.session.submit(payload, time.monotonic(), time.time())
        except (ValueError, TypeError, KeyError, OSError, subprocess.TimeoutExpired) as error:
            self.session.validation = {'accepted': False, 'reason': str(error)[:256]}
        self.valid_pub.publish(String(data=json.dumps(self.session.validation)))

    def bridge(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or not isinstance(value.get('settings'), dict):
                raise ValueError('invalid_bridge_status')
            self.samples['bridge'] = (value, time.monotonic())
        except (ValueError, TypeError):
            self.samples.pop('bridge', None)

    def receive_origin(self, msg):
        try:
            reference = PX4GlobalReference(msg.position.latitude, msg.position.longitude,
                                           stamp_ns(msg)//1000)
            if self.origin is not None and (reference.latitude_deg, reference.longitude_deg) == (
                    self.origin.latitude_deg, self.origin.longitude_deg):
                return
            self.origin = reference
        except (TypeError, ValueError):
            self.origin = None

    def target_received(self, msg):
        age = (self.get_clock().now().nanoseconds-stamp_ns(msg))/1e9
        if finite(msg.latitude, msg.longitude):
            self.target_feedback = dict(coordinate_frame=msg.coordinate_frame,type_mask=msg.type_mask,
                                        latitude=msg.latitude,longitude=msg.longitude,age_s=age)
        if msg.coordinate_frame in (0, 5) and not msg.type_mask & 3 and 0 <= age <= .5:
            self.session.applied_target(msg.latitude, msg.longitude, stamp_ns(msg))

    def snapshot(self):
        s = Snapshot(origin=self.origin, command_services_ready=all(
            client.service_is_ready() for key, client in self.command_clients.items()
            if self.settings.full_mission or key != 'takeoff_mode'),
            land_service_ready=self.command_clients['land'].service_is_ready())
        if 'state' in self.samples:
            msg = self.samples['state'][0]
            s.connected, s.armed, s.mode = msg.connected, msg.armed, msg.mode
            s.state_age_s = self.age('state')
        if 'landed' in self.samples:
            s.landed = self.samples['landed'][0].landed_state
            s.landed_age_s = self.age('landed')
        if 'estimator' in self.samples:
            msg = self.samples['estimator'][0]
            s.estimator_valid = native_estimator_valid(msg)
            s.estimator_age_s = self.age('estimator')
        if 'pose' in self.samples:
            msg = self.samples['pose'][0]
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            try:
                if msg.header.frame_id != 'map' or not finite(p.x, p.y, p.z, v.x, v.y, v.z):
                    raise ValueError('invalid_fc_pose')
                if msg.child_frame_id == 'base_link':
                    velocity = rotation_world_body((q.w, q.x, q.y, q.z)) @ [v.x, v.y, v.z]
                elif msg.child_frame_id == 'map':
                    velocity = (v.x, v.y, v.z)
                else:
                    raise ValueError('invalid_fc_velocity_frame')
                s.xy = tuple(enu_xy_to_map((p.x, p.y), self.settings))
                s.velocity_xy = tuple(enu_vector_to_map(velocity[:2], self.settings))
                s.vertical_speed_m_s = float(velocity[2])
                s.pose_stamp_ns, s.pose_age_s = stamp_ns(msg), self.age('pose')
                s.yaw_deg = enu_yaw_to_map(math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),
                                        1-2*(q.y*q.y+q.z*q.z))), self.settings)
            except (ValueError, TypeError):
                pass
        if 'uwb' in self.samples:
            msg = self.samples['uwb'][0]
            p = msg.pose.pose.position
            if msg.header.frame_id == 'uwb_map' and finite(p.x, p.y):
                s.uwb_xy, s.uwb_stamp_ns = (p.x, p.y), stamp_ns(msg)
                s.uwb_age_s = self.age('uwb')
        if 'height' in self.samples:
            msg = self.samples['height'][0]
            p = msg.pose.position
            if msg.header.frame_id == 'uwb_map' and finite(p.x, p.y, p.z):
                s.height_age_s = self.age('height')
        if 'bridge' in self.samples:
            value, received = self.samples['bridge']
            cfg = value.get('settings', {})
            s.bridge_ready = value.get('gate') == 'ready'
            s.bridge_gate = value.get('gate','')
            count = value.get('published')
            s.bridge_published = count if type(count) is int and count >= 0 else 0
            s.bridge_age_s = time.monotonic()-received
            stamp = value.get('last_observation_stamp_ns')
            if type(stamp) is int and stamp > 0:
                s.bridge_observation_age_s = (self.get_clock().now().nanoseconds-stamp)/1e9
            s.alignment_matches = (cfg.get('input_source') == 'btf_xy'
                and cfg.get('tag_id') == self.settings.drone_id
                and cfg.get('layout_confirmed') is True
                and (not self.settings.execute or cfg.get('ground_only') is False)
                and cfg.get('source_frame') == 'uwb_map'
                and type(cfg.get('map_y_axis_sign', 1)) is int
                and cfg.get('map_y_axis_sign', 1) == self.settings.map_y_axis_sign
                and all(finite(cfg.get(k)) and abs(cfg[k]-getattr(self.settings, k)) < 1e-6
                        for k in ('enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m',
                                  'expected_ev_delay_ms'))
                and all(finite(cfg.get('antenna_body_frd_'+axis+'_m'))
                        and abs(cfg['antenna_body_frd_'+axis+'_m']
                                -getattr(self.settings, 'expected_ev_pos_'+axis+'_m')) < 1e-6
                        for axis in ('x', 'y', 'z')))
        s.takeoff_alt_m = self.param_value
        s.takeoff_action = getattr(self, 'takeoff_action', None)
        s.mag_type = getattr(self, 'mag_type', None)
        s.rc_override = getattr(self, 'rc_override', None)
        s.rc_mode = getattr(self, 'rc_mode', None)
        s.rc_required = getattr(self, 'rc_required', True)
        if 'rc' in self.samples:
            channels = self.samples['rc'][0].channels
            s.rc_valid = len(channels) >= 4 and all(800 <= v <= 2200 for v in channels[:4])
            s.rc_age_s = self.age('rc')
            s.rc_channels = tuple(channels)
            mappings = getattr(self, 'rc_mappings', ())
            s.rc_mapping_valid = (len(mappings)==3 and all(type(v) is int and 0 <= v <= len(channels) for v in mappings)
                                  and mappings[0]>0 and all(800 <= channels[v-1] <= 2200 for v in mappings if v))
            if s.rc_mapping_valid:
                s.rc_switch_channels = tuple(sorted(set(v for v in mappings if v)))
        if 'battery' in self.samples:
            b = self.samples['battery'][0]
            if b.present and finite(b.voltage, b.percentage) and b.voltage > 0 and 0 <= b.percentage <= 1:
                s.battery, s.battery_age_s = b.percentage, self.age('battery')
        s.takeoff_param_age_s = time.monotonic()-self.param_received
        return s

    def query(self):
        now = time.monotonic()
        if self.param_future is not None and now-self.param_requested > 2.:
            self.param_client.remove_pending_request(self.param_future)
            self.param_future = None
        if self.param_future is None and self.param_client.service_is_ready():
            self.param_requested = now
            names = ['MIS_TAKEOFF_ALT', 'COM_TAKEOFF_ACT']
            if self.settings.full_mission:
                names.extend(['EKF2_MAG_TYPE', 'COM_RC_OVERRIDE', 'COM_RC_IN_MODE',
                              'RC_MAP_FLTMODE', 'RC_MAP_ARM_SW', 'RC_MAP_KILL_SW'])
            self.param_future = self.param_client.call_async(GetParameters.Request(names=names))
            self.param_future.add_done_callback(self.parameters_received)
        if self.stream_client.service_is_ready():
            if not self.streams_requested:
                # Measured ToF projection requires attitude within 20 ms. The
                # default 50 Hz attitude / 10 Hz range streams sit on that
                # boundary and intermittently starve valid UWB observations.
                rates = [(230, 10.), (245, 5.), (87, 10.)]
                if self.settings.full_mission:
                    rates += [(31, 100.), (105, 100.), (132, 40.), (32, 30.), (65, 10.)]
                for message, rate in rates:
                    request = CommandLong.Request(command=511, param1=float(message), param2=1e6/rate)
                    self.stream_client.call_async(request)
                self.streams_requested = True
            if self.origin is None and now-self.origin_requested > 5.:
                self.stream_client.call_async(CommandLong.Request(command=512, param1=49.))
                self.origin_requested = now

    def parameters_received(self, future):
        if future is not self.param_future:
            return
        self.param_future = None
        try:
            value = future.result().values[0]
            self.param_value = value.double_value if value.type == 3 else value.integer_value if value.type == 2 else None
            self.param_received = time.monotonic()
            action = future.result().values[1]
            self.takeoff_action = action.integer_value if action.type == 2 else None
            if self.settings.full_mission:
                mag = future.result().values[2]
                self.mag_type = mag.integer_value if mag.type == 2 else None
                rc = future.result().values[3]
                self.rc_override = rc.integer_value if rc.type == 2 else None
                mode = future.result().values[4]
                self.rc_mode = mode.integer_value if mode.type == 2 else None
                self.rc_mappings = tuple(v.integer_value if v.type == 2 else None for v in future.result().values[5:8])
        except Exception:
            self.param_value = None

    def dispatch(self, action):
        kind, token = action['kind'], action['token']
        client = self.command_clients['arm' if kind == 'disarm' else kind]
        if not client.service_is_ready():
            self.session.command_result(token, False, time.monotonic())
            return
        if kind == 'takeoff_mode':
            request = SetMode.Request(base_mode=0, custom_mode='AUTO.TAKEOFF')
        elif kind in ('arm', 'disarm'):
            request = CommandBool.Request(value=kind == 'arm')
        elif kind in ('takeoff', 'land'):
            # PX4 selects MIS_TAKEOFF_ALT/current location. No companion Z target.
            request = CommandTOL.Request(yaw=math.nan, latitude=math.nan,
                                         longitude=math.nan, altitude=math.nan)
        else:
            request = CommandInt.Request(broadcast=False, frame=0, command=192,
                param1=float(action['speed']), param2=0., param3=math.nan, param4=action.get('yaw_rad', math.nan),
                x=action['x'], y=action['y'], z=math.nan)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'uwb_map'
            pose.pose.position.x, pose.pose.position.y = self.session.monitor.target
            pose.pose.orientation.w = 1.
            self.target_pub.publish(pose)
        self.record(dict(type='command_requested', **action))
        if self.record_fault and kind not in ('land','disarm'):
            self.session.command_result(token, False, time.monotonic())
            self.session.failure = 'event_log_write_failed'
            return
        future = client.call_async(request)

        def completed(result):
            try:
                response = result.result()
                accepted = (response.mode_sent if kind == 'takeoff_mode' else
                            response.success and getattr(response, 'result', 0) == 0)
            except Exception:
                accepted = False
            self.session.command_result(token, accepted, time.monotonic())
            self.record(dict(type='command_response', token=token, kind=kind,
                             accepted=bool(accepted), acceptance_is_handoff=kind == 'reposition'))

        future.add_done_callback(completed)

    def tick(self):
        s = self.snapshot()
        try:
            actions = self.session.tick(s, time.monotonic(), time.time(), self.get_clock().now().nanoseconds)
        except OSError:
            self.session.intent = None
            if self.session.phase in self.session.ACTIVE:
                self.session.abort('request_ledger_write_failed', s, time.monotonic())
                actions = self.session.actions
            else:
                self.session.pending = None
                self.session.enter('FAILED', 'request_ledger_write_failed', time.monotonic())
                actions = []
        for action in actions:
            self.dispatch(action)
        requests = getattr(self.session, 'scan_requests', [])
        if (self.settings.full_mission and not requests
                and self.session.phase in ('ALIGNING','SCANNING')
                and time.monotonic()-self.scan_renewed_s >= .2):
            requests = [self.session.scan_request('ALIGN' if self.session.phase == 'ALIGNING' else 'SCAN')]
        for request in requests:
            self.scan_pub.publish(String(data=json.dumps(request, allow_nan=False)))
            self.record(dict(type='scan_request', value=request))
            self.scan_renewed_s = time.monotonic()
        status = self.session.status()
        fresh_state = 0 <= s.state_age_s <= self.settings.state_timeout_s
        status.update(fc_connected=s.connected if fresh_state else None,
                      fc_landed=s.landed if 0 <= s.landed_age_s <= self.settings.state_timeout_s else None,
                      fc_armed=s.armed if fresh_state else None,
                      fc_mode=s.mode if fresh_state else None,
                      px4_map_xy_m=list(s.xy) if s.xy and s.pose_age_s <= self.settings.pose_timeout_s else None)
        if self.settings.full_mission:
            status['preflight'] = field_preflight(self.settings, s,
                map_loaded=self.native_planner is not None,
                recording_ok=self.log is not None and not self.record_fault)
            status['target_feedback'] = self.target_feedback
            status['target_requested_global'] = self.session.target_global
            status['input_checks'] = dict(estimator_valid=s.estimator_valid,
                pose_age_s=s.pose_age_s if math.isfinite(s.pose_age_s) else None,
                velocity_xy=s.velocity_xy, yaw_deg=s.yaw_deg,
                vertical_speed_m_s=s.vertical_speed_m_s,
                estimator_age_s=s.estimator_age_s if math.isfinite(s.estimator_age_s) else None,
                uwb_age_s=s.uwb_age_s if math.isfinite(s.uwb_age_s) else None,
                height_age_s=s.height_age_s if math.isfinite(s.height_age_s) else None,
                bridge_ready=s.bridge_ready, alignment_matches=s.alignment_matches,
                bridge_observation_age_s=s.bridge_observation_age_s if math.isfinite(s.bridge_observation_age_s) else None,
                bridge_gate=self.samples.get('bridge',({},0))[0].get('gate'),
                bridge_reason=self.samples.get('bridge',({},0))[0].get('last_reason'),
                takeoff_alt_m=s.takeoff_alt_m, takeoff_action=s.takeoff_action, mag_type=s.mag_type,
                battery=s.battery, rc_required=s.rc_required, rc_valid=s.rc_valid,
                rc_age_s=s.rc_age_s if math.isfinite(s.rc_age_s) else None, rc_override=s.rc_override, rc_mode=s.rc_mode,
                rc_mapping_valid=s.rc_mapping_valid, rc_switch_channels=s.rc_switch_channels)
        changed = self.session.phase != self.previous_state
        if changed:
            self.record(dict(type='transition', **status))
            if self.log and self.session.phase in ('END','FAILED','UNCONFIRMED','PILOT_OVERRIDE'):
                try:
                    os.fsync(self.log.fileno())
                except OSError:
                    self.record_fault = True
            if self.record_fault and self.session.phase == 'END':
                self.session.end('FAILED','completion_log_write_failed',time.monotonic())
                self.session.failure = 'completion_log_write_failed'
                status.update(self.session.status())
        data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(String(data=data))
        if changed:
            self.get_logger().info(self.session.phase+': '+self.session.reason)
            if self.session.phase in ('LANDED', 'END', 'FAILED', 'UNCONFIRMED', 'PILOT_OVERRIDE'):
                self.result_pub.publish(String(data=data))
            self.previous_state = self.session.phase

    def destroy_node(self):
        if self.log:
            self.log.close()
        if self.writer_lock:
            self.writer_lock.close()
        return super().destroy_node()


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    from drone_uwb.integration.ros.lifecycle import init_for_main, shutdown_requested
    init_for_main(args)
    node = None
    try:
        node = FlightNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if not shutdown_requested():
            raise
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
