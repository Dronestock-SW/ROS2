"""MAVROS adapter for web-triggered PX4 flight trials."""

from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandInt, CommandLong, CommandTOL
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from drone_uwb.processing.gazebo_geometry import rotation_world_body
from drone_uwb.integration.sitl.sitl_target_contract import PX4GlobalReference
from .contracts import Settings, finite
from .session import FlightSession, Snapshot
from .writer_lock import WriterLock


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
        self.declare_parameter('ledger_file', str(Path.home()/'.local/state/dronestock-flight/requests.json'), descriptor)
        values = json.loads(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))
        self.settings = replace(Settings(**values), execute=self.get_parameter('execute').value)
        test_mode = self.get_parameter('test_mode').value
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
        self.session = FlightSession(self.settings, consumed, self.remember)
        self.samples = {}
        self.param_value = None
        self.param_received = float('-inf')
        self.param_future = None
        self.param_requested = float('-inf')
        self.streams_requested = False
        self.origin_requested = float('-inf')
        self.origin = None
        self.log = None
        directory = self.get_parameter('record_directory').value
        if directory:
            root = Path(directory)
            root.mkdir(parents=True, exist_ok=False)
            (root/'settings.json').write_text(json.dumps(asdict(self.settings), indent=2)+'\n', encoding='utf-8')
            self.log = (root/'events.jsonl').open('x', encoding='utf-8')
        self.status_pub = self.create_publisher(String, '/flight_state', 10)
        self.valid_pub = self.create_publisher(String, '/target_valid', 10)
        self.result_pub = self.create_publisher(String, '/mission_result', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/mission/assignment', self.assignment, 10)
        for name, cls, topic in (
                ('state', State, '/mavros/state'),
                ('landed', ExtendedState, '/mavros/extended_state'),
                ('estimator', EstimatorStatus, '/mavros/estimator_status'),
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
        self.param_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.stream_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.previous_state = None
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
            self.log.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
            self.log.flush()

    def receive(self, key, msg):
        self.samples[key] = (msg, time.monotonic())

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
            self.session.submit(json.loads(msg.data), time.monotonic(), time.time())
        except (ValueError, TypeError):
            self.session.validation = {'accepted': False, 'reason': 'invalid_assignment_json'}
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
        if msg.coordinate_frame in (0, 5) and not msg.type_mask & 3 and 0 <= age <= .5:
            self.session.applied_target(msg.latitude, msg.longitude, stamp_ns(msg))

    def snapshot(self):
        s = Snapshot(origin=self.origin, command_services_ready=all(
            client.service_is_ready() for client in self.command_clients.values()),
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
            s.estimator_valid = (msg.attitude_status_flag and msg.velocity_horiz_status_flag
                and msg.pos_horiz_rel_status_flag and not msg.const_pos_mode_status_flag
                and not msg.gps_glitch_status_flag and not msg.accel_error_status_flag)
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
                a = math.radians(self.settings.enu_yaw_deg)
                x, y = p.x-self.settings.enu_offset_x_m, p.y-self.settings.enu_offset_y_m
                s.xy = (math.cos(a)*x+math.sin(a)*y, -math.sin(a)*x+math.cos(a)*y)
                s.velocity_xy = (math.cos(a)*velocity[0]+math.sin(a)*velocity[1],
                                 -math.sin(a)*velocity[0]+math.cos(a)*velocity[1])
                s.pose_stamp_ns, s.pose_age_s = stamp_ns(msg), self.age('pose')
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
                and all(finite(cfg.get(k)) and abs(cfg[k]-getattr(self.settings, k)) < 1e-6
                        for k in ('enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m',
                                  'expected_ev_delay_ms'))
                and all(finite(cfg.get('antenna_body_frd_'+axis+'_m'))
                        and abs(cfg['antenna_body_frd_'+axis+'_m']
                                -getattr(self.settings, 'expected_ev_pos_'+axis+'_m')) < 1e-6
                        for axis in ('x', 'y', 'z')))
        s.takeoff_alt_m = self.param_value
        s.takeoff_param_age_s = time.monotonic()-self.param_received
        return s

    def query(self):
        now = time.monotonic()
        if self.param_future is not None and now-self.param_requested > 2.:
            self.param_client.remove_pending_request(self.param_future)
            self.param_future = None
        if self.param_future is None and self.param_client.service_is_ready():
            self.param_requested = now
            self.param_future = self.param_client.call_async(GetParameters.Request(names=['MIS_TAKEOFF_ALT']))
            self.param_future.add_done_callback(self.parameters_received)
        if self.stream_client.service_is_ready():
            if not self.streams_requested:
                for message, rate in ((230, 10.), (245, 5.), (87, 10.)):
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
        except Exception:
            self.param_value = None

    def dispatch(self, action):
        kind, token = action['kind'], action['token']
        client = self.command_clients['arm' if kind == 'disarm' else kind]
        if not client.service_is_ready():
            self.session.command_result(token, False, time.monotonic())
            return
        if kind in ('arm', 'disarm'):
            request = CommandBool.Request(value=kind == 'arm')
        elif kind in ('takeoff', 'land'):
            # PX4 selects MIS_TAKEOFF_ALT/current location. No companion Z target.
            request = CommandTOL.Request(yaw=math.nan, latitude=math.nan,
                                         longitude=math.nan, altitude=math.nan)
        else:
            request = CommandInt.Request(broadcast=False, frame=0, command=192,
                param1=float(action['speed']), param2=0., param3=math.nan, param4=math.nan,
                x=action['x'], y=action['y'], z=math.nan)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'uwb_map'
            pose.pose.position.x, pose.pose.position.y = self.session.monitor.target
            pose.pose.orientation.w = 1.
            self.target_pub.publish(pose)
        self.record(dict(type='command_requested', **action))
        future = client.call_async(request)

        def completed(result):
            try:
                response = result.result()
                accepted = response.success and getattr(response, 'result', 0) == 0
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
        status = self.session.status()
        fresh_state = 0 <= s.state_age_s <= self.settings.state_timeout_s
        status.update(fc_connected=s.connected if fresh_state else None,
                      fc_armed=s.armed if fresh_state else None,
                      fc_mode=s.mode if fresh_state else None,
                      px4_map_xy_m=list(s.xy) if s.pose_age_s <= .2 else None)
        data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(String(data=data))
        if self.session.phase != self.previous_state:
            self.get_logger().info(self.session.phase+': '+self.session.reason)
            self.record(dict(type='transition', **status))
            if self.session.phase in ('LANDED', 'FAILED', 'PILOT_OVERRIDE'):
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
    from drone_uwb.integration.ros.lifecycle import init_for_main
    init_for_main(args)
    node = None
    try:
        node = FlightNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
