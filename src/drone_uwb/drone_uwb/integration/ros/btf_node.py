"""Real-sensor B_TF observation node. No FC publisher or command client."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State, TimesyncStatus, ExtendedState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_uwb.processing.measured_btf import MeasuredBtf, MeasuredHeight, ground_xy_without_height, height_independent_b4
from drone_uwb.integration.ros.qos import received_stream_qos, btf_sensor_qos
from drone_uwb.integration.async_recording import AsyncRecording
from drone_uwb.integration.btf_trace import BtfTrace, safe_json
from drone_uwb.integration.clock_readiness import ClockReadiness


def stamp_ns(message):
    return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec


class BtfNode(Node):
    def __init__(self):
        super().__init__('uwb_btf_node')
        default = str(Path(get_package_share_directory('drone_uwb'))/'config/runtime/uwb_btf_real.json')
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('config_file', default, descriptor)
        self.declare_parameter('record_directory', '', descriptor)
        trace_default = str(Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share')))
                            /'dronestock'/'btf-trace'/('domain-'+os.environ.get('ROS_DOMAIN_ID', '0')))
        self.declare_parameter('trace_directory', trace_default, descriptor)
        self.declare_parameter('require_height_for_pose', False, descriptor)
        self.require_height = self.get_parameter('require_height_for_pose').value
        self.config = json.loads(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))
        if self.config['external_output_allowed'] is not False:
            raise ValueError('real_btf_is_observation_only')
        self.processor = MeasuredBtf(self.config)
        # Solver state belongs only to the default mutually exclusive group.
        # Sensor ingress never calls the solver and the solver never holds this
        # lock while fitting. A single snapshot serves all four anchor queries.
        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.sensor_node = Node('uwb_btf_sensor_ingress', context=self.context)
        self.sensor_lock = threading.RLock()
        self.sensor_height = MeasuredHeight(self.config['height'])
        self.sensor_counts = Counter()
        self.sensor_receive_id = 0
        self.sensor_epoch = 0
        self.files = {}
        directory = self.get_parameter('record_directory').value
        if directory:
            root = Path(directory)
            root.mkdir(parents=True, exist_ok=False)
            (root/'config.json').write_text(json.dumps(self.config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            for name in ('inputs', 'decisions', 'status'):
                self.files[name] = (root/(name+'.jsonl')).open('x', encoding='utf-8')
        self.recorder = AsyncRecording(self.files) if self.files else None
        trace_directory = self.get_parameter('trace_directory').value
        self.trace = (BtfTrace(trace_directory, dict(config=self.config,
            node_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            solver_source_sha256=hashlib.sha256((Path(__file__).parents[2]/'processing/measured_btf.py').read_bytes()).hexdigest(),
            imu_qos_depth=100, tof_qos_depth=100, executor_threads=2,
            sensor_callback_group='separate_node_and_single_thread_executor',
            require_height_for_pose=self.require_height)) if trace_directory else None)
        self.counts = Counter()
        self.height_counts = Counter()
        self.last = None
        self.last_source_ns = None
        self.clock_readiness = ClockReadiness()
        self.fc_state = None
        self.fc_state_at = self.landed_at = float('-inf')
        self.landed = None
        self.landed_stamp_ns = 0
        self.ground_velocity_ok = False
        self.ground_velocity_at = float('-inf')
        self.ground_height = self.config.get('ground_antenna_height_m')
        if self.ground_height is not None and (type(self.ground_height) not in (int,float)
                or not math.isfinite(self.ground_height) or not 0 < self.ground_height <= 1.):
            raise ValueError('invalid_ground_antenna_height')
        self.started = time.monotonic()
        self.status_pub = self.create_publisher(String, '/uwb/btf_status', 10)
        self.decision_pub = self.create_publisher(String, '/uwb/btf_decision', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb/btf_pose', qos_profile_sensor_data)
        self.xyz_pub = self.create_publisher(PoseStamped, '/uwb/btf_xyz', qos_profile_sensor_data)
        self.subs = [
            self.create_subscription(String, '/uwb/received', self.raw, received_stream_qos()),
            self.sensor_node.create_subscription(Range, self.config['tof_topic'], self.tof, btf_sensor_qos(),
                                     callback_group=self.sensor_group),
            self.sensor_node.create_subscription(Imu, self.config['imu_topic'], self.imu, btf_sensor_qos(),
                                     callback_group=self.sensor_group),
            self.create_subscription(TimesyncStatus, '/mavros/timesync_status', self.sync, qos_profile_sensor_data),
            self.create_subscription(State, '/mavros/state', self.state, qos_profile_sensor_data),
            self.create_subscription(ExtendedState, '/mavros/extended_state', self.extended, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/mavros/local_position/odom', self.ground_velocity, qos_profile_sensor_data),
        ]
        self.create_timer(1., self.status)
        self.create_service(Trigger, '/uwb/reset_observation_history', self.reset_ground_history)
        self.get_logger().info('Measured B_TF started; output=/uwb/btf_pose; FC output disabled')

    def record(self, name, row, *, encoded_row=None):
        if getattr(self, 'trace', None):
            self.trace.write(name, row, encoded_row=encoded_row)
        if name in self.files:
            if encoded_row is None:
                encoded_row = json.dumps(safe_json(row), ensure_ascii=False, allow_nan=False)
            self.recorder.write(name, encoded_row+'\n')

    def reset_ground_history(self, request, response):
        now = time.monotonic()
        response.success, response.message = self.processor.start_ground_session(
            connected=(self.fc_state or {}).get('connected'), armed=(self.fc_state or {}).get('armed'),
            landed=self.landed, state_age_s=now-self.fc_state_at, landed_age_s=now-self.landed_at,
            stationary=self.ground_velocity_ok, velocity_age_s=now-self.ground_velocity_at)
        row = dict(type='ground_session_reset', accepted=response.success, reason=response.message,
            observation_session=self.processor.observation_session,
            received_ros_ns=self.get_clock().now().nanoseconds)
        if response.success:
            self.last = None
            self.last_source_ns = None
        self.record('inputs', row)
        self.decision_pub.publish(String(data=json.dumps(row)))
        return response

    def state(self, msg):
        fresh = 0 <= self.get_clock().now().nanoseconds-stamp_ns(msg) <= 1_500_000_000
        if not fresh or not msg.connected:
            with self.sensor_lock:
                self.clock_readiness.reset()
                self.clear_sensors('fc_disconnected_or_stale')
        self.fc_state = dict(connected=msg.connected,armed=msg.armed,mode=msg.mode)
        self.fc_state_at = time.monotonic() if fresh else float('-inf')
        self.record('inputs',dict(type='fc_state',received_ros_ns=self.get_clock().now().nanoseconds,**self.fc_state))

    def extended(self, msg):
        self.landed = msg.landed_state
        self.landed_stamp_ns = stamp_ns(msg)
        self.landed_at = (time.monotonic() if 0 <= self.get_clock().now().nanoseconds-stamp_ns(msg) <= 1_500_000_000 else float('-inf'))
        self.record('inputs',dict(type='fc_landed',stamp_ns=stamp_ns(msg),landed_state=msg.landed_state))

    def ground_velocity(self, msg):
        v=msg.twist.twist.linear
        self.ground_velocity_ok=(all(math.isfinite(x) for x in (v.x,v.y,v.z)) and abs(v.z)<=.05
            and math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)<=.1
            and 0<=self.get_clock().now().nanoseconds-stamp_ns(msg)<=200_000_000)
        self.ground_velocity_at=time.monotonic()

    def sync(self, msg):
        with self.sensor_lock:
            accepted, changed = self.clock_readiness.observe(now=time.monotonic(),
                remote_ns=msg.remote_timestamp_ns, offset_ns=msg.estimated_offset_ns,
                rtt_ms=msg.round_trip_time_ms)
            if changed:
                self.clear_sensors('timesync_clock_changed')
        if changed:
            self.processor.height.samples['tof'].clear()
            self.processor.height.samples['imu'].clear()
            self.processor.reset_models()
        self.counts['timesync'] += 1
        self.record('inputs', {'type':'timesync','received_ros_ns':self.get_clock().now().nanoseconds,
             'remote_timestamp_ns':msg.remote_timestamp_ns,'estimated_offset_ns':msg.estimated_offset_ns,
             'round_trip_time_ms':msg.round_trip_time_ms,'stable_count':self.clock_readiness.stable_samples,
             'accepted':accepted,'clock_changed':changed})

    def sync_ready(self):
        with self.sensor_lock:
            return self.clock_readiness.ready(time.monotonic())

    def clear_sensors(self, reason):
        # Caller holds sensor_lock. The running/default-group solver snapshot
        # remains private and is reset separately at its normal clock gates.
        for samples in self.sensor_height.samples.values():
            samples.clear()
        self.sensor_epoch += 1
        self.record('inputs', dict(type='sensor_reset', reason=reason,
            sensor_epoch=self.sensor_epoch, received_monotonic_ns=time.monotonic_ns()))

    def freeze_sensors(self):
        with self.sensor_lock:
            ready = self.sync_ready()
            if not ready:
                self.clear_sensors('clock_not_ready_at_snapshot')
            self.processor.height = self.sensor_height.snapshot()
            meta = dict(receive_id=self.sensor_receive_id, epoch=self.sensor_epoch,
                        frozen_monotonic_ns=time.monotonic_ns(), clock_sync_ready=ready)
            for kind, samples in self.processor.height.samples.items():
                meta[kind] = dict(count=len(samples),
                    oldest_stamp_ns=samples[0]['stamp_ns'] if samples else None,
                    newest_stamp_ns=samples[-1]['stamp_ns'] if samples else None)
            return meta

    def sensor(self, kind, row):
        row['received_monotonic_ns'] = time.monotonic_ns()
        row['received_ros_ns'] = self.get_clock().now().nanoseconds
        with self.sensor_lock:
            row['clock_sync_ready'] = self.sync_ready()
            row['header_age_ns'] = row['received_ros_ns']-row['stamp_ns']
            if not row['clock_sync_ready'] or not 0 <= row['header_age_ns'] <= 500_000_000:
                row['valid'] = False
            self.sensor_receive_id += 1
            row['receive_id'] = self.sensor_receive_id
            row['sensor_epoch'] = self.sensor_epoch
            samples = self.sensor_height.samples[kind]
            previous = samples[-1]['stamp_ns'] if samples else None
            accepted = self.sensor_height.add(kind, row)
            reason = ('accepted' if accepted else 'invalid_stamp' if type(row['stamp_ns']) is not int
                or row['stamp_ns'] <= 0 else 'duplicate' if previous == row['stamp_ns'] else 'backward_stamp_reset')
            self.sensor_counts[kind] += 1
            self.sensor_counts[kind+'_valid'] += int(row['valid'])
            self.sensor_counts[kind+'_'+reason] += 1
            self.record('inputs', dict(row, type=kind, source='measured', buffer_accepted=accepted,
                buffer_reason=reason, previous_stamp_ns=previous, buffer_count=len(samples)))

    def tof(self, msg):
        valid = all(math.isfinite(v) for v in (msg.range,msg.min_range,msg.max_range)) and 0 < msg.min_range <= msg.range <= msg.max_range
        self.sensor('tof', {'stamp_ns':stamp_ns(msg),'range_m':float(msg.range),
             'min_m':float(msg.min_range),'max_m':float(msg.max_range),'valid':valid,'frame_id':msg.header.frame_id})

    def imu(self, msg):
        q = [msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z]
        valid = (all(math.isfinite(v) for v in q) and abs(sum(v*v for v in q)-1) < .02
                 and msg.orientation_covariance[0] >= 0 and msg.header.frame_id == 'base_link')
        self.sensor('imu', {'stamp_ns':stamp_ns(msg),'quaternion_wxyz':q,'valid':valid,'frame_id':msg.header.frame_id})

    def raw(self, msg):
        if self.recorder and self.recorder.error:
            self.counts['recording_failed'] += 1
            # Retain fresh observations when diagnostic storage fails.
            # status still reports the failure; validity gates below remain.
        started = time.perf_counter()
        callback_started_ns = time.monotonic_ns()
        now_ns = self.get_clock().now().nanoseconds
        try:
            event = json.loads(msg.data)
            age_ns = now_ns-event['host_received_ros_ns']
            if not 0 <= age_ns <= 150_000_000:
                self.counts['receiver_queue_expired'] += 1
                if age_ns < 0:
                    self.processor.reset()
                    with self.sensor_lock:
                        self.clear_sensors('receiver_clock_reversed')
                else:
                    self.processor.reject_queued_input()
                self.record('inputs',dict(type='uwb_rejected',event=event,reason='receiver_queue_expired',
                    reset_scope='all' if age_ns<0 else 'queued_models', age_ns=age_ns))
                self.decision_pub.publish(String(data=json.dumps(dict(
                    schema=1,ok=False,published=False,reason='receiver_queue_expired',
                    decision_stamp_ns=now_ns,input_received_ros_ns=event['host_received_ros_ns'],
                    receiver_age_ns=age_ns,observation_session=self.processor.observation_session))))
                return
            snapshot = self.freeze_sensors()
            sync_ready = snapshot['clock_sync_ready']
            self.record('inputs',dict(type='uwb',event=event,clock_sync_ready=sync_ready,
                                     sensor_snapshot=snapshot, callback_ros_ns=now_ns))
            ground = (self.ground_height is not None and sync_ready and (self.fc_state or {}).get('connected') is True
                and self.landed==1 and time.monotonic()-self.fc_state_at<=1.5
                and time.monotonic()-self.landed_at<=1.5 and self.ground_velocity_ok
                and time.monotonic()-self.ground_velocity_at<=.2)
            self.processor.height.ground_reference = (dict(height_m=self.ground_height,
                selection_stamp_ns=now_ns,ground_state_stamp_ns=self.landed_stamp_ns) if ground else None)
            result = self.processor.process(event)
            result['sensor_snapshot'] = snapshot
            result['callback_ros_ns'] = now_ns
            result['callback_started_monotonic_ns'] = callback_started_ns
            result['receiver_age_ns'] = age_ns
            if result.get('reason') == 'host_clock_jump':
                with self.sensor_lock:
                    self.clear_sensors('host_clock_jump')
        except (KeyError, TypeError, ValueError):
            self.counts['invalid_envelope'] += 1
            return
        self.counts[result['reason']] += 1
        if result['reason'].startswith('sideband_') or result['reason'] in ('status', 'awaiting_tdma'):
            return
        if 'seq' in result:
            self.last_source_ns = event['host_received_ros_ns']
            self.counts['cycles'] += 1
        for selection in result.get('height_selection',[]):
            self.height_counts[selection['reason']] += 1
        result['processing_ms'] = (time.perf_counter()-started)*1000
        result['processing_finished_monotonic_ns'] = time.monotonic_ns()
        result['published'] = False
        result['decision_stamp_ns'] = self.get_clock().now().nanoseconds
        result['input_received_ros_ns'] = event['host_received_ros_ns']
        if result['ok']:
            age = (self.get_clock().now().nanoseconds-result['stamp_ns'])/1e9
            result['publish_age_s'] = age
            ground_xy = ground_xy_without_height(result,
                enabled=self.config.get('allow_ground_xy_without_tof', False),
                connected=(self.fc_state or {}).get('connected'), landed=self.landed,
                state_age_s=time.monotonic()-self.fc_state_at, landed_age_s=time.monotonic()-self.landed_at)
            result['ground_xy_without_measured_height'] = ground_xy
            b4_xy = height_independent_b4(result,
                enabled=self.config.get('allow_b4_xy_without_tof', True))
            result['b4_xy_without_measured_height'] = b4_xy and result.get('xyz_m') is None
            if self.require_height and result.get('xyz_m') is None and not (ground_xy or b4_xy):
                result['pose_blocked_reason'] = 'measured_height_required'
                self.counts['measured_height_required'] += 1
            elif 0 <= age <= self.config['max_output_age_s']:
                pose = PoseWithCovarianceStamped()
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(result['stamp_ns'],1_000_000_000)
                pose.header.frame_id = 'uwb_map'
                pose.pose.pose.position.x, pose.pose.pose.position.y = result['xy_m']
                pose.pose.pose.orientation.w = 1.
                pose.pose.covariance[0] = pose.pose.covariance[7] = self.config['xy_stddev_m']**2
                for index in (14,21,28,35):
                    pose.pose.covariance[index] = 1e6
                self.pose_pub.publish(pose)
                if result.get('xyz_m') is not None and result.get('height_source')=='measured_tof_imu':
                    xyz = PoseStamped()
                    xyz.header = pose.header
                    xyz.pose.position.x, xyz.pose.position.y, xyz.pose.position.z = result['xyz_m']
                    xyz.pose.orientation.w = 1.
                    self.xyz_pub.publish(xyz)
                result['published'] = True
                self.counts['published'] += 1
            else:
                self.counts['output_expired'] += 1
                result['pose_blocked_reason'] = 'output_expired'
        self.last = result
        encoded = json.dumps(safe_json(result),ensure_ascii=False,allow_nan=False)
        self.record('decisions',result,encoded_row=encoded)
        self.decision_pub.publish(String(data=encoded))

    def status(self):
        now_ns = self.get_clock().now().nanoseconds
        with self.sensor_lock:
            counts = dict(self.counts + self.sensor_counts)
        row = {'utc':datetime.now(timezone.utc).isoformat(),'elapsed_s':time.monotonic()-self.started,
               'counts':counts,'height_counts':dict(self.height_counts),
               'last_reason':self.last.get('reason') if self.last else None,
               'jump_guard_enabled':False,
               'last_xy_m':self.last.get('xy_m') if self.last else None,
               'last_xyz_m':self.last.get('xyz_m') if self.last else None,
               'require_height_for_pose':self.require_height,
               'allow_b4_xy_without_tof':self.config.get('allow_b4_xy_without_tof', True),
               'enforce_b4_height_integrity':self.config.get('enforce_b4_height_integrity', False),
               'numerical_backend':self.processor.native.identity if self.processor.native is not None else {'backend':'python'},
               'input_age_s':(now_ns-self.last_source_ns)/1e9 if self.last_source_ns else None,
               'fc_output_enabled':False,'timestamp_calibrated':False,'timesync_ready':self.sync_ready(),
               'recording_error':self.recorder.error if self.recorder else None,
               'fc_state':self.fc_state,
               'height_mount_confirmed':self.config['height']['mount_confirmed'],
               'flat_floor_confirmed':self.config['height']['flat_floor_confirmed']}
        row['sensor_qos_depth'] = 100
        row['trace'] = self.trace.status() if self.trace else {'enabled':False}
        row['observation_session'] = self.processor.observation_session
        self.status_pub.publish(String(data=json.dumps(row,ensure_ascii=False)))
        self.record('status',row)
        if self.recorder:
            self.recorder.flush()

    def destroy_node(self):
        if rclpy.ok():
            self.status()
        if self.recorder:
            try:
                self.recorder.close()
            except OSError as exc:
                self.get_logger().error('Recording shutdown: ' + str(exc))
        if self.trace:
            self.trace.close()
            if self.trace.error:
                self.get_logger().error('Internal trace: '+self.trace.error)
        self.sensor_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    from .lifecycle import init_for_main, spin_until_stopped
    init_for_main(args, interrupt_spin=False)
    node = BtfNode()
    try:
        spin_until_stopped(node, ingress_node=node.sensor_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # spin_until_stopped joins ingress before closing files/publishers.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
