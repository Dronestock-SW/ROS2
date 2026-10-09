"""Real-sensor B_TF observation node. No FC publisher or command client."""
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State, TimesyncStatus
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String

from drone_uwb.processing.measured_btf import MeasuredBtf
from drone_uwb.integration.ros.qos import received_stream_qos


def safe_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(v) for v in value]
    return value


def stamp_ns(message):
    return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec


class BtfNode(Node):
    def __init__(self):
        super().__init__('uwb_btf_node')
        default = str(Path(get_package_share_directory('drone_uwb'))/'config/runtime/uwb_btf_real.json')
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('config_file', default, descriptor)
        self.declare_parameter('record_directory', '', descriptor)
        self.declare_parameter('require_height_for_pose', False, descriptor)
        self.require_height = self.get_parameter('require_height_for_pose').value
        self.config = json.loads(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))
        if self.config['external_output_allowed'] is not False:
            raise ValueError('real_btf_is_observation_only')
        self.processor = MeasuredBtf(self.config)
        self.files = {}
        directory = self.get_parameter('record_directory').value
        if directory:
            root = Path(directory)
            root.mkdir(parents=True, exist_ok=False)
            (root/'config.json').write_text(json.dumps(self.config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            for name in ('inputs', 'decisions', 'status'):
                self.files[name] = (root/(name+'.jsonl')).open('x', encoding='utf-8')
        self.counts = Counter()
        self.height_counts = Counter()
        self.last = None
        self.last_source_ns = None
        self.sync_received = None
        self.sync_offset = None
        self.sync_count = 0
        self.fc_state = None
        self.started = time.monotonic()
        self.status_pub = self.create_publisher(String, '/uwb/btf_status', 10)
        self.decision_pub = self.create_publisher(String, '/uwb/btf_decision', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb/btf_pose', qos_profile_sensor_data)
        self.xyz_pub = self.create_publisher(PoseStamped, '/uwb/btf_xyz', qos_profile_sensor_data)
        self.subs = [
            self.create_subscription(String, '/uwb/received', self.raw, received_stream_qos()),
            self.create_subscription(Range, self.config['tof_topic'], self.tof, qos_profile_sensor_data),
            self.create_subscription(Imu, self.config['imu_topic'], self.imu, qos_profile_sensor_data),
            self.create_subscription(TimesyncStatus, '/mavros/timesync_status', self.sync, qos_profile_sensor_data),
            self.create_subscription(State, '/mavros/state', self.state, qos_profile_sensor_data),
        ]
        self.create_timer(1., self.status)
        self.get_logger().info('Measured B_TF started; output=/uwb/btf_pose; FC output disabled')

    def record(self, name, row):
        if name in self.files:
            self.files[name].write(json.dumps(safe_json(row), ensure_ascii=False, allow_nan=False)+'\n')

    def state(self, msg):
        self.fc_state = dict(connected=msg.connected,armed=msg.armed,mode=msg.mode)
        self.record('inputs',dict(type='fc_state',received_ros_ns=self.get_clock().now().nanoseconds,**self.fc_state))

    def sync(self, msg):
        valid = (math.isfinite(msg.round_trip_time_ms) and 0 <= msg.round_trip_time_ms <= 20
                 and msg.remote_timestamp_ns > 0)
        if self.sync_offset is not None and abs(msg.estimated_offset_ns-self.sync_offset) > 5_000_000:
            valid = False
            self.processor.height.samples['tof'].clear()
            self.processor.height.samples['imu'].clear()
            self.processor.reset_models()
        self.sync_count = self.sync_count+1 if valid else 0
        self.sync_offset = msg.estimated_offset_ns
        self.sync_received = time.monotonic()
        self.counts['timesync'] += 1
        self.record('inputs', {'type':'timesync','received_ros_ns':self.get_clock().now().nanoseconds,
             'remote_timestamp_ns':msg.remote_timestamp_ns,'estimated_offset_ns':msg.estimated_offset_ns,
             'round_trip_time_ms':msg.round_trip_time_ms,'stable_count':self.sync_count})

    def sync_ready(self):
        return self.sync_count >= 30 and self.sync_received is not None and time.monotonic()-self.sync_received < .5

    def sensor(self, kind, row):
        row['received_ros_ns'] = self.get_clock().now().nanoseconds
        row['clock_sync_ready'] = self.sync_ready()
        if not row['clock_sync_ready'] or not 0 <= row['received_ros_ns']-row['stamp_ns'] <= 500_000_000:
            row['valid'] = False
        self.processor.height.add(kind, row)
        self.counts[kind] += 1
        self.counts[kind+'_valid'] += int(row['valid'])
        self.record('inputs', dict(row, type=kind, source='measured'))

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
        started = time.perf_counter()
        now_ns = self.get_clock().now().nanoseconds
        try:
            event = json.loads(msg.data)
            age_ns = now_ns-event['host_received_ros_ns']
            if not 0 <= age_ns <= 150_000_000:
                self.counts['receiver_queue_expired'] += 1
                if age_ns < 0:
                    self.processor.reset()
                else:
                    self.processor.reject_queued_input()
                self.record('inputs',dict(type='uwb_rejected',event=event,reason='receiver_queue_expired'))
                return
            sync_ready = self.sync_ready()
            self.record('inputs',dict(type='uwb',event=event,clock_sync_ready=sync_ready))
            if not sync_ready:
                self.processor.height.samples['tof'].clear()
                self.processor.height.samples['imu'].clear()
            result = self.processor.process(event)
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
        result['published'] = False
        if result['ok']:
            age = (self.get_clock().now().nanoseconds-result['stamp_ns'])/1e9
            result['publish_age_s'] = age
            if self.require_height and result.get('xyz_m') is None:
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
                if result.get('xyz_m') is not None:
                    xyz = PoseStamped()
                    xyz.header = pose.header
                    xyz.pose.position.x, xyz.pose.position.y, xyz.pose.position.z = result['xyz_m']
                    xyz.pose.orientation.w = 1.
                    self.xyz_pub.publish(xyz)
                result['published'] = True
                self.counts['published'] += 1
            else:
                self.counts['output_expired'] += 1
        self.last = result
        self.record('decisions',result)
        self.decision_pub.publish(String(data=json.dumps(safe_json(result),ensure_ascii=False,allow_nan=False)))

    def status(self):
        now_ns = self.get_clock().now().nanoseconds
        row = {'utc':datetime.now(timezone.utc).isoformat(),'elapsed_s':time.monotonic()-self.started,
               'counts':dict(self.counts),'height_counts':dict(self.height_counts),
               'last_reason':self.last.get('reason') if self.last else None,
               'last_xy_m':self.last.get('xy_m') if self.last else None,
               'last_xyz_m':self.last.get('xyz_m') if self.last else None,
               'require_height_for_pose':self.require_height,
               'input_age_s':(now_ns-self.last_source_ns)/1e9 if self.last_source_ns else None,
               'fc_output_enabled':False,'timestamp_calibrated':False,'timesync_ready':self.sync_ready(),
               'fc_state':self.fc_state,
               'height_mount_confirmed':self.config['height']['mount_confirmed'],
               'flat_floor_confirmed':self.config['height']['flat_floor_confirmed']}
        self.status_pub.publish(String(data=json.dumps(row,ensure_ascii=False)))
        self.record('status',row)
        for file in self.files.values():
            file.flush()

    def destroy_node(self):
        if rclpy.ok():
            self.status()
        for file in self.files.values():
            file.close()
        return super().destroy_node()


def main(args=None):
    from .lifecycle import init_for_main
    init_for_main(args)
    node = BtfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
