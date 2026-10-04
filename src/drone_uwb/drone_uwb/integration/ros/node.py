"""Live receive-only UWB node. Publishes observations, never flight commands."""
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from drone_uwb.processing.solvers.observations import Processor, Settings
from drone_uwb.contracts.protocol import InvalidSample, decode_line
from drone_uwb.acquisition.framing import LineFramer
from drone_uwb.integration.recording import open_record_files
from drone_uwb.acquisition.serial_io import SerialInput


class UwbNode(Node):
    def __init__(self):
        super().__init__('uwb_node')
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('port', '/dev/uwb', descriptor)
        self.declare_parameter('baudrate', 921600, descriptor)
        self.declare_parameter('anchor_file', '', descriptor)
        self.declare_parameter('watchdog_timeout_s', 0.5, descriptor)
        self.declare_parameter('record_directory', '', descriptor)
        self.declare_parameter('stop_after_s', 0.0, descriptor)
        for name, value in asdict(Settings()).items():
            self.declare_parameter(name, value, descriptor)
        values = {name: self.get_parameter(name).value for name in asdict(Settings())}
        self.settings = Settings(**values)
        path = self.get_parameter('anchor_file').value
        if not path:
            path = str(Path(get_package_share_directory('drone_uwb')) / 'config/anchors/anchors_20261004.json')
        self.layout = json.loads(Path(path).read_text(encoding='utf-8'))
        self.processor = Processor(self.layout, self.settings)
        self.framer = LineFramer()
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb_pose', qos_profile_sensor_data)
        self.raw_pub = self.create_publisher(String, '/uwb/raw', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/uwb/status', 10)
        self.counts = Counter()
        self.last_decision = 'starting'
        self.last_cycle_mono = None
        self.started = time.monotonic()
        self.serial = None
        self.record_files = {}
        self.finished = False
        self.parse_streak = 0
        self.last_open_attempt = 0.0
        directory = self.get_parameter('record_directory').value
        if directory:
            self.record_files = open_record_files(directory, {
                'start_utc': datetime.now(timezone.utc).isoformat(),
                'port': self.get_parameter('port').value,
                'settings': asdict(self.settings), 'layout': self.layout,
                'serial_payload_bytes_written': 0,
                'flight_state_estimator': 'PX4 EKF2 only',
            })
        self.create_timer(0.005, self.poll)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('UWB source=' + self.settings.source_mode + ', frame=' + self.layout['coordinate_frame'])

    def record(self, name, value):
        if name in self.record_files:
            self.record_files[name].write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')

    def poll(self):
        now = time.monotonic()
        stop = self.get_parameter('stop_after_s').value
        if stop > 0 and now - self.started >= stop:
            self.finished = True
            return
        if self.serial is None:
            if now - self.last_open_attempt < 1.0:
                return
            self.last_open_attempt = now
            try:
                self.serial = SerialInput(self.get_parameter('port').value, self.get_parameter('baudrate').value)
                self.framer = LineFramer()
                self.processor.disconnect()
                self.last_decision = 'waiting_status'
            except OSError as exc:
                self.last_decision = 'serial_unavailable'
                self.get_logger().warning(str(exc))
            return
        try:
            chunk = self.serial.read()
        except OSError as exc:
            self.get_logger().warning(str(exc))
            self.close_serial()
            self.processor.disconnect()
            self.last_decision = 'serial_disconnected'
            return
        if chunk is None:
            return
        mono_ns, ros_ns = time.monotonic_ns(), self.get_clock().now().nanoseconds
        if 'raw' in self.record_files:
            self.record_files['raw'].write(chunk)
        for line in self.framer.feed(chunk):
            try:
                msg = decode_line(line)
            except InvalidSample:
                self.counts['parse_error'] += 1
                self.parse_streak += 1
                if self.parse_streak >= 100:
                    self.get_logger().error('No valid JSON protocol: check port and baudrate.')
                    self.finished = True
                continue
            self.parse_streak = 0
            self.counts['json_messages'] += 1
            if msg.get('type') == 'uwb_raw_cycle':
                self.counts['cycles'] += 1
                self.last_cycle_mono = mono_ns
            self.raw_pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))
            self.record('received', {'host_received_monotonic_ns': mono_ns,
                                    'host_received_ros_ns': ros_ns, 'message': msg})
            result = self.processor.process(msg, mono_ns, ros_ns)
            self.last_decision = result.reason
            self.counts[result.reason] += 1
            self.record('decisions', {'host_received_monotonic_ns': mono_ns,
                                     'reason': result.reason, 'details': result.details,
                                     'observation': asdict(result.observation) if result.observation else None})
            if result.observation:
                obs = result.observation
                pose = PoseWithCovarianceStamped()
                pose.header.stamp.sec = obs.stamp_ns // 1_000_000_000
                pose.header.stamp.nanosec = obs.stamp_ns % 1_000_000_000
                pose.header.frame_id = self.layout['coordinate_frame']
                pose.pose.pose.position.x, pose.pose.pose.position.y = obs.x, obs.y
                # Unobserved message fields, never height/attitude control targets.
                pose.pose.pose.orientation.w = 1.0
                pose.pose.covariance[0] = pose.pose.covariance[7] = obs.variance
                for index in (14, 21, 28, 35):
                    pose.pose.covariance[index] = 1e6
                self.pose_pub.publish(pose)

    def publish_status(self):
        age = ((time.monotonic_ns() - self.last_cycle_mono) / 1e9
               if self.last_cycle_mono is not None else None)
        value = {'counts': dict(self.counts), 'last_decision': self.last_decision,
                 'connected': self.serial is not None,
                 'cycle_age_s': age,
                 'watchdog_ok': age is not None and age <= self.get_parameter('watchdog_timeout_s').value,
                 'source_mode': self.settings.source_mode,
                 'layout_id': self.layout['layout_id'],
                 'frame_id': self.layout['coordinate_frame'],
                 'tag_status': self.processor.status,
                 'timestamp_method': 'approximate_mean_report_read_time',
                 'timestamp_calibrated': False,
                 'framing_overflows': self.framer.overflows}
        if rclpy.ok():
            self.status_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))
        self.record('status', value)
        for file in self.record_files.values():
            file.flush()

    def close_serial(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except OSError as exc:
                self.get_logger().warning('Serial restore: ' + str(exc))
            self.serial = None

    def destroy_node(self):
        self.publish_status()
        self.close_serial()
        for file in self.record_files.values():
            file.close()
        return super().destroy_node()


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    rclpy.init(args=args)
    node = None
    try:
        node = UwbNode()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
