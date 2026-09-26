"""Local demo-domain publishers. No MAVROS, serial, server, or flight interface."""
import json
import os
import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Range

from .core import DemoRun, load_inputs
from .z_gate import DemoZGate


def require_demo_environment(environ):
    if environ.get('ROS_DOMAIN_ID') != '99' or environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('Use ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 for demo inputs.')


def pose_message(xyz, stamp, frame):
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame
    message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, xyz)
    message.pose.orientation.w = 1.0
    return message


def observation_message(obs, frame):
    message = PoseWithCovarianceStamped()
    message.header.stamp.sec = obs['stamp_ns'] // 1_000_000_000
    message.header.stamp.nanosec = obs['stamp_ns'] % 1_000_000_000
    message.header.frame_id = frame
    message.pose.pose.position.x = obs['x']
    message.pose.pose.position.y = obs['y']
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = message.pose.covariance[7] = obs['variance']
    for index in (14, 21, 28, 35):
        message.pose.covariance[index] = 1e6
    return message


class DemoNode(Node):
    def __init__(self):
        require_demo_environment(os.environ)
        super().__init__('demo_node')
        if self.context.get_domain_id() != 99:
            raise ValueError('demo context must use domain 99')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('demo_node uses wall time; do not set use_sim_time')
        descriptor = ParameterDescriptor(read_only=True)
        for name, default in (('config_file', ''), ('scenario', '')):
            self.declare_parameter(name, default, descriptor)
        self.declare_parameter('synthetic_z_enabled', True, descriptor)
        self.declare_parameter('real_tof_topic', '/tof/range', descriptor)
        self.z_gate = DemoZGate(self.get_parameter('synthetic_z_enabled').value)
        config, layout, settings = load_inputs(
            self.get_parameter('config_file').value,
            scenario=self.get_parameter('scenario').value or None)
        self.run = DemoRun(config, layout, settings)
        self.truth_pub = self.create_publisher(PoseStamped, '/demo_pose', 10)
        self.target_pub = self.create_publisher(
            PoseStamped, '/target_pose', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.uwb_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/uwb_pose', qos_profile_sensor_data)
        self.raw_pub = self.create_publisher(String, '/uwb/raw', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/demo_status', 10)
        self.tof_sub = self.create_subscription(
            Range, self.get_parameter('real_tof_topic').value,
            self.on_real_tof, qos_profile_sensor_data)
        self.started_ns = time.monotonic_ns()
        self.last_index = -1
        self.last_phase = None
        self.finished = False
        self.timer = self.create_timer(1/config.rate_hz, self.tick)
        self.get_logger().info(
            f'DEMO ONLY: {config.scenario}, sine z={config.z_min_m}..{config.z_max_m} m, '
            f'domain 99, {config.duration_s} s. No flight interface.')

    def on_real_tof(self, message):
        if self.z_gate.observe(message.range, message.min_range, message.max_range):
            if not self.finished:
                self.get_logger().warn('Real ToF detected; stopping demo publishers.')
                self.timer.cancel()
                self.finished = True

    def tick(self):
        if self.z_gate.blocked:
            self.timer.cancel()
            self.finished = True
            return
        mono_ns = time.monotonic_ns()
        elapsed = (mono_ns - self.started_ns) / 1e9
        if elapsed >= self.run.config.duration_s:
            self.timer.cancel()
            self.finished = True
            self.get_logger().info('Demo completed.')
            return
        index = int(elapsed * self.run.config.rate_hz)
        if index <= self.last_index:
            return
        stamp = self.get_clock().now()
        sample = self.run.sample(index, mono_ns, stamp.nanoseconds)
        if self.last_index < 0:
            self.target_pub.publish(pose_message(
                sample['target_xyz_m'], stamp.to_msg(), sample['frame_id']))
        self.last_index = index
        self.truth_pub.publish(pose_message(sample['truth_xyz_m'], stamp.to_msg(), sample['frame_id']))
        for record in sample['received']:
            self.raw_pub.publish(String(data=json.dumps(record['message'], allow_nan=False)))
        if sample['observation']:
            self.uwb_pub.publish(observation_message(sample['observation'], sample['frame_id']))
        status = {k: v for k, v in sample.items() if k not in ('received', 'observation')}
        status['uwb_observation_published'] = sample['observation'] is not None
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        phase = (sample['trajectory_phase'], sample['uwb_available'])
        if phase != self.last_phase:
            self.get_logger().info(
                f"t={sample['time_s']:.2f}s {phase[0]}, UWB available={phase[1]}")
            self.last_phase = phase


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    try:
        require_demo_environment(os.environ)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    rclpy.init(args=args)
    node = None
    try:
        node = DemoNode()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
