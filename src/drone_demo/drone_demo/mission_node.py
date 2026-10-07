"""ROS 2 adapter for the demo-only mission monitor."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from .mission import MissionMonitor, load_mission_config
from .node import require_demo_environment


class MissionNode(Node):
    def __init__(self):
        require_demo_environment(os.environ)
        super().__init__('demo_mission_node')
        if self.context.get_domain_id() != 99 or self.get_parameter('use_sim_time').value:
            raise ValueError('demo mission monitor requires domain 99 and wall time')
        descriptor = ParameterDescriptor(read_only=True)
        for name in ('mission_config', 'record_directory'):
            self.declare_parameter(name, '', descriptor)
        self.monitor = MissionMonitor(load_mission_config(self.get_parameter('mission_config').value))
        self.record = None
        directory = self.get_parameter('record_directory').value
        if directory:
            target = Path(directory)
            target.mkdir(parents=True, exist_ok=False)
            (target / 'config.json').write_text(
                json.dumps(asdict(self.monitor.config), indent=2) + '\n', encoding='utf-8')
            self.record = (target / 'mission_state.jsonl').open('x', encoding='utf-8')
        self.publisher = self.create_publisher(String, '/demo_mission_state', 10)
        self.subscriptions_ = [
            self.create_subscription(PoseStamped, '/target_pose', self.on_target,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(PoseStamped, '/demo_pose', self.on_pose, 10),
            self.create_subscription(PoseWithCovarianceStamped, '/uwb_pose', self.on_uwb,
                                     qos_profile_sensor_data),
        ]
        self.started = time.monotonic()
        self.previous_state = None
        self.create_timer(0.05, self.tick)

    def on_target(self, message):
        p = message.pose.position
        self.monitor.set_target(p.x, p.y, message.header.frame_id)

    def receive(self, kind, position, header):
        stamp_ns = header.stamp.sec*1_000_000_000 + header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        self.monitor.update(kind, position.x, position.y, stamp_ns,
                            time.monotonic(), age, header.frame_id)

    def on_pose(self, message):
        self.receive('pose', message.pose.position, message.header)

    def on_uwb(self, message):
        self.receive('uwb', message.pose.pose.position, message.header)

    def tick(self):
        now = time.monotonic()
        status = self.monitor.evaluate(now)
        status['monitor_elapsed_s'] = now-self.started
        data = json.dumps(status, allow_nan=False)
        self.publisher.publish(String(data=data))
        if self.record:
            self.record.write(data + '\n')
            self.record.flush()
        if status['state'] != self.previous_state:
            self.get_logger().info(status['state'] + ': ' + status['reason'])
            self.previous_state = status['state']

    def destroy_node(self):
        if self.record:
            self.record.close()
        return super().destroy_node()


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
        node = MissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
