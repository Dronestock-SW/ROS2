"""Forward verified horizontal observations to MAVROS; disabled by default.

No flight commands, parameter writes, or PX4 pose/attitude feedback.
Configuration is immutable after startup; restart to apply surveyed values.
"""
from dataclasses import asdict
import json
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String

from drone_uwb.integration.ros.frames import BridgeSettings, gate, rotate_xy_covariance


class UwbPx4Bridge(Node):
    def __init__(self):
        super().__init__('uwb_px4_bridge')
        for name, value in asdict(BridgeSettings()).items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.settings = BridgeSettings(**{k: self.get_parameter(k).value for k in asdict(BridgeSettings())})
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        self.status_pub = self.create_publisher(String, '/uwb/bridge_status', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/uwb_pose', self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self.receive_state, qos_profile_sensor_data)
        self.client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.future = None
        self.request_time = 0.0
        self.params = {}
        self.param_time = self.state_time = float('-inf')
        self.connected = False
        self.last_stamp = None
        self.published = self.rejected = 0
        self.last_reason = 'starting'
        self.create_timer(1.0, self.monitor)

    def receive_state(self, msg):
        if not msg.connected or not self.connected:
            self.params = {}
            self.param_time = float('-inf')
            self.last_stamp = None
        self.connected, self.state_time = msg.connected, time.monotonic()

    def current_gate(self):
        now = time.monotonic()
        return gate(self.settings, self.connected, now-self.state_time, self.params, now-self.param_time)

    def monitor(self):
        # MAVROS exposes its FC parameter mirror. This node never changes it.
        # The operator must verify it against the FC after external configuration changes.
        now = time.monotonic()
        if self.future is not None and now - self.request_time > 2.0:
            self.client.remove_pending_request(self.future)
            self.future = None
        if self.connected and self.future is None and self.client.service_is_ready():
            request = GetParameters.Request(names=['EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD'])
            self.request_time = now
            self.future = self.client.call_async(request)
            self.future.add_done_callback(self.parameters_received)
        self.status_pub.publish(String(data=json.dumps({
            'gate': self.current_gate(), 'last_reason': self.last_reason,
            'published': self.published, 'rejected': self.rejected,
            'parameters': self.params, 'settings': asdict(self.settings),
        }, ensure_ascii=False)))

    def parameters_received(self, future):
        if future is not self.future:
            return
        self.future = None
        if future.exception() is not None:
            return
        names = ('EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD')
        self.params = {name: val.integer_value if val.type == 2 else val.double_value if val.type == 3 else None
                       for name, val in zip(names, future.result().values)}
        self.param_time = time.monotonic()

    def receive_pose(self, msg):
        reason = self.current_gate()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp)/1e9
        if reason == 'ready' and (msg.header.frame_id != self.settings.source_frame
                                  or not 0 <= age <= self.settings.max_age_s
                                  or (self.last_stamp is not None and stamp <= self.last_stamp)):
            reason = 'invalid_frame_or_timestamp'
        if reason == 'ready':
            try:
                c = msg.pose.covariance
                xy, covariance = rotate_xy_covariance(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                                       [[c[0], c[1]], [c[6], c[7]]], self.settings)
            except (ValueError, ArithmeticError):
                reason = 'invalid_xy_or_covariance'
        self.last_reason = reason
        if reason != 'ready':
            self.rejected += 1
            return
        output = PoseWithCovarianceStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = 'map'
        output.pose.pose.position.x, output.pose.pose.position.y = map(float, xy)
        output.pose.pose.orientation.w = 1.0
        for index, value in zip((0, 1, 6, 7), covariance.flat):
            output.pose.covariance[index] = float(value)
        for index in (14, 21, 28, 35):
            output.pose.covariance[index] = 1e6
        self.publisher.publish(output)
        self.last_stamp = stamp
        self.published += 1


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    rclpy.init(args=args)
    node = UwbPx4Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
