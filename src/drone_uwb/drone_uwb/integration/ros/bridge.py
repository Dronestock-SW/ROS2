"""Forward verified horizontal observations to MAVROS; disabled by default.

No flight commands, parameter writes, or PX4 pose/attitude feedback.
Configuration is immutable after startup; restart to apply surveyed values.
"""
from dataclasses import asdict
import json
import math
import os
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State, TimesyncStatus
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String

from drone_uwb.integration.ros.frames import BridgeSettings, gate, observation_xy

PARAMETERS = ('EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD',
              'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z')


class UwbPx4Bridge(Node):
    def __init__(self):
        super().__init__('uwb_px4_bridge')
        self.declare_parameter('test_mode', False, ParameterDescriptor(read_only=True))
        self.test_mode = self.get_parameter('test_mode').value
        if self.test_mode and not (self.context.get_domain_id() == 99
                                   and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
            raise ValueError('bridge_test_mode_requires_local_domain_99')
        for name, value in asdict(BridgeSettings()).items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.settings = BridgeSettings(**{k: self.get_parameter(k).value for k in asdict(BridgeSettings())})
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        self.status_pub = self.create_publisher(String, '/uwb/bridge_status', 10)
        self.input_topic = self.settings.input_topic
        self.create_subscription(PoseWithCovarianceStamped, self.input_topic, self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self.receive_state, qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus, '/mavros/timesync_status', self.receive_sync, qos_profile_sensor_data)
        self.client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.future = None
        self.request_time = 0.0
        self.params = {}
        self.param_time = self.state_time = float('-inf')
        self.connected = False
        self.armed = False
        self.sync_count = 0
        self.sync_time = float('-inf')
        self.sync_offset = None
        self.sync_remote_ns = 0
        self.last_stamp = None
        self.published = self.rejected = 0
        self.last_reason = 'starting'
        self.create_timer(0.1, self.monitor)

    def receive_state(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if stamp <= 0 or not 0 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= self.settings.state_timeout_s:
            self.connected = False
            self.sync_count = 0
            return
        if not msg.connected or not self.connected:
            self.params = {}
            self.param_time = float('-inf')
            self.sync_count = 0
        self.connected, self.state_time = msg.connected, time.monotonic()
        self.armed = msg.armed

    def receive_sync(self, msg):
        now = time.monotonic()
        valid = (msg.remote_timestamp_ns > self.sync_remote_ns and math.isfinite(msg.round_trip_time_ms)
                 and 0 <= msg.round_trip_time_ms <= 20)
        stable = (now-self.sync_time < .5 and self.sync_offset is not None
                  and abs(msg.estimated_offset_ns-self.sync_offset) <= 5_000_000)
        self.sync_count = self.sync_count+1 if valid and stable else int(valid)
        self.sync_time, self.sync_offset = now, msg.estimated_offset_ns
        self.sync_remote_ns = msg.remote_timestamp_ns

    def current_gate(self):
        now = time.monotonic()
        result = gate(self.settings, self.connected, now-self.state_time, self.params,
                      now-self.param_time, armed=self.armed)
        if result != 'ready':
            return result
        if not self.test_mode and self.context.get_domain_id() != self.settings.ros_domain_id:
            return 'tag_domain_mismatch'
        if self.sync_count < 30 or not 0 <= now-self.sync_time < .5:
            return 'timesync_unavailable_or_unstable'
        if self.count_publishers(self.input_topic) != 1 or self.count_publishers('/mavros/state') != 1:
            return 'ambiguous_or_missing_publisher'
        return 'ready'

    def monitor(self):
        # MAVROS exposes its FC parameter mirror. This node never changes it.
        # The operator must verify it against the FC after external configuration changes.
        now = time.monotonic()
        if self.future is not None and now - self.request_time > 2.0:
            self.client.remove_pending_request(self.future)
            self.future = None
        if (self.connected and self.future is None and now-self.request_time >= 1.0
                and self.client.service_is_ready()):
            request = GetParameters.Request(names=list(PARAMETERS))
            self.request_time = now
            self.future = self.client.call_async(request)
            self.future.add_done_callback(self.parameters_received)
        self.status_pub.publish(String(data=json.dumps({
            'gate': self.current_gate(), 'last_reason': self.last_reason,
            'published': self.published, 'rejected': self.rejected,
            'last_observation_stamp_ns': self.last_stamp,
            'parameters': self.params, 'settings': asdict(self.settings),
            'input_topic': self.input_topic, 'position_reference': 'uwb_antenna',
            'lever_arm_owner': 'PX4_EKF2', 'z_observed': False,
            'flight_commands_enabled': False, 'fusion_verified': False,
        }, ensure_ascii=False)))

    def parameters_received(self, future):
        if future is not self.future:
            return
        self.future = None
        if future.exception() is not None:
            return
        self.params = {name: val.integer_value if val.type == 2 else val.double_value if val.type == 3 else None
                       for name, val in zip(PARAMETERS, future.result().values)}
        self.param_time = time.monotonic()

    def receive_pose(self, msg):
        reason = self.current_gate()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if reason == 'ready':
            try:
                c = msg.pose.covariance
                xy, covariance = observation_xy(self.settings, frame=msg.header.frame_id,
                    stamp_ns=stamp, now_ns=self.get_clock().now().nanoseconds,
                    last_stamp_ns=self.last_stamp, x=msg.pose.pose.position.x,
                    y=msg.pose.pose.position.y, covariance=[[c[0], c[1]], [c[6], c[7]]])
            except (ValueError, ArithmeticError) as exc:
                reason = 'invalid_frame_or_timestamp' if str(exc) == 'invalid_frame_or_timestamp' else 'invalid_xy_or_covariance'
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
    from .lifecycle import init_for_main
    init_for_main(args)
    node = UwbPx4Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
