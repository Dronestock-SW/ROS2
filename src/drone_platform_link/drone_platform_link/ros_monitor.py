"""ROS telemetry and opt-in assignment forwarding; no actuator command writer."""

import json
import queue
import threading
from .telemetry import SOURCES


def start(observations, config=None, mission_queue=None, stop_event=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    stop_event = stop_event if stop_event is not None else threading.Event()

    def run():
        # The asyncio main loop owns OS signals and joins this thread on exit.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=Node('platform_telemetry_monitor')
        def pose(msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            position=msg.pose.position if observations.source=='px4_local' else msg.pose.pose.position
            observations.receive_pose(position.x,position.y,msg.header.frame_id,stamp,
                now_ns=node.get_clock().now().nanoseconds,z=position.z)
        def battery(msg):
            observations.receive_battery(msg.percentage, msg.voltage, msg.present)

        kind = PoseStamped if observations.source == 'px4_local' else PoseWithCovarianceStamped
        node.create_subscription(kind, SOURCES[observations.source][0], pose,
                                 qos_profile_sensor_data)
        node.create_subscription(BatteryState, '/mavros/battery', battery,
                                 qos_profile_sensor_data)
        node.create_subscription(PoseStamped, '/uwb/btf_xyz', lambda msg: observations.receive_height(
            msg.pose.position.z, msg.header.frame_id,
            msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec), qos_profile_sensor_data)
        node.create_subscription(State, '/mavros/state', lambda msg: observations.receive_fc(
            msg.connected, msg.armed, msg.mode), qos_profile_sensor_data)
        node.create_subscription(Odometry, '/mavros/local_position/odom', lambda msg: observations.receive_position(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            msg.header.frame_id, msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec), qos_profile_sensor_data)

        def mission(msg):
            try:
                observations.receive_mission(json.loads(msg.data))
            except ValueError:
                pass

        node.create_subscription(String, '/flight_state', mission, 10)
        if mission_queue is not None:
            publisher = node.create_publisher(String, '/mission/assignment', 10)

            def forward():
                try:
                    payload = mission_queue.get_nowait()
                except queue.Empty:
                    return
                publisher.publish(String(data=json.dumps(payload)))

            node.create_timer(.05, forward)
        try:
            while not stop_event.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    thread=threading.Thread(target=run,name='platform-ros-monitor',daemon=True)
    thread.start()
    return thread
