"""Subscribe to observation topics only; never publish a flight command."""

import threading


def start(observations):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import BatteryState
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    def run():
        rclpy.init()
        node = Node('platform_telemetry_monitor')

        def pose(msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            position = msg.pose.pose.position
            observations.receive_pose(position.x, position.y, msg.header.frame_id, stamp)

        def battery(msg):
            observations.receive_battery(msg.percentage, msg.voltage, msg.present)

        node.create_subscription(PoseWithCovarianceStamped, '/uwb_pose', pose,
                                 qos_profile_sensor_data)
        node.create_subscription(BatteryState, '/mavros/battery', battery,
                                 qos_profile_sensor_data)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    thread = threading.Thread(target=run, name='platform-ros-monitor', daemon=True)
    thread.start()
    return thread
