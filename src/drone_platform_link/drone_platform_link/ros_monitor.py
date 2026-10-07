"""Subscribe to one selected onboard source; never publish a flight command."""
import threading
from .telemetry import SOURCES


def start(observations):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import BatteryState
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    def run():
        rclpy.init()
        node=Node('platform_telemetry_monitor')
        def pose(msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            position=msg.pose.position if observations.source=='px4_local' else msg.pose.pose.position
            observations.receive_pose(position.x,position.y,msg.header.frame_id,stamp,
                now_ns=node.get_clock().now().nanoseconds,z=position.z)
        def battery(msg):
            observations.receive_battery(msg.percentage,msg.voltage,msg.present)
        kind=PoseStamped if observations.source=='px4_local' else PoseWithCovarianceStamped
        subscriptions=[node.create_subscription(kind,SOURCES[observations.source][0],pose,qos_profile_sensor_data),
            node.create_subscription(BatteryState,'/mavros/battery',battery,qos_profile_sensor_data)]
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    thread=threading.Thread(target=run,name='platform-ros-monitor',daemon=True)
    thread.start()
    return thread
