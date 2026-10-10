"""Fixed telemetry interval requests, once per DISARM connection; no flight commands."""
import time

# Same measured sensor rates as the field recorder; no global-origin polling.
RATES = ((230, 10.), (245, 5.), (31, 100.), (105, 100.),
         (132, 40.), (32, 30.), (65, 10.), (106, 10.), (331, 30.))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandLong

    class Streams(Node):
        def __init__(self):
            super().__init__('manual_observation_streams')
            self.state = None
            self.received = float('-inf')
            self.index = 0
            self.pending = None
            self.sent_at = 0.
            self.client = self.create_client(CommandLong, '/mavros/cmd/command')
            self.create_subscription(State, '/mavros/state', self.receive, qos_profile_sensor_data)
            self.create_timer(.25, self.tick)

        def receive(self, msg):
            if not msg.connected:
                self.index = 0
            self.state, self.received = msg, time.monotonic()

        def tick(self):
            now = time.monotonic()
            if self.pending is not None:
                if self.pending.done():
                    try:
                        reply = self.pending.result()
                        self.get_logger().info(f'telemetry interval ACK {reply.result}')
                    except Exception as exc:
                        self.get_logger().warning(f'telemetry interval error {type(exc).__name__}')
                    self.pending = None
                elif now-self.sent_at > 3.:
                    self.client.remove_pending_request(self.pending)
                    self.pending = None
                    self.get_logger().warning('telemetry interval ACK timeout')
                return
            if (self.state is None or not self.state.connected or self.state.armed
                    or now-self.received > 1.5 or self.index >= len(RATES)
                    or not self.client.service_is_ready()):
                return
            message, rate = RATES[self.index]
            self.index += 1
            self.sent_at = now
            # Only SET_MESSAGE_INTERVAL. No parameter set, origin, ARM or mode.
            self.pending = self.client.call_async(CommandLong.Request(
                command=511, param1=float(message), param2=1e6/rate))

    rclpy.init()
    node = Streams()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
