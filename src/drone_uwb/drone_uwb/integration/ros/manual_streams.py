"""Fixed telemetry interval requests, once per DISARM connection; no flight commands."""
import time

# TFmini Plus target: sensor 100 Hz, PX4 reads 50 Hz, DISTANCE_SENSOR 50 Hz.
# These are requests, not evidence of the measured rates. No global-origin polling.
RATES = ((230, 10.), (245, 5.), (31, 100.), (105, 100.),
         (132, 50.), (32, 30.), (65, 10.), (106, 10.), (331, 30.))
ACK_TIMEOUT_S = 3.


class IntervalRequests:
    """Bounded, serialized requests; an ACK confirms acceptance, not output Hz."""
    def __init__(self, rates=RATES):
        self.rates = rates
        self.reset()

    def reset(self):
        self.index = 0
        self.attempts = 0
        self.retry_at = 0.

    def begin(self, now):
        if self.index >= len(self.rates) or now < self.retry_at:
            return None
        self.attempts += 1
        return self.rates[self.index]

    def finish(self, accepted, now):
        if accepted or self.attempts >= 3:
            outcome = 'accepted' if accepted else 'exhausted'
            self.index += 1
            self.attempts = 0
            self.retry_at = 0.
            return outcome
        self.retry_at = now + 1.
        return 'retry'


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
            self.requests = IntervalRequests()
            self.pending = None
            self.pending_message = None
            self.sent_at = 0.
            self.client = self.create_client(CommandLong, '/mavros/cmd/command')
            self.create_subscription(State, '/mavros/state', self.receive, qos_profile_sensor_data)
            self.create_timer(.25, self.tick)

        def receive(self, msg):
            if not msg.connected:
                if self.pending is not None and not self.pending.done():
                    self.client.remove_pending_request(self.pending)
                    self.pending.cancel()
                self.pending = None
                self.pending_message = None
                self.requests.reset()
            self.state, self.received = msg, time.monotonic()

        def finish(self, accepted, now, detail):
            outcome = self.requests.finish(accepted, now)
            message = (f'telemetry interval msg={self.pending_message} '
                       f'{outcome}: {detail}; actual rate unverified')
            if accepted:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            self.pending = None
            self.pending_message = None

        def tick(self):
            now = time.monotonic()
            if self.pending is not None:
                if self.pending.done():
                    try:
                        reply = self.pending.result()
                        # success=False can carry a default result=0 on failure.
                        self.finish(bool(reply.success) and reply.result == 0,
                                    now, f'ACK {reply.result}, success={reply.success}')
                    except Exception as exc:
                        self.finish(False, now, type(exc).__name__)
                elif now-self.sent_at > ACK_TIMEOUT_S:
                    self.client.remove_pending_request(self.pending)
                    self.pending.cancel()
                    self.finish(False, now, 'ACK timeout')
                return
            if (self.state is None or not self.state.connected or self.state.armed
                    or now-self.received > 1.5
                    or not self.client.service_is_ready()):
                return
            request = self.requests.begin(now)
            if request is None:
                return
            message, rate = request
            self.pending_message = message
            self.sent_at = now
            # Only SET_MESSAGE_INTERVAL. No parameter set, origin, ARM or mode.
            try:
                self.pending = self.client.call_async(CommandLong.Request(
                    command=511, param1=float(message), param2=1e6/rate))
            except Exception as exc:
                self.finish(False, now, type(exc).__name__)

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
