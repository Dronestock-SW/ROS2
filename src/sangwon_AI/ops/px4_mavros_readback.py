"""Fixed diagnostic queries through an existing MAVROS router; no serial open."""
from collections import deque
import os
import time


def allowed_shell_packet(device, flags, timeout, baudrate, count, data, queries):
    """Allow only the exact read-only query list or a shell release packet."""
    if device != 10 or timeout != 0 or baudrate != 0 or len(data) != 70:
        return False
    if flags == 0 and count == 0:
        return all(value == 0 for value in data)
    if flags != 6 or not 0 < count <= 70 or any(data[count:]):
        return False
    try:
        payload = bytes(data[:count]).decode('ascii')
    except (ValueError, UnicodeDecodeError):
        return False
    return payload in {query + '\n' for query in queries}


class MavrosReadback:
    def __init__(self, domain, queries):
        if int(os.environ.get('ROS_DOMAIN_ID', '0')) != domain:
            raise RuntimeError('ROS_DOMAIN_ID must match --ros-domain')
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from mavros_msgs.msg import Mavlink, State
        from mavros.mavlink import convert_to_bytes, convert_to_rosmsg
        from pymavlink import mavutil
        self.rclpy = rclpy
        self.to_bytes, self.to_ros = convert_to_bytes, convert_to_rosmsg
        self.parser = mavutil.mavlink.MAVLink(None)
        self.encoder = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=197)
        self.queries = tuple(queries)
        self.queue = deque(maxlen=1024)
        self.state = self.heartbeat = None
        self.state_at = self.heartbeat_at = float('-inf')
        self.decode_error = None
        rclpy.init(args=[])
        self.node = rclpy.create_node('px4_fixed_sensor_readback')
        self.publisher = self.node.create_publisher(Mavlink, '/uas1/mavlink_sink', qos_profile_sensor_data)
        # The raw router also carries high-rate IMU/odometry. Preserve shell
        # response bursts instead of losing them in the sensor default depth 5.
        raw_qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Mavlink, '/uas1/mavlink_source', self.receive, raw_qos)
        self.node.create_subscription(State, '/mavros/state', self.receive_state, qos_profile_sensor_data)
        self.mav = self  # Same narrow serial_control_send interface as serial mode.

    def receive_state(self, msg):
        self.state, self.state_at = msg, time.monotonic()

    def receive(self, msg):
        if msg.sysid != 1 or msg.compid != 1 or msg.msgid not in (0, 126):
            return
        try:
            packet = self.parser.decode(self.to_bytes(msg))
        except Exception as exc:
            self.decode_error = str(exc)
            return  # Invalid frame cannot establish FC state or shell completion.
        if packet.get_type() == 'HEARTBEAT':
            self.heartbeat, self.heartbeat_at = packet, time.monotonic()
        self.queue.append(packet)

    def recv_match(self, blocking=False, timeout=0.):
        end = time.monotonic() + (timeout if blocking else 0.)
        while True:
            self.rclpy.spin_once(self.node, timeout_sec=0.)
            if self.queue:
                return self.queue.popleft()
            remaining = end - time.monotonic()
            if remaining <= 0:
                return None
            self.rclpy.spin_once(self.node, timeout_sec=min(.05, remaining))

    def wait_heartbeat(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.recv_match(blocking=True, timeout=.1)
            if (self.heartbeat is not None and self.state is not None
                    and self.publisher.get_subscription_count() == 1):
                # The source may be discovered before the router's sink.
                # Wait for discovery without weakening the per-query guard.
                return self.heartbeat
        if self.decode_error:
            raise RuntimeError('MAVROS frame decode failed: '+self.decode_error)
        return None

    def serial_control_send(self, device, flags, timeout, baudrate, count, data):
        if not allowed_shell_packet(device, flags, timeout, baudrate, count, data, self.queries):
            raise RuntimeError('Only fixed read-only queries and shell release are allowed')
        if count:
            now = time.monotonic()
            if (self.state is None or self.heartbeat is None
                    or now-self.state_at > 2.5 or now-self.heartbeat_at > 2.5
                    or not self.state.connected or self.state.armed
                    or self.heartbeat.base_mode & 128):
                raise RuntimeError('Fresh connected and disarmed FC required for every query')
            if self.publisher.get_subscription_count() != 1:
                raise RuntimeError('Expected one existing MAVROS router sink')
        packet = self.encoder.serial_control_encode(device, flags, timeout, baudrate, count, data)
        packet.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq + 1) % 256
        self.publisher.publish(self.to_ros(packet))

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
