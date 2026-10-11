"""Relay framed synthetic ROS scan observations into private C++ IPC. No device output."""
import argparse
import fcntl
import hashlib
import os
import signal
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_web.common import decode, encode
from sangwon_web.ipc import CoreClient, CoreError


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    cfg = decode(args.config.read_bytes())
    if (cfg['scope'] != 'SYNTHETIC_SENSOR_REPLAY' or cfg['flight_authority'] is not False
            or not 170 <= cfg['ros_domain_id'] <= 199
            or any(not cfg[k].startswith('/sangwon_synthetic_') for k in ('request_topic', 'observation_topic', 'receipt_topic'))
            or len({cfg[k] for k in ('request_topic', 'observation_topic', 'receipt_topic')}) != 3):
        raise ValueError('ISOLATED_SCAN_REPLAY_REQUIRED')
    os.environ['ROS_DOMAIN_ID'] = str(cfg['ros_domain_id'])
    core = CoreClient(cfg['core_socket'], timeout=.4)
    status = core.call('status')
    if (status['profile'] != 'REPLAY' or not status['drone_id'].startswith('TEST-')
            or status['physical_output_enabled'] is not False or status['readiness']['flight_authority'] is not False):
        raise ValueError('ISOLATED_SCAN_REPLAY_REQUIRED')
    lock = (Path(cfg['core_socket']).parent / 'scan_bridge.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('sangwon_scan_replay_bridge')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(String, cfg['request_topic'], qos)
    receipts = node.create_publisher(String, cfg['receipt_topic'],
        QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE))
    counters = {'accepted': 0, 'rejected': 0};reasons = {}

    def receive(msg):
        packet = None;raw = msg.data.encode('utf-8');receipt = None
        try:
            if len(raw) > 128 * 1024:raise CoreError('SCAN_INPUT_TOO_LARGE')
            packet = decode(raw)
            if node.count_publishers(cfg['observation_topic']) != 1:raise CoreError('SCAN_SOURCE_NOT_UNIQUE')
            # This relay preserves source time/context. It cannot turn a String
            # from the old /qr_reader/data or /qr/data topics into a valid scan.
            receipt = core.call('scan.observe', packet)
            receipt['status'] = 'INGRESS_ACCEPTED'
            counters['accepted'] += 1
        except (ValueError, CoreError, OSError) as exc:
            counters['rejected'] += 1
            code = str(exc) if isinstance(exc, CoreError) else type(exc).__name__
            if len(code) <= 64 and code.replace('_', '').isalnum():
                reasons[code] = reasons.get(code, 0) + 1
            transient = isinstance(exc, OSError) or (isinstance(exc, CoreError)
                and str(exc) in ('IPC_INCOMPLETE_RESPONSE', 'IPC_RESPONSE_MISMATCH', 'IPC_CLOSED', 'IPC_TIMEOUT'))
            receipt = {'schema_version': 'sangwon-scan-receipt/1', 'scope': 'SYNTHETIC_SENSOR_REPLAY',
                'flight_authority': False, 'accepted': False, 'acceptance_scope': 'INPUT_VALIDATION_ONLY',
                'status': 'RETRYABLE' if transient else 'REJECTED', 'code': 'CORE_UNAVAILABLE' if transient else code}
        # No raw QR/image data appears in receipts or logs. Bind the exact bytes
        # actually delivered over ROS; the source retries those bytes unchanged.
        keys = ('observation_id', 'request_id', 'context', 'producer_id', 'kind', 'sequence')
        if isinstance(packet, dict) and all(k in packet for k in keys) and node.context.ok():
            receipt.update({k: packet[k] for k in keys})
            receipt['transport_sha256'] = hashlib.sha256(raw).hexdigest()
            receipts.publish(String(data=encode(receipt).decode('utf-8')))

    def request():
        try:
            message = encode(core.call('scan.request')).decode('utf-8')
            if node.context.ok():publisher.publish(String(data=message))
        except (CoreError, OSError):
            # No cached request publication when the authority process is absent.
            pass

    node.create_subscription(String, cfg['observation_topic'], receive,
        QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_timer(.025, request)
    running = True
    def stop(signum, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop);signal.signal(signal.SIGINT, stop)
    try:
        while running and rclpy.ok():rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lock.close()
        print('SCAN_REPLAY_BRIDGE accepted={} rejected={} codes={} flight_authority=false'.format(counters['accepted'], counters['rejected'], reasons))


if __name__ == '__main__':
    main()
