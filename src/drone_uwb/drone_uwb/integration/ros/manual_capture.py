"""Receive-only manual-flight capture, markers and status. Never opens an FC port."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

from drone_uwb.integration.manual_capture import Capture, add_marker, config_evidence, inspect_capture


TOPICS = {
    '/uwb/received': 'std_msgs/msg/String',
    '/uwb/status': 'std_msgs/msg/String',
    '/uwb/btf_decision': 'std_msgs/msg/String',
    '/uwb/btf_status': 'std_msgs/msg/String',
    '/uwb/btf_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    '/uwb/btf_xyz': 'geometry_msgs/msg/PoseStamped',
    '/uwb/bridge_status': 'std_msgs/msg/String',
    '/mavros/state': 'mavros_msgs/msg/State',
    '/mavros/extended_state': 'mavros_msgs/msg/ExtendedState',
    '/mavros/rc/in': 'mavros_msgs/msg/RCIn',
    '/mavros/imu/data': 'sensor_msgs/msg/Imu',
    '/mavros/local_position/odom': 'nav_msgs/msg/Odometry',
    '/mavros/downward_0': 'sensor_msgs/msg/Range',
    '/mavros/downward_1': 'sensor_msgs/msg/Range',
    '/mavros/timesync_status': 'mavros_msgs/msg/TimesyncStatus',
    '/mavros/estimator_status': 'mavros_msgs/msg/EstimatorStatus',
    '/mavros/vision_pose/pose_cov': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    '/mavros/px4flow/raw/optical_flow_rad': 'mavros_msgs/msg/OpticalFlowRad',
    '/uas1/mavlink_source': 'mavros_msgs/msg/Mavlink',
    # Do not subscribe to the command sink: fixed FC readback tools require
    # exactly one sink subscriber (the MAVROS router). EV intent is captured
    # above at vision_pose/pose_cov; FC receipt/fusion needs the matching ULog.
}
REQUIRED = ['/uwb/received', '/uwb/btf_decision', '/mavros/state',
            '/mavros/extended_state', '/mavros/imu/data', '/mavros/local_position/odom',
            '/mavros/timesync_status', '/mavros/downward_0', '/mavros/rc/in',
            '/uas1/mavlink_source']


class FlightEnd:
    """Automatic stop only after observed ARM, then fresh landed + DISARM."""
    def __init__(self):
        self.ever_armed = False
        self.state = self.landed = None
        self.state_at = self.landed_at = float('-inf')
        self.ground_since = None

    def observe(self, topic, data, now, fresh):
        if topic == '/mavros/state':
            self.state_at = now if fresh else float('-inf')
            self.state = data
            self.ever_armed |= fresh and data.get('armed') is True and data.get('connected') is True
        elif topic == '/mavros/extended_state':
            self.landed_at = now if fresh else float('-inf')
            self.landed = data.get('landed_state')

    def finished(self, now, tail_s):
        ground = (self.ever_armed and self.state is not None and self.state.get('connected') is True
            and self.state.get('armed') is False and self.landed == 1
            and 0 <= now-self.state_at <= 1.5 and 0 <= now-self.landed_at <= 1.5)
        if not ground:
            self.ground_since = None
            return False
        if self.ground_since is None:
            self.ground_since = now
        return now-self.ground_since >= tail_s


def record(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(args=[])
    node = Node('manual_flight_capture_'+str(os.getpid()), enable_rosout=False,
                start_parameter_services=False)
    domain = node.context.get_domain_id()
    expected_domain = {'A':1, 'B':2}[args.tag]
    if domain != expected_domain and not (domain == 99 and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
        node.destroy_node(); rclpy.shutdown()
        raise ValueError('capture_tag_domain_mismatch')
    topics = dict(TOPICS)
    # A new observer uses BEST_EFFORT so it never pressures reliable sensor writers.
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    capture = Capture(args.output, dict(tag=args.tag, tag_id={'A':'5','B':'6'}[args.tag],
        ros_domain_id=domain, topics=topics, required_topics=REQUIRED,
        source_revision=args.source_revision, operator_note=args.note,
        config_evidence=config_evidence(args.config),
        requested_seconds=args.seconds, stop_after_disarm_s=args.post_disarm_seconds,
        qos='BEST_EFFORT KEEP_LAST 100; upstream/DDS loss is possible'),
        max_bytes=args.max_mib*1024*1024, min_free_bytes=args.reserve_mib*1024*1024)
    flight_end = FlightEnd()
    stopped = []
    def stop(signum, frame):
        stopped.append('signal_'+str(signum))
    old_handlers = {s:signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}

    def callback(msg, topic, type_name):
        mono, ros = time.monotonic_ns(), node.get_clock().now().nanoseconds
        header = getattr(msg, 'header', None)
        stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec if header is not None else None
        data = message_to_ordereddict(msg)
        capture.add(topic, type_name, data, mono_ns=mono, ros_ns=ros, header_ns=stamp)
        flight_end.observe(topic, data, mono/1e9,
            stamp is not None and stamp > 0 and 0 <= ros-stamp <= 1_500_000_000)

    reason = 'duration_limit'
    subscriptions = []
    try:
        for topic, name in topics.items():
            subscriptions.append(node.create_subscription(get_message(name), topic,
                lambda msg,t=topic,n=name:callback(msg,t,n), qos))
        started = last_checkpoint = time.monotonic()
        capture.checkpoint()
        print(json.dumps(dict(recording=str(args.output), pid=os.getpid(),
             fc_commands_sent=0, flight_authorized=False)), flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if stopped:
                reason = stopped[0]; break
            if capture.stop_requested:
                reason = 'operator_stop_file'; break
            if capture.storage_error:
                reason = 'recording_error'; break
            if now-started >= args.seconds:
                break
            if flight_end.finished(now, args.post_disarm_seconds):
                reason = 'disarmed_on_ground_tail_complete'; break
            if now-last_checkpoint >= 2.:
                capture.checkpoint(); last_checkpoint = now
    except (KeyboardInterrupt, ExternalShutdownException):
        reason = 'interrupted'
    except Exception as exc:
        reason = 'capture_exception:'+type(exc).__name__
        capture.error = reason
        raise
    finally:
        summary = capture.close(reason)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        print(json.dumps(dict(summary=str(args.output/'summary.json'), reason=reason,
              error=summary['error'], missing=summary['required_topics_missing'])), flush=True)
    return 1 if summary['error'] else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('record')
    run.add_argument('--output', type=Path, required=True)
    run.add_argument('--tag', choices=['A','B'], default='B')
    run.add_argument('--seconds', type=float, default=900.)
    run.add_argument('--post-disarm-seconds', type=float, default=20.)
    run.add_argument('--max-mib', type=int, default=192)
    run.add_argument('--reserve-mib', type=int, default=512)
    run.add_argument('--config', type=Path, action='append', default=[])
    run.add_argument('--source-revision', default='unspecified')
    run.add_argument('--note', default='')
    marker = sub.add_parser('mark')
    marker.add_argument('directory', type=Path)
    marker.add_argument('label')
    status = sub.add_parser('status')
    status.add_argument('directory', type=Path)
    args = parser.parse_args(argv)
    if args.command == 'mark':
        print(json.dumps(add_marker(args.directory, args.label), ensure_ascii=False)); return 0
    if args.command == 'status':
        result = inspect_capture(args.directory)
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result['recording_active'] else 1
    if not (10 <= args.seconds <= 3600 and 5 <= args.post_disarm_seconds <= 120
            and 1 <= args.max_mib <= 1024 and args.reserve_mib >= 256):
        parser.error('seconds 10..3600; post-disarm 5..120; max MiB 1..1024; reserve >=256 MiB')
    return record(args)


if __name__ == '__main__':
    raise SystemExit(main())
