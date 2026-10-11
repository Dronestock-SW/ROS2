"""Read-only ROS sensor subscriber. No decoder commands, flight outputs or raw recording."""
import argparse
import datetime as dt
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_sensors.perception import PerceptionHealth, config
from sangwon_web.common import atomic_json, decode, runtime_path
from health_monitor import boot_id, notify


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def header(message):
    stamp = message.header.stamp
    if not 0 <= stamp.nanosec < 1000000000:
        return float('nan'), message.header.frame_id
    return stamp.sec + stamp.nanosec / 1e9, message.header.frame_id


def finite(values):
    return all(math.isfinite(v) for v in values)


class Observer:
    def __init__(self, health):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import String
        from aruco_opencv_msgs.msg import ArucoDetection
        self.health, self.ros = health, rclpy
        rclpy.init()
        self.node = rclpy.create_node('sangwon_perception_observer_' + str(os.getpid()))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.subscriptions = []
        for name, message_type, callback in (('image', Image, self.image), ('camera_info', CameraInfo, self.camera_info),
                ('markers', ArucoDetection, self.markers), ('scanner_qr', String, self.scanner_qr), ('camera_qr', String, self.camera_qr)):
            self.subscriptions.append(self.node.create_subscription(message_type, health.streams[name].topic, callback, qos))

    def observe(self, name, msg, metadata, valid):
        stamp, frame = header(msg)
        self.health.streams[name].stamped(stamp, frame, time.time(), time.monotonic(), metadata, valid)

    def image(self, msg):
        valid = (0 < msg.width <= 8192 and 0 < msg.height <= 8192 and 0 < msg.step <= 8192 * 16
            and len(msg.data) == msg.height * msg.step and len(msg.data) <= 64 * 1024**2 and 0 < len(msg.encoding) <= 32)
        self.observe('image', msg, {'width': msg.width, 'height': msg.height}, valid)

    def camera_info(self, msg):
        valid = (0 < msg.width <= 8192 and 0 < msg.height <= 8192 and len(msg.d) <= 16 and finite((*msg.k, *msg.d, *msg.r, *msg.p))
            and msg.k[0] > 0 and msg.k[4] > 0 and msg.p[0] > 0 and msg.p[5] > 0)
        self.observe('camera_info', msg, {'width': msg.width, 'height': msg.height}, valid)

    def markers(self, msg):
        valid = len(msg.markers) <= 128
        if not valid:
            self.observe('markers', msg, {}, False)
            return
        identities = set()
        for marker in msg.markers:
            p, q = marker.pose.position, marker.pose.orientation
            if (marker.marker_id in identities or not finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                    or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .01):
                valid = False
            identities.add(marker.marker_id)
        # A live detection array with zero markers is valid transport, not a
        # successful target acquisition. Marker dictionary/size/layout are absent.
        self.observe('markers', msg, {'marker_count': len(msg.markers)}, valid)

    def qr(self, name, msg):
        length = len(msg.data.encode('utf-8')) if len(msg.data) <= 65536 else 65537
        self.health.streams[name].unstamped_qr(length, time.monotonic())

    def scanner_qr(self, msg):
        self.qr('scanner_qr', msg)

    def camera_qr(self, msg):
        self.qr('camera_qr', msg)

    def spin(self):
        self.ros.spin_once(self.node, timeout_sec=.05)

    def graph(self):
        for stream in self.health.streams.values():
            stream.publisher_count = self.node.count_publishers(stream.topic)

    def close(self):
        self.node.destroy_node()
        self.ros.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/perception.observe.json')
    parser.add_argument('--state-dir', default='.runtime/perception')
    parser.add_argument('--ros-domain-id', type=int, help='Read-only probe override; does not alter other ROS nodes')
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--duration-s', type=float, default=3)
    args = parser.parse_args()
    configuration = decode(args.config.read_bytes())
    if args.ros_domain_id is not None:
        configuration['ros_domain_id'] = args.ros_domain_id
    config(configuration)
    if not math.isfinite(args.duration_s) or not .1 <= args.duration_s <= 30:
        raise ValueError('INVALID_PROBE_DURATION')
    os.environ['ROS_DOMAIN_ID'] = str(configuration['ros_domain_id'])
    os.umask(0o077)
    directory = runtime_path(ROOT, args.state_dir)
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (directory / 'observer.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Perception observer already owns this state directory') from None
        health = PerceptionHealth(configuration, boot_id())
        observer, code = None, 'OK'
        try:
            observer = Observer(health)
        except ImportError:
            code = 'ROS_DEPENDENCIES_UNAVAILABLE'
        running = True
        def stop(*_):
            nonlocal running
            running = False
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        start, next_report = time.monotonic(), 0
        try:
            while running:
                if observer:
                    observer.spin()
                else:
                    time.sleep(.05)
                now = time.monotonic()
                if now >= next_report:
                    if observer:
                        observer.graph()
                    report = health.snapshot(now, utc_now(), code)
                    atomic_json(directory / 'health.json', report)
                    notify(('READY=1\n' if health.sequence == 1 else '') + 'WATCHDOG=1\nSTATUS=Read-only perception transport; no flight authority')
                    next_report = now + .5
                if args.once and now - start >= args.duration_s:
                    report = health.snapshot(now, utc_now(), code)
                    atomic_json(directory / 'health.json', report)
                    print(json.dumps(report))
                    break
        finally:
            if observer:
                observer.close()
            notify('STOPPING=1\nSTATUS=Perception observer stopped')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
