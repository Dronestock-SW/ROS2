"""Actual ROS transport in an isolated topic namespace, with synthetic sensor data."""
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_sensors.perception import host_checks


def main():
    os.environ['ROS_DOMAIN_ID'] = '181'
    import rclpy
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
    rclpy.init()
    node = rclpy.create_node('sangwon_synthetic_sensor_' + uuid.uuid4().hex[:10])
    with tempfile.TemporaryDirectory(prefix='perception-test-', dir=ROOT / '.runtime') as name:
        test_root = Path(name).resolve()
        cfg = json.loads((ROOT / 'config/perception.observe.json').read_text())
        cfg.update(ros_domain_id=181, observation_source='SYNTHETIC_ROS_TEST')
        prefix = '/sangwon_synthetic_' + uuid.uuid4().hex
        cfg['topics'] = {key: prefix + '/' + key for key in cfg['topics']}
        path = test_root / 'config.json';path.write_text(json.dumps(cfg))
        state = test_root / 'observer';state_relative = state.relative_to(ROOT).as_posix()
        types = {'image': Image, 'camera_info': CameraInfo, 'markers': ArucoDetection, 'scanner_qr': String, 'camera_qr': String}
        publishers = {key: node.create_publisher(kind, cfg['topics'][key], 1) for key, kind in types.items()}
        log = (test_root / 'observer.log').open('wb')
        process = subprocess.Popen([sys.executable, str(ROOT / 'ops/perception_monitor.py'), '--config', str(path),
                                    '--state-dir', state_relative], stdout=log, stderr=subprocess.STDOUT,
                                    env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
        def publish():
            image = Image();image.header.stamp = node.get_clock().now().to_msg();image.header.frame_id = 'synthetic_optical_frame'
            image.width = image.height = 2;image.step = 2;image.encoding = 'mono8';image.data = [0, 0, 0, 0]
            info = CameraInfo();info.header = image.header;info.width = info.height = 2
            info.k = [1., 0., 1., 0., 1., 1., 0., 0., 1.];info.p = [1., 0., 1., 0., 0., 1., 1., 0., 0., 0., 1., 0.]
            detection = ArucoDetection();detection.header = image.header
            marker = MarkerPose();marker.marker_id = 7;marker.pose.orientation.w = 1.;marker.pose.position.z = .5
            detection.markers = [marker]
            for key, msg in (('image', image), ('camera_info', info), ('markers', detection)):
                publishers[key].publish(msg)
            for key in ('scanner_qr', 'camera_qr'):
                publishers[key].publish(String(data='SYNTHETIC_UNBOUND_QR_DO_NOT_RECORD'))
        def wait(condition, send=True, seconds=12):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if process.poll() is not None:
                    raise AssertionError('Observer stopped: ' + str(process.returncode))
                if send:publish()
                rclpy.spin_once(node, timeout_sec=.01);time.sleep(.025)
                try:
                    value = json.loads((state / 'health.json').read_text())
                    if condition(value):return value
                except (OSError, ValueError, KeyError):pass
            raise AssertionError('No matching synthetic ROS observation state')
        try:
            report = wait(lambda s: all(s['streams'][x]['status'] == 'LIVE' for x in ('image', 'camera_info', 'markers'))
                          and all(s['streams'][x]['received_count'] > 0 for x in ('scanner_qr', 'camera_qr')))
            assert report['observation_source'] == 'SYNTHETIC_ROS_TEST'
            assert report['flight_authority'] is False and report['can_start'] is False
            assert all(not report['streams'][x]['source_observation_valid'] for x in ('scanner_qr', 'camera_qr'))
            assert 'SYNTHETIC_UNBOUND_QR_DO_NOT_RECORD' not in json.dumps(report)
            assert host_checks(report, report['boot_id'], time.monotonic())[0]['status'] == 'UNKNOWN'
            old_sequence = report['monitor_seq'];old_count = report['streams']['image']['accepted_count']
            expired = wait(lambda s: s['streams']['image']['status'] == s['streams']['markers']['status'] == 'STALE', send=False, seconds=3)
            assert expired['monitor_seq'] > old_sequence
            assert expired['streams']['image']['accepted_count'] == old_count
            resumed = wait(lambda s: s['streams']['image']['status'] == 'LIVE' and s['streams']['image']['accepted_count'] > old_count)
            assert resumed['can_start'] is False
            print('PASS isolated ROS image/calibration/marker transport, legacy QR exclusion, expiry and source recovery; no flight authority')
        finally:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
            log.close()
            node.destroy_node();rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
