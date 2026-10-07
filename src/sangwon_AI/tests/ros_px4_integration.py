"""Isolated synthetic MAVROS messages to the actual C++ ROS subscriber; no FC output."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_sensors.px4 import host_checks


def main():
    os.environ['ROS_DOMAIN_ID'] = '183'
    import rclpy
    from mavros_msgs.msg import State, ExtendedState, RCIn
    from sensor_msgs.msg import BatteryState
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node = rclpy.create_node('sangwon_synthetic_px4_' + uuid.uuid4().hex[:8])
    with tempfile.TemporaryDirectory(prefix='px4-observe-test-', dir=ROOT / '.runtime') as name:
        test = Path(name).resolve()
        cfg = json.loads((ROOT / 'config/px4.observe.json').read_text())
        cfg.update(ros_domain_id=183, observation_source='SYNTHETIC_MAVROS_TEST')
        cfg['topics']['local_position'] = '/mavros/local_position/pose'
        cfg['max_header_age_ms']['local_position'] = 200
        prefix = '/sangwon_synthetic_' + uuid.uuid4().hex
        cfg['topics'] = {key: prefix + '/' + key for key in cfg['topics']}
        path = test / 'config.json'; path.write_text(json.dumps(cfg))
        state_dir = test / 'observer'
        types = {'state': State, 'extended_state': ExtendedState, 'battery': BatteryState, 'rc': RCIn, 'local_position': PoseStamped}
        publishers = {key: node.create_publisher(kind, cfg['topics'][key], 1) for key, kind in types.items()}
        log = (test / 'observer.log').open('wb')
        args = [sys.argv[1], '--root', str(ROOT), '--config', str(path), '--state-dir', str(state_dir.relative_to(ROOT)), '--duration-s', '30']
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, env={**os.environ, 'NOTIFY_SOCKET': ''})
        fixed_stamp = None
        percentage = float('nan')

        def publish():
            stamp = fixed_stamp or node.get_clock().now().to_msg()
            s = State(); s.header.stamp = stamp; s.connected = True; s.armed = False; s.mode = 'POSCTL'; s.system_status = 3
            e = ExtendedState(); e.header.stamp = stamp; e.landed_state = 1
            b = BatteryState(); b.header.stamp = stamp; b.voltage = 16.2; b.current = float('nan')
            b.present = True; b.percentage = percentage; b.serial_number = 'SYNTH_PRIVATE_SERIAL_NOT_TO_REPORT'
            r = RCIn(); r.header.stamp = stamp; r.channels = [1500] * 8; r.rssi = 255
            p = PoseStamped(); p.header.stamp = stamp; p.header.frame_id = 'map'
            p.pose.position.x = 1.; p.pose.position.y = 2.; p.pose.position.z = .4; p.pose.orientation.w = 1.
            for key, msg in (('state', s), ('extended_state', e), ('battery', b), ('rc', r), ('local_position', p)):
                publishers[key].publish(msg)

        def wait(condition, send=True, seconds=8):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if process.poll() is not None:
                    raise AssertionError('Observer stopped; exit=' + str(process.returncode))
                if send:
                    publish()
                rclpy.spin_once(node, timeout_sec=.01); time.sleep(.03)
                try:
                    value = json.loads((state_dir / 'health.json').read_text())
                    if condition(value):
                        return value
                except (OSError, ValueError, KeyError):
                    pass
            raise AssertionError('No matching C++ MAVROS observation state')

        try:
            r = wait(lambda value: all(s['status'] == 'LIVE' for s in value['streams'].values()))
            assert r['streams']['battery']['fields']['percentage'] is None
            assert all(r[k] is False for k in ('can_start', 'flight_authority', 'physical_output_enabled', 'source_sample_time_verified', 'aircraft_identity_verified'))
            assert host_checks(r, r['boot_id'], time.monotonic())[0]['status'] == 'UNKNOWN'
            assert 'SYNTH_PRIVATE_SERIAL_NOT_TO_REPORT' not in json.dumps(r)
            assert (state_dir / 'health.json').stat().st_mode & 0o777 == 0o600
            observer = next(n for n in node.get_node_names() if n.startswith('sangwon_px4_observer_'))
            assert node.get_publisher_names_and_types_by_node(observer, '/') == []
            assert node.get_service_names_and_types_by_node(observer, '/') == []
            assert node.get_client_names_and_types_by_node(observer, '/') == []
            other = subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5, check=False)
            assert other.returncode == 2
            fixed_stamp = node.get_clock().now().to_msg()
            duplicated = wait(lambda value: value['streams']['state']['code'] == 'HEADER_TIMESTAMP_NOT_NEW')
            accepted = duplicated['streams']['state']['accepted_count']
            repeated = wait(lambda value: value['monitor_seq'] > duplicated['monitor_seq'] and value['streams']['state']['code'] == 'HEADER_TIMESTAMP_NOT_NEW')
            assert repeated['streams']['state']['accepted_count'] == accepted
            assert repeated['streams']['state']['fields'] == {}
            fixed_stamp = None
            recovered = wait(lambda value: value['streams']['state']['status'] == 'LIVE')
            assert recovered['streams']['state']['accepted_count'] > accepted
            percentage = 1.5
            invalid = wait(lambda value: value['streams']['battery']['code'] == 'INVALID_MAVROS_MESSAGE')
            assert not invalid['streams']['battery']['transport_observation_valid']
            percentage = float('nan')
            wait(lambda value: value['streams']['battery']['status'] == 'LIVE')
            extra = node.create_publisher(State, cfg['topics']['state'], 1)
            wait(lambda value: value['streams']['state']['publisher_count'] == 2 and value['streams']['state']['fields'] == {})
            node.destroy_publisher(extra)
            live = wait(lambda value: value['streams']['state']['status'] == 'LIVE')
            expired = wait(lambda value: all(s['status'] == 'STALE' for s in value['streams'].values()), send=False, seconds=4)
            assert expired['streams']['state']['accepted_count'] == live['streams']['state']['accepted_count'] or expired['streams']['state']['accepted_count'] > live['streams']['state']['accepted_count']
            count = expired['streams']['state']['accepted_count']
            still = wait(lambda value: value['monitor_seq'] > expired['monitor_seq'], send=False, seconds=2)
            assert still['streams']['state']['accepted_count'] == count
            wait(lambda value: all(s['status'] == 'LIVE' for s in value['streams'].values()))
            print('PASS C++ MAVROS transport: subscription-only, singleton/private report, NaN/RC unknown, duplicate/stale/ambiguous publisher rejection and recovery; no flight authority')
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=3)
            log.close(); node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
