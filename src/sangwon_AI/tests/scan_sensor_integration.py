"""Source -> isolated ROS -> Python relay -> actual C++ action/ledger. Fake flight only."""
import copy
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_sensors.scan_window import CaptureGate
from sangwon_sensors.delivery import PendingObservations
from sangwon_web.common import encode
from sangwon_web.ipc import CoreError
from replay_scan_fixture import bundle
from web_service_integration import Rig, stop


def compact_bundle():
    raw, cfg = bundle();snapshot = json.loads(raw)
    snapshot['takeoff_z_m'] = .6;snapshot['labels'][0]['position_m']['z'] = .6
    for task in snapshot['route_tasks']:
        if task['type'] == 'waypoint':task.update(position_m={'x': 3.88, 'y': 1.5, 'z': .6}, hold_s=0)
    cfg['approved_replay_launch_pose']['position_m'].update(x=3.88)
    cfg.update(replay_scan_source='EXTERNAL_SENSOR_REPLAY', approved_replay_sensor_producer='synthetic-capture')
    cfg.pop('replay_scan_outcomes')
    for name, changes in (('motion-demo', {'arrival_stable_s': .1, 'max_speed_xy_mps': .1}),
            ('scan-demo', {'scanner_attempt_timeout_s': .25, 'camera_qr_timeout_s': 1., 'settle_s': .1})):
        key = name + '@r1';value = json.loads(cfg['replay_profile_artifacts'][key]);value.update(changes)
        text = json.dumps(value, separators=(',', ':'), sort_keys=True);cfg['replay_profile_artifacts'][key] = text
        next(ref for ref in snapshot['profile_artifacts'] if ref['id'] == name)['sha256'] = hashlib.sha256(text.encode()).hexdigest()
    raw = json.dumps(snapshot, separators=(',', ':'), sort_keys=True).encode()
    cfg['approved_replay_snapshot_sha256'] = [hashlib.sha256(raw).hexdigest()]
    return raw, cfg


def rejection(core, packet, code):
    try:core.call('scan.observe', packet)
    except CoreError as exc:assert code in str(exc), (code, str(exc))
    else:raise AssertionError('Unsafe scan observation accepted: ' + code)


def run(outcome):
    os.environ['ROS_DOMAIN_ID'] = '182'
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    raw, cfg = compact_bundle()
    with Rig(steps=1, snapshot=raw, overrides=cfg) as rig:
        # Rig's web-only subprocess environment replaces PYTHONPATH. Preserve
        # the ROS runtime discovered by the sourced setup for this ROS helper.
        rig.env['PYTHONPATH'] = str(ROOT / 'python') + os.pathsep + os.environ.get('PYTHONPATH', '')
        rclpy.init();node = rclpy.create_node('sangwon_scan_synthetic_source_' + uuid.uuid4().hex[:8])
        prefix = '/sangwon_synthetic_' + uuid.uuid4().hex
        requests, observations, receipts = prefix + '/request', prefix + '/observation', prefix + '/receipt'
        gate = CaptureGate('synthetic-capture');seen = {};active_packet = None;windows = set();camera_checked = False
        pending = PendingObservations();dropped_packet = dropped_receipt = False
        attempted_raw = [];first_qr_receipt = None;duplicate_qr_receipt = None;health_packet = None
        camera_probe = False;source_thread = None;source_stop = threading.Event();source_errors = []
        qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)
        publisher = node.create_publisher(String, observations, qos)
        node.create_subscription(String, requests, lambda msg: gate.update(json.loads(msg.data)), qos)

        def received(msg):
            nonlocal dropped_receipt, first_qr_receipt, duplicate_qr_receipt, camera_checked
            if node.count_publishers(receipts) != 1:return
            receipt = json.loads(msg.data)
            if receipt.get('kind') == 'QR_CAMERA' and receipt.get('code') == 'CAMERA_ACTIVATION_MISMATCH':camera_checked = True
            if outcome == 'SCANNER_SUCCESS' and receipt.get('kind') == 'QR_SCANNER' and receipt.get('accepted') is True:
                if not dropped_receipt:
                    dropped_receipt = True;first_qr_receipt = copy.deepcopy(receipt);return
                if receipt.get('duplicate') is True:duplicate_qr_receipt = copy.deepcopy(receipt)
            pending.receive(receipt)

        node.create_subscription(String, receipts, received,
            QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE))

        def publish():
            nonlocal active_packet, camera_checked, dropped_packet, camera_probe
            for raw_retry in pending.due():
                if outcome == 'SCANNER_SUCCESS':
                    attempted_raw.append(raw_retry)
                    if not dropped_packet:dropped_packet = True;continue
                publisher.publish(String(data=raw_retry.decode('utf-8')))
            request = gate.request
            if request is None or not request['active'] or time.monotonic() >= request['expires_monotonic_s']:return
            if outcome == 'STALE_HEALTH_RETRY':
                publisher.publish(String(data=encode(health_packet).decode('utf-8')));return
            def event(kind, data):
                packet = gate.capture(kind, data, time.monotonic())
                if kind in ('QR_SCANNER', 'QR_CAMERA'):
                    pending.offer(packet, request['source_max_age_s'][kind])
                else:publisher.publish(String(data=encode(packet).decode('utf-8')))
                return packet
            event('STATUS', {'camera_healthy': True, 'scanner_healthy': True})
            if request['observe_marker']:
                marker = copy.deepcopy(request['marker_contract']);marker.pop('nominal_reader_pose')
                marker.update(desired_vehicle_pose_map=request['marker_contract']['nominal_reader_pose'], valid=True, reprojection_px=0.1)
                event('MARKER', marker)
            if request['open_reader']:
                windows.add(request['context']['window_id'])
                if outcome == 'SCANNER_SUCCESS' and request['request_id'] not in seen:
                    active_packet = event('QR_SCANNER', {'raw': json.dumps(json.loads(raw)['qr_match_rules'][0]['expected_fields'])})
                    seen[request['request_id']] = active_packet
            if request['enable_camera']:
                if not camera_checked:
                    if not camera_probe:
                        event('QR_CAMERA', {'raw': '{}'});camera_probe = True
                    return
                event('CAMERA_ACK', {'enabled': True})
                if outcome == 'CAMERA_SUCCESS':
                    event('QR_CAMERA', {'raw': json.dumps(json.loads(raw)['qr_match_rules'][0]['expected_fields'])})

        node.create_timer(.025, publish)
        path = rig.root / 'config/bridge.json'
        path.write_text(json.dumps({'scope': 'SYNTHETIC_SENSOR_REPLAY', 'flight_authority': False,
            'core_socket': rig.core.path, 'ros_domain_id': 182, 'request_topic': requests, 'observation_topic': observations}))
        config = json.loads(path.read_text());config['receipt_topic'] = receipts;path.write_text(json.dumps(config))
        bridge = rig.launch('sensor-bridge', [sys.executable, str(ROOT / 'ops/scan_replay_bridge.py'), '--config', str(path)])
        try:
            # Discovery is observed before START; publication alone is insufficient.
            until = time.monotonic() + 10
            while node.count_publishers(requests) != 1:
                assert time.monotonic() < until, 'No scan request publisher';rclpy.spin_once(node, timeout_sec=.05)
            rig.connect();assert rig.prepare()['can_start']
            start = rig.command();assert rig.core.call('command', start)['status'] == 'ACCEPTED'
            request = rig.core.call('scan.request');gate.update(request)
            packet = gate.capture('STATUS', {'camera_healthy': True, 'scanner_healthy': True}, time.monotonic())
            altered = copy.deepcopy(packet);altered['producer_id'] = 'other';rejection(rig.core, altered, 'SCAN_PRODUCER_MISMATCH')
            altered = copy.deepcopy(packet);altered['schema_version'] = 'sangwon-scan-observation/1'
            rejection(rig.core, altered, 'SCAN_INPUT_SCOPE_MISMATCH')
            altered = copy.deepcopy(packet);altered['scope'] = 'FLIGHT';rejection(rig.core, altered, 'SCAN_INPUT_SCOPE_MISMATCH')
            altered = copy.deepcopy(packet);altered['context']['drone_id'] = 'TEST-OTHER';rejection(rig.core, altered, 'STALE_SCAN_REQUEST')
            altered = copy.deepcopy(packet);altered['observed_monotonic_s'] = time.monotonic() + 1;rejection(rig.core, altered, 'SCAN_SOURCE_TIME_INVALID')
            altered = copy.deepcopy(packet);altered['observed_monotonic_s'] = request['issued_monotonic_s'] - 1;rejection(rig.core, altered, 'SCAN_SOURCE_TIME_INVALID')
            altered = copy.deepcopy(packet);altered['observed_at'] = (dt.datetime.now(dt.timezone.utc) + dt.timedelta(seconds=1)).isoformat()
            rejection(rig.core, altered, 'SCAN_SOURCE_CLOCK_MISMATCH')
            accepted = rig.core.call('scan.observe', packet);assert accepted['accepted'] and not accepted['duplicate']
            duplicate = rig.core.call('scan.observe', packet)
            assert duplicate['duplicate'] and duplicate['accepted_monotonic_s'] == accepted['accepted_monotonic_s']
            altered = copy.deepcopy(packet);altered['data']['camera_healthy'] = False
            rejection(rig.core, altered, 'SCAN_OBSERVATION_ID_CONFLICT')
            altered = copy.deepcopy(packet);altered['observation_id'] = str(uuid.uuid4())
            rejection(rig.core, altered, 'SCAN_OBSERVATION_OUT_OF_ORDER')
            altered = copy.deepcopy(packet);altered['observation_id'] = str(uuid.uuid4());altered['sequence'] += 1;altered['observed_monotonic_s'] = request['issued_monotonic_s'] - 1
            rejection(rig.core, altered, 'SCAN_SOURCE_TIME_INVALID')
            health_packet = packet
            def spin_source():
                try:
                    while not source_stop.is_set():rclpy.spin_once(node, timeout_sec=.01)
                except BaseException as exc:source_errors.append(exc)
            # Source capture does not stop while the operator queries the daemon.
            # This models independent source processes in the field pipeline.
            source_thread = threading.Thread(target=spin_source, daemon=True);source_thread.start()
            until = time.monotonic() + 65;last_link = 0
            while True:
                assert time.monotonic() < until, 'Scan sensor mission did not finish'
                if source_errors:raise source_errors[0]
                try:
                    if time.monotonic() - last_link > .3:
                        rig.core.call('link.update', {'connected': True});last_link = time.monotonic()
                    result = rig.core.call('command.get', {'control_request_id': start['control_request_id']})
                except OSError:
                    # A read timeout is not a terminal mission state or a new START.
                    continue
                if result['status'] in ('COMPLETED', 'FAILED'):break
                time.sleep(.025)
            source_stop.set();source_thread.join(timeout=3);assert not source_thread.is_alive()
            assert result['status'] == 'COMPLETED' and result['visited'] == 3, result
            reports = [r['body'] for r in rig.core.call('outbox.list') if r['route'] == 'scan-task-results']
            assert len(reports) == 2, reports
            final = reports[-1];assert final['egress_status'] == 'COMPLETED'
            assert final['source'] == 'EXTERNAL_SENSOR_REPLAY' and final['task_status'] == 'COMPLETED'
            if outcome == 'SCANNER_SUCCESS':
                assert final['reader_source'] == 'SCANNER' and final['scanner_attempts_started'] == 1 and not final['camera_qr_used'], final
                assert dropped_packet and dropped_receipt and len(attempted_raw) >= 3
                assert all(raw_retry == attempted_raw[0] for raw_retry in attempted_raw)
                assert duplicate_qr_receipt and duplicate_qr_receipt['accepted_monotonic_s'] == first_qr_receipt['accepted_monotonic_s']
                assert pending.accepted == 1 and not pending.pending
            elif outcome == 'STALE_HEALTH_RETRY':
                assert final['failure_code'] == 'CAMERA_UNAVAILABLE' and final['scanner_attempts_started'] == 0, final
                late = rig.core.call('scan.observe', health_packet)
                assert late['duplicate'] and late['accepted_monotonic_s'] == accepted['accepted_monotonic_s']
            else:
                assert len(windows) == 3 and final['scanner_attempts_started'] == 3 and final['camera_qr_used'] and camera_checked, final
                assert final['scan_outcome'] == ('SUCCEEDED' if outcome == 'CAMERA_SUCCESS' else 'FAILED'), final
            if active_packet:
                stale = copy.deepcopy(active_packet);stale['observation_id'] = str(uuid.uuid4())
                rejection(rig.core, stale, 'SCAN_INPUT_INACTIVE')
            if outcome == 'SCANNER_SUCCESS':
                decoded = dt.datetime.fromisoformat(final['decoded_at'].replace('Z', '+00:00'))
                capture = dt.datetime.fromisoformat(active_packet['observed_at'])
                assert abs((decoded - capture).total_seconds()) < .002, 'Persistence replaced source decode time'
            status = rig.core.call('status');assert not status['physical_output_enabled'] and not status['readiness']['flight_authority']
            print('PASS source/ROS/C++ ' + outcome + '; timestamp, producer, sequence, activation, task/window and durable egress boundaries')
        finally:
            source_stop.set()
            if source_thread:source_thread.join(timeout=3)
            stop(bridge);node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    for mode in ('SCANNER_SUCCESS', 'CAMERA_SUCCESS', 'QR_UNREADABLE', 'STALE_HEALTH_RETRY'):run(mode)
    with Rig(profile='HOST_OBSERVE') as rig:
        assert not rig.core.call('scan.request')['active']
        rejection(rig.core, {}, 'SCAN_INPUT_NOT_ENABLED')
    print('PASS HOST_OBSERVE refuses scan input; no device, PX4, inventory or UWB output')
