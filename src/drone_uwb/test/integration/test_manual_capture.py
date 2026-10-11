"""Recording faults and fresh flight-state boundaries, without ROS/FC outputs."""
import json
from pathlib import Path

import pytest

from drone_uwb.integration.manual_capture import Capture, add_marker, atomic_json, config_evidence
from drone_uwb.integration.ros.manual_capture import FlightEnd, TOPICS, main
from drone_uwb.integration import manual_capture


def capture(tmp_path, **kwargs):
    return Capture(tmp_path/'capture', dict(required_topics=['/uwb/received'], topics=TOPICS),
                   min_free_bytes=0, **kwargs)


def test_status_does_not_call_an_old_boot_or_stale_checkpoint_active(tmp_path, monkeypatch):
    monkeypatch.setattr(manual_capture, 'boot_id', lambda:'first')
    c=capture(tmp_path)  # Initial status is written before subscribers are created.
    assert manual_capture.inspect_capture(c.directory)['recording_active']
    monkeypatch.setattr(manual_capture, 'boot_id', lambda:'second')
    assert not manual_capture.inspect_capture(c.directory)['recording_active']
    monkeypatch.setattr(manual_capture, 'boot_id', lambda:'first')
    s=json.loads((c.directory/'summary.json').read_text()); s['updated']['monotonic_ns']-=10_000_000_000
    atomic_json(c.directory/'summary.json',s)
    assert not manual_capture.inspect_capture(c.directory)['recording_active']
    c.close('test')
    assert not manual_capture.inspect_capture(c.directory)['recording_active']


def test_keeps_original_measurement_and_receipt_clocks_nonfinite_and_raw(tmp_path):
    c = capture(tmp_path)
    raw = '{"sample_time_us":[12,15],"raw_slant_m":[1.2,3.4]}'
    assert c.add('/uwb/received', 'std_msgs/msg/String', {'data':raw},
                 mono_ns=100, ros_ns=200, header_ns=None)
    assert c.add('/tof', 'sensor_msgs/msg/Range', {'range':float('nan'), 'missing':None},
                 mono_ns=300, ros_ns=500, header_ns=450)
    summary = c.close('test')
    rows = [json.loads(line) for line in (c.directory/'events.jsonl').read_text().splitlines()]
    assert rows[0]['data']['data'] == raw
    assert rows[1]['header_ns'] == 450 and rows[1]['received_ros_ns'] == 500
    assert rows[1]['data'] == {'range':'NaN', 'missing':None}
    assert summary['required_topics_missing'] == []
    assert summary['writer_drained'] and not summary['flight_authorized']
    assert not summary['alignment_confirmed'] and not summary['timing_confirmed']


def test_byte_limit_leaves_readable_prefix_and_reports_incomplete(tmp_path):
    c = capture(tmp_path, max_bytes=500)
    assert c.add('a', 'test', {'v':1}, mono_ns=1, ros_ns=1)
    assert not c.add('a', 'test', {'v':'x'*600}, mono_ns=2, ros_ns=2)
    s = c.close('limit')
    assert s['error'] == 'capture_byte_limit' and s['rejected_events'] == 1
    assert not s['writer_drained']
    rows = (c.directory/'events.jsonl').read_text().splitlines()
    assert len(rows) == 1 and json.loads(rows[0])['data']['v'] == 1


def test_writer_failure_is_not_recording_success(tmp_path):
    class FailedWriter:
        error = None
        def __init__(self, streams, **kwargs): self.streams = streams
        def write(self, *args): self.error = 'disk_fault'; return False
        def close(self):
            for stream in self.streams.values(): stream.close()
            raise OSError(self.error)
    c = capture(tmp_path, writer_factory=FailedWriter)
    assert not c.add('a', 'test', {}, mono_ns=1, ros_ns=1)
    s = c.close('failure')
    assert s['error'] == 'disk_fault' and not s['writer_drained']


def test_missing_sensors_and_mavlink_flow_are_explicit(tmp_path):
    c = capture(tmp_path)
    assert not c.report()['raw_optical_flow_received']
    c.add('/uas1/mavlink_source', 'mavros_msgs/msg/Mavlink', {'msgid':106, 'payload64':[1]},
          mono_ns=1, ros_ns=1)
    s = c.close('test')
    assert s['raw_optical_flow_received'] and s['required_topics_missing'] == ['/uwb/received']
    assert not s['independent_reference_verified'] and not s['px4_ulog_attached']


def test_gap_and_fc_transition_preserved_without_commands(tmp_path):
    c = capture(tmp_path)
    for mono,armed in [(1_000_000_000,False),(1_400_000_000,True),(1_500_000_000,False)]:
        c.add('/mavros/state', 'mavros_msgs/msg/State', dict(connected=True,armed=armed,mode='POSCTL'),
              mono_ns=mono, ros_ns=mono, header_ns=mono+1)
    s = c.close('test')
    assert [r['armed'] for r in s['state_transitions']] == [False,True,False]
    assert s['topics']['/mavros/state']['max_gap_s'] == pytest.approx(.4)
    assert s['topics']['/mavros/state']['negative_header_age_count'] == 3
    assert s['fc_commands_sent'] == s['parameter_writes'] == 0


def test_existing_session_not_overwritten_and_marker_is_annotation(tmp_path):
    c = capture(tmp_path)
    c.close('test')
    with pytest.raises(FileExistsError): capture(tmp_path)
    marker = add_marker(c.directory, '창고 +X 시작')
    assert marker['kind'] == 'operator_annotation_not_measurement'
    manifest = json.loads((c.directory/'manifest.json').read_text())
    manifest['boot_id'] = 'different_boot'
    atomic_json(c.directory/'manifest.json', manifest)
    with pytest.raises(ValueError, match='capture_host_boot'): add_marker(c.directory, 'wrong clock')


def test_configuration_snapshot_is_exact(tmp_path):
    p = tmp_path/'field.json'; p.write_text('{"alignment_confirmed":false}', encoding='utf-8')
    evidence = config_evidence([p])[0]
    assert len(evidence['sha256']) == 64
    assert evidence['content_utf8'] == p.read_text()


def update(end, now, *, armed=False, landed=1, fresh=True):
    end.observe('/mavros/state', dict(connected=True,armed=armed), now, fresh)
    end.observe('/mavros/extended_state', dict(landed_state=landed), now, fresh)


def test_landed_before_arm_and_signal_loss_do_not_end_capture():
    end = FlightEnd()
    update(end,0)
    assert not end.finished(0,5)
    update(end,1,armed=True)
    update(end,2,armed=False,landed=2)
    assert not end.finished(2,5)
    update(end,3)
    assert not end.finished(3,5)
    assert not end.finished(5,5)  # Stale state resets the grounded tail.
    for second in range(6,12):
        update(end,second)
        assert end.finished(second,5) is (second == 11)


def test_stale_arm_does_not_count_as_flight():
    end = FlightEnd()
    update(end,1,armed=True,fresh=False)
    update(end,2)
    assert not end.ever_armed and not end.finished(2,5)


def test_cli_duration_bound_and_no_control_dependencies():
    with pytest.raises(SystemExit): main(['record','--output','unused','--seconds','nan'])
    source = Path(__file__).parents[2]/'drone_uwb/integration/ros/manual_capture.py'
    text = source.read_text(encoding='utf-8')
    assert 'create_client(' not in text and 'create_publisher(' not in text
    assert 'serial.Serial' not in text and 'subprocess' not in text


def test_capture_does_not_change_command_sink_subscriber_guard():
    assert '/uas1/mavlink_source' in TOPICS
    assert '/uas1/mavlink_sink' not in TOPICS
    assert '/mavros/vision_pose/pose_cov' in TOPICS


def test_lidar_capture_is_optional_and_retains_raw_transform_topics():
    from drone_uwb.integration.ros.manual_capture import selected_topics
    assert '/scan' not in selected_topics()
    topics=selected_topics(True)
    assert topics['/scan']=='sensor_msgs/msg/LaserScan'
    assert topics['/tf_static']==topics['/tf']=='tf2_msgs/msg/TFMessage'
    assert set(TOPICS).issubset(topics)
    assert '/uas1/mavlink_sink' not in topics
