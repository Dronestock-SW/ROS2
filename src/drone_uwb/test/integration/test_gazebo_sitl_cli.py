"""Run the recorder CLI with deterministic Gazebo messages, no live simulator."""
import hashlib
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import pytest

from drone_uwb.integration.gazebo.gazebo_live_shadow import main
from test_gazebo_live_shadow import configuration, cycle, sensor_rows


@pytest.mark.parametrize('session_case', ['none', 'normal', 'stop_error'])
def test_cli_shadow_records_complete_observation_path_and_input_hashes(tmp_path, monkeypatch, session_case):
    config = configuration()
    profile, _, _ = sensor_rows(1_000_000, 1.)
    root = Path(__file__).resolve().parents[2]
    config_path, profile_path = tmp_path/'config.json', tmp_path/'height.json'
    config_path.write_text(json.dumps(config), encoding='utf-8')
    profile_path.write_text(json.dumps(profile), encoding='utf-8')
    unsubscribed = []
    lifecycle = []
    factory = None

    class Node:
        def subscribe(self, message_type, topic, callback):
            for seq, stamp in enumerate((1_000_000, 1_040_000, 1_080_000)):
                header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nsec=(stamp-1_000_000)*1000))
                if topic.endswith('/ranges'):
                    message = SimpleNamespace(header=header, data=json.dumps(
                        cycle(config, seq, stamp, [2.09, 1.68, 1.])))
                elif topic.endswith('/scan'):
                    message = SimpleNamespace(header=header, ranges=[.65], range_min=.1,
                                              range_max=12., frame='lidar')
                elif topic.endswith('/imu'):
                    message = SimpleNamespace(header=header, entity_name='imu',
                        orientation=SimpleNamespace(w=1., x=0., y=0., z=0.),
                        angular_velocity=SimpleNamespace(x=0., y=0., z=0.))
                else:
                    assert topic.endswith('/clock')
                    message = SimpleNamespace(sim=header.stamp)
                callback(message)
            return True

        def unsubscribe(self, topic):
            unsubscribed.append(topic)

    for module_name, attr, value in (
        ('gz.transport13', 'Node', Node), ('gz.msgs10.stringmsg_pb2', 'StringMsg', object),
        ('gz.msgs10.laserscan_pb2', 'LaserScan', object), ('gz.msgs10.imu_pb2', 'IMU', object),
        ('gz.msgs10.clock_pb2', 'Clock', object)):
        module = ModuleType(module_name)
        setattr(module, attr, value)
        monkeypatch.setitem(sys.modules, module_name, module)
    if session_case == 'none':
        # Shadow must work without pymavlink, including with a configured observer.
        monkeypatch.setitem(sys.modules, 'pymavlink', None)
    else:
        from test_sitl_observer import Connection, DIALECT
        link = Connection()
        link.close = lambda: lifecycle.append('connection_closed')
        connections = []
        def connect(*args, **kwargs):
            connections.append((args, kwargs))
            return link
        module = ModuleType('pymavlink')
        module.mavutil = SimpleNamespace(mavlink=DIALECT, mavlink_connection=connect)
        monkeypatch.setitem(sys.modules, 'pymavlink', module)
        def stop(reason):
            lifecycle.append(('stop', reason))
            if session_case == 'stop_error':
                raise OSError('stop failed for test')
        def factory(observer, output):
            assert observer.connection is link
            return SimpleNamespace(on_message=lambda msg: lifecycle.append('message'),
                on_observation=lambda event: lifecycle.append('observation'),
                tick=lambda: lifecycle.append('tick'), stop=stop,
                close=lambda: lifecycle.append('session_closed'))
    monkeypatch.setattr(sys, 'stdout', SimpleNamespace(write=lambda value: len(value),
                        flush=lambda: None, reconfigure=lambda **kwargs: None))
    output = tmp_path/'capture'
    arguments = ['--config', str(config_path), '--height-profile', str(profile_path),
          '--output', str(output), '--duration-s', '.08', '--holdback-ms', '0',
          '--sitl-odometry', str(root/'config/gazebo_sitl_odometry.json'),
          '--sitl-observer', str(root/'config/gazebo_sitl_observer.json')]
    if session_case != 'none':
        arguments += ['--sitl-mode', 'timesync']
    if session_case == 'stop_error':
        with pytest.raises(SystemExit) as error:
            main(arguments, session_factory=factory)
        assert error.value.code == 2
    else:
        main(arguments, session_factory=factory)
    summary = json.loads((output/'capture.json').read_text(encoding='utf-8'))
    assert summary['stop_reason'] == 'duration_complete'
    assert summary['counters']['processed'] == 3 and summary['observations_sent'] == 0
    assert summary['external_output_allowed'] is False
    events = [json.loads(line) for line in (output/'sitl_events.jsonl').read_text(encoding='utf-8').splitlines()]
    assert len(events) == 3 and all(not row['transmitted'] for row in events)
    if session_case == 'none':
        assert all(row['reason'] == 'shadow_only' for row in events)
    else:
        assert len(connections) == 1
        assert connections[0][0] == ('udpin:127.0.0.1:14540',)
        assert 'observation' in lifecycle and 'tick' in lifecycle
        assert lifecycle[-3:] == [('stop', 'duration_complete'), 'session_closed', 'connection_closed']
    assert bool(summary['cleanup_errors']) == (session_case == 'stop_error')
    assert all(row['candidate'] is not None for row in events)
    assert len(unsubscribed) == 4
    index = json.loads((output/'input_index.json').read_text(encoding='utf-8'))
    for name in ('raw_ranges.jsonl', 'tof.jsonl', 'attitude.jsonl', 'clock.jsonl', 'results.jsonl'):
        assert index[name]['sha256'] == hashlib.sha256((output/name).read_bytes()).hexdigest()
    manifest = json.loads((output/'manifest.json').read_text(encoding='utf-8'))
    assert 'integration/sitl/sitl_observer.py' in manifest['source_sha256']
