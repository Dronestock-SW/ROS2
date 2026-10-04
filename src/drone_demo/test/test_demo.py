"""Check reusable fixtures against the production observation contract."""
from dataclasses import replace
import json
from pathlib import Path

import pytest

from drone_demo.core import DemoConfig, DemoRun
from drone_demo.export import write_demo
from drone_demo.node import observation_message, require_demo_environment
from drone_demo.z_gate import DemoZGate
from drone_uwb.processing.solvers.observations import Processor, Settings


ROOT = Path(__file__).resolve().parents[2]
CONFIG = DemoConfig(**json.loads((ROOT / 'drone_demo/config/demo.json').read_text(encoding='utf-8')))
LAYOUT = json.loads((ROOT / 'drone_uwb/config/anchors/anchors_20261004.json').read_text(encoding='utf-8'))


def samples(config):
    run = DemoRun(config, LAYOUT)
    return [run.sample(i) for i in range(config.sample_count)]


def test_demo_sine_height_bounds_and_uwb_altitude_stays_unobserved():
    config = replace(CONFIG, scenario='stationary', duration_s=8.0,
                     range_noise_stddev_m=0.0)
    rows = samples(config)
    for index, expected in ((0, 1.2), (80, 2.2), (160, 1.2), (240, 0.2)):
        row = rows[index]
        assert row['truth_xyz_m'][2] == pytest.approx(expected)
        assert row['z_source'] == 'demo_sine'
        assert row['z_measured'] is False
        if row['observation']:
            message = observation_message(row['observation'], 'uwb_map')
            assert message.pose.pose.position.z == 0.0
            assert message.pose.covariance[14] == 1e6
    assert all(0.2 - 1e-12 <= row['truth_xyz_m'][2] <= 2.2 + 1e-12 for row in rows)
    assert rows[0]['received'][-1]['message']['raw_slant_m'][0] != rows[80]['received'][-1]['message']['raw_slant_m'][0]


def test_gap_has_no_received_messages_or_observations_then_recovers():
    rows = samples(replace(CONFIG, scenario='gap', range_noise_stddev_m=0.0))
    gap = [r for r in rows if 4.0 <= r['time_s'] < 5.0]
    assert len(gap) == 40
    assert all(r['received'] == [] and r['observation'] is None for r in gap)
    assert all(r['decision'] == 'demo_gap' for r in gap)
    assert gap[-1]['truth_xyz_m'] != gap[0]['truth_xyz_m']
    assert any(r['observation'] for r in rows if r['time_s'] > 5.0)
    assert rows[-1]['truth_xyz_m'][:2] == [3.09, 2.68]
    assert rows[-1]['trajectory_phase'] == 'ARRIVED'


def test_export_is_repeatable_and_replays_through_original_processor(tmp_path):
    config = replace(CONFIG, scenario='gap', duration_s=6.0)
    settings = Settings()
    first, second = tmp_path / 'first', tmp_path / 'second'
    write_demo(first, config, LAYOUT, settings)
    write_demo(second, config, LAYOUT, settings)
    for name in ('samples.jsonl', 'received.jsonl', 'poses.csv', 'summary.json'):
        assert (first / name).read_bytes() == (second / name).read_bytes()
    processor = Processor(LAYOUT, settings)
    replayed = []
    for line in (first / 'received.jsonl').read_text(encoding='utf-8').splitlines():
        row = json.loads(line)
        decision = processor.process(row['message'], row['host_received_monotonic_ns'],
                                     row['host_received_ros_ns'])
        if decision.observation:
            replayed.append((decision.observation.seq, decision.observation.x, decision.observation.y))
    original = [r['observation'] for r in samples(config) if r['observation']]
    assert replayed == [(o['seq'], o['x'], o['y']) for o in original]
    with pytest.raises(FileExistsError):
        write_demo(first, config, LAYOUT, settings)


@pytest.mark.parametrize('domain', ('', '0', '1', '2', '98'))
def test_ros_publisher_refuses_non_demo_domain(domain):
    with pytest.raises(ValueError):
        require_demo_environment({'ROS_DOMAIN_ID': domain, 'ROS_LOCALHOST_ONLY': '1'})
    with pytest.raises(ValueError):
        require_demo_environment({'ROS_DOMAIN_ID': '99'})
    require_demo_environment({'ROS_DOMAIN_ID': '99', 'ROS_LOCALHOST_ONLY': '1'})


@pytest.mark.parametrize('changes', [
    {'z_min_m': float('nan')}, {'z_max_m': 0.1}, {'z_period_s': 0.0}, {'rate_hz': 0.0}, {'speed_m_s': -1.0},
    {'duration_s': 0.0}, {'seed': True}, {'target_xy_m': [1, float('inf')]},
])
def test_invalid_fixture_config_fails_before_publishing(changes):
    with pytest.raises(ValueError):
        replace(CONFIG, **changes)


def test_real_tof_gate_latches_after_valid_measurement():
    gate = DemoZGate()
    assert not gate.observe(float('nan'), 0.1, 4.0)
    assert not gate.observe(0.05, 0.1, 4.0)
    assert gate.observe(1.2, 0.1, 4.0)
    assert gate.reason == 'real_tof_detected'
    assert gate.observe(float('nan'), 0.1, 4.0)
    assert DemoZGate(enabled=False).blocked
