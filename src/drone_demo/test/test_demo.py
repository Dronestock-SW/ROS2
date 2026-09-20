"""Check reusable fixtures against the production observation contract."""
from dataclasses import replace
import json
from pathlib import Path

import pytest

from drone_demo.core import DemoConfig, DemoRun
from drone_demo.export import write_demo
from drone_demo.node import observation_message, require_demo_environment
from drone_uwb.core import Processor, Settings


ROOT = Path(__file__).resolve().parents[2]
CONFIG = DemoConfig(**json.loads((ROOT / 'drone_demo/config/demo.json').read_text(encoding='utf-8')))
LAYOUT = json.loads((ROOT / 'drone_uwb/config/anchors_20260906.json').read_text(encoding='utf-8'))


def samples(config):
    run = DemoRun(config, LAYOUT)
    return [run.sample(i) for i in range(config.sample_count)]


def test_demo_height_changes_distances_but_never_becomes_uwb_altitude():
    measured_ranges = []
    for height in (0.3, 1.2, 2.8):
        config = replace(CONFIG, scenario='stationary', fixed_z_m=height,
                         duration_s=2.0, range_noise_stddev_m=0.0)
        row = samples(config)[-1]
        assert row['truth_xyz_m'] == [2.09, 1.68, height]
        assert row['observation']['x'] == pytest.approx(2.09, abs=1e-10)
        assert row['observation']['y'] == pytest.approx(1.68, abs=1e-10)
        message = observation_message(row['observation'], 'uwb_map')
        assert message.pose.pose.position.z == 0.0
        assert message.pose.covariance[14] == 1e6
        measured_ranges.append(row['received'][-1]['message']['raw_slant_m'][0])
    assert len(set(measured_ranges)) == 3


def test_gap_has_no_received_messages_or_observations_then_recovers():
    rows = samples(replace(CONFIG, scenario='gap', range_noise_stddev_m=0.0))
    gap = [r for r in rows if 4.0 <= r['time_s'] < 5.0]
    assert len(gap) == 40
    assert all(r['received'] == [] and r['observation'] is None for r in gap)
    assert all(r['decision'] == 'demo_gap' for r in gap)
    assert gap[-1]['truth_xyz_m'] != gap[0]['truth_xyz_m']
    assert any(r['observation'] for r in rows if r['time_s'] > 5.0)
    assert rows[-1]['truth_xyz_m'] == [3.09, 2.68, 1.2]
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
    {'fixed_z_m': float('nan')}, {'rate_hz': 0.0}, {'speed_m_s': -1.0},
    {'duration_s': 0.0}, {'seed': True}, {'target_xy_m': [1, float('inf')]},
])
def test_invalid_fixture_config_fails_before_publishing(changes):
    with pytest.raises(ValueError):
        replace(CONFIG, **changes)
