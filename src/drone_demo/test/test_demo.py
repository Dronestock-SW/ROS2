"""Check XY fixtures and measured ToF readout without inventing altitude."""
from dataclasses import replace
import csv
import json
from pathlib import Path

import pytest

from drone_demo.core import DemoConfig, DemoRun
from drone_demo.export import write_demo
from drone_demo.node import observation_message, pose_message, require_demo_environment
from drone_demo.tof_readout import TofReadout
from builtin_interfaces.msg import Time
from drone_uwb.processing.solvers.observations import Settings


ROOT = Path(__file__).resolve().parents[2]
CONFIG = DemoConfig(**json.loads((ROOT / 'drone_demo/config/demo.json').read_text(encoding='utf-8')))
LAYOUT = json.loads((ROOT / 'drone_uwb/config/anchors/anchors_20261004.json').read_text(encoding='utf-8'))


def samples(config):
    run = DemoRun(config, LAYOUT)
    return [run.sample(i) for i in range(config.sample_count)]


def test_xy_demo_has_no_generated_height_or_ranges():
    rows = samples(replace(CONFIG, scenario='stationary', duration_s=8.0,
                           xy_noise_stddev_m=0.0))
    for row in rows:
        assert row['truth_xy_m'] == CONFIG.start_xy_m
        assert row['z_m'] is None
        assert row['z_source'] == 'unobserved'
        assert row['z_measured'] is False
        assert 'truth_xyz_m' not in row and 'received' not in row
        assert row['observation']['source_mode'] == 'demo_xy'
        assert [row['observation'][k] for k in ('x', 'y')] == CONFIG.start_xy_m
        message = observation_message(row['observation'], 'uwb_map')
        assert message.pose.pose.position.z == 0.0
        assert message.pose.covariance[14] == 1e6
        assert pose_message(row['truth_xy_m'], Time(), 'uwb_map').pose.position.z == 0.0


def test_gap_has_no_received_messages_or_observations_then_recovers():
    rows = samples(replace(CONFIG, scenario='gap', xy_noise_stddev_m=0.0))
    gap = [r for r in rows if 4.0 <= r['time_s'] < 5.0]
    assert len(gap) == 40
    assert all(r['observation'] is None for r in gap)
    assert all(r['decision'] == 'demo_gap' for r in gap)
    assert gap[-1]['truth_xy_m'] != gap[0]['truth_xy_m']
    assert any(r['observation'] for r in rows if r['time_s'] > 5.0)
    assert rows[-1]['truth_xy_m'][:2] == [3.09, 2.68]
    assert rows[-1]['trajectory_phase'] == 'ARRIVED'


def test_export_is_repeatable_and_contains_only_xy_fixtures(tmp_path):
    config = replace(CONFIG, scenario='gap', duration_s=6.0)
    first, second = tmp_path / 'first', tmp_path / 'second'
    write_demo(first, config, LAYOUT, Settings())
    write_demo(second, config, LAYOUT, Settings())
    for name in ('samples.jsonl', 'poses.csv', 'summary.json'):
        assert (first / name).read_bytes() == (second / name).read_bytes()
    assert not (first / 'received.jsonl').exists()
    rows = [json.loads(line) for line in (first / 'samples.jsonl').read_text(
        encoding='utf-8').splitlines()]
    assert rows == samples(config)
    with (first / 'poses.csv').open(encoding='utf-8') as stream:
        records = list(csv.DictReader(stream))
    assert len(records) == config.sample_count
    assert all('z' not in key for key in records[0])
    assert all(r['uwb_x_m'] == '' for r in records if 4 <= float(r['time_s']) < 5)
    with pytest.raises(FileExistsError):
        write_demo(first, config, LAYOUT, Settings())


@pytest.mark.parametrize('domain', ('', '0', '1', '2', '98'))
def test_ros_publisher_refuses_non_demo_domain(domain):
    with pytest.raises(ValueError):
        require_demo_environment({'ROS_DOMAIN_ID': domain, 'ROS_LOCALHOST_ONLY': '1'})
    with pytest.raises(ValueError):
        require_demo_environment({'ROS_DOMAIN_ID': '99'})
    require_demo_environment({'ROS_DOMAIN_ID': '99', 'ROS_LOCALHOST_ONLY': '1'})


@pytest.mark.parametrize('changes', [
    {'xy_noise_stddev_m': float('nan')}, {'rate_hz': 0.0}, {'speed_m_s': -1.0},
    {'duration_s': 0.0}, {'seed': True}, {'target_xy_m': [1, float('inf')]},
])
def test_invalid_fixture_config_fails_before_publishing(changes):
    with pytest.raises(ValueError):
        replace(CONFIG, **changes)


def test_tof_missing_fresh_duplicate_stale_and_recovery():
    tof = TofReadout(timeout_s=0.2)
    assert tof.snapshot(1_000_000_000, 2_000_000_000)['range_m'] is None
    assert tof.observe(1.37, 0.1, 4.0, 2_000_000_000, 'tof_link',
                       1_000_000_000, 2_050_000_000)
    value = tof.snapshot(1_050_000_000, 2_100_000_000)
    assert value['range_m'] == 1.37 and value['available']
    assert value['quantity'] == 'downward_range'
    assert value['age_s'] == pytest.approx(0.1)
    assert not tof.observe(1.5, 0.1, 4.0, 2_000_000_000, 'tof_link',
                           1_100_000_000, 2_150_000_000)
    assert tof.snapshot(1_100_000_000, 2_150_000_000)['range_m'] == 1.37
    assert tof.snapshot(1_200_000_000, 2_250_000_000)['range_m'] is None
    assert tof.observe(1.4, 0.1, 4.0, 2_300_000_000, 'tof_link',
                       1_300_000_000, 2_300_000_000)
    assert tof.snapshot(1_300_000_000, 2_300_000_000)['range_m'] == 1.4


@pytest.mark.parametrize('distance,stamp,frame', [
    (float('nan'), 2_000_000_000, 'tof_link'),
    (float('inf'), 2_000_000_000, 'tof_link'),
    (0.05, 2_000_000_000, 'tof_link'),
    (5.0, 2_000_000_000, 'tof_link'),
    (1.2, 2_100_000_000, 'tof_link'),
    (1.2, 1_000_000_000, 'tof_link'),
    (1.2, 0, 'tof_link'),
    (1.2, 2_000_000_000, ''),
])
def test_bad_tof_clears_readout_without_synthetic_fallback(distance, stamp, frame):
    tof = TofReadout()
    tof.observe(1.3, 0.1, 4.0, 1_950_000_000, 'tof_link', 900_000_000, 1_950_000_000)
    assert not tof.observe(distance, 0.1, 4.0, stamp, frame, 1_000_000_000, 2_000_000_000)
    value = tof.snapshot(1_000_000_000, 2_000_000_000)
    assert not value['available'] and value['range_m'] is None


def test_removed_virtual_height_settings_are_rejected():
    from dataclasses import asdict
    with pytest.raises(TypeError, match='z_min_m'):
        DemoConfig(**asdict(CONFIG), z_min_m=0.2)
