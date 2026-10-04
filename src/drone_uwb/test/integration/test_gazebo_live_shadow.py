"""Live UWB comparison uses sensor records and never needs truth poses."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from drone_uwb.processing.experiments.gazebo_live_shadow import GazeboLiveShadow
from drone_uwb.integration.gazebo.gazebo_live_shadow import raw_record, with_callback_clock


ROOT = Path(__file__).resolve().parents[2]
CONFIG = json.loads((ROOT/'config/gazebo_shadow.json').read_text(encoding='utf-8'))
PROFILE = json.loads((ROOT/'config/gazebo_sensor_height_profile.json').read_text(encoding='utf-8'))


def configuration():
    config = deepcopy(CONFIG)
    config['faults'] = []
    config['tag_offset_body_flu_m'] = [0., 0., .3]
    return config


def cycle(config, seq, stamp, xyz):
    anchors = np.asarray(config['anchors_xyz_m'])
    raw = np.linalg.norm(anchors-np.asarray(xyz), axis=1)+np.asarray(config['bias_m'])
    return dict(schema=1, source='simulation', type='sim_uwb_cycle',
                clock_domain='gazebo_sim_us', seq=seq, time_us=stamp,
                sample_time_us=[stamp]*4, anchor_order=['A1', 'A2', 'A3', 'A4'],
                raw_slant_m=raw.tolist(), valid_mask=15, range_bias_applied=False,
                external_output_allowed=False)


def sensor_rows(stamp, tag_z):
    profile = deepcopy(PROFILE)
    profile.update(orientation_alignment_confirmed=True, flat_floor_confirmed=True)
    # The body sits 0.3 m below the antenna. With a 0.05 m downward
    # rangefinder offset, the measured range is body height minus 0.05 m.
    tof_distance = tag_z-.3-.05
    tof = dict(schema=1, source='gazebo_sensor', type='tof_sample',
               clock_domain='gazebo_sim_us', time_us=stamp, valid=True,
               reason='ok', distance_m=tof_distance, range_min_m=.1,
               range_max_m=12.)
    imu = dict(schema=1, source='gazebo_sensor', type='imu_attitude_sample',
               clock_domain='gazebo_sim_us', time_us=stamp, valid=True,
               reason='ok', quaternion_wxyz=[1., 0., 0., 0.])
    return profile, tof, imu


def test_live_shadow_estimates_from_ranges_and_sensors_without_truth():
    config = configuration()
    tag = [2.09, 1.68, 1.0]
    profile, _, _ = sensor_rows(1_000_000, tag[2])
    processor = GazeboLiveShadow(config, profile)
    rows = []
    for seq in range(4):
        stamp = 1_000_000+seq*40_000
        _, tof, imu = sensor_rows(stamp, tag[2])
        processor.add_sensor(tof)
        processor.add_sensor(imu)
        rows.append(processor.process_cycle(cycle(config, seq, stamp, tag)))
    assert rows[-1]['height_source'] == 'gazebo_tof_imu'
    assert rows[-1]['truth_used'] is False
    assert rows[-1]['external_output_allowed'] is False
    for name in ('A', 'B', 'C', 'D', 'WLS'):
        assert rows[-1]['models'][name]['ok'], (name, rows[-1]['models'][name]['reason'])
        np.testing.assert_allclose(rows[-1]['models'][name]['xy_m'], tag[:2], atol=1e-5)
    assert rows[0]['models']['B']['ok'] is False


def test_closed_height_gate_blocks_position_models_but_not_h80_diagnostic():
    config = configuration()
    processor = GazeboLiveShadow(config, PROFILE)
    row = processor.process_cycle(cycle(config, 0, 1_000_000, [2., 1., 1.]))
    assert row['height_m'] is None
    assert row['models']['A']['reason'] == 'orientation_alignment_unconfirmed'
    assert row['models']['C']['ok'] is False
    assert row['models']['B']['reason'] == 'insufficient_anchors'


def test_duplicate_and_bad_clock_are_rejected_without_reusing_observation():
    config = configuration()
    processor = GazeboLiveShadow(config, PROFILE)
    first = cycle(config, 0, 1_000_000, [2., 1., 1.])
    processor.process_cycle(first)
    with pytest.raises(ValueError, match='non_increasing_cycle'):
        processor.process_cycle(first)
    reversed_stamp = cycle(config, 1, 999_000, [2., 1., 1.])
    with pytest.raises(ValueError, match='non_increasing_cycle'):
        processor.process_cycle(reversed_stamp)
    second = cycle(config, 1, 1_040_000, [2., 1., 1.])
    second['sample_time_us'] = [second['time_us']-1000]*4
    with pytest.raises(ValueError, match='non_simultaneous_range'):
        processor.process_cycle(second)
    assert processor.process_cycle(cycle(config, 1, 1_040_000, [2., 1., 1.]))['seq'] == 1


def test_new_invalid_tof_does_not_reuse_earlier_good_height():
    config = configuration()
    profile, tof, imu = sensor_rows(1_000_000, 1.)
    processor = GazeboLiveShadow(config, profile)
    processor.add_sensor(tof)
    processor.add_sensor(imu)
    bad = dict(tof, time_us=1_030_000, valid=False, reason='no_valid_return', distance_m=None)
    processor.add_sensor(bad)
    row = processor.process_cycle(cycle(config, 0, 1_040_000, [2., 1., 1.]))
    assert row['models']['A']['reason'] == 'tof_invalid'
    assert row['height_m'] is None


def test_transport_stamp_must_match_cycle_and_nonfinite_json_is_refused():
    row = cycle(configuration(), 0, 1_000_000, [2., 1., 1.])
    message = SimpleNamespace(data=json.dumps(row), header=SimpleNamespace(
        stamp=SimpleNamespace(sec=1, nsec=0)))
    assert raw_record(message, '/test')['seq'] == 0
    message.header.stamp.nsec = 1_000_000
    with pytest.raises(ValueError, match='raw_transport_stamp_mismatch'):
        raw_record(message, '/test')
    message.header.stamp.nsec = 0
    message.data = message.data.replace('"raw_slant_m": [', '"raw_slant_m": [NaN, ')
    with pytest.raises(ValueError):
        raw_record(message, '/test')


def test_callback_clock_brackets_parse_without_changing_sensor_stamp():
    row = cycle(configuration(), 0, 1_000_000, [2., 1., 1.])
    recorded = with_callback_clock(row, 9_000_000_000, 9_000_100_000)
    assert recorded['time_us'] == row['time_us']
    assert recorded['clock_domain'] == 'gazebo_sim_us'
    assert recorded['host_clock_domain'] == 'wsl_monotonic_ns'
    assert recorded['host_callback_start_monotonic_ns'] == 9_000_000_000
    assert recorded['host_callback_end_monotonic_ns'] == 9_000_100_000
    assert 'host_callback_start_monotonic_ns' not in row
    with pytest.raises(ValueError, match='invalid_host_callback_clock'):
        with_callback_clock(row, 9_000_100_000, 9_000_000_000)
