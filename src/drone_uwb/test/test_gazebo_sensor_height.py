"""Sensor-fed Gazebo replay must never substitute simulator truth for missing height."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.processing.experiments.gazebo_height import GazeboSensorHeight
from drone_uwb.processing.experiments.gazebo_trial import compare_poses, run


ROOT = Path(__file__).resolve().parents[1]
PROFILE = json.loads((ROOT/'config/gazebo_sensor_height_profile.json').read_text(encoding='utf-8'))
CONFIG = json.loads((ROOT/'config/gazebo_shadow.json').read_text(encoding='utf-8'))
CONFIG['tag_offset_body_flu_m'] = [0., 0., .3]


def fixture(count=16):
    poses, tof, attitude = [], [], []
    for index in range(count):
        stamp = 1_000_000+index*25_000
        poses.append(dict(source='synthetic_pose_fixture', clock_domain='gazebo_sim_us',
                          model='test_model', time_us=stamp, position_xyz_m=[.4, .5, 1.],
                          quaternion_wxyz=[1., 0., 0., 0.]))
        tof.append(dict(schema=1, source='gazebo_sensor', type='tof_sample',
                        clock_domain='gazebo_sim_us', time_us=stamp, topic='/tof',
                        sensor_frame='tof', distance_m=.95, range_min_m=.1,
                        range_max_m=12., valid=True, reason='ok'))
        attitude.append(dict(schema=1, source='gazebo_sensor', type='imu_attitude_sample',
                             clock_domain='gazebo_sim_us', time_us=stamp, topic='/imu',
                             sensor_frame='imu', orientation_reference='initial_imu_frame_unverified',
                             quaternion_wxyz=[1., 0., 0., 0.], valid=True, reason='ok'))
    return poses, tof, attitude


def profile(open_gate):
    result = deepcopy(PROFILE)
    result['orientation_alignment_confirmed'] = open_gate
    result['flat_floor_confirmed'] = open_gate
    return result


def test_causal_height_gate_and_latest_invalid_sample():
    poses, tof, attitude = fixture()
    closed = GazeboSensorHeight(tof, attitude, PROFILE, CONFIG['tag_offset_body_flu_m'])
    assert closed.height_at(poses[5]['time_us'])[1]['reason'] == 'orientation_alignment_unconfirmed'
    open_height = GazeboSensorHeight(tof, attitude, profile(True), CONFIG['tag_offset_body_flu_m'])
    assert open_height.height_at(poses[5]['time_us'])[0] == pytest.approx(1.3)
    assert open_height.height_at(poses[0]['time_us']-1)[1]['reason'] == 'tof_unavailable'
    bad = deepcopy(tof)
    bad[5]['valid'], bad[5]['reason'], bad[5]['distance_m'] = False, 'no_valid_return', None
    height = GazeboSensorHeight(bad, attitude, profile(True), CONFIG['tag_offset_body_flu_m'])
    assert height.height_at(poses[5]['time_us'])[1]['reason'] == 'tof_invalid'
    attitude[5]['time_us'] = attitude[4]['time_us']
    with pytest.raises(ValueError, match='invalid_or_unordered'):
        GazeboSensorHeight(tof, attitude, profile(True), CONFIG['tag_offset_body_flu_m'])


def test_sensor_height_changes_only_height_dependent_models():
    poses, tof, attitude = fixture()
    base = list(compare_poses(poses, CONFIG))
    sensor = GazeboSensorHeight(tof, attitude, profile(True), CONFIG['tag_offset_body_flu_m'])
    same = list(compare_poses(poses, CONFIG, height_provider=sensor))
    for original, supplied in zip(base, same):
        assert supplied['height_source'] == 'gazebo_tof_imu'
        assert supplied['height_m'] == pytest.approx(1.3)
        assert supplied['models']['B'] == original['models']['B']
        np.testing.assert_allclose(supplied['models']['A']['xy_m'], original['models']['A']['xy_m'])
    changed_tof = deepcopy(tof)
    changed_tof[8]['distance_m'] += .2
    changed = list(compare_poses(poses, CONFIG, height_provider=GazeboSensorHeight(
        changed_tof, attitude, profile(True), CONFIG['tag_offset_body_flu_m'])))
    assert changed[8]['models']['A']['xy_m'] != same[8]['models']['A']['xy_m']
    assert changed[8]['models']['B'] == same[8]['models']['B']
    blocked = list(compare_poses(poses, CONFIG, height_provider=GazeboSensorHeight(
        tof, attitude, PROFILE, CONFIG['tag_offset_body_flu_m'])))
    assert blocked[8]['models']['A']['reason'] == 'orientation_alignment_unconfirmed'
    assert blocked[8]['models']['B']['ok'] is True


def test_file_replay_preserves_sensor_inputs_and_gate_state(tmp_path):
    poses, tof, attitude = fixture()
    paths = [tmp_path/name for name in ('poses.jsonl', 'tof.jsonl', 'attitude.jsonl')]
    for path, records in zip(paths, (poses, tof, attitude)):
        path.write_text(''.join(json.dumps(row)+'\n' for row in records), encoding='utf-8')
    config_path, profile_path = tmp_path/'config.json', tmp_path/'height_profile.json'
    config_path.write_text(json.dumps(CONFIG), encoding='utf-8')
    profile_path.write_text(json.dumps(PROFILE), encoding='utf-8')
    output = tmp_path/'trial'
    summary = run(paths[0], config_path, output, tof_path=paths[1],
                  attitude_path=paths[2], height_profile_path=profile_path)
    assert summary['scope'] == 'gazebo_sensor_UWB_shadow'
    assert summary['height_selection_reasons'] == {'orientation_alignment_unconfirmed': 16}
    assert summary['evaluation']['models']['A']['count'] == 0
    assert summary['evaluation']['models']['B']['count'] == 14
    assert (output/'tof.jsonl').read_bytes() == paths[1].read_bytes()
    assert (output/'attitude.jsonl').read_bytes() == paths[2].read_bytes()
