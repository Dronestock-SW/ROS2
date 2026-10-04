"""A geometry diagnostic must expose mount errors without authorizing fusion."""
from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from drone_uwb.processing.experiments.gazebo_height_diagnostic import assess, run


ROOT = Path(__file__).resolve().parents[2]
PROFILE = json.loads((ROOT/'config/gazebo_sensor_height_profile.json').read_text(encoding='utf-8'))


def fixture():
    poses, tof, attitude = [], [], []
    for index in range(20):
        pitch = math.radians(index)
        c, s = math.cos(pitch), math.sin(pitch)
        stamp = 1_000_000+index*25_000
        quat = [math.cos(pitch/2), 0., math.sin(pitch/2), 0.]
        poses.append(dict(source='synthetic_pose_fixture', clock_domain='gazebo_sim_us',
                          model='test_model', time_us=stamp, position_xyz_m=[.4, .5, 1.],
                          quaternion_wxyz=quat))
        tof.append(dict(schema=1, source='gazebo_sensor', type='tof_sample',
                        clock_domain='gazebo_sim_us', time_us=stamp, valid=True,
                        reason='ok', distance_m=(1.-.05*c)/c,
                        range_min_m=.1, range_max_m=12.))
        attitude.append(dict(schema=1, source='gazebo_sensor', type='imu_attitude_sample',
                             clock_domain='gazebo_sim_us', time_us=stamp, valid=True,
                             reason='ok', quaternion_wxyz=quat))
    return poses, tof, attitude


def test_diagnostic_detects_orientation_error_but_never_confirms_it():
    poses, tof, attitude = fixture()
    untouched = deepcopy(PROFILE)
    records, summary = assess(poses, tof, attitude, PROFILE, [0., 0., .3])
    assert summary['height_candidate']['count'] == 20
    assert summary['height_candidate']['max_abs_m'] < 1e-12
    assert summary['tof_range_residual']['max_abs_m'] < 1e-12
    assert summary['tilt_range_deg']['max'] == pytest.approx(19.)
    assert summary['eligible_for_navigation'] is False
    assert PROFILE == untouched
    assert all(row['external_output_allowed'] is False for row in records)

    # A wrong IMU orientation changes the candidate while the same ToF truth stays fixed.
    wrong_attitude = deepcopy(attitude)
    for row in wrong_attitude:
        row['quaternion_wxyz'] = [1., 0., 0., 0.]
    _, wrong = assess(poses, tof, wrong_attitude, PROFILE, [0., 0., .3])
    assert wrong['height_candidate']['max_abs_m'] > .04
    assert wrong['alignment_confirmed'] is False


def test_missing_range_and_file_record_keep_diagnostic_provenance(tmp_path):
    poses, tof, attitude = fixture()
    tof[5]['valid'], tof[5]['reason'], tof[5]['distance_m'] = False, 'no_valid_return', None
    _, summary = assess(poses, tof, attitude, PROFILE, [0., 0., .3])
    assert summary['height_candidate']['count'] == 19
    assert summary['selection_reasons']['tof_invalid'] == 1
    config = {'tag_offset_body_flu_m': [0., 0., .3]}
    paths = [tmp_path/name for name in ('poses.jsonl', 'tof.jsonl', 'attitude.jsonl')]
    for path, rows in zip(paths, (poses, tof, attitude)):
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
    profile_path, config_path = tmp_path/'profile.json', tmp_path/'config.json'
    profile_path.write_text(json.dumps(PROFILE), encoding='utf-8')
    config_path.write_text(json.dumps(config), encoding='utf-8')
    output = tmp_path/'diagnostic'
    saved = run(*paths, profile_path, config_path, output)
    assert saved['session_identity_verified'] is False
    assert saved['eligible_for_navigation'] is False
    assert (output/'height_profile.json').read_bytes() == profile_path.read_bytes()
    assert len((output/'diagnostics.jsonl').read_text(encoding='utf-8').splitlines()) == 20
    with pytest.raises(FileExistsError):
        run(*paths, profile_path, config_path, output)
