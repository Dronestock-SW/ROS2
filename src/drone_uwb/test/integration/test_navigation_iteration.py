"""Iteration records must preserve provenance and refuse false 7 cm claims."""
import hashlib
import json
from pathlib import Path

import pytest

from drone_uwb.processing.experiments.navigation_iteration import execute


CONFIG = Path(__file__).resolve().parents[2]/'config/gazebo_shadow.json'


def _prepare(tmp_path, *, invalid_pose=False):
    input_path = tmp_path/'poses.jsonl'
    poses = [dict(source='synthetic_pose_fixture', clock_domain='gazebo_sim_us',
                  model='fixture', time_us=1_000_000+25_000*i,
                  position_xyz_m=[.5+.01*i, .4, 1.2], quaternion_wxyz=[1., 0., 0., 0.])
             for i in range(20)]
    if invalid_pose:
        poses[3]['time_us'] = poses[2]['time_us']
    input_path.write_text(''.join(json.dumps(p)+'\n' for p in poses), encoding='utf-8')
    output = tmp_path/'iter_0001'
    plan_path = tmp_path/'plan.json'
    plan = dict(run_id=output.name, parent_run_id=None, dataset_split='exploratory',
                hypothesis='Record the accuracy and unavailable ticks on a synthetic path.',
                change={'kind':'baseline_replay'}, criteria={'target_m':.07,'min_coverage':.95},
                scenario_intervals=[dict(name='first',start_s=0.,end_s=.25)],
                input_sha256=hashlib.sha256(input_path.read_bytes()).hexdigest(),
                config_sha256=hashlib.sha256(CONFIG.read_bytes()).hexdigest())
    plan_path.write_text(json.dumps(plan), encoding='utf-8')
    return input_path, plan_path, output


def test_iteration_records_failures_and_unverified_achievement(tmp_path):
    source, plan, output = _prepare(tmp_path)
    record = execute(source, CONFIG, plan, output)
    assert record['achieved_7cm'] is False
    assert record['source'] == 'synthetic_pose_fixture'
    assert record['model_metrics']['B']['missing_count'] >= 2
    assert record['model_status']['B'] != 'achieved'
    failures = [json.loads(line) for line in (output/'failures.jsonl').read_text(encoding='utf-8').splitlines()]
    assert sum(row['model'] == 'B' for row in failures) >= 2
    assert (output/'scenario_metrics.csv').read_text(encoding='utf-8').count('first') == 5
    assert (output/'input_index.json').exists()
    assert (output/'plan.json').read_bytes() == plan.read_bytes()
    with pytest.raises(FileExistsError):
        execute(source, CONFIG, plan, output)


def test_failed_iteration_keeps_plan_and_failure_reason(tmp_path):
    source, plan, output = _prepare(tmp_path, invalid_pose=True)
    with pytest.raises(ValueError, match='non_increasing_simulator_time'):
        execute(source, CONFIG, plan, output)
    failure = json.loads((output/'failed_attempt.json').read_text(encoding='utf-8'))
    assert failure['error_type'] == 'ValueError'
    assert (output/'plan.json').read_bytes() == plan.read_bytes()
    assert not (output/'summary.json').exists()


def test_sensor_files_require_predeclared_hashes_and_keep_height_gate_closed(tmp_path):
    source, plan_path, output = _prepare(tmp_path)
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    config['tag_offset_body_flu_m'] = [0., 0., .3]
    config_path = tmp_path/'sensor_config.json'
    config_path.write_text(json.dumps(config), encoding='utf-8')
    profile = json.loads((CONFIG.parent/'gazebo_sensor_height_profile.json').read_text(encoding='utf-8'))
    profile_path = tmp_path/'height_profile.json'
    profile_path.write_text(json.dumps(profile), encoding='utf-8')
    poses = [json.loads(line) for line in source.read_text(encoding='utf-8').splitlines()]
    tof, attitude = tmp_path/'tof.jsonl', tmp_path/'attitude.jsonl'
    tof.write_text(''.join(json.dumps(dict(schema=1, source='gazebo_sensor', type='tof_sample',
                  clock_domain='gazebo_sim_us', time_us=p['time_us'], valid=True,
                  reason='ok', distance_m=1.15, range_min_m=.1, range_max_m=12.))+'\n'
                  for p in poses), encoding='utf-8')
    attitude.write_text(''.join(json.dumps(dict(schema=1, source='gazebo_sensor',
                       type='imu_attitude_sample', clock_domain='gazebo_sim_us',
                       time_us=p['time_us'], valid=True, reason='ok',
                       quaternion_wxyz=[1., 0., 0., 0.]))+'\n'
                       for p in poses), encoding='utf-8')
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    plan.update(config_sha256=sha(config_path), tof_sha256=sha(tof),
                attitude_sha256=sha(attitude), height_profile_sha256=sha(profile_path))
    plan_path.write_text(json.dumps(plan), encoding='utf-8')
    with pytest.raises(ValueError, match='predeclared_sensor_capture_mismatch'):
        execute(source, config_path, plan_path, output, tof_path=tof,
                attitude_path=attitude, height_profile_path=CONFIG)
    record = execute(source, config_path, plan_path, output, tof_path=tof,
                     attitude_path=attitude, height_profile_path=profile_path)
    assert record['model_metrics']['A']['count'] == 0
    assert record['model_metrics']['B']['count'] == 18
    assert record['achieved_7cm'] is False
    index = json.loads((output/'input_index.json').read_text(encoding='utf-8'))
    assert index['tof_sha256'] == sha(tof)
    assert index['attitude_sha256'] == sha(attitude)
    assert index['height_profile_sha256'] == sha(profile_path)
