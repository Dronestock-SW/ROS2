"""Gazebo adapter contracts tested with explicitly synthetic pose fixtures."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as Obj

import numpy as np
import pytest

from drone_uwb.integration.gazebo_capture import pose_record
from drone_uwb.processing.experiments.gazebo_trial import compare_poses, evaluate, run
from drone_uwb.processing.experiments.gazebo_scenarios import scenarios


PROFILE = json.loads((Path(__file__).resolve().parents[1]/'config/gazebo_shadow.json').read_text())


def poses(count=80):
    return [dict(source='synthetic_pose_fixture', clock_domain='gazebo_sim_us', model='test_model',
                 time_us=1_000_000+k*25000, position_xyz_m=[.2+k*.004, .3, 1.1+k*.001],
                 quaternion_wxyz=[1., 0., 0., 0.]) for k in range(count)]


def test_exact_pose_identity_and_simulator_timestamp():
    model = Obj(name='x500_lidar_down_0', position=Obj(x=1., y=2., z=3.), orientation=Obj(w=1., x=0., y=0., z=0.))
    msg = Obj(header=Obj(stamp=Obj(sec=5, nsec=123456000)), pose=[model])
    record = pose_record(msg, model.name)
    assert record['time_us'] == 5_123456
    assert record['source'] == 'gazebo' and record['position_xyz_m'] == [1., 2., 3.]
    assert pose_record(msg, 'x500') is None  # No prefix matching other models/links.
    model.position.x = float('nan')
    with pytest.raises(ValueError, match='invalid_gazebo_pose'):
        pose_record(msg, model.name)
    model.position.x = 1.
    msg.pose.append(model)
    assert pose_record(msg, model.name) is None


def test_same_weights_equal_A_and_weights_do_not_modify_other_models():
    config = deepcopy(PROFILE)
    one = list(compare_poses(poses(), config))
    config['wls_sigma_m'][1] = .3
    two = list(compare_poses(poses(), config))
    for original, changed in zip(one, two):
        assert original['raw_slant_m'] == changed['raw_slant_m']
        assert all(original['models'][m] == changed['models'][m] for m in ('A', 'B', 'C', 'D'))
        if original['models']['A']['ok'] and original['models']['WLS']['ok']:
            np.testing.assert_allclose(original['models']['A']['xy_m'], original['models']['WLS']['xy_m'], atol=1e-7)
    assert any(a['models']['WLS']['xy_m'] != b['models']['WLS']['xy_m'] for a, b in zip(one, two))


def test_dropout_remains_in_denominator_and_B_restarts():
    config = deepcopy(PROFILE)
    config['faults'] = [dict(start_s=.5, end_s=.75, drop_all=True)]
    rows = list(compare_poses(poses(), config))
    assert sum(r['frame_dropped'] for r in rows) == 10
    assert all(not rows[20]['models'][m]['ok'] for m in rows[20]['models'])
    assert rows[30]['models']['B']['reset_reason'] == 'gap'
    assert evaluate(rows)['models']['A']['coverage'] == 70/80


def test_scenario_quality_assumptions_can_help_or_hurt():
    variants = scenarios(PROFILE)
    # Identical stationary observations isolate the declared quality effect.
    records = poses(120)
    for row in records:
        row['position_xyz_m'] = [.7, .3, 1.3]
    results = {name: list(compare_poses(records, variants[name])) for name in
               ('noisy_A2_equal', 'noisy_A2_weighted', 'noisy_A2_wrong_weight')}
    assert [[r['raw_slant_m'] for r in rows] for rows in results.values()].count(
        [r['raw_slant_m'] for r in results['noisy_A2_equal']]) == 3
    errors = {name: evaluate(rows)['models']['WLS']['rmse_m'] for name, rows in results.items()}
    assert errors['noisy_A2_weighted'] < errors['noisy_A2_equal']
    assert errors['noisy_A2_wrong_weight'] > errors['noisy_A2_equal']


@pytest.mark.parametrize('fault', ['duplicate', 'time_reversed', 'clock', 'model', 'source', 'nan'])
def test_bad_or_mixed_pose_stream_fails(fault):
    records = poses(4)
    if fault == 'duplicate':
        records[2] = deepcopy(records[1])
    elif fault == 'time_reversed':
        records[2]['time_us'] = 0
    elif fault == 'clock':
        records[2]['clock_domain'] = 'host'
    elif fault == 'model':
        records[2]['model'] = 'other'
    elif fault == 'source':
        records[2]['source'] = 'gazebo'
    else:
        records[2]['position_xyz_m'][1] = float('nan')
    with pytest.raises(ValueError):
        list(compare_poses(records, PROFILE))


def test_replay_and_source_are_explicit(tmp_path):
    source, config = tmp_path/'poses.jsonl', tmp_path/'config.json'
    source.write_text(''.join(json.dumps(p)+'\n' for p in poses(15)), encoding='utf-8')
    config.write_text(json.dumps(PROFILE), encoding='utf-8')
    result = run(source, config, tmp_path/'one')
    assert run(source, config, tmp_path/'two') == result
    assert result['source'] == 'synthetic_pose_fixture'
    assert result['external_output_allowed'] is False
    assert (tmp_path/'one/results.jsonl').read_bytes() == (tmp_path/'two/results.jsonl').read_bytes()
    with pytest.raises(FileExistsError):
        run(source, config, tmp_path/'one')
