"""File replay, paired denominators and immutable A/B baseline checks."""
from copy import deepcopy
import json

import pytest

from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import run as run_ab
from drone_uwb.processing.experiments.static_a import estimate_bias, read_capture
from drone_uwb.processing.experiments.subset_comparison import evaluate, run
from test_static_a import ANCHORS, FIRMWARE, LAYOUT, REFERENCE, SOLVER, capture


def trial(tmp_path):
    train, test = tmp_path/'train.jsonl', tmp_path/'test.jsonl'
    capture(train)
    capture(test, excursion=True, missing=True)
    calibration = estimate_bias(read_capture(train, '5', FIRMWARE), ANCHORS, REFERENCE)
    calibration['source_sha256'] = digest(train)
    objects = dict(locked_calibration=calibration, anchors=LAYOUT,
                   evaluation_metadata={'reference_position_m': dict(zip(('x', 'y', 'z'), REFERENCE))},
                   excursion_annotation={'events': [{'first_seq': 101, 'last_seq': 101}]})
    files = dict(calibration_raw=train, evaluation_raw=test)
    for name, obj in objects.items():
        files[name] = tmp_path/(name+'.json')
        files[name].write_text(json.dumps(obj), encoding='utf-8')
    config = dict(target_mode='reference_slant', tag_id='5', firmware=FIRMWARE, A=SOLVER, B={}, C=SOLVER, D={}, models=['C', 'D'],
                  files={name: dict(path=str(p), sha256=digest(p)) for name, p in files.items()})
    cp = tmp_path/'config.json'
    cp.write_text(json.dumps(config), encoding='utf-8')
    run_ab(cp, tmp_path, tmp_path/'ab')
    prior = tmp_path/'ab/results.jsonl'
    config['files']['prior_AB_results'] = dict(path=str(prior), sha256=digest(prior))
    cp.write_text(json.dumps(config), encoding='utf-8')
    return cp, config


def test_file_replay_prior_baseline_and_all_failed_ticks_preserved(tmp_path):
    cp, config = trial(tmp_path)
    a, b = tmp_path/'a', tmp_path/'b'
    result = run(cp, tmp_path, a)
    assert result == run(cp, tmp_path, b)
    assert result['prior_A_B_exactly_preserved'] is True
    assert result['groups']['all']['input_ticks'] == 120
    assert result['groups']['all']['models']['C']['count'] < 120
    for name in ('results.jsonl', 'summary.json', 'positions.csv'):
        assert (a/name).read_bytes() == (b/name).read_bytes()
    rows = [json.loads(s) for s in (a/'results.jsonl').read_text(encoding='utf-8').splitlines()]
    assert rows[10]['models']['C']['reason'] == rows[10]['models']['D']['reason'] == 'four_anchors_required'
    before = deepcopy(rows)
    altered = evaluate(rows, [100, 200], [101, 101], result['models'])
    assert rows == before and altered['all']['models']['C']['rmse_m'] > 100
    assert all(not r['flight_valid'] and not r['external_output_allowed'] for r in rows)
    with pytest.raises(FileExistsError):
        run(cp, tmp_path, a)


def test_different_success_subsets_use_same_times_for_each_pair():
    rows = []
    for i in range(3):
        models = {m: dict(ok=True, reason='ok', xy_m=[i+1., 0]) for m in ('A', 'B', 'C', 'D')}
        if i != 2:
            models['D'] = dict(ok=False, reason='no_intersection', xy_m=None)
        rows.append(dict(seq=i, models=models))
    summary = evaluate(rows, [0, 0], [100, 101], ['A', 'B', 'C', 'D'])['all']
    assert summary['models']['D']['coverage'] == 1/3
    assert summary['pairwise']['A_D']['models']['A']['rmse_m'] == 3
    assert summary['all_model_common']['count'] == 1
    assert summary['pairwise']['A_C']['count'] == 3


@pytest.mark.parametrize('fault', ['hash', 'same_capture', 'changed_baseline'])
def test_trial_provenance_guards(tmp_path, fault):
    cp, config = trial(tmp_path)
    if fault == 'hash':
        config['files']['evaluation_raw']['sha256'] = 'wrong'
        reason = 'input_hash_mismatch'
    elif fault == 'same_capture':
        config['files']['evaluation_raw'] = config['files']['calibration_raw']
        reason = 'calibration_evaluation_must_be_separate'
    else:
        config['A']['step_tol_m'] = .1
        reason = 'A_or_B_changed_from_locked_baseline'
    cp.write_text(json.dumps(config), encoding='utf-8')
    with pytest.raises(ValueError, match=reason):
        run(cp, tmp_path, tmp_path/'bad')
    assert not (tmp_path/'bad').exists()
