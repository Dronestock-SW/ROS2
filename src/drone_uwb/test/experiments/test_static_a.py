"""Independent capture calibration and explicit manual-reference experiment boundaries."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.static_a import calculate, estimate_bias, evaluate, read_capture, run
from drone_uwb.processing.runner import json_line


LAYOUT = json.loads((Path(__file__).resolve().parents[2]/'config'/'anchors_20260906.json').read_text(encoding='utf-8'))
ANCHORS = np.array(LAYOUT['anchors_xyz_m'])
REFERENCE = [2.09, 1.68, 1.19]
BIAS = [-.14, .20, -.17, -.06]
FIRMWARE = 'uwb-tag-for-jetson-v1.8-raw-xy'
SOLVER = dict(max_iterations=40, step_tol_m=1e-7, condition_max=1e6)


def capture(path, count=120, bias=BIAS, missing=False, excursion=False):
    status = dict(type='uwb_raw_status', schema=1, tag_id='5', uwb_ready=True,
                  firmware=FIRMWARE, anchor_order=['A1', 'A2', 'A3', 'A4'], anchor_count=4,
                  clock_domain='esp32_monotonic_boot_us', temporal_filter_applied=False)
    events = [dict(message=status, host_received_monotonic_ns=1_000_000_000)]
    ranges = np.linalg.norm(ANCHORS-REFERENCE, axis=1)+bias
    for k in range(count):
        start = 1_000_000+k*25000
        raw = ranges.copy()
        if excursion and k == 100:
            raw[3] += 1
        msg = dict(type='uwb_raw_cycle', schema=1, tag_id='5', seq=k+1,
                   cycle_start_us=start, cycle_end_us=start+20000,
                   valid_mask=15, raw_slant_m=raw.tolist(), failure=['ok']*4,
                   sample_time_us=[start+4000*i for i in range(1, 5)])
        if missing and k == 10:
            msg['valid_mask'] = 11
            msg['raw_slant_m'][2] = None
            msg['failure'][2] = 'response_timeout'
        events.append(dict(message=msg, host_received_monotonic_ns=10_000_000_000+(start+21000)*1000))
    path.write_text(''.join(map(json_line, events)), encoding='utf-8')
    return events


def test_bias_sign_partial_anchor_and_unmodified_input(tmp_path):
    path = tmp_path/'train.jsonl'
    capture(path, missing=True)
    original = path.read_bytes()
    rows = read_capture(path, '5', FIRMWARE)
    fit = estimate_bias(rows, ANCHORS, REFERENCE)
    np.testing.assert_allclose(fit['bias_m'], BIAS, atol=1e-12)
    assert fit['samples_per_anchor'] == [120, 120, 119, 120]
    assert fit['candidate_only'] and not fit['production_calibration_confirmed']
    assert path.read_bytes() == original


def test_separate_evaluation_recovers_xy_and_keeps_excursion(tmp_path):
    train, test = tmp_path/'train.jsonl', tmp_path/'test.jsonl'
    capture(train)
    capture(test, excursion=True)
    calibration = estimate_bias(read_capture(train, '5', FIRMWARE), ANCHORS, REFERENCE)
    bias_before = deepcopy(calibration)
    rows = read_capture(test, '5', FIRMWARE)
    result = calculate(rows, ANCHORS, REFERENCE[2], calibration['bias_m'], SOLVER)
    assert calibration == bias_before
    assert len(result) == 120
    np.testing.assert_allclose(result[0]['fits']['with_candidate_bias']['xy_m'], REFERENCE[:2], atol=1e-9)
    assert result[100]['fits']['with_candidate_bias']['ok']
    summary = evaluate(result, REFERENCE[:2], [101, 101])
    assert summary['reported_excursion_span']['models']['with_candidate_bias']['rmse_m'] > .1
    assert summary['outside_reported_span']['models']['with_candidate_bias']['rmse_m'] < 1e-9
    before = deepcopy(result)
    changed = evaluate(result, [10, 10], [101, 101])
    assert changed['all']['models']['with_candidate_bias']['rmse_m'] > 5
    assert result == before
    assert all(not r['flight_valid'] and not r['external_output_allowed'] for r in result)


def test_missing_anchor_does_not_create_four_anchor_position(tmp_path):
    path = tmp_path/'missing.jsonl'
    capture(path, missing=True)
    result = calculate(read_capture(path, '5', FIRMWARE), ANCHORS, REFERENCE[2], BIAS, SOLVER)
    assert result[10]['reason'] == 'four_anchors_required'
    assert result[10]['fits'] == {}
    assert len(result) == 120


@pytest.mark.parametrize('fault', ['sequence_type', 'duplicate', 'host_time'])
def test_rejected_cycle_remains_in_evaluation_denominator(tmp_path, fault):
    path = tmp_path/'bad.jsonl'
    events = capture(path)
    if fault == 'sequence_type':
        events[20]['message']['seq'] = 'bad'
    elif fault == 'duplicate':
        events[20] = deepcopy(events[19])
    else:
        events[20]['host_received_monotonic_ns'] = 0
    path.write_text(''.join(map(json_line, events)), encoding='utf-8')
    rows = read_capture(path, '5', FIRMWARE)
    result = calculate(rows, ANCHORS, REFERENCE[2], BIAS, SOLVER)
    metrics = evaluate(result, REFERENCE[:2], [101, 101])
    assert metrics['all']['input_records'] == 120
    assert metrics['all']['models']['with_candidate_bias']['count'] == 119


def test_insufficient_calibration_and_inconsistent_firmware_are_rejected(tmp_path):
    path = tmp_path/'small.jsonl'
    capture(path, count=3)
    with pytest.raises(ValueError, match='insufficient_calibration_samples'):
        estimate_bias(read_capture(path, '5', FIRMWARE), ANCHORS, REFERENCE)
    with pytest.raises(ValueError, match='inconsistent_session_metadata'):
        read_capture(path, '5', 'other-firmware')


def test_run_hash_split_and_reproducibility(tmp_path):
    train, test = tmp_path/'train.jsonl', tmp_path/'test.jsonl'
    capture(train)
    capture(test, excursion=True)
    meta = tmp_path/'metadata.json'
    meta.write_text(json.dumps({'reference_position_m': dict(zip(('x', 'y', 'z'), REFERENCE))}), encoding='utf-8')
    layout = tmp_path/'anchors.json'
    layout.write_text(json.dumps(LAYOUT), encoding='utf-8')
    annotation = tmp_path/'excursion.json'
    annotation.write_text(json.dumps({'events': [{'first_seq': 101, 'last_seq': 101}]}), encoding='utf-8')
    files = {'calibration_raw': train, 'evaluation_raw': test, 'calibration_metadata': meta,
             'evaluation_metadata': meta, 'anchors': layout, 'excursion_annotation': annotation}
    config = dict(tag_id='5', firmware=FIRMWARE, minimum_calibration_samples=100, solver=SOLVER,
                  files={name: dict(path=str(p), sha256=digest(p)) for name, p in files.items()})
    config_path = tmp_path/'config.json'
    config_path.write_text(json.dumps(config), encoding='utf-8')
    a, b = tmp_path/'a', tmp_path/'b'
    assert run(config_path, tmp_path, a) == run(config_path, tmp_path, b)
    assert (a/'results.jsonl').read_bytes() == (b/'results.jsonl').read_bytes()
    assert (a/'evaluation_raw.jsonl').read_bytes() == test.read_bytes()
    with pytest.raises(FileExistsError):
        run(config_path, tmp_path, a)
    config['files']['evaluation_raw'] = config['files']['calibration_raw']
    config_path.write_text(json.dumps(config), encoding='utf-8')
    with pytest.raises(ValueError, match='calibration_evaluation_must_be_separate'):
        run(config_path, tmp_path, tmp_path/'same')
    config['files']['calibration_raw']['sha256'] = 'invalid'
    config_path.write_text(json.dumps(config), encoding='utf-8')
    with pytest.raises(ValueError, match='input_hash_mismatch'):
        run(config_path, tmp_path, tmp_path/'wrong')
    assert not (tmp_path/'wrong').exists()
