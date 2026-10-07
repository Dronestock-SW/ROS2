"""B causal windows, numerical failures, session resets and paired evaluation."""
from copy import deepcopy
import json

import numpy as np
import pytest

from drone_uwb.acquisition.validation import Cycle
from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import (
    BSettings, H80Window, calculate, evaluate, read_events, run,
)
from drone_uwb.processing.solvers.h80 import fit_h80
from drone_uwb.processing.runner import json_line
from test_static_a import ANCHORS, BIAS, FIRMWARE, LAYOUT, REFERENCE, SOLVER, capture


def moving_cycle(k, noise=None):
    start = 1_000_000+k*25000
    samples = [start+4000*i for i in range(1, 5)]
    times = np.array(samples)/1e6
    positions = np.column_stack([.7+.3*times, 1.8-.1*times, 1.1+.02*times])
    raw = np.linalg.norm(positions-ANCHORS, axis=1)+BIAS
    if noise is not None:
        raw += noise
    return Cycle(k, start, start+20000, 15, raw, samples, ['ok']*4, list(range(4)),
                 host_mono_ns=(start+21000)*1000)


def as_events(cycles):
    return [dict(input_line=i+1, seq=c.seq, cycle=c, reset_reason=None, reason='ok', tick=True)
            for i, c in enumerate(cycles)]


def test_exact_async_movement_velocity_units_and_causal_window():
    window = H80Window(ANCHORS, BIAS)
    for k in range(100):
        cycle = moving_cycle(k)
        before = cycle.raw.copy()
        fit = window.process(cycle, k+1)
        np.testing.assert_array_equal(before, cycle.raw)
        if k < 2:
            assert not fit['ok'] and fit['reason'] == 'insufficient_anchors'
        else:
            t = cycle.end_us/1e6
            assert fit['ok'] and fit['fit']['rank'] == 7
            np.testing.assert_allclose(fit['xy_m'], [.7+.3*t, 1.8-.1*t], atol=1e-8)
            np.testing.assert_allclose(fit['velocity_m_s'], [.3, -.1], atol=1e-8)
            assert cycle.end_us-800000 <= fit['oldest_sample_us'] <= fit['newest_sample_us'] <= cycle.end_us
            assert len(fit['fit']['coefficients']) == 7
            json_line(fit)


def test_four_anchors_and_no_fresh_output():
    window = H80Window(ANCHORS, BIAS)
    for k in range(40):
        cycle = moving_cycle(k)
        cycle.indices = [0, 1, 2]
        result = window.process(cycle, k)
        assert not result['ok']
    assert result['counts'][3] == 0
    for k in range(40, 44):
        result = window.process(moving_cycle(k), k)
    assert result['ok']
    cycle = moving_cycle(44)
    cycle.indices = []
    result = window.process(cycle, 44)
    assert not result['ok'] and result['reason'] == 'no_fresh_observations'


@pytest.mark.parametrize('kind', ['gap', 'boot', 'backwards'])
def test_reset_drops_previous_window(kind):
    window = H80Window(ANCHORS, BIAS)
    for k in range(20):
        assert window.process(moving_cycle(k), k)['ok'] == (k >= 2)
    if kind == 'boot':
        window.reset('boot')
    k = 40 if kind == 'gap' else 0
    result = window.process(moving_cycle(k), 100)
    assert result['counts'] == [1]*4 and not result['ok']
    assert result['reset_reason'] == {'gap': 'gap', 'boot': 'boot', 'backwards': 'source_time_backwards'}[kind]


def test_gap_threshold_is_strictly_greater_than_150ms():
    window = H80Window(ANCHORS, BIAS)
    for k in range(10):
        window.process(moving_cycle(k), k)
    assert window.process(moving_cycle(15), 15)['reset_reason'] is None
    assert window.process(moving_cycle(22), 22)['reset_reason'] == 'gap'


def test_frames_without_usable_ranges_do_not_bridge_observation_gap():
    window = H80Window(ANCHORS, BIAS)
    for k in range(20):
        window.process(moving_cycle(k), k)
    for k in range(20, 30):
        cycle = moving_cycle(k)
        cycle.indices = []
        assert window.process(cycle, k)['reason'] == 'no_fresh_observations'
    result = window.process(moving_cycle(30), 30)
    assert result['reset_reason'] == 'observation_gap'
    assert result['counts'] == [1]*4 and not result['ok']


@pytest.mark.parametrize('fault,reason', [('nan', 'invalid_input'), ('rank', 'rank_deficient'),
                                         ('future', 'outside_causal_window'), ('overflow', 'solver_failed')])
def test_math_failures_are_finite_json(fault, reason):
    idx = np.tile(np.arange(4), 4)
    times = np.repeat([-.3, -.2, -.1, 0.], 4)
    ranges = np.tile(np.linalg.norm(ANCHORS-REFERENCE, axis=1), 4)
    if fault == 'nan':
        ranges[0] = np.nan
    elif fault == 'rank':
        times[:] = 0
    elif fault == 'future':
        times[-1] = .01
    else:
        ranges[:] = 1e308
    from dataclasses import asdict
    fit = fit_h80(ANCHORS, idx, times, ranges, 0., BSettings())
    assert not fit.ok and fit.reason == reason
    json_line(asdict(fit))


def test_linalg_failure_and_map_assumption(monkeypatch):
    window = H80Window(ANCHORS, BIAS)
    window.process(moving_cycle(0), 0)
    window.process(moving_cycle(1), 1)
    def fail(*args, **kwargs):
        raise np.linalg.LinAlgError('injected')
    monkeypatch.setattr(np.linalg, 'lstsq', fail)
    result = window.process(moving_cycle(2), 2)
    assert result['reason'] == 'solver_failed'
    json_line(result)
    anchors = ANCHORS.copy()
    anchors[2, 2] += .1
    with pytest.raises(ValueError, match='equal_anchor_heights'):
        H80Window(anchors, BIAS)


def test_prefix_results_do_not_depend_on_future_ranges_or_reference():
    cycles = [moving_cycle(k) for k in range(60)]
    prefix, _, _ = calculate(as_events(cycles[:40]), ANCHORS, REFERENCE[2], BIAS, SOLVER)
    cycles[50].raw += .8
    entire, _, _ = calculate(as_events(cycles), ANCHORS, REFERENCE[2], BIAS, SOLVER)
    assert prefix == entire[:40]
    before = deepcopy(entire)
    metrics = evaluate(entire, [2, 2], [1000, 1001])
    evaluate(entire, [200, 200], [1000, 1001])
    assert entire == before
    assert metrics['all']['input_ticks'] == 60
    assert metrics['all']['models']['A']['coverage'] == 1
    assert metrics['all']['models']['B']['count'] == metrics['all']['paired']['count'] == 58


def test_parser_boot_backward_duplicate_and_bad_future_time(tmp_path):
    path = tmp_path/'capture.jsonl'
    events = capture(path, count=12)
    events[5] = deepcopy(events[4])
    events[6]['message']['sample_time_us'][0] = events[6]['message']['cycle_end_us']+1
    boot = deepcopy(events[0])
    boot['host_received_monotonic_ns'] = events[7]['host_received_monotonic_ns']-1
    boot['message']['event'] = 'boot'
    events.insert(7, boot)
    # Source reversal without a new boot/status must invalidate the session.
    events[-2]['message']['cycle_end_us'] = 1
    path.write_text(''.join(map(json_line, events)), encoding='utf-8')
    parsed = list(read_events(path, '5', FIRMWARE))
    assert parsed[5]['reason'] == 'duplicate_or_out_of_order'
    assert parsed[6]['reason'] == 'invalid_report_time'
    assert parsed[7]['reset_reason'] == 'boot'
    assert parsed[-2]['reset_reason'] == 'source_time_backwards'
    assert parsed[-1]['reason'] == 'status_unavailable'
    results, _, _ = calculate(parsed, ANCHORS, REFERENCE[2], BIAS, SOLVER)
    after_boot = next(r for r in results if r['input_line'] == 9)
    assert after_boot['models']['B']['counts'] == [1]*4
    assert len(results) == 12


def test_input_timing_and_corrected_range_guards():
    window = H80Window(ANCHORS, BIAS)
    cycle = moving_cycle(0)
    cycle.raw[0] = -10
    assert window.process(cycle, 1)['excluded']['0'] == 'invalid_corrected_range'
    cycle = moving_cycle(1)
    cycle.end_us += 50000
    assert window.process(cycle, 2)['reason'] == 'extrapolation_limit'
    with pytest.raises(ValueError):
        BSettings(required_anchor_count=3)
    with pytest.raises(ValueError):
        BSettings(h80_irls_iterations=2.5)


def test_run_locked_calibration_hashes_and_deterministic_replay(tmp_path):
    train, test = tmp_path/'train.jsonl', tmp_path/'test.jsonl'
    capture(train)
    capture(test, excursion=True)
    from drone_uwb.processing.experiments.static_a import estimate_bias, read_capture
    calibration = estimate_bias(read_capture(train, '5', FIRMWARE), ANCHORS, REFERENCE)
    calibration['source_sha256'] = digest(train)
    objects = dict(locked_calibration=calibration, anchors=LAYOUT,
                   evaluation_metadata={'reference_position_m': dict(zip(('x', 'y', 'z'), REFERENCE))},
                   excursion_annotation={'events': [{'first_seq': 101, 'last_seq': 101}]})
    files = dict(calibration_raw=train, evaluation_raw=test)
    for name, obj in objects.items():
        files[name] = tmp_path/(name+'.json')
        files[name].write_text(json.dumps(obj), encoding='utf-8')
    config = dict(target_mode='reference_slant', tag_id='5', firmware=FIRMWARE, A=SOLVER, B={},
                  files={name: dict(path=str(p), sha256=digest(p)) for name, p in files.items()})
    cp = tmp_path/'config.json'
    cp.write_text(json.dumps(config), encoding='utf-8')
    a, b = tmp_path/'a', tmp_path/'b'
    assert run(cp, tmp_path, a) == run(cp, tmp_path, b)
    for name in ('results.jsonl', 'summary.json', 'positions.csv'):
        assert (a/name).read_bytes() == (b/name).read_bytes()
    with pytest.raises(FileExistsError):
        run(cp, tmp_path, a)
    test.write_text(test.read_text(encoding='utf-8')+'\n', encoding='utf-8')
    with pytest.raises(ValueError, match='input_hash_mismatch'):
        run(cp, tmp_path, tmp_path/'invalid')
