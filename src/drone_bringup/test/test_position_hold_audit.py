"""Guard against optimistic hold scores from missing or misaligned records."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SPEC = importlib.util.spec_from_file_location(
    'hold_audit', Path(__file__).parents[1] / 'tools/analyze_position_hold.py')
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def test_alignment_never_uses_future_stale_or_missing_data():
    data = {'timestamp': np.array([1_000_000, 2_000_000], dtype=np.uint64),
            'x': np.array([3.0, 9.0])}
    output = AUDIT.align(data, 'x', [900_000, 1_100_000, 1_900_000, 2_100_000], 0.25)
    assert np.isnan(output[[0, 2]]).all()
    assert output[[1, 3]].tolist() == [3.0, 9.0]
    assert np.isnan(AUDIT.align(data, 'missing', [1_100_000], 0.25)).all()
    assert np.isnan(AUDIT.align({}, 'x', [1_100_000], 0.25)).all()
    with pytest.raises(ValueError, match='Non-monotonic'):
        AUDIT.align({'timestamp': [2, 1], 'x': [3, 4]}, 'x', [2], 1)


def hold_samples():
    count = 10
    samples = {k: np.zeros(count) for k in (
        'target_x', 'target_y', 'roll', 'pitch', 'xy_reset_counter',
        'vxy_reset_counter', 'heading_reset_counter')}
    samples.update(time_s=1.0 + np.arange(count) / 10,
                   target_timestamp_us=1_000_000 + np.arange(count) * 100_000,
                   qualified=np.ones(count, dtype=bool), manual_valid=np.ones(count),
                   error_m=np.full(count, np.hypot(0.03, 0.04)),
                   speed_m_s=np.full(count, 0.02))
    return samples


SEGMENT = {'start_s': 1.0, 'end_s': 2.0, 'nav_state': 2, 'mode': 'POSCTL'}


def test_fixed_hold_score_is_tracking_error_only():
    result = AUDIT.summarize_window(hold_samples(), SEGMENT, 0)
    assert result['position_hold_candidate']
    assert result['error_m']['rms'] == pytest.approx(0.05)
    assert result['qualification_is_not_flight_acceptance']


@pytest.mark.parametrize('issue', ['old_target', 'missing_rc', 'moving_target',
                                  'stick_input', 'reset', 'missing_reset', 'low_coverage'])
def test_incomplete_or_moving_intervals_are_not_hold_candidates(issue):
    samples = hold_samples()
    if issue == 'old_target':
        samples['target_timestamp_us'][:] = 999_999
    elif issue == 'missing_rc':
        samples['manual_valid'][:] = np.nan
    elif issue == 'moving_target':
        samples['target_x'][:] = np.arange(10) * 0.1
    elif issue == 'stick_input':
        samples['roll'][5] = 0.2
    elif issue == 'reset':
        samples['xy_reset_counter'][5:] = 1
    elif issue == 'missing_reset':
        samples['xy_reset_counter'][:] = np.nan
    else:
        samples['qualified'][:5] = False
    result = AUDIT.summarize_window(samples, SEGMENT, 0)
    assert not result['position_hold_candidate']
    if issue == 'old_target':
        assert result['error_m'] is None
