"""Synthetic checks validate diagnostic math, not physical timing accuracy."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest


PATH = Path(__file__).resolve().parents[2] / 'tools' / 'analyze_lidar_yaw_timing.py'
SPEC = importlib.util.spec_from_file_location('lidar_yaw_timing', PATH)
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


@pytest.mark.parametrize('offset', [-.035, 0., .025])
def test_offset_sign_and_magnitude(offset):
    times = np.arange(0., 10., .002)
    integral = .3 * np.sin(times * 2.3) + .07 * np.sin(times * 5.1)
    start = np.arange(1., 9., .1)
    end = start + .1
    yaw = np.interp(end + offset, times, integral) - np.interp(start + offset, times, integral)
    best, _ = analysis.fit_offsets(start, end, yaw, times, integral)
    assert best['offset_s'] == pytest.approx(offset)
    assert best['rms_deg'] < 1e-9


def test_constant_rotation_cannot_identify_offset():
    times = np.arange(0., 10., .002)
    start = np.arange(1., 9., .1)
    end = start + .1
    _, profile = analysis.fit_offsets(start, end, .2 * (end - start), times, .2 * times)
    assert np.ptp([p['rms_deg'] for p in profile]) < 1e-10


def test_deskew_without_pairs_cannot_claim_timing():
    result = analysis.deskew_profile([], 0., np.array([0., 1.]), np.zeros(2), [], analysis.Config())
    assert result['common_pairs'] == 0
    assert 'best_offset_s' not in result
