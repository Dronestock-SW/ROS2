"""Declared synthetic path must preserve position and time across reversals."""
from copy import deepcopy
import json
from pathlib import Path
import runpy

import pytest


ROOT = Path(__file__).resolve().parents[2]
GENERATE = runpy.run_path(str(ROOT/'tools/generate_gazebo_dynamic_path.py'))['generate']
SPEC = json.loads((ROOT/'config/gazebo_dynamic_path_20260928.json').read_text(encoding='utf-8'))


def test_axis_reversals_are_continuous_and_clock_is_strict():
    rows = list(GENERATE(SPEC))
    assert len(rows) == 480
    assert all(b['time_us'] > a['time_us'] for a, b in zip(rows, rows[1:]))
    assert rows[0]['source'] == 'synthetic_pose_fixture'
    for second, xy in ((0, (2.09, 1.68)), (2, (2.09, 1.68)),
                       (4, (2.69, 1.68)), (6, (2.09, 1.68)),
                       (8.2, (2.09, 2.04)), (9.4, (2.09, 1.68))):
        assert rows[round(second*40)]['position_xyz_m'][:2] == pytest.approx(xy)


def test_overlapping_or_nonfinite_segments_are_rejected():
    spec = deepcopy(SPEC)
    spec['velocity_segments'][1]['start_s'] = 3.
    with pytest.raises(ValueError, match='invalid_path_segment'):
        list(GENERATE(spec))
    spec = deepcopy(SPEC)
    spec['velocity_segments'][0]['vx_m_s'] = float('nan')
    with pytest.raises(ValueError, match='invalid_path_segment'):
        list(GENERATE(spec))
