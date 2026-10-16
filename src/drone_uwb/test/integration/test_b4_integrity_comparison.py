import importlib.util
import json
from pathlib import Path

import pytest

path = Path(__file__).parents[2]/'tools/compare_b4_integrity.py'
spec = importlib.util.spec_from_file_location('b4_compare', path)
tool = importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)


def test_censored_or_all_rejected_gap_is_not_reported_as_zero():
    rows = [(0., (True, False)), (1., (True, False)), (3., (True, False))]
    assert tool.gaps(rows, 1)['longest_including_boundary_s'] == 3.
    assert tool.gaps(rows, 1)['longest_complete_s'] is None
    assert tool.gaps(rows, 1)['all_rejected']


def test_gap_does_not_cross_ground_or_filtered_interval(tmp_path):
    rows = [dict(seq=i, received_monotonic_ns=t*1_000_000_000, airborne=air,
                 replay_publish=True, reason='ok', xy_m=[0., 0.],
                 models={'B_TF': {'source': 'unchanged_B4'}})
            for i, (t, air) in enumerate([(1, True), (2, True), (3, False), (20, True), (21, True)])]
    before = tmp_path/'before'; after = tmp_path/'after'
    before.write_text(''.join(json.dumps(r)+'\n' for r in rows), encoding='utf-8')
    after.write_bytes(before.read_bytes())
    result = tool.compare(before, after)
    assert result['before']['longest_complete_s'] == 1.
    assert result['shared_B4_xy_delta_max_m'] == 0.
    assert tool.compare(before, after, start=20)['before']['published'] == 2
    rows[-1]['seq'] += 1
    after.write_text(''.join(json.dumps(r)+'\n' for r in rows), encoding='utf-8')
    with pytest.raises(ValueError, match='identity'):
        tool.compare(before, after)
