"""Compare fixed geometry reuse with the original solver and restart contract."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from drone_uwb.contracts.protocol import InvalidSample
from drone_uwb.processing.cached_xy import CachedXY
from drone_uwb.processing.measured_btf import MeasuredBtf
from drone_uwb.processing.solvers.observations import solve_xy


PACKAGE = Path(__file__).resolve().parents[2]
CONFIG = PACKAGE / 'config/runtime/uwb_btf_real.json'


def config():
    return json.loads(CONFIG.read_text(encoding='utf-8'))


def rectangle(width):
    return np.array([[0, 0, .15], [width, 0, .15],
                     [0, 4.6, .15], [width, 4.6, .15]])


def outcome(solve):
    try:
        xy, residual = solve()
        return 'ok', xy, residual
    except InvalidSample as exc:
        return str(exc), None, None


def test_numeric_and_rejection_contract_in_2763_cases():
    rng = np.random.default_rng(20261015)
    anchors = rectangle(6.3)
    layouts = [anchors]
    counts = {'ok': 0, 'degenerate_geometry': 0}

    def condition(width):
        points = rectangle(width)
        matrix = 2 * (points[1:, :2] - points[0, :2])
        _, _, _, singular = np.linalg.lstsq(matrix, np.ones(3), rcond=None)
        return float(singular[0] / singular[-1])

    # Locate the actual singular-value boundary, not an aspect-ratio proxy.
    for side, low, high in (('narrow', .001, 4.6), ('wide', 4.6, 1000.)):
        for _ in range(80):
            middle = (low + high) / 2
            if (condition(middle) > 30) == (side == 'narrow'):
                low = middle
            else:
                high = middle
        widths = [low * (1 - 1e-10), low, high, high * (1 + 1e-10)]
        assert any(condition(w) > 30 for w in widths)
        assert any(condition(w) <= 30 for w in widths)
        layouts.extend(rectangle(w) for w in widths)
    for width in (0., .001, .1, 4.6/30*(1-1e-12), 4.6/30,
                  4.6/30*(1+1e-12), 1., 6.3, 30., 138.,
                  138.*(1+1e-12), 1000.):
        layouts.append(rectangle(width))
    for _ in range(20):
        width, height = rng.uniform(2, 20, 2)
        angle = rng.uniform(-np.pi, np.pi)
        rotation = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
        xy = np.array([[0, 0], [width, 0], [0, height], [width, height]])
        xy = xy @ rotation.T + rng.uniform(-100, 100, 2)
        layouts.append(np.column_stack((xy, np.full(4, .15))))

    def check(points, ranges, indices, cache):
        original = outcome(lambda: solve_xy(points, ranges, list(indices)))
        cached = outcome(lambda: cache.solve(ranges))
        assert original[0] == cached[0]
        counts[original[0]] += 1
        if original[0] == 'ok':
            np.testing.assert_allclose(original[1], cached[1], rtol=0, atol=1e-9)
            assert abs(original[2] - cached[2]) <= 1e-9

    for points in layouts:
        for indices in ((0, 1, 2, 3), (3, 2, 1, 0), (0, 1, 2)):
            cache = CachedXY(points, indices)
            for _ in range(20):
                ranges = rng.uniform(.1, 80, 4) - [.01, -.02, .03, -.04]
                check(points, ranges, indices, cache)
            assert cache.builds == 1 and cache.calls == 20
    cache = CachedXY(anchors)
    for point in (anchors[0], np.array([2., 2., .15]), np.array([2., 2., 1.5])):
        ranges = np.linalg.norm(anchors - point, axis=1)
        for noise in np.linspace(-.2, .2, 101):
            varied = ranges.copy()
            varied[3] += noise
            check(anchors, varied, (0, 1, 2, 3), cache)
    assert sum(counts.values()) == 2763
    assert counts['ok'] > 0 and counts['degenerate_geometry'] > 0


def test_geometry_is_owned_and_read_only_while_ranges_remain_live():
    points = rectangle(6.3)
    original = points.copy()
    cache = CachedXY(points)
    identity = cache.audit()['geometry_sha256']
    ranges = np.array([3.1, 4.2, 5.3, 6.4])
    before, _ = cache.solve(ranges)
    points[1, 0] += 2
    after, _ = cache.solve(ranges)
    np.testing.assert_array_equal(before, after)
    assert cache.audit()['geometry_sha256'] == identity
    for array in (cache.others, cache.matrix, cache.constant,
                  cache.denominator, cache.inverse):
        with pytest.raises(ValueError):
            array.flat[0] = 0
    ranges[2] += .1
    changed, residual = cache.solve(ranges)
    expected, expected_residual = solve_xy(original, ranges, [0, 1, 2, 3])
    assert not np.array_equal(changed, before)
    np.testing.assert_allclose(changed, expected, rtol=0, atol=1e-9)
    assert abs(residual - expected_residual) <= 1e-9


def test_clock_and_model_reset_preserve_fixed_geometry_cache():
    processor = MeasuredBtf(config())
    cache = processor.instant_xy_cache
    processor.reset_models()
    processor.reset()
    assert processor.instant_xy_cache is cache
    assert cache.audit()['builds'] == 1


def test_config_file_change_takes_effect_in_a_new_os_process(tmp_path):
    old = config()
    new = copy.deepcopy(old)
    new['anchors_xyz_m'] = [[1., 2., .15], [9.1, 2., .15],
                           [1., 7.2, .15], [9.1, 7.2, .15]]
    path = tmp_path / 'config.json'
    replacement = tmp_path / 'replacement.json'
    path.write_text(json.dumps(old), encoding='utf-8')
    replacement.write_text(json.dumps(new), encoding='utf-8')
    # The actual processor reads once. A file change cannot mutate its cache.
    worker = '''
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, sys.argv[1])
from drone_uwb.processing.measured_btf import MeasuredBtf
path = Path(sys.argv[2])
processor = MeasuredBtf(json.loads(path.read_text(encoding='utf-8')))
def query():
    xy, residual = processor.instant_xy_cache.solve(np.array([3.1,4.2,5.3,6.4]))
    return dict(xy=xy.tolist(), residual=residual, cache=processor.instant_xy_cache.audit())
before = query()
if len(sys.argv) > 3:
    path.write_text(Path(sys.argv[3]).read_text(encoding='utf-8'), encoding='utf-8')
print(json.dumps(dict(before=before, after=query())))
'''

    def run(*extra):
        result = subprocess.run(
            [sys.executable, '-c', worker, str(PACKAGE), str(path), *extra],
            check=True, capture_output=True, text=True, encoding='utf-8', timeout=30)
        return json.loads(result.stdout)

    first = run(str(replacement))
    restarted = run()['before']
    assert first['before']['xy'] == first['after']['xy']
    assert first['before']['cache']['geometry_sha256'] == first['after']['cache']['geometry_sha256']
    assert first['before']['cache']['geometry_sha256'] != restarted['cache']['geometry_sha256']
    for settings, result in ((old, first['before']), (new, restarted)):
        xy, residual = solve_xy(np.array(settings['anchors_xyz_m']),
                                np.array([3.1, 4.2, 5.3, 6.4]), [0, 1, 2, 3])
        np.testing.assert_allclose(result['xy'], xy, rtol=0, atol=1e-9)
        assert abs(result['residual'] - residual) <= 1e-9
        assert result['cache']['builds'] == 1
    assert np.max(np.abs(np.array(first['before']['xy']) - restarted['xy'])) > .1
