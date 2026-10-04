"""Three-anchor fits, circle branch handling, and honest candidate fusion."""
from copy import deepcopy
from itertools import permutations

import numpy as np
import pytest

from drone_uwb.processing.solvers.candidate_fusion import uniform_fuse
from drone_uwb.processing.solvers.intersections import DSettings, circle_intersections, select_intersection
from drone_uwb.processing.solvers.intersections import make_candidates as model_d
from drone_uwb.processing.solvers.triplets import make_candidates as model_c, make_triplets
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy
from drone_uwb.processing.runner import json_line
from test_static_a import ANCHORS, REFERENCE


RANGES = np.linalg.norm(ANCHORS-REFERENCE, axis=1)


@pytest.mark.parametrize('model', [model_c, model_d])
def test_exact_geometry_and_all_permutations(model):
    original = model(ANCHORS, RANGES, REFERENCE[2])
    assert original['ok']
    np.testing.assert_allclose(original['xy_m'], REFERENCE[:2], atol=1e-6)
    assert len(original['candidates']) == 4
    assert original['weights'] == {x: .25 for x in ('S123', 'S124', 'S134', 'S234')}
    assert original['covariance_kind'] == 'unknown'
    for order in permutations(range(4)):
        index = list(order)
        result = model(ANCHORS[index], RANGES[index], REFERENCE[2],
                       anchor_ids=[f'A{i+1}' for i in order])
        assert result == original
    if model is model_d:
        assert len(original['pair_candidates']) == 12
        assert all(s['ok'] for s in original['pair_candidates'])
        np.testing.assert_allclose(original['xy_m'],
                                   np.mean([s['xy_m'] for s in original['pair_candidates']], axis=0))
    json_line(original)


def test_C_one_anchor_bias_affects_exactly_its_three_subsets():
    contaminated = RANGES.copy()
    contaminated[0] += .5
    result = model_c(ANCHORS, contaminated, REFERENCE[2])
    assert result['ok']
    for c in result['candidates']:
        error = np.linalg.norm(np.array(c['xy_m'])-REFERENCE[:2])
        assert (error < 1e-6) == (c['candidate_id'] == 'S234')
    assert np.linalg.norm(np.array(result['xy_m'])-REFERENCE[:2]) > .1


def test_C_failure_does_not_silently_reduce_candidate_count_or_change_A_contract():
    anchors = ANCHORS.copy()
    anchors[:3, :2] = [[0, 0], [1, 0], [2, 0]]
    ranges = np.linalg.norm(anchors-REFERENCE, axis=1)
    result = model_c(anchors, ranges, REFERENCE[2])
    assert not result['ok'] and result['reason'] == 'incomplete_candidate_set'
    assert result['candidates'][0]['reason'] == 'degenerate_anchor_geometry'
    assert any(c['ok'] for c in result['candidates'][1:])
    assert not solve_uniform_xy(ANCHORS[:3], RANGES[:3], REFERENCE[2]).ok


@pytest.mark.parametrize('model', [model_c, model_d])
@pytest.mark.parametrize('fault', ['missing_z', 'missing_anchor', 'bad_distance', 'duplicate_id'])
def test_model_rejects_missing_or_invalid_frame(model, fault):
    a, r, z = ANCHORS.copy(), RANGES.copy(), REFERENCE[2]
    ids = ['A1', 'A2', 'A3', 'A4']
    if fault == 'missing_z':
        z = None
    elif fault == 'missing_anchor':
        r[3] = np.nan
    elif fault == 'bad_distance':
        r[1] = -1
    else:
        ids[3] = 'A3'
    result = model(a, r, z, anchor_ids=ids)
    assert not result['ok'] and result['xy_m'] is None
    json_line(result)


@pytest.mark.parametrize('ra,rb,distance,reason,count', [
    (2., 2., 2., 'two_intersections', 2), (1., 1., 2., 'tangent', 1),
    (1., 1., 3., 'separate_circles', 0), (3., 1., 1., 'contained_circles', 0),
    (1., 1., 0., 'coincident_centers', 0), (2., 1., 1., 'tangent', 1),
])
def test_circle_branches(ra, rb, distance, reason, count):
    result = circle_intersections([0, 0], ra, [distance, 0], rb)
    assert result['reason'] == reason and len(result['points_xy_m']) == count
    for p in result['points_xy_m']:
        assert abs(np.linalg.norm(p)-ra) < 1e-9
        assert abs(np.linalg.norm(np.array(p)-[distance, 0])-rb) < 1e-9


def test_third_anchor_selects_mirror_and_preserves_ambiguity():
    points = [[1, 1], [1, -1]]
    selected = select_intersection(points, [1, 3, 2], np.sqrt(5), 1, 1e-6)
    assert selected['ok'] and selected['xy_m'] == [1, 1]
    ambiguous = select_intersection(points, [3, 0, 2], np.sqrt(6), 1, 1e-6)
    assert not ambiguous['ok'] and ambiguous['reason'] == 'ambiguous_intersection'


def test_numeric_roundoff_is_not_radius_expansion():
    result = circle_intersections([0, 0], 1, [2+1e-12, 0], 1)
    assert result['ok'] and result['numeric_clamped']
    assert len(result['points_xy_m']) == 1
    separated = circle_intersections([0, 0], 1, [2+1e-5, 0], 1)
    assert not separated['ok']
    with pytest.raises(ValueError):
        DSettings(intersection_policy='bounded_expansion')


def test_D_impossible_projection_records_failed_slots_without_cloning():
    ranges = RANGES.copy()
    ranges[0] = .1
    result = model_d(ANCHORS, ranges, REFERENCE[2])
    assert not result['ok'] and result['projection_reasons'][0] == 'negative_horizontal_range_sq'
    assert len(result['pair_candidates']) == 12
    assert sum(s['ok'] for s in result['pair_candidates']) == 3
    assert len(result['used_obs_ids']) == 0
    assert result['correction_m'] == [0.]*4


def test_fusion_rejects_duplicates_stale_candidates_and_preserves_provenance():
    candidates = model_c(ANCHORS, RANGES, REFERENCE[2], t_ref_us=10)['candidates']
    ids = [c['candidate_id'] for c in candidates]
    result = uniform_fuse(candidates, ids, 10)
    assert result['used_obs_ids'] == ['A1', 'A2', 'A3', 'A4']
    assert uniform_fuse(candidates+candidates[:1], ids, 10)['reason'] == 'duplicate_candidate_id'
    changed = deepcopy(candidates)
    changed[0]['t_ref_us'] = 9
    assert uniform_fuse(changed, ids, 10)['reason'] == 'stale_candidate'
    assert not uniform_fuse(candidates[:3], ids, 10)['ok']


def test_per_sample_heights_and_candidate_inputs_are_not_modified():
    z = np.array([1.1, 1.12, 1.14, 1.16])
    positions = np.column_stack([np.full(4, REFERENCE[0]), np.full(4, REFERENCE[1]), z])
    ranges = np.linalg.norm(ANCHORS-positions, axis=1)
    before = ranges.copy()
    for model in (model_c, model_d):
        result = model(ANCHORS, ranges, z)
        assert result['ok']
        np.testing.assert_allclose(result['xy_m'], REFERENCE[:2], atol=1e-6)
        np.testing.assert_array_equal(ranges, before)
    assert len(make_triplets()) == 4
