"""Independent checks of weighting, correlation and covariance contracts."""
import numpy as np
import pytest

from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy
from drone_uwb.processing.weighted_xy import solve_weighted_xy


ANCHORS = np.array([[-3., -2.5, 2.2], [3., -2.5, 2.2], [-3., 2.5, 2.2], [3.1, 2.3, 2.2]])
POINT = np.array([.7, .3, 1.3])
RANGES = np.linalg.norm(ANCHORS-POINT, axis=1)


def test_equal_quality_matches_A_without_changing_A():
    ranges = RANGES+np.array([.03, -.01, .07, .02])
    a = solve_uniform_xy(ANCHORS, ranges, POINT[2])
    w = solve_weighted_xy(ANCHORS, ranges, POINT[2], np.eye(4)*.03**2)
    assert a.ok and w['ok']
    np.testing.assert_allclose(w['xy_m'], a.xy_m, atol=1e-10)


def test_unreliable_range_has_less_effect_when_declared_less_precise():
    ranges = RANGES+np.array([0, .35, 0, 0])
    uniform = solve_weighted_xy(ANCHORS, ranges, POINT[2], np.eye(4)*.03**2)
    weighted = solve_weighted_xy(ANCHORS, ranges, POINT[2], np.diag([.03, .3, .03, .03])**2)
    assert uniform['ok'] and weighted['ok']
    error = lambda fit: np.linalg.norm(np.array(fit['xy_m'])-POINT[:2])
    assert error(weighted) < error(uniform)/10
    # A remains equally weighted; no reference position is given to either fit.


def test_correlated_noise_optimum_covariance_and_permutation():
    ranges = RANGES+np.array([.02, -.01, .04, .01])
    covariance = np.diag([.02, .08, .03, .05])**2+np.ones((4, 4))*.01**2
    fit = solve_weighted_xy(ANCHORS, ranges, POINT[2], covariance)
    assert fit['ok']
    diff = np.array(fit['xy_m'])-ANCHORS[:, :2]
    predicted = np.sqrt(np.sum(diff**2, axis=1)+(POINT[2]-ANCHORS[:, 2])**2)
    jacobian = diff/predicted[:, None]
    residual = predicted-ranges
    np.testing.assert_allclose(jacobian.T@np.linalg.solve(covariance, residual), [0, 0], atol=1e-4)
    information = jacobian.T@np.linalg.solve(covariance, jacobian)
    np.testing.assert_allclose(information@fit['covariance_xy_m2'], np.eye(2), atol=1e-10)
    permutation = [2, 0, 3, 1]
    reordered = solve_weighted_xy(ANCHORS[permutation], ranges[permutation], POINT[2],
                                   covariance[np.ix_(permutation, permutation)])
    assert reordered['ok']
    np.testing.assert_allclose(reordered['xy_m'], fit['xy_m'], atol=1e-7)


@pytest.mark.parametrize('covariance', [None, np.zeros((4, 4)), np.eye(3),
                                       np.diag([1., -1., 1., 1.]), np.full((4, 4), np.nan),
                                       [[1, .2, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]])
def test_invalid_covariance_is_not_silently_uniform(covariance):
    result = solve_weighted_xy(ANCHORS, RANGES, POINT[2], covariance)
    assert not result['ok'] and result['reason'] == 'invalid_range_covariance'


def test_degenerate_geometry_and_missing_height():
    anchors = ANCHORS.copy()
    anchors[:, 1] = 0
    assert solve_weighted_xy(anchors, RANGES, POINT[2], np.eye(4))['reason'] == 'degenerate_anchor_geometry'
    assert not solve_weighted_xy(ANCHORS, RANGES, None, np.eye(4))['ok']
