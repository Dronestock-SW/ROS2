"""Shared equal-weight slant-range least squares for A and C subsets."""
from dataclasses import dataclass

import numpy as np


@dataclass
class RangeFit:
    ok: bool
    reason: str
    xy_m: list = None
    residuals_m: list = None
    rms_m: float = None
    condition: float = None
    iterations: int = 0


def solve_slant_xy(anchors, ranges, z_m, max_iterations=40, step_tol_m=1e-7,
                     condition_max=1e6, required_count=4):
    """Use all required ranges; no robust weights, spatial gate, or residual rejection.

    z_m is one supplied height or per-anchor time-aligned heights. XY is approximated
    as one position across the short measurement frame. Success needs convergence.
    """
    a, r, z = (np.asarray(v, dtype=float) for v in (anchors, ranges, z_m))
    if z.ndim == 0:
        z = np.full(required_count, float(z))
    if (required_count not in (3, 4) or a.shape != (required_count, 3)
            or r.shape != (required_count,) or z.shape != (required_count,)
            or not all(np.isfinite(v).all() for v in (a, r, z)) or np.any(r <= 0)):
        return RangeFit(False, 'invalid_solver_input')
    if (not isinstance(max_iterations, int) or isinstance(max_iterations, bool)
            or max_iterations < 1 or not np.isfinite([step_tol_m, condition_max]).all()
            or step_tol_m <= 0 or condition_max <= 1):
        raise ValueError('invalid_solver_settings')
    if np.linalg.matrix_rank(a[:, :2]-a[0, :2]) < 2:
        return RangeFit(False, 'degenerate_anchor_geometry')
    dz = z-a[:, 2]
    horizontal_sq = r*r-dz*dz
    matrix = 2*(a[1:, :2]-a[0, :2])
    rhs = (horizontal_sq[0]-horizontal_sq[1:]
           + np.sum(a[1:, :2]**2, axis=1)-np.sum(a[0, :2]**2))
    p = np.linalg.lstsq(matrix, rhs, rcond=None)[0]
    for iteration in range(1, max_iterations+1):
        diff = p-a[:, :2]
        predicted = np.sqrt(np.sum(diff*diff, axis=1)+dz*dz)
        if np.any(predicted <= 1e-12):
            return RangeFit(False, 'singular_range_geometry', iterations=iteration)
        residual = predicted-r
        jacobian = diff/predicted[:, None]
        singular = np.linalg.svd(jacobian, compute_uv=False)
        condition = singular[0]/singular[-1] if singular[-1] > 0 else np.inf
        if not np.isfinite(condition) or condition > condition_max:
            return RangeFit(False, 'ill_conditioned', iterations=iteration)
        step = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
        if np.max(np.abs(step)) <= step_tol_m:
            return RangeFit(True, 'ok', p.tolist(), residual.tolist(),
                              float(np.sqrt(np.mean(residual**2))), float(condition), iteration)
        cost = float(residual @ residual)
        for halvings in range(20):
            trial = p + step*(.5**halvings)
            trial_residual = np.sqrt(np.sum((trial-a[:, :2])**2, axis=1)+dz*dz)-r
            if np.isfinite(trial).all() and float(trial_residual @ trial_residual) <= cost:
                p = trial
                break
        else:
            return RangeFit(False, 'no_descent', iterations=iteration)
    return RangeFit(False, 'iteration_limit', iterations=max_iterations)
