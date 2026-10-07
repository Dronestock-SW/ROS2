"""Four-anchor weighted slant-range fit with a declared distance covariance.

The covariance is an input noise model, never inferred from reference XY.
Returned XY covariance is conditional on that model and supplied heights/map.
"""
import numpy as np


def solve_weighted_xy(anchors, ranges, heights, range_covariance_m2,
                      max_iterations=40, step_tol_m=1e-7, condition_max=1e6):
    result = dict(ok=False, reason='invalid_solver_input', xy_m=None, residuals_m=None,
                  rms_m=None, weighted_cost=None, covariance_xy_m2=None,
                  covariance_kind='conditional_range_noise_model', iterations=0, condition=None)
    try:
        a, r, z, covariance = (np.asarray(v, float) for v in
                                (anchors, ranges, heights, range_covariance_m2))
        if z.ndim == 0:
            z = np.full(4, float(z))
        if (a.shape != (4, 3) or r.shape != (4,) or z.shape != (4,)
                or not all(np.isfinite(v).all() for v in (a, r, z)) or np.any(r <= 0)):
            return result
        if (covariance.shape != (4, 4) or not np.isfinite(covariance).all()
                or not np.allclose(covariance, covariance.T, rtol=1e-10, atol=1e-14)):
            result['reason'] = 'invalid_range_covariance'
            return result
        try:
            chol = np.linalg.cholesky(covariance)
        except np.linalg.LinAlgError:
            result['reason'] = 'invalid_range_covariance'
            return result
        if (not isinstance(max_iterations, int) or isinstance(max_iterations, bool) or max_iterations < 1
                or not np.isfinite([step_tol_m, condition_max]).all()
                or step_tol_m <= 0 or condition_max <= 1):
            raise ValueError('invalid_solver_settings')
        if np.linalg.matrix_rank(a[:, :2]-a[0, :2]) < 2:
            result['reason'] = 'degenerate_anchor_geometry'
            return result
        dz = z-a[:, 2]
        h2 = r*r-dz*dz
        matrix = 2*(a[1:, :2]-a[0, :2])
        rhs = h2[0]-h2[1:]+np.sum(a[1:, :2]**2, axis=1)-np.sum(a[0, :2]**2)
        p = np.linalg.lstsq(matrix, rhs, rcond=None)[0]
        for iteration in range(1, max_iterations+1):
            result['iterations'] = iteration
            diff = p-a[:, :2]
            predicted = np.sqrt(np.sum(diff*diff, axis=1)+dz*dz)
            if np.any(predicted <= 1e-12):
                result['reason'] = 'singular_range_geometry'
                return result
            residual = predicted-r
            weighted_residual = np.linalg.solve(chol, residual)
            jacobian = np.linalg.solve(chol, diff/predicted[:, None])
            _, singular, vt = np.linalg.svd(jacobian, full_matrices=False)
            condition = singular[0]/singular[-1] if singular[-1] > 0 else np.inf
            if not np.isfinite(condition) or condition > condition_max:
                result['reason'] = 'ill_conditioned'
                return result
            step = np.linalg.lstsq(jacobian, -weighted_residual, rcond=None)[0]
            cost = float(weighted_residual@weighted_residual)
            if np.max(np.abs(step)) <= step_tol_m:
                position_covariance = (vt.T/(singular*singular))@vt
                result.update(ok=True, reason='ok', xy_m=p.tolist(), residuals_m=residual.tolist(),
                              rms_m=float(np.sqrt(np.mean(residual**2))), weighted_cost=cost,
                              covariance_xy_m2=position_covariance.tolist(), condition=float(condition))
                return result
            for halves in range(20):
                trial = p+step*.5**halves
                trial_e = np.sqrt(np.sum((trial-a[:, :2])**2, axis=1)+dz*dz)-r
                trial_w = np.linalg.solve(chol, trial_e)
                if np.isfinite(trial).all() and float(trial_w@trial_w) <= cost:
                    p = trial
                    break
            else:
                result['reason'] = 'no_descent'
                return result
        result['reason'] = 'iteration_limit'
    except (np.linalg.LinAlgError, FloatingPointError, OverflowError):
        result['reason'] = 'numeric_failure'
    return result
