"""H80: space-time range-squared regression over the last window (handoff section 10).

With u_j = (s_j - t)/W, x(u) = x0 + x1 u, y(u) = y0 + y1 u and q(u) = q0 + q1 u + q2 u^2 (which
absorbs the antenna height, valid only for equal anchor heights):

    r_j^2 - ax^2 - ay^2 = q(u) - 2 ax x(u) - 2 ay y(u)      ->  A_j theta = c_j

theta = [x0, y0, x1, y1, q0, q1, q2]. Solved by column-scaled weighted least squares (SVD) with
Huber IRLS. No matrix inverse is formed.
"""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class H80Fit:
    ok: bool
    reason: str = None
    t_ref_s: float = None
    x0: float = None
    y0: float = None
    x1: float = None
    y1: float = None
    rms_m: float = None
    max_m: float = None
    condition: float = None
    n: int = 0
    mask: int = 0
    counts: list = field(default_factory=list)
    iterations: int = 0
    rank: int = 0
    coefficients: list = field(default_factory=list)
    converged: bool = False

    def position(self, t_s, window_s):
        u = (t_s - self.t_ref_s) / window_s
        return self.x0 + self.x1 * u, self.y0 + self.y1 * u


def participating(idx, settings):
    counts = np.bincount(idx, minlength=4)
    part = [i for i in range(4) if counts[i] >= settings.min_samples_per_anchor]
    return part, counts.tolist()


def fit_h80(anchors, idx, s, r, t_ref_s, settings):
    """idx: anchor index per observation, s: source time [s], r: calibrated slant range [m]."""
    anchors, idx, s, r = (np.asarray(v) for v in (anchors, idx, s, r))
    fit = H80Fit(False, reason='invalid_input')
    if (anchors.shape != (4, 3) or idx.ndim != 1 or s.shape != idx.shape or r.shape != idx.shape
            or not np.issubdtype(idx.dtype, np.integer) or np.any((idx < 0) | (idx > 3))
            or not all(np.isfinite(v).all() for v in (anchors, s, r))
            or not np.isfinite(t_ref_s) or np.any(r <= 0)):
        return fit
    if np.ptp(anchors[:, 2]) > 1e-9:
        fit.reason = 'unequal_anchor_heights'
        return fit
    if (not np.isfinite([settings.window_s, settings.sigma_r_m, settings.huber_m,
                         settings.max_scaled_condition]).all()
            or min(settings.window_s, settings.sigma_r_m, settings.huber_m) <= 0
            or settings.max_scaled_condition <= 1
            or not isinstance(settings.h80_irls_iterations, int)
            or isinstance(settings.h80_irls_iterations, bool) or settings.h80_irls_iterations < 1):
        raise ValueError('invalid_h80_settings')
    if np.any(s > t_ref_s) or np.any(s < t_ref_s-settings.window_s-1e-9):
        fit.reason = 'outside_causal_window'
        return fit
    part, counts = participating(idx, settings)
    # idx is already validated in [0, 3]. A fixed membership table avoids the
    # generic sorting/range dispatch of isin on every four-anchor fit.
    included = np.zeros(4, dtype=bool)
    included[part] = True
    keep = included[idx]
    n = int(keep.sum())
    mask = sum(1 << i for i in part)
    fit = H80Fit(False, t_ref_s=t_ref_s, n=n, mask=mask, counts=counts)
    if len(part) < getattr(settings, 'required_anchor_count', 3) or n < settings.min_samples_total:
        fit.reason = 'insufficient_anchors'
        return fit
    ii, rr = idx[keep], r[keep]
    u = (s[keep] - t_ref_s) / settings.window_s
    ax, ay = anchors[ii, 0], anchors[ii, 1]
    design = np.column_stack([-2 * ax, -2 * ay, -2 * ax * u, -2 * ay * u, np.ones_like(u), u, u * u])
    with np.errstate(over='ignore', invalid='ignore'):
        rhs = rr**2 - ax**2 - ay**2
        var = 4 * rr**2 * settings.sigma_r_m**2 + 2 * settings.sigma_r_m**4
    if not all(np.isfinite(v).all() for v in (design, rhs, var)) or np.any(var <= 0):
        fit.reason = 'solver_failed'
        return fit
    scale = np.max(np.abs(design), axis=0)
    if np.any(scale == 0):
        fit.reason = 'rank_deficient'
        return fit
    scaled = design / scale
    denom = 2 * np.maximum(rr, 0.1)
    base_weight = 1.0 / var
    weight = base_weight

    def solve(w):
        root = np.sqrt(w)
        phi, _, rank, sing = np.linalg.lstsq(scaled * root[:, None], rhs * root, rcond=None)
        fit.rank = int(rank)
        if rank < 7:
            raise ValueError('rank_deficient')
        condition = float(sing[0] / sing[-1])
        if not np.isfinite(condition):
            raise ValueError('solver_failed')
        fit.condition = condition
        if condition > settings.max_scaled_condition:
            raise ValueError('ill_conditioned')
        return phi / scale

    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            for _ in range(settings.h80_irls_iterations):
                theta = solve(weight)
                e = (design @ theta - rhs) / denom
                updated = base_weight * np.minimum(1.0, settings.huber_m / np.maximum(np.abs(e), 1e-12))
                # Weights reaching this point are finite nonnegative floats.
                # This is allclose(rtol=1e-6, atol=0) without its scalar/NaN/
                # infinity dispatch. Keep the same SVD and stopping tolerance.
                fit.converged = bool(np.all(np.abs(updated-weight) <= 1e-6*np.abs(weight)))
                weight = updated
                fit.iterations += 1
                if fit.converged:
                    break
            theta = solve(weight)
    except (np.linalg.LinAlgError, FloatingPointError):
        fit.reason = 'solver_failed'
        return fit
    except ValueError as exc:
        fit.reason = str(exc)
        return fit
    predicted_sq = design @ theta + ax**2 + ay**2
    residual = np.sqrt(np.maximum(predicted_sq, 0.0)) - rr
    fit.rms_m, fit.max_m = float(np.sqrt(np.mean(residual**2))), float(np.max(np.abs(residual)))
    if not np.isfinite(theta).all() or not np.isfinite([fit.condition, fit.rms_m, fit.max_m]).all():
        fit.reason = 'solver_failed'
        fit.rms_m = fit.max_m = None
    elif fit.condition > settings.max_scaled_condition:
        fit.reason = 'ill_conditioned'
    else:
        fit.ok = True
        fit.reason = 'ok'
        fit.coefficients = [float(v) for v in theta]
        fit.x0, fit.y0, fit.x1, fit.y1 = (float(v) for v in theta[:4])
    return fit
