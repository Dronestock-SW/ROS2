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

    def position(self, t_s, window_s):
        u = (t_s - self.t_ref_s) / window_s
        return self.x0 + self.x1 * u, self.y0 + self.y1 * u


def participating(idx, settings):
    counts = np.bincount(idx, minlength=4)
    part = [i for i in range(4) if counts[i] >= settings.min_samples_per_anchor]
    return part, counts.tolist()


def fit_h80(anchors, idx, s, r, t_ref_s, settings):
    """idx: anchor index per observation, s: source time [s], r: calibrated slant range [m]."""
    part, counts = participating(idx, settings)
    keep = np.isin(idx, part)
    n = int(keep.sum())
    mask = sum(1 << i for i in part)
    fit = H80Fit(False, t_ref_s=t_ref_s, n=n, mask=mask, counts=counts)
    if len(part) < 3 or n < settings.min_samples_total:
        fit.reason = 'insufficient_anchors'
        return fit
    ii, rr = idx[keep], r[keep]
    u = (s[keep] - t_ref_s) / settings.window_s
    ax, ay = anchors[ii, 0], anchors[ii, 1]
    design = np.column_stack([-2 * ax, -2 * ay, -2 * ax * u, -2 * ay * u, np.ones_like(u), u, u * u])
    rhs = rr**2 - ax**2 - ay**2
    scale = np.max(np.abs(design), axis=0)
    scaled = design / scale
    var = 4 * rr**2 * settings.sigma_r_m**2 + 2 * settings.sigma_r_m**4
    denom = 2 * np.maximum(rr, 0.1)
    weight = 1.0 / var

    def solve(w):
        root = np.sqrt(w)
        phi = np.linalg.lstsq(scaled * root[:, None], rhs * root, rcond=None)[0]
        return phi / scale

    for _ in range(settings.h80_irls_iterations):
        theta = solve(weight)
        e = (design @ theta - rhs) / denom
        updated = (1.0 / var) * np.minimum(1.0, settings.huber_m / np.maximum(np.abs(e), 1e-12))
        done = np.allclose(updated, weight, rtol=1e-6, atol=0.0)
        weight = updated
        fit.iterations += 1
        if done:
            break
    theta = solve(weight)
    sing = np.linalg.svd(scaled * np.sqrt(weight)[:, None], compute_uv=False)
    fit.condition = float(sing[0] / sing[-1]) if sing[-1] > 0 else float('inf')
    predicted_sq = design @ theta + ax**2 + ay**2
    residual = np.sqrt(np.maximum(predicted_sq, 0.0)) - rr
    fit.rms_m, fit.max_m = float(np.sqrt(np.mean(residual**2))), float(np.max(np.abs(residual)))
    if not np.isfinite(theta).all() or not np.isfinite(fit.condition):
        fit.reason = 'solver_failed'
    elif fit.condition > settings.max_scaled_condition:
        fit.reason = 'ill_conditioned'
    else:
        fit.ok = True
        fit.x0, fit.y0, fit.x1, fit.y1 = (float(v) for v in theta[:4])
    return fit
