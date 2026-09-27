"""Q_S10: direct 3D range fit with per-anchor temporary positive bias (handoff section 11).

theta_Q = [x0, y0, x1, y1, b1, b2, b3, b4]   (8 unknowns)
    d_j = sqrt((x0 + x1 u_j - ax)^2 + (y0 + y1 u_j - ay)^2 + (z_ant - az)^2)
    e_j = d_j + b_i - r_j                       0 <= b_i <= 1.0 m
    J   = sum rho_0.12(e_j)/sigma^2 + (lambda_v/2)(x1^2 + y1^2) + sum b_i^2 / (2 * 0.10^2)

Bounded damped Gauss-Newton (Levenberg-Marquardt) on the stacked, Huber-reweighted residual rows;
each step is a least-squares (SVD) solve, halved up to `q_step_halvings` times until the cost drops.
Biases are not remembered: every window estimates them again.
"""
from dataclasses import dataclass, field

import numpy as np

from drone_uwb.processing.h80 import participating


@dataclass
class QFit:
    ok: bool
    reason: str = None
    t_ref_s: float = None
    x0: float = None
    y0: float = None
    x1: float = None
    y1: float = None
    bias: list = None
    converged: bool = False
    iterations: int = 0
    cost: float = None
    rms_m: float = None
    max_m: float = None
    condition: float = None
    n: int = 0
    mask: int = 0
    counts: list = field(default_factory=list)


def solve_q_s10(anchors, idx, s, r, t_ref_s, z_ant, init, settings):
    """init = (x0, y0, x1, y1). Returns a QFit; ok only if converged, finite and well conditioned."""
    part, counts = participating(idx, settings)
    keep = np.isin(idx, part)
    n = int(keep.sum())
    fit = QFit(False, t_ref_s=t_ref_s, n=n, mask=sum(1 << i for i in part), counts=counts)
    if len(part) < 3 or n < settings.min_samples_total:
        fit.reason = 'insufficient_anchors'
        return fit
    ii, rr = idx[keep], r[keep]
    u = (s[keep] - t_ref_s) / settings.window_s
    ax, ay = anchors[ii, 0], anchors[ii, 1]
    dz2 = (z_ant - anchors[ii, 2])**2
    sigma, delta = settings.sigma_r_m, settings.huber_m
    sqrt_lv, sb = np.sqrt(settings.q_velocity_weight), 1.0 / settings.q_bias_prior_m
    prior = np.zeros((6, 8))
    prior[0, 2] = prior[1, 3] = sqrt_lv
    for k in range(4):
        prior[2 + k, 4 + k] = sb
    rows = np.arange(n)

    def evaluate(theta):
        dx = theta[0] + theta[2] * u - ax
        dy = theta[1] + theta[3] * u - ay
        d = np.sqrt(dx * dx + dy * dy + dz2)
        return d + theta[4 + ii] - rr, dx / d, dy / d

    def cost(theta):
        e = evaluate(theta)[0]
        ae = np.abs(e)
        rho = np.where(ae <= delta, 0.5 * e * e, delta * (ae - 0.5 * delta))
        return float(np.sum(rho) / sigma**2 + 0.5 * np.sum((prior @ theta)**2))

    def project(theta):
        theta = theta.copy()
        theta[4:] = np.clip(theta[4:], 0.0, settings.q_bias_max_m)
        return theta

    theta = project(np.array([init[0], init[1], init[2], init[3], 0.0, 0.0, 0.0, 0.0], dtype=float))
    current, damping = cost(theta), 1e-3
    jac_final = None
    for iteration in range(1, settings.q_iterations + 1):
        e, gx, gy = evaluate(theta)
        weight = np.where(np.abs(e) <= delta, 1.0, delta / np.maximum(np.abs(e), 1e-12))
        jac = np.zeros((n, 8))
        jac[:, 0], jac[:, 1], jac[:, 2], jac[:, 3] = gx, gy, gx * u, gy * u
        jac[rows, 4 + ii] = 1.0
        root = np.sqrt(weight) / sigma
        stacked = np.vstack([jac * root[:, None], prior])
        residual = np.concatenate([e * root, prior @ theta])
        jac_final = stacked
        fit.iterations = iteration
        moved, improved = 0.0, False
        while True:
            step = np.linalg.lstsq(np.vstack([stacked, np.sqrt(damping) * np.eye(8)]),
                                   np.concatenate([-residual, np.zeros(8)]), rcond=None)[0]
            if not np.isfinite(step).all():
                damping *= 10.0
                if damping > 1e8:
                    break
                continue
            scale = 1.0
            for _ in range(settings.q_step_halvings + 1):
                trial = project(theta + scale * step)
                trial_cost = cost(trial)
                if np.isfinite(trial_cost) and trial_cost < current:
                    improved = True
                    break
                scale *= 0.5
            if improved:
                moved = float(np.max(np.abs(trial - theta)))
                theta, current = trial, trial_cost
                damping = max(damping / 3.0, 1e-9)
                break
            if float(np.max(np.abs(step))) < 1e-7:
                fit.converged = True   # cost cannot drop: already at the (bounded) optimum
                break
            damping *= 10.0
            if damping > 1e8:
                break
        if fit.converged or moved < 1e-7 and improved:
            fit.converged = True
            break
        if not improved:
            break
    e, _, _ = evaluate(theta)
    fit.cost = current
    fit.rms_m, fit.max_m = float(np.sqrt(np.mean(e**2))), float(np.max(np.abs(e)))
    norms = np.linalg.norm(jac_final, axis=0)
    sing = np.linalg.svd(jac_final / np.where(norms > 0, norms, 1.0), compute_uv=False)
    fit.condition = float(sing[0] / sing[-1]) if sing[-1] > 0 else float('inf')
    fit.bias = [float(v) for v in theta[4:]]
    if not (np.isfinite(theta).all() and np.isfinite(fit.rms_m) and np.isfinite(fit.condition)):
        fit.reason = 'solver_failed'
    elif not fit.converged:
        fit.reason = 'solver_failed'
    elif fit.condition > settings.max_scaled_condition:
        fit.reason = 'ill_conditioned'
    else:
        fit.ok = True
        fit.x0, fit.y0, fit.x1, fit.y1 = (float(v) for v in theta[:4])
    return fit
