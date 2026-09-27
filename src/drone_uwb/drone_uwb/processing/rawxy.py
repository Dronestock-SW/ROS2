"""Single-cycle, unsmoothed XY (diagnostic only). 3D range model with a fixed antenna height."""
from dataclasses import dataclass

import numpy as np

from drone_uwb.processing.observations import InvalidSample, solve_xy


@dataclass
class RawXY:
    valid: bool
    x: float = None
    y: float = None
    mask: int = 0
    rms_m: float = None
    max_m: float = None
    condition: float = None
    reason: str = None


def solve_raw_xy(anchors, ranges, indices, z_ant, settings):
    """Gauss-Newton on sum_i [d_i(p) - r_i]^2 over the given anchors; needs >= 3 anchors.

    Steps use least squares (SVD), never a normal-equation inverse. Any failed check returns
    valid=False with x/y left as None so nothing downstream can mistake it for a position.
    """
    mask = sum(1 << i for i in indices)
    if len(indices) < 3:
        return RawXY(False, mask=mask, reason='insufficient_anchors')
    idx = list(indices)
    a = anchors[idx]
    r = ranges[idx]
    try:
        p, _ = solve_xy(anchors, ranges, idx)   # linear start; common height term cancels
    except InvalidSample:
        return RawXY(False, mask=mask, reason='ill_conditioned')
    dz = z_ant - a[:, 2]
    for _ in range(20):
        diff = p - a[:, :2]
        d = np.sqrt(np.sum(diff**2, axis=1) + dz**2)
        step = np.linalg.lstsq(diff / d[:, None], -(d - r), rcond=None)[0]
        p = p + step
        if np.max(np.abs(step)) < 1e-7:
            break
    diff = p - a[:, :2]
    d = np.sqrt(np.sum(diff**2, axis=1) + dz**2)
    jac = diff / d[:, None]
    sing = np.linalg.svd(jac, compute_uv=False)
    condition = float(sing[0] / sing[-1]) if sing[-1] > 0 else float('inf')
    residual = d - r
    rms, peak = float(np.sqrt(np.mean(residual**2))), float(np.max(np.abs(residual)))
    result = RawXY(False, mask=mask, rms_m=rms, max_m=peak, condition=condition)
    low = anchors[:, :2].min(axis=0) - settings.raw_xy_region_margin_m
    high = anchors[:, :2].max(axis=0) + settings.raw_xy_region_margin_m
    if not np.isfinite(p).all() or not np.isfinite(condition) or condition > settings.raw_xy_max_condition:
        result.reason = 'ill_conditioned'
    elif rms > settings.raw_xy_max_rms_m:
        result.reason = 'residual_high'
    elif np.any(p < low) or np.any(p > high):
        result.reason = 'out_of_region'
    else:
        result.valid, result.x, result.y = True, float(p[0]), float(p[1])
    return result
