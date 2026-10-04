"""C: four three-anchor slant fits followed by equal candidate weighting."""
from dataclasses import asdict
from itertools import combinations

import numpy as np

from drone_uwb.processing.solvers.candidate_fusion import uniform_fuse
from drone_uwb.processing.solvers.range_solver import solve_slant_xy


ANCHOR_IDS = ('A1', 'A2', 'A3', 'A4')


def make_triplets(anchor_ids=ANCHOR_IDS):
    if len(anchor_ids) != 4 or set(anchor_ids) != set(ANCHOR_IDS):
        raise ValueError('four_unique_anchor_ids_required')
    return list(combinations(sorted(anchor_ids), 3))


def prepare_inputs(anchors, ranges, heights, anchor_ids, obs_ids):
    make_triplets(anchor_ids)
    if heights is None:
        raise ValueError('missing_z')
    a, r, z = (np.asarray(x, float) for x in (anchors, ranges, heights))
    if z.ndim == 0:
        z = np.full(4, float(z))
    if (a.shape != (4, 3) or r.shape != (4,) or z.shape != (4,)
            or not all(np.isfinite(x).all() for x in (a, r, z)) or np.any((r <= 0) | (r > 80))):
        raise ValueError('invalid_frame')
    obs = list(obs_ids) if obs_ids is not None else list(anchor_ids)
    if len(obs) != 4 or len(set(obs)) != 4 or not all(isinstance(v, str) and v for v in obs):
        raise ValueError('invalid_observation_ids')
    order = [list(anchor_ids).index(v) for v in ANCHOR_IDS]
    return a[order], r[order], z[order], [obs[i] for i in order]


def candidate_record(candidate_id, indices, t_ref_us, obs_ids):
    return dict(candidate_id=candidate_id, anchor_ids=[ANCHOR_IDS[i] for i in indices],
                t_ref_us=t_ref_us, used_obs_ids=[obs_ids[i] for i in indices],
                ok=False, reason='pending', xy_m=None)


def residuals_at(xy, anchors, ranges, heights):
    return np.sqrt(np.sum((np.asarray(xy)-anchors[:, :2])**2, axis=1)
                   +(heights-anchors[:, 2])**2)-ranges


def solve_triplet_xy(anchors, ranges, heights, settings=None):
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            return asdict(solve_slant_xy(anchors, ranges, heights, required_count=3, **(settings or {})))
    except (np.linalg.LinAlgError, FloatingPointError):
        return dict(ok=False, reason='solver_failed', xy_m=None)


def make_candidates(anchors, ranges, heights, *, t_ref_us=0, anchor_ids=ANCHOR_IDS,
                    obs_ids=None, settings=None):
    result = dict(model_id='C', variant='uniform4', ok=False, reason=None, xy_m=None, candidates=[])
    try:
        a, r, z, obs = prepare_inputs(anchors, ranges, heights, anchor_ids, obs_ids)
    except (ValueError, TypeError) as exc:
        result['reason'] = str(exc)
        return result
    for subset in combinations(range(4), 3):
        cid = 'S'+''.join(str(i+1) for i in subset)
        index = list(subset)
        omitted = next(i for i in range(4) if i not in subset)
        candidate = candidate_record(cid, index, t_ref_us, obs)
        fit = solve_triplet_xy(a[index], r[index], z[index], settings)
        candidate.update(ok=fit['ok'], reason=fit['reason'], xy_m=fit['xy_m'], fit=fit,
                         omitted_anchor_id=ANCHOR_IDS[omitted], omitted_residual_m=None,
                         residuals_all_anchors_m=None, rank=2 if fit['ok'] else None)
        if fit['ok']:
            residual = residuals_at(fit['xy_m'], a, r, z)
            candidate.update(residuals_all_anchors_m=residual.tolist(), omitted_residual_m=float(residual[omitted]))
        result['candidates'].append(candidate)
    result.update(uniform_fuse(result['candidates'], ['S123', 'S124', 'S134', 'S234'], t_ref_us))
    if result['ok']:
        result['residuals_all_anchors_m'] = residuals_at(result['xy_m'], a, r, z).tolist()
    return result
