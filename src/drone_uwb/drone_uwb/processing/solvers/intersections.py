"""D strict: circle intersections selected by the third slant range.

Three selected slots form each triplet candidate; all four triplet candidates
must succeed. No radius expansion, truth-based choice, or old candidate reuse.
"""
from dataclasses import asdict, dataclass
from itertools import combinations

import numpy as np

from drone_uwb.processing.solvers.candidate_fusion import uniform_fuse
from drone_uwb.processing.solvers.triplets import ANCHOR_IDS, candidate_record, prepare_inputs, residuals_at


@dataclass(frozen=True)
class DSettings:
    intersection_policy: str = 'strict'
    tie_margin_m: float = 1e-6
    min_horizontal_range_m: float = 1e-4
    numeric_tolerance_m2: float = 1e-10
    min_center_separation_m: float = 1e-6

    def __post_init__(self):
        if self.intersection_policy != 'strict':
            raise ValueError('only_strict_intersections_implemented')
        for name, value in asdict(self).items():
            if name == 'intersection_policy':
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError('invalid_D_setting:'+name)
            if value < 0 or (name != 'tie_margin_m' and value == 0):
                raise ValueError('invalid_D_setting:'+name)


def circle_intersections(center_a, radius_a, center_b, radius_b, settings=None):
    cfg = settings or DSettings()
    a, b = np.asarray(center_a, float), np.asarray(center_b, float)
    radii = np.array([radius_a, radius_b], float)
    result = dict(ok=False, reason='invalid_circle', points_xy_m=[], numeric_clamped=False)
    if (a.shape != (2,) or b.shape != (2,) or not all(np.isfinite(x).all() for x in (a, b, radii))
            or np.any(radii < 0)):
        return result
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            delta = b-a
            distance = float(np.linalg.norm(delta))
            if distance <= cfg.min_center_separation_m:
                result['reason'] = 'coincident_centers'
                return result
            along = (radii[0]**2-radii[1]**2+distance**2)/(2*distance)
            height_sq = float(radii[0]**2-along**2)
            result['intersection_height_sq_m2'] = height_sq
            if height_sq < -cfg.numeric_tolerance_m2:
                result['reason'] = 'separate_circles' if distance > radii.sum() else 'contained_circles'
                return result
            result['numeric_clamped'] = height_sq < 0
            height = float(np.sqrt(max(height_sq, 0.)))
            direction = delta/distance
            base = a+along*direction
            points = [base] if height == 0 else [base+height*np.array([-direction[1], direction[0]]),
                                                base-height*np.array([-direction[1], direction[0]])]
            if not np.isfinite(points).all():
                result['reason'] = 'numeric_failure'
                return result
            result.update(ok=True, reason='tangent' if len(points) == 1 else 'two_intersections',
                          points_xy_m=[p.tolist() for p in points])
    except (FloatingPointError, OverflowError):
        result['reason'] = 'numeric_failure'
    return result


def select_intersection(points, third_anchor, third_range, third_height, tie_margin_m):
    """Select using held-in third measurement only, never reference XY."""
    a = np.asarray(third_anchor, float)
    p = np.asarray(points, float)
    residuals = np.sqrt(np.sum((p-a[:2])**2, axis=1)+(third_height-a[2])**2)-third_range
    result = dict(ok=False, reason='ambiguous_intersection', xy_m=None,
                  third_residuals_m=residuals.tolist(), selected_index=None)
    scores = np.abs(residuals)
    if len(points) == 2 and abs(scores[0]-scores[1]) <= tie_margin_m:
        return result
    index = int(np.argmin(scores))
    result.update(ok=True, reason='ok', xy_m=p[index].tolist(), selected_index=index)
    return result


def make_candidates(anchors, ranges, heights, *, t_ref_us=0, anchor_ids=ANCHOR_IDS,
                    obs_ids=None, settings=None):
    cfg = settings or DSettings()
    result = dict(model_id='D', variant='strict_uniform4', ok=False, reason=None, xy_m=None,
                  candidates=[], pair_candidates=[], correction_m=[0.]*4)
    try:
        a, r, z, obs = prepare_inputs(anchors, ranges, heights, anchor_ids, obs_ids)
    except (ValueError, TypeError) as exc:
        result['reason'] = str(exc)
        return result
    try:
        with np.errstate(over='raise', invalid='raise'):
            horizontal_sq = r*r-(z-a[:, 2])**2
    except FloatingPointError:
        result['reason'] = 'numeric_failure'
        return result
    reasons, horizontal, clamped = [], [], []
    for value in horizontal_sq:
        clamped.append(bool(-cfg.numeric_tolerance_m2 <= value < 0))
        h = float(np.sqrt(max(value, 0)))
        reason = ('negative_horizontal_range_sq' if value < -cfg.numeric_tolerance_m2
                  else 'horizontal_range_too_small' if h < cfg.min_horizontal_range_m else 'ok')
        reasons.append(reason)
        horizontal.append(h if reason == 'ok' else None)
    result.update(horizontal_range_m=horizontal, projection_reasons=reasons, projection_clamped=clamped)
    # A pair is reused by two triplets, but its third-anchor selector differs.
    pair_geometry = {}
    for i, j in combinations(range(4), 2):
        pair_geometry[i, j] = (circle_intersections(a[i, :2], horizontal[i], a[j, :2], horizontal[j], cfg)
                               if reasons[i] == reasons[j] == 'ok' else None)
    for subset in combinations(range(4), 3):
        group_id = 'S'+''.join(str(i+1) for i in subset)
        slots = []
        for pair in combinations(subset, 2):
            third = next(i for i in subset if i not in pair)
            sid = group_id+':P'+''.join(str(i+1) for i in pair)
            slot = candidate_record(sid, list(subset), t_ref_us, obs)
            slot.update(pair_anchor_ids=[ANCHOR_IDS[i] for i in pair],
                        selector_anchor_id=ANCHOR_IDS[third], geometry=pair_geometry[pair])
            if any(reasons[i] != 'ok' for i in subset):
                slot['reason'] = 'projection_failed'
            elif not pair_geometry[pair]['ok']:
                slot['reason'] = pair_geometry[pair]['reason']
            else:
                slot.update(select_intersection(pair_geometry[pair]['points_xy_m'], a[third], r[third],
                                                z[third], cfg.tie_margin_m))
            slots.append(slot)
        group = candidate_record(group_id, list(subset), t_ref_us, obs)
        group.update(uniform_fuse(slots, [s['candidate_id'] for s in slots], t_ref_us))
        if not group['ok']:
            group['reason'] = 'incomplete_pair_set'
        group['pair_candidate_ids'] = [s['candidate_id'] for s in slots]
        group['residuals_all_anchors_m'] = residuals_at(group['xy_m'], a, r, z).tolist() if group['ok'] else None
        result['candidates'].append(group)
        result['pair_candidates'].extend(slots)
    result.update(uniform_fuse(result['candidates'], ['S123', 'S124', 'S134', 'S234'], t_ref_us))
    if result['ok']:
        result['residuals_all_anchors_m'] = residuals_at(result['xy_m'], a, r, z).tolist()
    return result
