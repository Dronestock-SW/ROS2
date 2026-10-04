"""Uniform candidate fusion without an independence or covariance claim."""
import numpy as np


def uniform_fuse(candidates, required_ids, t_ref_us):
    result = dict(ok=False, reason='incomplete_candidate_set', xy_m=None, weights={},
                  dispersion_m2=None, covariance_kind='unknown', used_obs_ids=[])
    ids = [c['candidate_id'] for c in candidates]
    if len(ids) != len(set(ids)):
        result['reason'] = 'duplicate_candidate_id'
        return result
    if set(ids) != set(required_ids) or not ids or not all(c['ok'] for c in candidates):
        return result
    if any(c['t_ref_us'] != t_ref_us for c in candidates):
        result['reason'] = 'stale_candidate'
        return result
    ordered = sorted(candidates, key=lambda c: c['candidate_id'])
    points = np.asarray([c['xy_m'] for c in ordered], float)
    if points.shape != (len(ordered), 2) or not np.isfinite(points).all():
        result['reason'] = 'invalid_candidate'
        return result
    mean = np.mean(points, axis=0)
    difference = points-mean
    result.update(ok=True, reason='ok', xy_m=mean.tolist(),
                  weights={c['candidate_id']: 1/len(ordered) for c in ordered},
                  dispersion_m2=(difference.T@difference/len(ordered)).tolist(),
                  used_obs_ids=sorted({obs for c in ordered for obs in c['used_obs_ids']}))
    return result
