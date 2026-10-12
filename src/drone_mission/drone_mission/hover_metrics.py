"""Offline fixed-target hover metrics. No commands or changes to mission limits."""
import math

import numpy as np


def metrics(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return dict(n=0,rms=None,p95=None,maximum=None)
    if not np.isfinite(a).all():raise ValueError('nonfinite_hover_metric')
    return dict(n=len(a),rms=float(np.sqrt(np.mean(a*a))),
                p95=float(np.quantile(np.abs(a),.95)),maximum=float(np.max(np.abs(a))))


def evaluate_window(rows, *, target_xy, height_target_m, required_duration_s=10.,
                    max_gap_s=.12, xy_rms_limit_m=.03):
    """Score all samples in a preselected hold window, never subtract its mean.

    The caller must declare the window before seeing its RMS (the mission's
    continuous-stability interval or the first fixed ten seconds after arrival).
    A constant target, maximum sample gap and full elapsed duration are required.
    """
    if (not math.isfinite(required_duration_s) or required_duration_s<=0
            or not math.isfinite(max_gap_s) or max_gap_s<=0
            or len(target_xy)!=2 or not all(math.isfinite(v) for v in target_xy)):
        raise ValueError('explicit_hover_window_required')
    if not rows:return dict(complete=False,precision_pass=False,reasons=['no_samples'])
    times=np.asarray([r['clock_s'] for r in rows],dtype=float)
    duration=float(times[-1]-times[0])
    gaps=np.diff(times)
    target_changed=any(math.dist(r.get('target_before_tick',target_xy),target_xy)>1e-6
                       or abs(r.get('height_before_tick_m',height_target_m)-height_target_m)>1e-6
                       for r in rows)
    xy=[math.dist(r['pose_xy'],target_xy) for r in rows]
    truth=[math.dist(r['truth_xy'],target_xy) for r in rows]
    speed=[math.hypot(*r['velocity_xy']) for r in rows]
    z=[abs(r['snapshot']['altitude_enu_m']-height_target_m) for r in rows]
    vz=[abs(r['snapshot']['vertical_speed_m_s']) for r in rows]
    uwb=[math.dist(r['snapshot']['uwb_xy'],target_xy) for r in rows]
    reasons=[]
    if duration+1e-6<required_duration_s:reasons.append('short_hold')
    if len(gaps) and (np.any(gaps<=0) or np.max(gaps)>max_gap_s):reasons.append('invalid_sample_continuity')
    if target_changed:reasons.append('target_changed')
    if any(not r['armed'] or r['landed']!=2 or r['mode']!='OFFBOARD' for r in rows):
        reasons.append('flight_state_lost')
    for name,values,limit in [('xy',xy,.15),('speed',speed,.1),('z',z,.15),('vz',vz,.1),('uwb',uwb,.25)]:
        if max(values)>limit+1e-6:reasons.append(name+'_limit_exceeded')
    fc_metric=metrics(xy);truth_metric=metrics(truth)
    continuous=not reasons
    if fc_metric['rms']>xy_rms_limit_m:reasons.append('fc_goal_xy_rms_exceeded')
    if truth_metric['rms']>xy_rms_limit_m:reasons.append('truth_goal_xy_rms_exceeded')
    return dict(samples=len(rows),duration_s=duration,max_gap_s=float(max(gaps)) if len(gaps) else None,
        target_xy_m=list(target_xy),height_target_m=height_target_m,
        fc_goal_xy_m=fc_metric,truth_goal_xy_m=truth_metric,fc_speed_m_s=metrics(speed),
        fc_height_error_m=metrics(z),fc_vertical_speed_m_s=metrics(vz),uwb_goal_xy_m=metrics(uwb),
        complete=continuous,precision_pass=not reasons,reasons=reasons,
        criterion=dict(duration_s=required_duration_s,xy_rms_m=xy_rms_limit_m,
                       scope='fixed target; no mean subtraction; both FC and simulator truth'))
