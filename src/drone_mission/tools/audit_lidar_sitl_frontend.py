"""Evaluate scan-derived interval velocity against offline native SIH truth."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def audit(root):
    rows=[json.loads(s) for s in (root/'scan_observations.jsonl').read_text().splitlines()]
    u=ULog(str(next(root.rglob('*.ulg'))))
    d=next(d.data for d in u.data_list if d.name=='vehicle_local_position_groundtruth')
    t=d['timestamp']/1e6
    reasons=Counter();errors=[];scores=[];ages=[];intervals=[]
    for row in rows:
        obs=row['observation']
        if not obs['accepted']:
            reasons.update(obs['reasons']);continue
        dt=obs['interval_s'];end=obs['stamp_s'];start=end-dt
        v=np.array(obs['velocity_ne_m_s']);c=np.array(obs['covariance_ne_m2_s2'])
        ref=np.array([(np.interp(end,t,d[k])-np.interp(start,t,d[k]))/dt for k in ('x','y')])
        error=v-ref
        errors.append(error);scores.append(float(error@np.linalg.solve(c,error)))
        ages.append(row['clock_s']-obs['sample_midpoint_s']);intervals.append(dt)
    value=dict(run=str(root),scans=len(rows),accepted=len(errors),reject_reasons=dict(reasons),
        scope='offline SIH interval displacement; no ground truth is passed to scan matcher',
        covariance_scope='explicit simulation model; not physical calibration')
    if errors:
        error=np.array(errors)
        value.update(velocity_error_rms_xy_m_s=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
            velocity_error_rms_axes_m_s=np.sqrt(np.mean(error**2,axis=0)).tolist(),
            velocity_error_p95_xy_m_s=float(np.quantile(np.linalg.norm(error,axis=1),.95)),
            nominal_95_percent_covariance_coverage=float(np.mean(np.array(scores)<=5.991)),
            midpoint_age_s=dict(min=float(min(ages)),max=float(max(ages))),
            interval_s=dict(min=float(min(intervals)),max=float(max(intervals))))
    return value


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--runs',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();v=[audit(r) for r in a.runs]
    a.output.write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8');print(json.dumps(v,indent=2))
