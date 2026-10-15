"""Audit recorded ranges against anchor geometry without using FC XY as truth.

Triangle deficits account for sequential measurement times at a stated speed.
Best physical fits are diagnostics, never replacement flight observations.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares


def audit(row, anchors, speed):
    ranges = row.get('cal_slant_m', row.get('raw_slant_m'))
    times = row.get('sample_time_us')
    if not ranges or not times:
        return None
    ids = [i for i, r in enumerate(ranges) if r is not None and r > 0 and times[i]]
    if len(ids) < 3:
        return None
    a = np.asarray(anchors,dtype=float)[ids]; r = np.asarray([ranges[i] for i in ids],dtype=float)
    deficits = []
    for k, i in enumerate(ids):
        for j in ids[k+1:]:
            separation = float(np.linalg.norm(np.asarray(anchors[i])-anchors[j]))
            allowance = speed*abs(times[i]-times[j])/1e6
            deficit = separation-ranges[i]-ranges[j]-allowance
            deficits.append(dict(anchors=[i+1,j+1], deficit_m=max(0.,deficit),
                                 sequential_motion_allowance_m=allowance))
    initial = row.get('raw_xy_m') or row.get('xy_m') or np.mean(a[:,:2], axis=0)
    heights = row.get('sample_height_m') or [None]*4
    # The lower bound assumes equal anchor heights, as required by B4.
    if np.ptp(a[:,2]) > 1e-8:
        raise ValueError('equal anchor heights required')
    free = least_squares(lambda p:np.sqrt(np.sum((a[:,:2]-p[:2])**2,axis=1)+p[2]**2)-r,
                         [*initial,.3], bounds=([-np.inf,-np.inf,0.],[np.inf]*3))
    # Include the boundary solution: z=anchor_z can be the physical optimum.
    boundary = least_squares(lambda p:np.linalg.norm(a[:,:2]-p,axis=1)-r,initial)
    lower_rms = min(float(np.sqrt(np.mean(free.fun**2))),float(np.sqrt(np.mean(boundary.fun**2))))
    result = dict(seq=row.get('seq'), callback_ros_ns=row['callback_ros_ns'],
                  reason=row['reason'], published=row.get('replay_publish',False),
                  triangle=max(deficits,key=lambda x:x['deficit_m']),
                  physical_3d_best_rms_m=lower_rms,
                  measured_height_available=all(heights[i] is not None for i in ids))
    if result['measured_height_available']:
        z=np.asarray([heights[i] for i in ids])-a[:,2]
        fixed=least_squares(lambda p:np.sqrt(np.sum((a[:,:2]-p)**2,axis=1)+z*z)-r,initial)
        result['measured_height_best_rms_m']=float(np.sqrt(np.mean(fixed.fun**2)))
    return result


def run(args):
    if not np.isfinite(args.speed_m_s) or args.speed_m_s < 0:
        raise ValueError('finite nonnegative speed assumption required')
    config=json.loads(args.config.read_text(encoding='utf-8'))
    rows=[json.loads(l) for l in args.decisions.open(encoding='utf-8')]
    rows=[r for r in rows if args.start_ros_ns <= r['callback_ros_ns'] <= args.end_ros_ns]
    pub=[r for r in rows if r.get('replay_publish')]
    left,right=max(zip(pub,pub[1:]),key=lambda p:p[1]['received_monotonic_ns']-p[0]['received_monotonic_ns'])
    evaluated=[v for r in rows if (v:=audit(r,config['anchors_xyz_m'],args.speed_m_s)) is not None]
    gap=[r for r in evaluated if left['callback_ros_ns'] < r['callback_ros_ns'] < right['callback_ros_ns']]
    def summary(part):
        return dict(count=len(part),reasons=dict(Counter(r['reason'] for r in part)),
            triangle_deficit_over_6cm=sum(r['triangle']['deficit_m']>.06 for r in part),
            physical_best_rms_over_6cm=sum(r['physical_3d_best_rms_m']>=.06 for r in part),
            max_triangle_deficit_m=max((r['triangle']['deficit_m'] for r in part),default=0),
            physical_best_rms_range_m=[min((r['physical_3d_best_rms_m'] for r in part),default=0),max((r['physical_3d_best_rms_m'] for r in part),default=0)])
    result=dict(scope=__doc__,speed_assumption_m_s=args.speed_m_s,
        max_gap_s=(right['received_monotonic_ns']-left['received_monotonic_ns'])/1e9,
        gap_start_ros_ns=left['callback_ros_ns'],gap_end_ros_ns=right['callback_ros_ns'],
        full=summary(evaluated),gap=summary(gap),gap_rows=gap)
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    (args.output/'rows.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in evaluated),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='gap_rows'}))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('decisions','config','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--start-ros-ns',type=int,required=True)
    p.add_argument('--end-ros-ns',type=int,required=True)
    p.add_argument('--speed-m-s',type=float,default=1.)
    run(p.parse_args())
