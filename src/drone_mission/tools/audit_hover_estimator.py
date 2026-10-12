"""Compare delayed states, present outputs and actual sensor observations."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def audit(root):
    rows=[json.loads(s) for s in (root/'mission.jsonl').read_text().splitlines()]
    dwell=[r for r in rows if r['phase']=='DWELL']
    if not dwell:return dict(run=str(root),no_dwell=True)
    interval=(dwell[0]['clock_s'],dwell[-1]['clock_s'])
    u=ULog(str(next(root.rglob('*.ulg'))))
    ds={d.name:d.data for d in u.data_list if d.multi_id==0}
    truth=ds['vehicle_local_position_groundtruth'];tt=truth['timestamp']/1e6
    result=dict(run=str(root),interval_s=interval,topics={})
    for name,fields in (
        ('vehicle_local_position',('vx','vy','vz')),
        ('estimator_states',('states[4]','states[5]','states[6]')),
        ('estimator_optical_flow_vel',('vel_ne[0]','vel_ne[1]')),
        ('estimator_aid_src_aux_vel',('observation[0]','observation[1]'))):
        if name not in ds:continue
        d=ds[name];t=d.get('timestamp_sample',d['timestamp'])/1e6
        mask=(t>=interval[0])&(t<=interval[1]);t=t[mask]
        v=np.column_stack([d[k][mask] for k in fields])
        ref=np.column_stack([np.interp(t,tt,truth[k]) for k in ('vx','vy','vz')[:len(fields)]])
        error=v-ref
        result['topics'][name]=dict(n=len(t),error_rms_by_axis=np.sqrt(np.mean(error**2,axis=0)).tolist(),
            error_mean_by_axis=np.mean(error,axis=0).tolist(),
            observed_speed_rms=float(np.sqrt(np.mean(np.sum(v*v,axis=1)))),
            true_speed_rms=float(np.sqrt(np.mean(np.sum(ref*ref,axis=1)))),
            publish_minus_sample_s=float(np.mean(d['timestamp'][mask]/1e6-t)))
    for name in ('estimator_aid_src_rng_hgt','estimator_aid_src_baro_hgt','estimator_aid_src_ev_pos'):
        if name not in ds:continue
        d=ds[name];t=d['timestamp_sample']/1e6;mask=(t>=interval[0])&(t<=interval[1])
        result['topics'][name]={k:float(np.mean(d[k][mask])) for k in d if
            k=='fused' or k.startswith(('observation_variance','innovation_variance'))}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();value=[audit(r) for r in args.runs]
    args.output.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(value,indent=2))
