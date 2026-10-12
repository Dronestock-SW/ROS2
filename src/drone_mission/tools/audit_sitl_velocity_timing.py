"""Diagnose SIH sensor observation timing and current-state velocity response."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def audit(root):
    rows=[json.loads(s) for s in (root/'mission.jsonl').read_text().splitlines()]
    dwell=[r for r in rows if r['phase']=='DWELL']
    interval=(dwell[0]['clock_s'],dwell[-1]['clock_s'])
    log=ULog(str(next(root.rglob('*.ulg'))))
    topics={t.name:t.data for t in log.data_list if t.multi_id==0}
    truth=topics['vehicle_local_position_groundtruth'];tt=truth['timestamp']/1e6
    result=dict(run=str(root),dwell_interval=interval,parameters={k:v for k,v in log.initial_parameters.items()
        if k.startswith(('MPC_','EKF2_TAU','EKF2_DELAY','EKF2_ACC','EKF2_GYR','IMU_GYRO','MC_ROLL','MC_PITCH'))},topics={})
    for name,fields,ts in (
        ('vehicle_local_position',('vx','vy'),'timestamp_sample'),
        ('estimator_aid_src_aux_vel',('observation[0]','observation[1]'),'timestamp_sample'),
        ('vehicle_local_position_setpoint',('vx','vy'),'timestamp')):
        if name not in topics:continue
        d=topics[name];t=d[ts]/1e6
        mask=(t>=interval[0])&(t<=interval[1])
        t=t[mask];v=np.column_stack([d[k][mask] for k in fields])
        errors=[]
        # Positive lag means comparing current output with earlier truth.
        for lag in np.arange(-.5,.501,.01):
            ref=np.column_stack([np.interp(t-lag,tt,truth[k]) for k in ('vx','vy')])
            errors.append((float(lag),float(np.sqrt(np.mean(np.sum((v-ref)**2,axis=1))))))
        result['topics'][name]=dict(samples=len(t),rms_at_zero=errors[50][1],best_lag_and_rms=min(errors,key=lambda r:r[1]),
            speed_rms=float(np.sqrt(np.mean(np.sum(v*v,axis=1)))))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    value=audit(args.run);args.output.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(value))
