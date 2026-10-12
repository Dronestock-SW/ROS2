"""Offline causal clues from current-state errors and native attitude tracking."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation,Slerp


def analyze(root):
    rows=[json.loads(s) for s in (root/'mission.jsonl').read_text().splitlines()]
    dwell=[r for r in rows if r['phase']=='DWELL']
    if not dwell:return dict(run=str(root),no_dwell=True)
    start,end=dwell[0]['clock_s'],dwell[-1]['clock_s']
    log=ULog(str(next(root.rglob('*.ulg'))));ds={d.name:d.data for d in log.data_list if d.multi_id==0}
    result=dict(run=str(root),interval_s=[start,end],parameters={k:v for k,v in log.initial_parameters.items()
        if k.startswith(('EKF2_OF_','EKF2_ACC_','EKF2_GYR_','MC_ROLL','MC_PITCH','MPC_XY','MPC_VEL'))})
    truth=ds['vehicle_attitude_groundtruth'];tt=truth['timestamp']/1e6
    ref=Rotation.from_quat(np.column_stack([truth[f'q[{k}]'] for k in (1,2,3,0)]))
    for name,prefix in [('vehicle_attitude','q'),('vehicle_attitude_setpoint','q_d')]:
        if name not in ds:continue
        d=ds[name];t=d.get('timestamp_sample',d['timestamp'])/1e6
        mask=(t>=max(start,tt[0]))&(t<=min(end,tt[-1]));t=t[mask]
        r=Rotation.from_quat(np.column_stack([d[f'{prefix}[{k}]'][mask] for k in (1,2,3,0)]))
        error=(Slerp(tt,ref)(t).inv()*r).as_rotvec()*180/np.pi
        result[name]=dict(n=len(t),rotation_error_rms_deg=np.sqrt(np.mean(error**2,axis=0)).tolist(),
            rotation_error_mean_deg=np.mean(error,axis=0).tolist())
    d=ds['vehicle_local_position_groundtruth'];t=d['timestamp']/1e6
    keep=(t>=start)&(t<=end);t=t[keep]
    for axis in ('x','y'):
        values=d[axis][keep];dt=np.median(np.diff(t))
        spectrum=np.abs(np.fft.rfft(values-values.mean()))
        frequency=np.fft.rfftfreq(len(values),dt)
        index=np.argmax(spectrum[1:])+1
        result['truth_'+axis]=dict(std_m=float(np.std(values)),dominant_hz=float(frequency[index]))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=analyze(a.run)
    a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print(json.dumps(r,indent=2))
