"""Audit immutable mission/ULog windows against ten-second, 3 cm hover criteria."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog

from drone_mission.hover_metrics import evaluate_window, metrics


PARAMETERS=('EKF2_GPS_CTRL','EKF2_EV_CTRL','EKF2_EV_NOISE_MD','EKF2_EVP_NOISE',
    'EKF2_EV_DELAY','EKF2_EV_POS_X','EKF2_EV_POS_Y','EKF2_EV_POS_Z',
    'EKF2_OF_CTRL','EKF2_OF_DELAY','EKF2_OF_GYR_SRC','EKF2_RNG_CTRL','EKF2_RNG_NOISE',
    'EKF2_RNG_DELAY','EKF2_HGT_REF','EKF2_BARO_CTRL','EKF2_MAG_TYPE',
    'EKF2_DELAY_MAX','EKF2_TAU_POS','EKF2_TAU_VEL','EKF2_LID2_EN','EKF2_LID2_AGE',
    'EKF2_AVEL_DELAY','MPC_XY_P','MPC_XY_VEL_P_ACC','MPC_XY_VEL_I_ACC',
    'MPC_XY_VEL_D_ACC','MPC_VEL_LP','MPC_Z_P','MPC_Z_VEL_P_ACC',
    'COM_ARM_WO_GPS','COM_OF_LOSS_T','COM_OBL_RC_ACT','COM_RC_OVERRIDE',
    'MIS_TAKEOFF_ALT','EKF2_PREDICT_US','IMU_INTEG_RATE')
PARAMETERS+=('EKF2_ACC_NOISE','EKF2_GYR_NOISE','EKF2_RNG_SFE','MPC_XY_VEL_MAX')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def ulog_window(log,start_s,end_s,target_xy):
    result={}
    goal=np.array([target_xy[1]-2.,target_xy[0]-2.])
    for topic,key in [('vehicle_local_position','fc'),('vehicle_local_position_groundtruth','truth')]:
        data=next(d.data for d in log.data_list if d.name==topic)
        t=data.get('timestamp_sample',data['timestamp'])/1e6
        keep=(t>=start_s)&(t<=end_s)
        error=np.column_stack((data['x'][keep],data['y'][keep]))-goal
        result[key+'_goal_xy_m']=metrics(np.linalg.norm(error,axis=1))
        result[key+'_speed_m_s']=metrics(np.hypot(data['vx'][keep],data['vy'][keep]))
        result[key+'_vertical_speed_m_s']=metrics(data['vz'][keep])
    result['precision_pass']=all(result[k+'_goal_xy_m']['rms'] is not None and
        result[k+'_goal_xy_m']['rms']<=.03 for k in ('fc','truth'))
    result['scope']='High-rate native FC/physics samples in the preselected mission window'
    return result


def analyze(root):
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    status=json.loads((root/'run_status.json').read_text(encoding='utf-8'))
    rows=[json.loads(s) for s in (root/'mission.jsonl').read_text(encoding='utf-8').splitlines()]
    dwell=[r for r in rows if r.get('phase_before_tick',r['phase'])=='DWELL']
    logs=[ULog(str(root/f)) for f in status.get('ulog_files',[])]
    result=dict(run=str(root),scenario=manifest['scenario'],seed=manifest['seed'],
        status=status,manifest=manifest,parameters=[{k:u.initial_parameters.get(k) for k in PARAMETERS} for u in logs],
        raw_hashes={n:sha(root/n) for n in ('mission.jsonl','telemetry.jsonl','manifest.json','run_status.json')},
        ulog_hashes={f:sha(root/f) for f in status.get('ulog_files',[])},
        dwell_samples=len(dwell),mission_10s_complete=False,precision_hover_pass=False)
    if not dwell:return result
    target=dwell[0]['target_before_tick'];height=dwell[0]['height_before_tick_m']
    result['whole_dwell']=evaluate_window(dwell,target_xy=target,height_target_m=height)
    first_window=[r for r in dwell if r['clock_s']<=dwell[0]['clock_s']+10.06]
    result['first_10s_from_arrival']=evaluate_window(first_window,target_xy=target,height_target_m=height)
    result['first_10s_ulog']=[ulog_window(u,dwell[0]['clock_s'],dwell[0]['clock_s']+10.,target)
        for u in logs] if dwell[-1]['clock_s']-dwell[0]['clock_s']>=10. else []
    exit_row=next((r for r in dwell if r['phase']!='DWELL' and r['phase'] not in ('LANDING','FAILED')),None)
    if exit_row is not None:
        previous=[r for r in dwell if r['clock_s']<=exit_row['clock_s'] and r.get('stable_since_s') is not None]
        start=previous[-1]['stable_since_s'] if previous else exit_row['clock_s']
        window=[r for r in dwell if start-1e-6<=r['clock_s']<=exit_row['clock_s']]
        result['accepted_hold']=evaluate_window(window,target_xy=target,height_target_m=height)
        result['accepted_hold']['start_s']=start;result['accepted_hold']['end_s']=exit_row['clock_s']
        result['accepted_hold_ulog']=[ulog_window(u,start,exit_row['clock_s'],target) for u in logs]
        result['mission_10s_complete']=(manifest.get('hover_duration_s',2.)>=10.
            and result['accepted_hold']['complete'])
        result['precision_hover_pass']=(result['mission_10s_complete']
            and result['accepted_hold']['precision_pass']
            and bool(logs) and all(v['precision_pass'] for v in result['accepted_hold_ulog']))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs',type=Path,nargs='+');p.add_argument('--matrix',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    runs=args.runs
    if args.matrix:
        plan=json.loads((args.matrix/'plan.json').read_text())
        done=json.loads((args.matrix/'results.json').read_text())
        if len(plan['jobs'])!=len(done):raise ValueError('incomplete_matrix')
        runs=[args.matrix/r['run'] for r in done]
    if not runs:p.error('runs or matrix required')
    results=[analyze(root) for root in runs]
    args.output.write_text(json.dumps(dict(schema=1,results=results),indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for r in results:
        print(json.dumps(dict(run=r['run'],reason=r['status']['final_reason'],
            ten_seconds=r['mission_10s_complete'],precision=r['precision_hover_pass'],
            whole_xy=r.get('whole_dwell',{}).get('fc_goal_xy_m'),
            accepted=r.get('accepted_hold'))),flush=True)


if __name__=='__main__':main()
