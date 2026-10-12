"""Audit completed isolated SIH trials against their predeclared criteria."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def metrics(values):
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    return dict(n=len(x), rms=float(np.sqrt(np.mean(x*x))) if len(x) else None,
        p95=float(np.quantile(np.abs(x), .95)) if len(x) else None,
        maximum=float(np.max(np.abs(x))) if len(x) else None)


def truth_errors(log, start_us, end_us):
    """Compare logged NED states on one simulation clock, with bounded gaps."""
    topic={t.name:t.data for t in log.data_list if t.multi_id==0}
    if not all(k in topic for k in ('vehicle_local_position','vehicle_local_position_groundtruth')):
        return dict(available=False)
    state=topic['vehicle_local_position'];truth=topic['vehicle_local_position_groundtruth']
    time=state['timestamp_sample'].astype(float)
    tt=truth['timestamp'].astype(float)
    indices=np.searchsorted(tt,time)
    bounded=(indices>0)&(indices<len(tt))
    indices=np.clip(indices,1,len(tt)-1)
    bounded &= (time-tt[indices-1]<=40000)&(tt[indices]-time<=40000)
    bounded &= (time>=start_us)&(time<=end_us)
    error={k:state[k][bounded]-np.interp(time[bounded],tt,truth[k])
        for k in ('x','y','z','vx','vy','vz')}
    return dict(available=True,start_us=start_us,end_us=end_us,samples=int(bounded.sum()),
        coordinate_frame='NED simulator origin; unaligned absolute state differences',
        xy_m=metrics(np.hypot(error['x'],error['y'])),z_m=metrics(error['z']),
        vxy_m_s=metrics(np.hypot(error['vx'],error['vy'])),vz_m_s=metrics(error['vz']),
        z_signed_mean_m=float(np.mean(error['z'])) if bounded.any() else None,
        z_change_error_m=metrics(error['z']-error['z'][0]) if bounded.any() else None,
        z_initial_error_m=float(error['z'][0]) if bounded.any() else None,
        z_change_scope='error relative to first retained moving sample; constant origin offset removed, no independent height calibration',
        timestamp_contract='FC output timestamp_sample vs interpolated simulator timestamp; both brackets <=40 ms')


def analyze(root):
    manifest = json.loads((root/'manifest.json').read_text())
    status = json.loads((root/'run_status.json').read_text())
    rows = [json.loads(line) for line in (root/'mission.jsonl').read_text().splitlines()]
    moving = [r for r in rows if r['phase'] in ('MOVING', 'RETURNING', 'DWELL')]
    # Include braking and landing after first motion for clearance, as does the
    # runtime counter. Restrict estimation metrics to in-air mission motion.
    xy = [np.linalg.norm(np.asarray(r['pose_xy'])-r['truth_xy']) for r in moving]
    velocity = [np.linalg.norm(np.asarray(r['velocity_xy'])-r['truth_velocity_xy'])
        for r in moving if r.get('truth_velocity_xy') is not None]
    audit = dict(source=str(root), scenario=manifest['scenario'], seed=manifest['seed'],
        lidar_aiding=manifest['lidar_aiding'], status=status,
        moving_samples=len(moving), moving_xy_error_m=metrics(xy),
        moving_velocity_error_m_s=metrics(velocity),
        mission_phase_samples=dict(Counter(r['phase'] for r in rows)),
        velocity_error_bound_exceeded_fraction=(float(np.mean(np.asarray(velocity)>manifest['guard'].get('velocity_error_m_s',0.)))
            if velocity else None),
        guard_reasons=dict(Counter((r.get('guard') or {}).get('reason') for r in moving)),
        ulogs=[], scope='Synthetic SIH observations and truth; not physical or raw-scan accuracy')
    for name in status['ulog_files']:
        path=root/name
        log=ULog(str(path))
        info=dict(file=name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            parameters={k:v for k,v in log.initial_parameters.items()
                if k in ('EKF2_LID2_EN','EKF2_LID2_AGE','EKF2_EV_CTRL','EKF2_EV_DELAY',
                         'EKF2_AVEL_DELAY','EKF2_OF_CTRL','EKF2_RNG_CTRL','EKF2_GPS_CTRL',
                         'EKF2_DELAY_MAX','EKF2_ASP_DELAY','EKF2_TAU_VEL','MPC_XY_P')},
            admission=[], aiding=[], position_resets=[])
        fault_start=(moving[0]['clock_s']+1.4)*1e6 if moving else float('inf')
        fault_end=(moving[0]['clock_s']+2.9)*1e6 if moving else float('-inf')
        for topic in log.data_list:
            d=topic.data
            if topic.name=='lidar_velocity2d_status':
                middle=(d['timestamp']>=fault_start)&(d['timestamp']<=fault_end)
                info['admission'].append(dict(instance=topic.multi_id, samples=len(d['timestamp']),
                    reasons={str(k):int(v) for k,v in Counter(d['reason']).items()},
                    fault_middle_reasons={str(k):int(v) for k,v in Counter(d['reason'][middle]).items()},
                    accepted_last=int(d['accepted'][-1]),rejected_last=int(d['rejected'][-1])))
            if topic.name in ('estimator_aid_src_aux_vel','estimator_aid_src_ev_pos','estimator_aid_src_optical_flow'):
                stamp=d['timestamp_sample']
                valid=stamp>0
                # One record per new sample, rather than counting republished
                # aid status as independent observations.
                unique=np.r_[True,np.diff(stamp)!=0]&valid
                fused=unique&(d['fused']>0)
                middle=(stamp>=fault_start)&(stamp<=fault_end)
                fields=[key for key in d if key.startswith('test_ratio[')]
                info['aiding'].append(dict(topic=topic.name,instance=topic.multi_id,
                    log_records=len(stamp),new_samples=int(unique.sum()),fused_samples=int(fused.sum()),
                    fault_middle_new_samples=int((unique&middle).sum()),
                    fault_middle_fused_samples=int((fused&middle).sum()),
                    max_test_ratio=max(float(np.max(d[k][unique])) for k in fields) if unique.any() and fields else None,
                    last_sample_us=int(stamp[-1])))
            if topic.name=='vehicle_local_position':
                start=moving[0]['clock_s']*1e6 if moving else float('inf')
                motion=d['timestamp']>=start
                info['position_resets'].append(dict(instance=topic.multi_id,
                    counters={k:int(d[k][-1])-int(d[k][0]) for k in
                        ('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter') if k in d},
                    after_motion={k:int(d[k][motion][-1])-int(d[k][motion][0]) for k in
                        ('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter') if k in d and motion.any()}))
        if moving:
            info['moving_truth_errors']=truth_errors(log,moving[0]['clock_s']*1e6,moving[-1]['clock_s']*1e6)
        audit['ulogs'].append(info)
    aux=sum(item['fused_samples'] for log in audit['ulogs'] for item in log['aiding']
        if item['topic']=='estimator_aid_src_aux_vel')
    evidence=all(status.get(k,False) for k in ('armed_seen','airborne_seen','offboard_seen','moving_seen'))
    evidence=evidence and status.get('feedback_count',0)>0 and not status['errors']
    clearance=status.get('min_wall_surface_clearance_m')
    audit['criteria']=dict(exercised=bool(evidence),
        clear_mission_completed=(status['final_phase']=='END') if manifest['scenario']=='clear' else None,
        wall_clearance=(clearance>=manifest['predeclared']['min_wall_surface_clearance_m'])
            if evidence and clearance is not None and manifest['scenario']=='wall' else None,
        aux_fusion_observed=bool(aux>0),
        stale_zero_within_limit=(status['scan_fault_to_zero_command_s']<=manifest['predeclared']['stale_scan_zero_command_within_s'])
            if evidence and status['scan_fault_to_zero_command_s'] is not None else None)
    return audit


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--runs',type=Path,nargs='+')
    source.add_argument('--matrix',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    runs=args.runs
    if args.matrix:
        plan=json.loads((args.matrix/'plan.json').read_text(encoding='utf-8'))
        runs=sorted(path.parent for path in args.matrix.glob('*/run_status.json'))
        if len(runs)!=len(plan['jobs']):
            raise ValueError('incomplete matrix; preserve existing output and finish or explicitly analyze individual runs')
    results=[analyze(root) for root in runs]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(schema=1,results=results),indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for x in results:
        print(json.dumps({k:x[k] for k in ('source','scenario','lidar_aiding','criteria','moving_xy_error_m','moving_velocity_error_m_s')}))
