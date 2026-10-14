"""Audit HTTP/C++/PX4 SIH takeoff with independent ULog truth and aiding."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from analyze_lidar_independent_sih import aligned


def analyze(path):
    manifest=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
    result=json.loads((path/'summary.json').read_text(encoding='utf-8'))
    trace=json.loads((path/'trace.json').read_text(encoding='utf-8'))
    events=json.loads((path/'events.json').read_text(encoding='utf-8'))
    logs=list(path.rglob('*.ulg'))
    if len(logs)!=1 or not trace:
        raise ValueError('single completed ULog and trace required: '+str(path))
    ulog=ULog(str(logs[0]))
    data={d.name:d.data for d in ulog.data_list if d.multi_id==0}
    local=data['vehicle_local_position'];truth=data['vehicle_local_position_groundtruth']
    flags=data['estimator_status_flags'];aid=data.get('estimator_aid_src_aux_vel')
    admission=data.get('lidar_velocity2d_status')
    failures=[];metrics={};aiding={}
    starts=[e for e in events if e.get('type')=='http_request' and e.get('action')=='takeoff']
    takeoff=[r for r in trace if r['phase'] in ('TAKING_OFF','POSITION_HANDOFF','HOLD_READY')]
    hold=[r for r in trace if r['phase']=='HOLD_READY']
    if not result['goal_achieved']:failures.append('harness_goal_criteria_failed')
    if starts and takeoff:
        start=starts[0]['clock_us'];end=takeoff[-1]['clock_us']
        initial=np.searchsorted(truth['timestamp'],start,side='right')-1
        initial=max(initial,0)
        selected=(truth['timestamp']>=start)&(truth['timestamp']<=end)
        displacement=np.hypot(truth['x']-truth['x'][initial],truth['y']-truth['y'][initial])
        metrics['max_takeoff_and_hold_truth_xy_m']=float(displacement[selected].max())
        metrics['max_ascent_truth_height_m']=float((truth['z'][initial]-truth['z'][selected]).max())
        if metrics['max_takeoff_and_hold_truth_xy_m']>.25:failures.append('truth_xy_exceeds_0.25m')
        active=(local['timestamp']>=start)&(local['timestamp']<=end)
        metrics['reset_counts_after_web_takeoff']={key:int((np.r_[0,np.diff(local[key].astype(np.int64))%256])[active].sum())
            for key in ('xy_reset_counter','vxy_reset_counter','z_reset_counter','vz_reset_counter')}
        if hold:
            lo,hi=hold[0]['clock_us'],hold[-1]['clock_us']
            selected=(truth['timestamp']>=lo)&(truth['timestamp']<=hi)
            metrics.update(hold_duration_s=(hi-lo)/1e6,
                max_hold_truth_xy_m=float(displacement[selected].max()),
                max_hold_truth_speed_m_s=float(np.hypot(truth['vx'][selected],truth['vy'][selected]).max()),
                max_hold_truth_height_error_m=float(np.abs(truth['z'][initial]-truth['z'][selected]-1.7).max()),
                max_hold_estimated_height_error_m=max(abs(r['estimated_height_m']-r['height_reference_m']) for r in hold),
                all_hold_rows_posctl=all(r['mode']=='POSCTL' for r in hold))
            for name,part in [('uwb_absent',[r for r in hold if not r['uwb_present']]),
                              ('uwb_restored',[r for r in hold if r['uwb_present']])]:
                if part:
                    ix=(truth['timestamp']>=part[0]['clock_us'])&(truth['timestamp']<=part[-1]['clock_us'])
                    metrics['hold_'+name]=dict(duration_s=(part[-1]['clock_us']-part[0]['clock_us'])/1e6,
                        max_truth_xy_m=float(displacement[ix].max()),
                        max_truth_speed_m_s=float(np.hypot(truth['vx'][ix],truth['vy'][ix]).max()))
            for key,maximum in (('max_hold_truth_xy_m',.15),('max_hold_truth_speed_m_s',.1),
                    ('max_hold_truth_height_error_m',.15),('max_hold_estimated_height_error_m',.15)):
                if metrics[key]>maximum:failures.append(key)
            if metrics['hold_duration_s']<20 or not metrics['all_hold_rows_posctl']:failures.append('continuous_20s_position_hold_required')
        else:failures.append('position_hold_not_reached')
        gap=[r for r in trace if r['armed'] and not r['uwb_present'] and r['phase'] in ('TAKING_OFF','POSITION_HANDOFF','HOLD_READY')]
        if gap:
            # EV position may remain active briefly after its last observation.
            lo,hi=gap[0]['clock_us']+2e6,gap[-1]['clock_us']
            selected=(local['timestamp']>=lo)&(local['timestamp']<=hi)
            times=local['timestamp'][selected]
            if len(times) and hi-lo>=1e6:
                fresh=np.r_[False,np.diff(aid['time_last_fuse'].astype(np.int64))>0]
                count=int((fresh&(aid['timestamp']>=lo)&(aid['timestamp']<=hi)).sum())
                aiding=dict(uwb_absent_window_s=(hi-lo)/1e6,lidar_fusion_updates=count,
                    lidar_active_fraction=float(aligned(times,admission,'fusion_active').mean()),
                    validity={k:float(local[k][selected].mean()) for k in ('xy_valid','v_xy_valid','z_valid','v_z_valid')},
                    other_horizontal={k:float(aligned(times,flags,k).mean()) for k in
                        ('cs_ev_pos','cs_ev_vel','cs_gnss_pos','cs_gnss_vel','cs_opt_flow')})
                if aiding['lidar_active_fraction']<.98 or count/(hi-lo)*1e6<7:failures.append('independent_lidar_fusion_unavailable')
                if any(v<.98 for v in aiding['validity'].values()):failures.append('invalid_estimate_during_uwb_gap')
                if any(aiding['other_horizontal'].values()):failures.append('other_horizontal_aid_present_in_gap')
            else:failures.append('independent_lidar_gap_too_short')
        elif manifest['scenario']=='uwb_gap':failures.append('uwb_gap_not_exercised')
    else:failures.append('web_takeoff_not_reached')
    if ulog.dropouts:failures.append('ulog_dropout')
    transitions=[{k:e[k] for k in ('phase','reason','elapsed_s','truth_xyz')} for e in events if e['type']=='transition']
    return dict(run=path.name,scope=manifest['scope'],physical_flight=False,checks_passed=not failures,
        failures=failures,metrics=metrics,aiding=aiding,transitions=transitions,
        synthetic_parameters={k:manifest.get(k,'not_recorded') for k in
            ('scenario','seed','source_clock','lidar_noise_std_m_s','lidar_reported_std_m_s','height_reference','mag_type',
             'imu_profile','range_source','synthetic_range_mount_z_ned_m','range_noise_model','output_tau_s','velocity_feedback_cutoff_hz')},
        core_sha256=manifest['core_sha256'],script_sha256=manifest['script_sha256'],
        ulog_sha256=hashlib.sha256(logs[0].read_bytes()).hexdigest(),
        error_types=[e.split('(',1)[0] for e in result['errors']],
        terminal_reason=transitions[-1]['reason'] if transitions else None,
        commands=result['commands'],dropout_count=len(ulog.dropouts))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rows=[analyze(path) for path in args.runs]
    args.output.write_text(json.dumps(rows,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    for row in rows:print(json.dumps({k:row[k] for k in ('run','checks_passed','failures','metrics')},ensure_ascii=False))
    raise SystemExit(0 if all(row['checks_passed'] for row in rows) else 1)
