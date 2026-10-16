"""Read-only ULog comparison of actual height-publication outages and recovery."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from analyze_lidar_state_output import hold, metrics, sha256

PHASES = [('before', 25, 40), ('outage', 55, 85), ('restored', 105, 120),
          ('outage_full', 40, 90), ('restoration_transition', 90, 105)]


def analyze(run):
    manifest = json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    status = json.loads((run/'run_status.json').read_text(encoding='utf-8'))
    if status['errors'] or len(status['ulog_files']) != 1:
        raise ValueError(f'incomplete run: {run}')
    log = run/status['ulog_files'][0]
    u = ULog(str(log))
    if u.msg_info_dict['ver_sw'] != manifest['firmware_commit']:
        raise ValueError('firmware identity differs')
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    base = status['first_clock_us']
    effective_parameters=dict(u.initial_parameters)
    for t,k,v in u.changed_parameters:
        if t < base+20e6:effective_parameters[k]=v
    s, l, truth = (d[x] for x in ['estimator_states', 'vehicle_local_position',
                                 'vehicle_local_position_groundtruth'])
    flags = d['estimator_status_flags']
    # With HGT_REF=barometer, the EKF state is -AMSL even when ref_alt is
    # unpublished (global XY origin absent). Never replace a NaN origin by 0.
    # Compare changes from the pre-outage mean for a common local datum.
    sp = (s['timestamp_sample']>=base+25e6)&(s['timestamp_sample']<base+40e6)
    lp = (l['timestamp']>=base+25e6)&(l['timestamp']<base+40e6)
    internal_reference = float(np.mean(s['states[9]'][sp].astype(float)-
        np.interp(s['timestamp_sample'][sp],truth['timestamp'],truth['z'])))
    output_reference = float(np.mean(l['z'][lp].astype(float)-
        np.interp(l['timestamp'][lp],truth['timestamp'],truth['z'])))
    vz_deltas = np.r_[0, np.diff(l['vz_reset_counter'].astype(np.int64)) % 256]
    z_deltas = np.r_[0, np.diff(l['z_reset_counter'].astype(np.int64)) % 256]
    events = [v for line in (run/'inputs.jsonl').read_text(encoding='utf-8').splitlines()
              if 'event' in (v := json.loads(line))]
    phases = {}
    for name, lo, hi in PHASES:
        lower, upper = base+lo*1e6, base+hi*1e6
        si = (s['timestamp_sample'] >= lower) & (s['timestamp_sample'] < upper)
        li = (l['timestamp'] >= lower) & (l['timestamp'] < upper)
        t = s['timestamp_sample'][si]
        z = s['states[9]'][si].astype(float)-internal_reference
        ts = l['timestamp'][li]
        aid = {}
        for topic in ['estimator_aid_src_aux_vel','estimator_aid_src_ev_vel',
                      'estimator_aid_src_ev_pos','estimator_aid_src_baro_hgt','estimator_aid_src_rng_hgt']:
            if topic not in d:
                aid[topic] = None
                continue
            item = d[topic]
            ii = (item['timestamp'] >= lower) & (item['timestamp'] < upper)
            # Count changes, not held copies of the previous successful fuse.
            fresh = np.r_[False,np.diff(item['time_last_fuse'].astype(np.int64))>0]
            aid[topic] = dict(n=int(ii.sum()), fused_samples=int(item['fused'][ii].sum()),
                              fuse_updates=int((ii & fresh).sum()),
                              rejected_samples=int(item['innovation_rejected'][ii].sum()))
        sensors = {}
        for topic in ['sensor_baro','vehicle_air_data','distance_sensor','lidar_velocity2d']:
            items = [x for x in u.data_list if x.name == topic]
            # Barometer restarts advertise a new uORB instance; include all.
            sensors[topic] = {str(x.multi_id):int(((x.data['timestamp']>=lower)&
                                                  (x.data['timestamp']<upper)).sum()) for x in items}
        phases[name] = dict(
            ekf_z_drift_m=metrics(z-np.interp(t,truth['timestamp'],truth['z'])),
            ekf_vz_error_mps=metrics(s['states[6]'][si]-np.interp(t,truth['timestamp'],truth['vz'])),
            output_z_drift_m=metrics(l['z'][li]-output_reference-np.interp(ts,truth['timestamp'],truth['z'])),
            output_vz_error_mps=metrics(l['vz'][li]-np.interp(ts,truth['timestamp'],truth['vz'])),
            ekf_z_variance=metrics(s['covariances[8]'][si]),
            ekf_vz_variance=metrics(s['covariances[5]'][si]),
            vz_resets=int(vz_deltas[li].sum()),z_resets=int(z_deltas[li].sum()),
            validity={k:dict(true=int(l[k][li].sum()),n=int(li.sum())) for k in ['z_valid','v_z_valid','xy_valid','v_xy_valid']},
            flags={k:dict(true=int(hold(ts,flags,k).sum()),n=int(li.sum()))
                   for k in ['cs_ev_vel','cs_ev_pos','cs_fake_hgt','cs_baro_hgt','cs_rng_hgt','cs_in_air']},
            aiding=aid,logged_sensor_samples=sensors)
    transitions = {}
    for k in ['z_valid','v_z_valid']:
        changed = np.r_[True,np.diff(l[k].astype(int)) != 0]
        transitions[k] = [dict(elapsed_s=float((float(t)-base)/1e6),value=bool(v))
                          for t,v in zip(l['timestamp'][changed],l[k][changed])]
    li = np.flatnonzero((l['timestamp']>=base)&(l['timestamp']<base+120e6))[::25]
    si = np.flatnonzero((s['timestamp_sample']>=base)&(s['timestamp_sample']<base+120e6))[::25]
    admission = d.get('lidar_velocity2d_status')
    reset_indices=np.flatnonzero(vz_deltas)
    return dict(name=run.name,raw_directory=str(run),manifest=manifest,status=status,events=events,
        record_sha256={str(p.relative_to(run)):sha256(p) for p in run.rglob('*') if p.is_file()},
        phases=phases,validity_transitions=transitions,
        vz_reset_events=[dict(elapsed_s=(int(l['timestamp'][i])-base)/1e6,count=int(vz_deltas[i]),
                             delta_vz_mps=float(l['delta_vz'][i]),
                             fake_height=bool(hold(l['timestamp'][i:i+1],flags,'cs_fake_hgt')[0]))
                         for i in reset_indices],
        admission=None if admission is None else dict(
            accepted=int(admission['accepted'][-1]),rejected=int(admission['rejected'][-1]),
            reasons={str(k):v for k,v in Counter(admission['reason'].tolist()).items()}),
        parameters_at_input_start={k:effective_parameters.get(k) for k in ['EKF2_EV_CTRL','EKF2_LID2_EN','EKF2_BARO_CTRL',
                    'EKF2_RNG_CTRL','EKF2_HGT_REF','EKF2_EV_NOISE_MD','EKF2_AVEL_DELAY','SIH_DISTSNSR_OVR']},
        changed_parameters=[dict(timestamp=int(t),name=k,value=v) for t,k,v in u.changed_parameters],
        ulog_dropout_count=len(u.dropouts),
        ulog_dropouts=[dict(elapsed_s=(x.timestamp-base)/1e6,duration_ms=int(x.duration)) for x in u.dropouts],
        vertical_datum=dict(method='subtract each stream pre-outage (25-40s) mean error to truth; drift not absolute height accuracy',
                            state_down_reference_m=internal_reference,output_z_reference_m=output_reference),
        trace=dict(time_s=((l['timestamp'][li].astype(float)-base)/1e6).tolist(),
                   output_z_drift=(l['z'][li].astype(float)-output_reference).tolist(),output_vz=l['vz'][li].astype(float).tolist(),
                   z_valid=l['z_valid'][li].astype(int).tolist(),
                   vz_reset_count=np.cumsum(vz_deltas)[li].astype(int).tolist(),
                   ekf_time_s=((s['timestamp_sample'][si].astype(float)-base)/1e6).tolist(),
                   ekf_z_drift=(s['states[9]'][si].astype(float)-internal_reference).tolist()))


def checks(runs):
    failures=[]
    for r in runs:
        tag=r['name'];m=r['manifest'];p=r['phases'];out=p['outage']
        def require(condition,message):
            if not condition:failures.append(tag+': '+message)
        require(r['ulog_dropout_count']==0,'ULog dropout')
        require(r['status']['simulated_duration_s']>=120,'duration too short')
        for phase in p.values():
            require(phase['flags']['cs_in_air']['true']==0,'unexpected airborne state')
            require(phase['ekf_z_drift_m']['finite']==phase['ekf_z_drift_m']['n'],'nonfinite internal z')
        if m['path']=='lidar2d':
            require(r['admission'] is not None and r['admission']['accepted']>900,'LiDAR2D admission absent')
            require(out['aiding']['estimator_aid_src_aux_vel'] is not None
                    and out['aiding']['estimator_aid_src_aux_vel']['fuse_updates']>250,'2D fusion not demonstrated')
        if m['scenario']!='nominal':
            require(sum(out['logged_sensor_samples']['distance_sensor'].values())==0,'range publications did not stop')
            require(sum(p['before']['logged_sensor_samples']['distance_sensor'].values())>100,'range baseline absent')
            require(sum(p['restored']['logged_sensor_samples']['distance_sensor'].values())>100,'range did not resume')
        if m['scenario']=='all_height_gap':
            require(sum(out['logged_sensor_samples']['sensor_baro'].values())==0,'barometer publications did not stop')
            require(sum(p['restored']['logged_sensor_samples']['sensor_baro'].values())>5,'barometer did not resume')
            require(out['flags']['cs_baro_hgt']['true']==0 and out['flags']['cs_rng_hgt']['true']==0,'height aid remains active')
        else:
            require(out['flags']['cs_baro_hgt']['true']==out['flags']['cs_baro_hgt']['n'],'barometer aiding missing')
        if m['path']!='legacy3d':
            require(out['vz_resets']==0,'unexpected repeated vertical-velocity reset')
            if m['scenario']=='all_height_gap':
                require(out['validity']['z_valid']['true']==0,'height remains marked valid without height aid')
                require(p['restored']['validity']['z_valid']['true']==p['restored']['validity']['z_valid']['n'],
                        'height validity did not recover')
        require(not any(x['name'].startswith('EKF2_') and x['timestamp']>=r['status']['first_clock_us']+20e6
                        for x in r['changed_parameters']), 'EKF2 parameter changed after input start')
    return dict(run_count=len(runs),failures=failures,
                note='Diagnostic checks verify stimulus and measured behavior; not a flight acceptance gate.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix',type=Path,required=True)
    parser.add_argument('--retry-matrix',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--figure',type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing overwrite')
    runs=[analyze(p.parent) for p in sorted(args.matrix.glob('*/manifest.json'))]
    excluded=[]
    if args.retry_matrix:
        for manifest in sorted(args.retry_matrix.glob('*/manifest.json')):
            retry=analyze(manifest.parent)
            old=next(r for r in runs if r['name']==retry['name'])
            if old['ulog_dropout_count']==0 or checks([retry])['failures']:
                raise ValueError('replacement requires original dropout and a clean retry')
            excluded.append(dict(reason='ULog dropout; same configuration rerun in separate directory',run=old))
            runs[runs.index(old)]=retry
    result=dict(schema=1,scope='stationary, disarmed SIH; synthetic observations; no hardware or raw scan validation',
                phases=PHASES,runs=runs,excluded_runs=excluded,checks=checks(runs))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for r in runs:
        o=r['phases']['outage']
        print(r['name'],json.dumps(dict(z_drift_rms=o['ekf_z_drift_m']['rms'],vz_rms=o['output_vz_error_mps']['rms'],
              vz_resets=o['vz_resets'],valid=o['validity']['z_valid'],admission=r['admission'])),flush=True)
    print(json.dumps(result['checks']),flush=True)
    if args.figure:
        render(runs,args.figure)


def render(runs,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected={r['manifest']['path']:r for r in runs if r['manifest']['scenario']=='all_height_gap'
              and r['manifest']['seed']==11}
    if len(selected)!=3:raise ValueError('figure requires all three seed11 outage runs')
    fig,axes=plt.subplots(3,2,figsize=(14,10),sharex=True)
    styles={'lidar2d':('#087f8c','LiDAR2D','-'),'position_only':('#6356a7','Position only','--'),
            'legacy3d':('#c84545','Legacy 3D',':')}
    for path,r in selected.items():
        c,label,style=styles[path];t=r['trace'];x=t['time_s']
        axes[0,0].plot(t['ekf_time_s'],t['ekf_z_drift'],color=c,label=label,ls=style,lw=1.5)
        axes[1,0].plot(x,t['output_vz'],color=c,label=label,ls=style,lw=1.2)
        axes[2,0].plot(x,t['z_valid'],color=c,label=label,ls=style,lw=2)
        axes[2,1].plot(x,t['vz_reset_count'],color=c,label=label,ls=style,lw=1.7)
        if path!='legacy3d':
            axes[0,1].plot(t['ekf_time_s'],np.asarray(t['ekf_z_drift'])*100,color=c,label=label,ls=style)
            axes[1,1].plot(x,t['output_vz'],color=c,label=label,ls=style)
    titles=[['Internal height drift: all paths','Internal height drift: two horizontal paths'],
            ['Output vertical velocity: all paths','Output vertical velocity: two horizontal paths'],
            ['Published height validity','Cumulative vertical-velocity resets']]
    labels=[['Drift from pre-outage mean (m)','Drift from pre-outage mean (cm)'],
            ['NED vz (m/s)','NED vz (m/s)'],['Valid = 1','Resets since log start']]
    for row in range(3):
        for col in range(2):
            ax=axes[row,col];ax.axvspan(40,90,color='#f0d0bd',alpha=.35,zorder=0)
            ax.axvline(40,color='#886655',alpha=.5,lw=.8);ax.axvline(90,color='#886655',alpha=.5,lw=.8)
            ax.set_xlim(20,120);ax.set_title(titles[row][col],fontsize=11,loc='left')
            ax.set_ylabel(labels[row][col]);ax.grid(alpha=.2)
            if row==2:ax.set_xlabel('Elapsed simulator time (s)')
    axes[2,0].set_ylim(-.1,1.1);axes[2,0].set_yticks([0,1])
    fig.suptitle('Height sensor publication outage | seed 11 | stationary, disarmed SIH',fontsize=16,x=.05,ha='left')
    fig.text(.05,.925,'Shaded: range + barometer stopped (40-90 s). Height drift uses the 25-40 s mean; this is not absolute altitude accuracy.',fontsize=10)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,ncol=3,loc='lower center',bbox_to_anchor=(.5,.003),frameon=False)
    fig.tight_layout(rect=[0,.04,1,.90]);output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
