"""Export bounded evidence from finished SIH runs, preserving original ULogs."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from lidar_ev_sitl import sha256


def range_of(values):
    a = [x for x in values if x is not None]
    return dict(n=len(a), minimum=min(a) if a else None, maximum=max(a) if a else None)


def extra_metrics(run, analysis):
    u = ULog(str(next(run.rglob('*.ulg'))))
    data = {d.name:d.data for d in u.data_list if d.multi_id == 0}
    base = analysis['status']['first_clock_us']
    scenario = analysis['manifest']['scenario']
    duration = analysis['manifest']['gap_s'] if scenario in ['lidar_gap','uwb_gap','nan_vz','invalid_velocity_frame','quality','stale_velocity'] else 5.
    fault_end = 40. + duration
    output={}
    for topic,field in [('actuator_armed','armed'),('vehicle_land_detected','landed'),('vehicle_status','arming_state')]:
        if topic in data:
            output[topic+'_'+field+'_values']=np.unique(data[topic][field]).astype(int).tolist()
    if output.get('actuator_armed_armed_values') != [0] or output.get('vehicle_land_detected_landed_values') != [1]:
        raise ValueError('Ulog does not verify the intended disarmed ground scope')
    local=data['vehicle_local_position']
    output['reset_increments']={}
    for phase,lo,hi in [('normal',25,40),('fault',40,45),('recovery',50,120)]:
        sel=(local['timestamp']>=base+lo*1e6)&(local['timestamp']<base+hi*1e6)
        output['reset_increments'][phase]={}
        for field in ['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter']:
            # These counters are uint8. A 255->0 rollover is one reset, not -255.
            delta=np.r_[0,np.diff(local[field].astype(np.int64)) % 256]
            output['reset_increments'][phase][field]=int(delta[sel].sum())
    for label in ['ev_pos','ev_vel']:
        d=data.get('estimator_aid_src_'+label)
        if d is None:
            output[label]=dict(max_aid_update_gap_s=None, first_fresh_aid_update_after_fault_s=None)
            continue
        clocks=np.unique(d['time_last_fuse'].astype(float))
        elapsed=(clocks-base)/1e6
        selected=elapsed[(elapsed>=35.) & (elapsed<=55.)]
        after=(d['timestamp_sample']>=base+fault_end*1e6) & (d['fused']>0) & (d['time_last_fuse']>0)
        output[label]=dict(max_aid_update_gap_s=float(np.max(np.diff(selected))) if len(selected)>1 else None,
             first_fresh_aid_update_after_fault_s=float((d['timestamp'][after][0]-base)/1e6-fault_end) if after.any() else None,
             aid_update_count_35_55=len(selected))
    output['recovery_validity']={}
    sel=(local['timestamp']>=base+50e6)&(local['timestamp']<base+120e6)
    for field in ['xy_valid','z_valid','v_xy_valid','v_z_valid']:
        if field in local:
            output['recovery_validity'][field]=dict(logged=int(sel.sum()), valid=int(local[field][sel].sum()))
    flags=data.get('estimator_status_flags',{})
    if flags:
        selected=(flags['timestamp']>=base+50e6)&(flags['timestamp']<base+120e6)
        output['recovery_flags']={field:np.unique(flags[field][selected]).astype(int).tolist()
            for field in ['cs_ev_pos','cs_ev_vel','cs_baro_hgt','cs_rng_hgt','cs_fake_hgt','cs_in_air'] if field in flags}
    output['changed_parameters']=u.changed_parameters
    d=data.get('vehicle_visual_odometry')
    if d is not None:
        lo,hi=base+40e6,base+45e6
        sel=(d['timestamp']>=lo)&(d['timestamp']<hi)
        lag=(d['timestamp'][sel].astype(float)-d['timestamp_sample'][sel].astype(float))/1e3
        output['fault_receive_minus_sample_ms']=dict(n=len(lag), median=float(np.median(lag)) if len(lag) else None,
            p95=float(np.quantile(lag,.95)) if len(lag) else None,
            minimum=float(np.min(lag)) if len(lag) else None,maximum=float(np.max(lag)) if len(lag) else None)
        output['fault_logged_finite']=dict(position_xy=int((np.isfinite(d['position[0]'][sel])&np.isfinite(d['position[1]'][sel])).sum()),
            velocity_xy=int((np.isfinite(d['velocity[0]'][sel])&np.isfinite(d['velocity[1]'][sel])).sum()), logged=int(sel.sum()))
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    plan=json.loads((args.matrix/'matrix_plan.json').read_text(encoding='utf-8'))
    matrix=json.loads((args.matrix/'matrix_results.json').read_text(encoding='utf-8'))
    if len(matrix['results']) != len(plan['jobs']):
        raise ValueError('matrix is incomplete')
    args.output.mkdir(parents=True,exist_ok=True)
    runs=[]
    failures=[]
    arguments={(name,seed):extra for name,seed,extra in plan['jobs']}
    for result in sorted(matrix['results'],key=lambda r:(r['name'],r['seed'])):
        run=Path(result['path'])
        if result['returncode'] or 'analysis_error' in result:
            failures.append(result)
            continue
        analysis=json.loads((run/'analysis.json').read_text(encoding='utf-8'))
        # Early matrix analysis used an imprecise name. PX4 also updates this
        # timestamp during a reset, so it is not a pure measurement-fusion count.
        for phase in analysis['phase'].values():
            for label in ['ev_pos','ev_vel']:
                if 'unique_fusion_times' in phase[label]:
                    phase[label]['unique_aid_update_times']=phase[label].pop('unique_fusion_times')
        analysis['aid_count_semantics']='time_last_fuse counts fusion or reset; reset increments are reported separately'
        analysis['case_name']=result['name']
        analysis['command_extra_args']=arguments[(result['name'],result['seed'])]
        analysis['extra']=extra_metrics(run,analysis)
        analysis['raw_location']=str(run)
        analysis['record_sha256']={n:sha256(run/n) for n in ['manifest.json','inputs.jsonl','telemetry.jsonl','px4.log','run_status.json','analysis.json']}
        runs.append(analysis)
    groups=defaultdict(list)
    for r in runs:
        groups[r['case_name']].append(r)
    summaries={}
    for name,items in groups.items():
        summaries[name]=dict(seeds=[r['manifest']['seed'] for r in items],
          sent=range_of([r['status']['odometry_sent'] for r in items]),
          variance_z=range_of([r['phase']['normal']['ev_vel'].get('observation_variance',{}).get('2',{}).get('median') for r in items]),
          nominal_xy_rms_m=range_of([r['phase']['normal']['error']['xy_m']['rms'] for r in items]),
          nominal_vxy_rms_mps=range_of([r['phase']['normal']['error']['vxy_mps']['rms'] for r in items]),
          recovery_z_rms_m=range_of([r['phase']['recovery']['error']['z_m']['rms'] for r in items]),
          recovery_vz_rms_mps=range_of([r['phase']['recovery']['error']['vz_mps']['rms'] for r in items]),
          fault_receive_minus_sample_ms=range_of([r['extra'].get('fault_receive_minus_sample_ms',{}).get('median') for r in items]),
          reset_increments={phase:{field:range_of([r['extra']['reset_increments'][phase][field] for r in items])
              for field in ['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter']}
              for phase in ['normal','fault','recovery']},
          aid_updates={label:dict(
              nominal_count=range_of([r['phase']['normal'][label]['unique_aid_update_times'] for r in items]),
              fault_count=range_of([r['phase']['fault'][label]['unique_aid_update_times'] for r in items]),
              recovery_count=range_of([r['phase']['recovery'][label]['unique_aid_update_times'] for r in items]),
              max_gap_s=range_of([r['extra'][label]['max_aid_update_gap_s'] for r in items]),
              recovery_s=range_of([r['extra'][label]['first_fresh_aid_update_after_fault_s'] for r in items])) for label in ['ev_pos','ev_vel']})
    output=dict(expected_runs=len(plan['jobs']), completed_runs=len(runs), failures=failures,
                actual_firmware_commits=sorted({r['firmware_commit'] for r in runs}),
                scope='stationary disarmed synthetic EV diagnostic; not full S01-S14 acceptance',
                acceptance='No mission error thresholds provided; S14 blocked by velocity reset hazard',
                groups=summaries)
    for name,value in [('sitl_summary.json',output),('sitl_runs.json',dict(runs=runs))]:
        (args.output/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in output.items() if k!='groups'},indent=2))


if __name__=='__main__':
    main()
