"""Verify actual PX4 ULog aiding transitions; synthetic SIH is not flight proof."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog


def aligned(times, data, field):
    indices = np.searchsorted(data['timestamp'], times, side='right') - 1
    return np.where(indices >= 0, data[field][np.maximum(indices, 0)], 0)


def analyze(path):
    manifest = json.loads((path/'manifest.json').read_text(encoding='utf-8'))
    status = json.loads((path/'run_status.json').read_text(encoding='utf-8'))
    if status['errors'] or len(status['ulog_files']) != 1:
        raise ValueError('completed_single_ulog_run_required')
    log = path/status['ulog_files'][0]
    ulog = ULog(str(log))
    data = {row.name:row.data for row in ulog.data_list if row.multi_id == 0}
    local, flags = data['vehicle_local_position'], data['estimator_status_flags']
    admission=data.get('lidar_velocity2d_status')
    aux = data.get('estimator_aid_src_aux_vel')
    ev = data.get('estimator_aid_src_ev_pos')
    base = status['first_clock_us']
    phases = {}
    failures = []
    for name, (lo,hi) in manifest['phases'].items():
        lower, upper = base+lo*1e6, base+hi*1e6
        selected = (local['timestamp']>=lower)&(local['timestamp']<upper)
        times = local['timestamp'][selected]
        if not len(times):
            raise ValueError('missing_phase_data:'+name)
        if times[0] > lower+100000 or times[-1] < upper-100000:
            failures.append(name+': incomplete_logged_window')
        item = dict(samples=len(times),
            validity={key:float(local[key][selected].mean()) for key in
                ['xy_valid','v_xy_valid','z_valid','v_z_valid']},
            flags={key:float(aligned(times,flags,key).mean()) for key in
                ['cs_ev_pos','cs_ev_vel','cs_gnss_pos','cs_gnss_vel','cs_opt_flow',
                 'cs_fake_pos','cs_inertial_dead_reckoning','cs_in_air','cs_baro_hgt','cs_rng_hgt']},
            fuse_updates={})
        item['flags']['cs_aux_vel'] = float(aligned(times,admission,'fusion_active').mean()) if admission is not None else 0.
        for key,aid in [('lidar',aux),('uwb',ev)]:
            fresh = np.r_[False,np.diff(aid['time_last_fuse'].astype(np.int64))>0] if aid is not None else []
            item['fuse_updates'][key] = int((fresh & (aid['timestamp']>=lower) & (aid['timestamp']<upper)).sum()) if aid is not None else 0
        phases[name]=item
        if item['flags']['cs_in_air']:
            failures.append(name+': unexpected_airborne_state')
        if name in ('lidar_only','uwb_gap','uwb_restored','all_restored'):
            if item['fuse_updates']['lidar'] < (hi-lo)*7:
                failures.append(name+': lidar_fusion_rate_below_7Hz')
            if item['flags']['cs_aux_vel'] < .98 or item['validity']['xy_valid'] < .98:
                failures.append(name+': independent_horizontal_aiding_unavailable')
        if name in ('lidar_only','uwb_gap'):
            if item['flags']['cs_ev_pos'] or item['flags']['cs_ev_vel'] or item['flags']['cs_gnss_pos'] or item['flags']['cs_gnss_vel'] or item['flags']['cs_opt_flow']:
                failures.append(name+': another_horizontal_source_active')
        if name=='all_horizontal_gap':
            if item['flags']['cs_aux_vel'] or item['fuse_updates']['lidar'] or item['validity']['xy_valid']:
                failures.append(name+': horizontal_loss_not_withdrawn')
    # Height outage overlaps UWB recovery and the independent LiDAR interval.
    height = (local['timestamp']>=base+62e6)&(local['timestamp']<base+75e6)
    if manifest['scenario']=='all_height_gap':
        if local['z_valid'][height].any() or local['v_z_valid'][height].any():
            failures.append('height_outage: unobserved_vertical_state_still_valid')
        if phases['all_restored']['validity']['z_valid'] < .98:
            failures.append('height_restoration_failed')
    selected=(local['timestamp']>=base+25e6)&(local['timestamp']<base+120e6)
    resets={key:int((np.r_[0,np.diff(local[key].astype(np.int64))%256])[selected].sum())
            for key in ['xy_reset_counter','vxy_reset_counter','z_reset_counter','vz_reset_counter']}
    if resets['vz_reset_counter'] > (2 if manifest['scenario']=='all_height_gap' else 0):
        failures.append('unexpected_vertical_velocity_resets')
    if ulog.dropouts:
        failures.append('ulog_dropouts')
    return dict(run=path.name,scope='SYNTHETIC_DISARM_SIH_NOT_PHYSICAL_FLIGHT',
        scenario=manifest['scenario'],seed=manifest['seed'],source_firmware=manifest['firmware_commit'],
        binary_sha256=manifest['binary_sha256'],ulog_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
        simulated_duration_s=status['simulated_duration_s'],phases=phases,reset_counts_25_to_120_s=resets,
        admission=None if admission is None else dict(accepted=int(admission['accepted'][-1]),rejected=int(admission['rejected'][-1])),
        dropout_count=len(ulog.dropouts),checks_passed=not failures,failures=failures)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    results=[analyze(path) for path in args.runs]
    args.output.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for row in results:
        print(json.dumps(dict(run=row['run'],checks_passed=row['checks_passed'],failures=row['failures']),ensure_ascii=False))
    raise SystemExit(0 if all(row['checks_passed'] for row in results) else 1)
