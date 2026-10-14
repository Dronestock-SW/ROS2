"""Read-only takeoff evidence audit; no ROS, MAVLink or flight commands.

Requires numpy and pyulog. Optional plotting requires matplotlib.
ULog distance samples describe FC logging, not companion callback receipt.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from pyulog import ULog


def audit(path: Path):
    ulog = ULog(str(path))

    def data(name, instance=0):
        return next(d.data for d in ulog.data_list
                    if d.name == name and d.multi_id == instance)

    def times(d):
        return d['timestamp'].astype(float) / 1e6

    status = data('vehicle_status')
    armed_indices = np.flatnonzero(status['arming_state'] == 2)
    if not len(armed_indices):
        raise ValueError('No armed interval in this ULog')
    arm = float(times(status)[armed_indices[0]])
    land = data('vehicle_land_detected')
    airborne = float(times(land)[np.flatnonzero((times(land) >= arm) & (land['landed'] == 0))[0]])
    nav = [(float(t), int(mode)) for t, mode in zip(times(status), status['nav_state']) if t >= arm]
    end = next(t for t, mode in nav if t > airborne and mode != 14)
    selection = data('estimator_selector_status')
    selected = selection['primary_instance'][(times(selection) >= arm) & (times(selection) <= end)]
    if len(set(map(int, selected))) != 1:
        raise ValueError('Multiple primary EKFs; segment the flight before this audit')
    instance = int(selected[0])
    flags = data('estimator_status_flags', instance)
    flow_start = float(times(flags)[np.flatnonzero((times(flags) >= arm) & (flags['cs_opt_flow'] == 1))[0]])
    ev = data('vehicle_visual_odometry')
    ev_t = times(ev)
    idx = np.flatnonzero((ev_t >= arm) & (ev_t < end))
    gaps = np.diff(ev_t[idx])
    largest = int(np.argmax(gaps))
    gap_start, gap_end = map(float, ev_t[idx[largest:largest + 2]])
    ev_stop = float(times(flags)[np.flatnonzero((times(flags) > gap_start) & (flags['cs_ev_pos'] == 0))[0]])
    ev_restart = float(times(flags)[np.flatnonzero((times(flags) > ev_stop) & (flags['cs_ev_pos'] == 1))[0]])
    pose = data('vehicle_local_position')
    reset_i = np.flatnonzero((np.diff(pose['xy_reset_counter'].astype(int), prepend=int(pose['xy_reset_counter'][0])) != 0)
                            & (times(pose) > ev_stop))[0]
    reset_t = float(times(pose)[reset_i])
    sp = data('trajectory_setpoint')
    stable_target = (times(sp) >= airborne) & (times(sp) < min(end, reset_t))
    sensor = data('distance_sensor')
    st = times(sensor)
    distance = sensor['current_distance']
    valid = (np.isfinite(distance) & (distance >= sensor['min_distance']) & (distance <= sensor['max_distance']))
    distance_windows = {}
    for name, lo, hi in [('arm_to_liftoff', arm, airborne), ('liftoff_to_flow', airborne, flow_start),
                         ('flow_to_ev_gap', flow_start, gap_start), ('ev_gap', gap_start, gap_end)]:
        mask = (st >= lo) & (st < hi)
        distance_windows[name] = dict(start_fc_s=lo, end_fc_s=hi, samples=int(mask.sum()),
            in_declared_range=int((valid & mask).sum()), min_m=float(distance[mask].min()),
            max_m=float(distance[mask].max()), max_logged_gap_s=float(np.diff(st[mask]).max()))
    first_valid = float(st[np.flatnonzero((st >= airborne) & valid)[0]])
    result = dict(schema=1, input_name=path.name, input_bytes=path.stat().st_size,
        input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        px4_revision=ulog.msg_info_dict.get('ver_sw'), primary_ekf=instance,
        ulog_dropout_count=len(ulog.dropouts),
        clock='PX4 boot seconds; no conversion from wall clock',
        scope='First armed takeoff, ending at first nav state other than OFFBOARD after liftoff',
        events=dict(arm=arm, liftoff=airborne, optical_flow_active=flow_start,
                    first_logged_distance_in_range=first_valid, ev_last_before_gap=gap_start,
                    ev_first_after_gap=gap_end, ev_pos_stopped=ev_stop, ev_pos_restarted=ev_restart,
                    xy_reset=reset_t, offboard_ended=end),
        ev_receiver_gap_s=gap_end-gap_start,
        ev_sample_gap_s=float((ev['timestamp_sample'][idx[largest+1]] - ev['timestamp_sample'][idx[largest]]) / 1e6),
        liftoff_to_flow_s=flow_start-airborne,
        liftoff_to_distance_in_range_s=first_valid-airborne,
        xy_reset_delta_ned_m=[float(pose[f'delta_xy[{i}]'][reset_i]) for i in range(2)],
        xy_target_span_m=[float(np.ptp(sp[f'position[{i}]'][stable_target])) for i in range(2)],
        xy_target_ned_m=[float(sp[f'position[{i}]'][stable_target][0]) for i in range(2)],
        distance_windows=distance_windows,
        distance_logged_gaps_over_150ms=[dict(start_fc_s=float(a), end_fc_s=float(b), duration_s=float(b-a))
            for a,b in zip(st[:-1], st[1:]) if airborne <= a < end and b-a > .15],
        initial_parameters={k: ulog.initial_parameters[k] for k in
            ['EKF2_EV_CTRL','EKF2_EV_DELAY','EKF2_DELAY_MAX','EKF2_GPS_CTRL','EKF2_OF_CTRL','EKF2_RNG_CTRL',
             'MPC_TKO_SPEED','MPC_Z_VEL_MAX_UP','SENS_FLOW_ROT']},
        limitations=[
            'FC estimated motion and estimator resets are not independent physical displacement.',
            'ULog cannot determine which B_TF input was rejected; companion decision records are required.',
            'Distance sample gaps are FC logged gaps, not proof of an I2C or MAVROS fault.',
            'Optical-flow activation and first in-range distance are different conditions.',
            'This flight is not the separately reported stable POSITION hover comparison.'])
    return result, dict(sensor=sensor, ev=ev, pose=pose, sp=sp, flags=flags)


def plot(result, streams, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    e = result['events']; arm = e['arm']; lo = arm-.5; hi = e['offboard_ended']+.2
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    def t(d):
        return d['timestamp']/1e6-arm
    d = streams['sensor']
    axes[0].plot(t(d), d['current_distance'], '.-', ms=3, label='FC logged downward range')
    min_distance=float(np.median(d['min_distance']))
    axes[0].axhline(min_distance, color='#b42318', ls='--', label=f'Reported minimum = {min_distance:.2f} m')
    axes[0].set(ylabel='Distance (m)', ylim=(0, 1.5))
    v = streams['ev']; ts = t(v)
    axes[1].vlines(ts, 0, 1, color='#2463a8', linewidth=.7)
    axes[1].set(ylabel='EV packet receipts', yticks=[])
    p=streams['pose'];sp=streams['sp']
    for i, name in enumerate(['N', 'E']):
        target=result['xy_target_ned_m'][i]
        shown=(t(p)>=lo-arm)&(t(p)<=hi-arm)
        axes[2].plot(t(p)[shown], p[['x','y'][i]][shown]-target, label=f'FC estimated {name} - fixed target')
    axes[2].set(ylabel='Estimated error (m)')
    f=streams['flags']
    for name,label in [('cs_ev_pos','EV position active'),('cs_opt_flow','Optical flow active')]:
        axes[3].step(t(f),f[name],where='post',label=label)
    axes[3].set(ylabel='Fusion flag',yticks=[0,1],ylim=(-.1,1.15),xlabel='Seconds after ARM')
    for ax in axes:
        ax.set_xlim(lo-arm,hi-arm);ax.grid(alpha=.2)
        ax.axvspan(e['ev_last_before_gap']-arm,e['ev_first_after_gap']-arm,color='#f3c64e',alpha=.22)
        ax.axvline(e['liftoff']-arm,color='#27834b',ls=':')
        ax.axvline(e['xy_reset']-arm,color='#b42318',ls=':')
        if ax.get_legend_handles_labels()[0]:ax.legend(loc='upper left',fontsize=8)
    axes[0].set_title(f"{result['input_name']}: initial range/flow availability and later EV outage\n"
        f"Green: liftoff | Yellow: {result['ev_receiver_gap_s']:.3f} s EV receipt gap | Red: XY estimator reset")
    fig.savefig(output,dpi=160)
    plt.close(fig)


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plot',type=Path)
    args=parser.parse_args()
    if args.output.resolve()==args.ulog.resolve() or (args.plot and args.plot.resolve()==args.ulog.resolve()):
        parser.error('Output must not overwrite the input ULog')
    result,streams=audit(args.ulog)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    if args.plot:
        args.plot.parent.mkdir(parents=True,exist_ok=True)
        plot(result,streams,args.plot)
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
