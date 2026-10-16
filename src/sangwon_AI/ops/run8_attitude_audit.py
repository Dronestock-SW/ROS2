"""Compare original PX4 attitude demand, tracking and XY control on one clock.

Offline ULog reader. CSV and figure are analysis; no physical position ground
truth, tuning decision, FC command or source-log modification is produced.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def hold(data, keys, timestamps, max_age_us=None):
    source=data['timestamp'];index=np.searchsorted(source,timestamps,side='right')-1
    values=np.array([data[k][np.maximum(index,0)] for k in keys],dtype=float).T
    valid=index>=0
    if max_age_us is not None:valid &= timestamps-source[np.maximum(index,0)]<=max_age_us
    values[~valid]=np.nan
    return values


def euler(q):
    w,x,y,z=q.T
    return np.degrees(np.array([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))]).T)


def finite_max(values):
    values=values[np.isfinite(values)]
    return float(np.max(values)) if len(values) else None


def main():
    from pyulog import ULog
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ulog',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--arm-boot-s',type=float,required=True);p.add_argument('--land-relative-s',type=float,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    u=ULog(str(args.ulog));d={x.name:x.data for x in u.data_list if x.multi_id==0}
    a=d['vehicle_attitude'];timestamps=a['timestamp'];t=timestamps/1e6-args.arm_boot_s
    q=np.array([a[f'q[{i}]'] for i in range(4)]).T
    qd=hold(d['vehicle_attitude_setpoint'],[f'q_d[{i}]' for i in range(4)],timestamps,100_000)
    actual=euler(q);desired=euler(qd)
    angle=np.degrees(2*np.arccos(np.clip(np.abs(np.sum(q*qd,axis=1)),0,1)))
    pos=hold(d['vehicle_local_position'],['x','y','z','vx','vy'],timestamps,150_000)
    target=hold(d['vehicle_local_position_setpoint'],['x','y','z','thrust[0]','thrust[1]','thrust[2]'],timestamps,150_000)
    error=target[:,:2]-pos[:,:2];distance=np.linalg.norm(error,axis=1);speed=np.linalg.norm(pos[:,3:5],axis=1)
    takeoff=hold(d['takeoff_status'],['takeoff_state','tilt_limit'],timestamps)
    control=hold(d['vehicle_control_mode'],['flag_control_offboard_enabled','flag_control_position_enabled','flag_control_attitude_enabled'],timestamps)
    flags=hold(d['estimator_status_flags'],['cs_opt_flow','cs_ev_pos'],timestamps)
    rows=np.column_stack([t,actual,desired,angle,distance,speed,-pos[:,2],target,takeoff,control,flags])
    headers=['arm_relative_s','actual_roll_deg','actual_pitch_deg','actual_yaw_deg','desired_roll_deg',
        'desired_pitch_deg','desired_yaw_deg','attitude_geodesic_error_deg','fc_xy_error_m','fc_xy_speed_m_s',
        'fc_enu_z_m','target_ned_x_m','target_ned_y_m','target_ned_z_m','thrust_ned_x','thrust_ned_y',
        'thrust_ned_z','takeoff_state','tilt_limit_rad','offboard_control','position_control','attitude_control',
        'flow_fusion','ev_position_fusion']
    with (args.output/'attitude_control.csv').open('w',encoding='utf-8',newline='') as out:
        writer=csv.writer(out);writer.writerow(headers);writer.writerows(rows)
    intervals=[]
    for start,end in [(0,1),(1,2.188),(2.188,3.188),(3.188,args.land_relative_s)]:
        mask=(t>=start)&(t<end)&np.isfinite(angle)
        if not np.any(mask):continue
        intervals.append(dict(start_s=start,end_s=end,samples=int(mask.sum()),
            attitude_error_median_deg=float(np.median(angle[mask])),attitude_error_p95_deg=float(np.percentile(angle[mask],95)),
            attitude_error_max_deg=float(np.max(angle[mask])),
            desired_roll_span_deg=[float(np.min(desired[mask,0])),float(np.max(desired[mask,0]))],
            desired_pitch_span_deg=[float(np.min(desired[mask,1])),float(np.max(desired[mask,1]))],
            xy_error_max_m=finite_max(distance[mask]),xy_speed_max_m_s=finite_max(speed[mask])))
    tr=d['takeoff_status'];mode=d['vehicle_control_mode']
    changes=lambda data,key:[dict(arm_relative_s=float(ts/1e6-args.arm_boot_s),value=float(value))
        for i,(ts,value) in enumerate(zip(data['timestamp'],data[key])) if i==0 or value!=data[key][i-1]]
    mask=(t>=2.188)&(t<args.land_relative_s)&np.isfinite(target[:,3])
    attitude_window=(t>=0)&(t<args.land_relative_s)&np.isfinite(angle)
    peak_index=np.flatnonzero(attitude_window)[np.argmax(angle[attitude_window])]
    product=np.sum(error*target[:,3:5],axis=1)
    nontrivial=mask&(distance>.03)&(np.linalg.norm(target[:,3:5],axis=1)>.001)
    summary=dict(scope='RUN8_FC_DEMAND_AND_TRACKING_ONLY',ulog_sha256=hashlib.sha256(args.ulog.read_bytes()).hexdigest(),
        px4_commit=u.msg_info_dict.get('ver_sw',''),arm_boot_s=args.arm_boot_s,land_relative_s=args.land_relative_s,
        attitude_target_pairing='causal_hold_max_age_100ms',physical_translation_ground_truth=False,
        intervals=intervals,takeoff_state_changes=changes(tr,'takeoff_state'),
        offboard_changes=changes(mode,'flag_control_offboard_enabled'),
        position_control_changes=changes(mode,'flag_control_position_enabled'),
        attitude_control_changes=changes(mode,'flag_control_attitude_enabled'),
        peak_pre_land_attitude_error=dict(arm_relative_s=float(t[peak_index]),error_deg=float(angle[peak_index]),
            actual_roll_pitch_deg=actual[peak_index,:2].tolist(),desired_roll_pitch_deg=desired[peak_index,:2].tolist()),
        horizontal_thrust_toward_estimated_goal_fraction=float(np.mean(product[nontrivial]>0)) if np.any(nontrivial) else None,
        nontrivial_thrust_samples=int(nontrivial.sum()),
        parameters={k:u.initial_parameters.get(k) for k in ['COM_SPOOLUP_TIME','MPC_TKO_RAMP_T','MPC_TILTMAX_LND',
            'MPC_TILTMAX_AIR','MPC_THR_HOVER','MPC_Z_VEL_MAX_UP','MPC_XY_P','MPC_XY_VEL_P_ACC',
            'MPC_XY_VEL_I_ACC','SENS_BOARD_X_OFF','SENS_BOARD_Y_OFF','SENS_FLOW_ROT']})
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
    for index,label in enumerate(['Roll','Pitch']):
        axes[index].plot(t,actual[:,index],label='FC actual');axes[index].plot(t,desired[:,index],label='PX4 demand')
        axes[index].set_ylabel(label+' (deg)');axes[index].legend(loc='upper right')
    axes[2].plot(t,distance,label='Estimated XY error (m)');axes[2].plot(t,speed,label='Estimated XY speed (m/s)');axes[2].legend()
    axes[3].step(t,takeoff[:,0],where='post',label='PX4 takeoff state')
    axes[3].step(t,flags[:,0],where='post',label='Optical flow fusion')
    axes[3].step(t,flags[:,1],where='post',label='EV position fusion');axes[3].legend()
    for ax in axes:
        ax.axvline(args.land_relative_s,color='red',linestyle='--');ax.grid(alpha=.25);ax.set_xlim(0,args.land_relative_s+.1)
    axes[0].set_ylim(-4,4);axes[1].set_ylim(-4,4);axes[2].set_ylim(0,.25)
    axes[-1].set_xlabel('Seconds after external ARM command (FC clock)')
    fig.tight_layout();fig.savefig(args.output/'attitude_control.png',dpi=150);plt.close(fig)
    print(json.dumps(summary))


if __name__=='__main__':main()
