"""Read-only PX4 takeoff diagnostics, including estimator and SIH truth."""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    trace=json.loads((args.run/'trace.json').read_text(encoding='utf-8'))
    base=trace[0]['clock_us']-trace[0]['elapsed_s']*1e6
    log=ULog(str(next(args.run.rglob('*.ulg'))))
    data={d.name:d.data for d in log.data_list if d.multi_id==0}
    selections={
        'vehicle_local_position':['x','y','z','vx','vy','vz','xy_reset_counter','z_reset_counter'],
        'vehicle_local_position_groundtruth':['x','y','z','vx','vy','vz'],
        'vehicle_local_position_setpoint':['x','y','z','vx','vy','vz'],
        'vehicle_attitude':['q[0]','q[1]','q[2]','q[3]'],
        'vehicle_attitude_groundtruth':['q[0]','q[1]','q[2]','q[3]'],
        'estimator_aid_src_aux_vel':['observation[0]','observation[1]','innovation[0]','innovation[1]','fused','time_last_fuse'],
        'estimator_status_flags':['cs_aux_vel','cs_ev_pos','cs_baro_hgt','cs_rng_hgt','cs_in_air','cs_inertial_dead_reckoning'],
        'lidar_velocity2d_status':['accepted','rejected','fusion_active','reason'],
        'vehicle_status':['nav_state','arming_state'],
        'vehicle_thrust_setpoint':['xyz[2]'],
        'distance_sensor':['current_distance'],
        'estimator_sensor_bias':['accel_bias[0]','accel_bias[1]','accel_bias[2]','gyro_bias[0]','gyro_bias[1]','gyro_bias[2]'],
        'estimator_aid_src_rng_hgt':['observation','innovation','fused','innovation_rejected'],
        'estimator_aid_src_baro_hgt':['observation','innovation','fused','innovation_rejected'],
    }
    output={}
    for t in (19,23,25,27,29,31,33):
        at=base+t*1e6; sample={}
        for name,fields in selections.items():
            d=data.get(name)
            if d is None:continue
            idx=np.searchsorted(d['timestamp'],at,side='right')-1
            if idx<0:continue
            sample[name]={key:float(d[key][idx]) for key in fields if key in d}
        output[str(t)]=sample
    output['parameters']={k:v for k,v in log.initial_parameters.items()
        if k.startswith(('EKF2_','IMU_','SIH_','MPC_'))}
    active=[r for r in trace if r['phase']=='TAKING_OFF' and r['elapsed_s']>=35]
    if active:
        d=data['vehicle_local_position'];t=data['vehicle_local_position_groundtruth']
        selected=(d['timestamp']>=active[0]['clock_us'])&(d['timestamp']<=active[-1]['clock_us'])
        ref=next(r for r in trace if r['phase']=='TAKING_OFF')['height_reference_m']
        home=next(r for r in trace if r['phase']=='PREPARING')['estimated_xy']
        measurements=dict(estimated_xy_error=np.hypot(d['y']-(home[0]-2),d['x']-(home[1]-2)),
            estimated_vxy=np.hypot(d['vx'],d['vy']),estimated_vz=np.abs(d['vz']),
            estimated_height_error=np.abs(-d['z']-ref))
        output['stability_35s_onward']={k:dict(p50=float(np.quantile(v[selected],.5)),
            p95=float(np.quantile(v[selected],.95)),maximum=float(v[selected].max())) for k,v in measurements.items()}
        ts=(t['timestamp']>=active[0]['clock_us'])&(t['timestamp']<=active[-1]['clock_us'])
        output['stability_35s_onward']['truth_vxy']={k:float(v) for k,v in zip(('p50','p95','maximum'),
            np.quantile(np.hypot(t['vx'][ts],t['vy'][ts]),[.5,.95,1.]))}
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
