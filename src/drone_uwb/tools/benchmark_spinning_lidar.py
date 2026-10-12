"""Reproducible sequential-ray motion/occlusion benchmark; never a flight claim.

Report interval velocity error, withholding, false publication in a
degenerate corridor, clock error and processing latency for every variant.
All variants see identical scans and measured IMU samples for each seed.
"""
import argparse
from collections import Counter
from dataclasses import replace
import json
import hashlib
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from drone_uwb.processing.experiments.lidar_spinning import PoseTimeline, SpinningLidar, SpinningScanOdometry


def pose(t, scenario):
    position = np.array([2+.2*np.sin(.7*t),2+.15*np.sin(.5*t),1.7])
    velocity = np.array([.14*np.cos(.7*t),.075*np.cos(.5*t),0.])
    attitude = Rotation.from_euler('ZYX', [.45*np.sin(1.7*t),.035*np.cos(2.6*t),.035*np.sin(3*t)])
    if scenario=='constant_twist':
        position = np.array([2+.08*t,2+.02*t,1.7])
        velocity = np.array([.08,.02,0.])
        attitude = Rotation.from_euler('z',.3*t)
    return position, attitude, velocity


def run(seed, scenario, mode, seconds):
    sensor = SpinningLidar(seed)
    tracker = SpinningScanOdometry(simulation_covariance_model=True,
        deskew=mode!='no_deskew', spatial_support=mode!='no_support')
    timeline = PoseTimeline()
    rng = np.random.default_rng(seed+1000)
    rows = []
    next_scan = .5
    for t in np.arange(0., seconds+.2, .005):
        position, attitude, velocity = pose(t,scenario)
        timeline.add(t,position,attitude.as_matrix(),velocity)
        before, after = pose(t-.0001,scenario)[1],pose(t+.0001,scenario)[1]
        omega = (before.inv()*after).as_rotvec()/.0002
        measured = omega+rng.normal(0,[.04,.025,.02])
        imu_gap = scenario=='imu_gap' and 3.<t<3.15
        if not imu_gap:
            tracker.imu.add_gyro(t,measured)
        if round(t/.005)%2==0:
            measured_attitude = attitude*Rotation.from_rotvec(rng.normal(0,.0008,3))
            tracker.imu.add_attitude(t,measured_attitude.as_matrix())
        if round(t/.005)%4==0:
            tracker.clock.add_exchange(*sensor.exchange(t))
        if t < next_scan+.12:
            continue
        person = None
        missing = None
        if scenario=='right_person':
            def person(times):
                return np.column_stack((2.+.7*np.sin(1.1*times),np.full(len(times),1.15))),.30
        if scenario=='partial_right':
            missing = (-np.pi*.8,-.12)
        scan = sensor.observe(timeline,first_ray_fc_s=next_scan,
            pillars=((.8,1.,.16),(4.3,4.,.2)),person=person,missing_sector=missing)
        if scenario=='parallel_walls':
            # Preserve only returns supported by the two parallel y walls.
            # No x translation is observable; passing is a false publication.
            angles=scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
            beam_time=next_scan+np.arange(len(scan.ranges))*.1/len(scan.ranges)
            poses, rotations=timeline.sample(beam_time)
            rays=np.einsum('nij,nj->ni',rotations,
                np.column_stack((np.cos(angles),np.sin(angles),np.zeros(len(angles)))))
            walls=np.where(rays[:,1]>0,(5.-poses[:,1])/np.maximum(rays[:,1],1e-8),
                -poses[:,1]/np.minimum(rays[:,1],-1e-8))
            scan=replace(scan,ranges=np.where(walls<4.,walls+sensor.rng.normal(0,.01,len(walls)),np.nan))
        started=time.perf_counter()
        observation=tracker.observe_scan(scan,now_local_s=sensor.local_time(t),height_above_floor_m=1.7)
        row=dict(first_ray_fc_s=next_scan,observation=observation,process_ms=(time.perf_counter()-started)*1000)
        if 'timing' in observation:
            row['clock_error_s']=observation['timing']['first_ray_fc_s']-next_scan
        if observation['accepted']:
            end=observation['stamp_s'];begin=end-observation['interval_s']
            expected=(pose(end,scenario)[0]-pose(begin,scenario)[0])[:2][::-1]/(end-begin)
            error=np.asarray(observation['velocity_ne_m_s'])-expected
            row['velocity_error_m_s']=error.tolist()
            covariance=np.asarray(observation['covariance_ne_m2_s2'])
            row['mahalanobis_squared']=float(error@np.linalg.solve(covariance,error))
        rows.append(row)
        next_scan+=.1
    accepted=[r for r in rows if r['observation']['accepted']]
    error=np.array([r['velocity_error_m_s'] for r in accepted])
    reasons=Counter(v for r in rows for v in r['observation']['reasons'])
    return dict(seed=seed,scenario=scenario,mode=mode,scans=len(rows),accepted=len(accepted),
        withholding_fraction=1-len(accepted)/len(rows),reasons=dict(reasons),
        velocity_rms_m_s=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))) if len(error) else None,
        velocity_p95_m_s=float(np.quantile(np.linalg.norm(error,axis=1),.95)) if len(error) else None,
        nominal_95_covariance_coverage=float(np.mean([r['mahalanobis_squared']<=5.991 for r in accepted])) if accepted else None,
        process_p95_ms=float(np.quantile([r['process_ms'] for r in rows],.95)),
        clock_max_error_ms=max(abs(r.get('clock_error_s',0))*1000 for r in rows),rows=rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=10.)
    p.add_argument('--seeds',type=int,nargs='+',default=[11,22,33])
    p.add_argument('--scenarios',nargs='+',default=['clear','constant_twist','right_person','partial_right','parallel_walls','imu_gap'])
    a=p.parse_args()
    if a.output.exists():raise ValueError('new_evidence_file_required')
    import drone_uwb.processing.experiments.lidar_spinning as module
    paths=[Path(__file__),*[Path(module.__file__).with_name(name) for name in
        ('lidar_scanmatch.py','lidar_spinning.py','lidar_sitl_observation.py','lidar_timing.py')]]
    snapshot=a.output.with_suffix('.source');snapshot.mkdir(exist_ok=False)
    hashes={}
    for path in paths:
        data=path.read_bytes();(snapshot/path.name).write_bytes(data)
        hashes[path.name]=hashlib.sha256(data).hexdigest()
    results=[]
    for seed in a.seeds:
        for scenario in a.scenarios:
            for mode in ('full','no_deskew','no_support'):
                r=run(seed,scenario,mode,a.seconds);results.append(r)
                print(json.dumps({k:v for k,v in r.items() if k!='rows'}),flush=True)
    a.output.write_text(json.dumps(dict(scope='synthetic_sensor_only_not_flight',source_hashes=hashes,results=results),indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
