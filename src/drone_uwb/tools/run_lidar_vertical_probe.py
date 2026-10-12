"""Compare diagnostic EKF interventions with identical synthetic sensor seeds."""
import argparse
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys

import numpy as np
from analyze_lidar_state_output import metrics, sha256


def fusion_updates(samples, key, mask):
    """Count new fuse timestamps, excluding one held across a phase boundary."""
    stamps = np.array([r[key] for r in samples], dtype=np.uint64)
    changed = np.r_[False, stamps[1:] != stamps[:-1]]
    return int(np.count_nonzero(mask & changed & (stamps > 0)))


def analyze(path):
    records={key:[] for key in ['SAMPLE','AXIS','VRESET']}
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            key, _, payload=line.partition(' ')
            if key in records:
                # C printf emits lowercase non-finite literals before EKF init.
                # Preserve them in the raw log; count finite samples separately.
                payload=re.sub(r'(?<=:)-?nan(?=[,}])','NaN',payload)
                payload=re.sub(r'(?<=:)(-?)inf(?=[,}])',r'\1Infinity',payload)
                records[key].append(json.loads(payload))
    samples=records['SAMPLE']
    if len(samples)!=12000:
        raise ValueError(f'incomplete probe: {len(samples)} samples')
    phases={}
    for name,lo,hi in [('before',25,40),('after',50,120)]:
        chosen=[r for r in samples if lo<=r['t']<hi]
        increments=np.r_[0,np.diff(np.array([r['vz_reset'] for r in samples],dtype=np.int64)) % 256]
        mask=np.array([lo<=r['t']<hi for r in samples])
        phases[name]=dict(n=len(chosen),
            metrics={key:metrics([r[key] for r in chosen]) for key in
                     ['ekf_z','ekf_vz','out_z','out_vz','ekf_vx','ekf_vy','vz_var','z_var']},
            vz_resets=int(increments[mask].sum()),
            flags={key:dict(true=sum(r[key] for r in chosen),n=len(chosen)) for key in
                   ['internal_z_valid','internal_vz_valid','ev_vel','fake_hgt','baro']},
            aux_updates=fusion_updates(samples,'aux_last_fuse',mask),
            ev_updates=fusion_updates(samples,'ev_last_fuse',mask))
    axis={}
    for i in range(3):
        rows=[r for r in records['AXIS'] if r['axis']==i and 50e6<=r['us']<120e6]
        axis[str(i)]={key:metrics([r[key] for r in rows]) for key in ['dz','dvz','gain_z','gain_vz','innovation']}
        axis[str(i)]['dz_sum']=sum(r['dz'] for r in rows)
    resets=[r for r in records['VRESET'] if 50e6<=r['us']<120e6]
    return dict(raw_sha256=sha256(path),phases=phases,axis_updates=axis,
                height_reset_branch_entries=len(resets),
                height_reset_branch_dz_sum=sum(r['dz'] for r in resets),
                height_age_s=metrics([(r['us']-r['last_hgt_us'])/1e6 for r in resets]),
                vertical_fuse_age_s=metrics([(r['us']-r['last_ver_vel_us'])/1e6 for r in resets]),
                trace={key:[r[key] if np.isfinite(r[key]) else None for r in samples[::10]] for key in
                       ['t','ekf_z','ekf_vz','out_z','out_vz','vz_reset','internal_z_valid','internal_vz_valid']})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seeds',type=int,nargs='+',default=[11,22,33,44,55])
    p.add_argument('--tilts',type=float,nargs='+',default=[0,20])
    p.add_argument('--modes',nargs='+',default=['stock','no_vreset','rotated_vreset','position_only','aux2d','ned3d','no_output_reset'])
    a=p.parse_args()
    parent=os.environ.get('LIDAR_PROBE_PARENT_NETNS')
    if not parent:
        env=dict(os.environ,LIDAR_PROBE_PARENT_NETNS=os.readlink('/proc/self/ns/net'))
        result=subprocess.run(['unshare','-Unc','--keep-caps',sys.executable,str(Path(__file__).resolve()),*sys.argv[1:]],env=env)
        raise SystemExit(result.returncode)
    namespace=os.readlink('/proc/self/ns/net')
    if namespace==parent or {n for _,n in socket.if_nameindex()}!={'lo'}:
        raise ValueError('private loopback-only namespace required')
    build_manifest=a.probe.resolve().parent/'manifest.json'
    if build_manifest.is_file():
        build=json.loads(build_manifest.read_text(encoding='utf-8'))
        if sha256(a.probe)!=build['binary_sha256']:
            raise ValueError('probe binary differs from build manifest')
        if not build.get('diagnostic_overlay_linked',True):
            unsupported=set(a.modes)&{'no_vreset','rotated_vreset','no_output_reset'}
            if unsupported:
                raise ValueError('causal modes require diagnostic overlay: '+','.join(sorted(unsupported)))
    a.output.mkdir(parents=True,exist_ok=False)
    results=[]
    for tilt in a.tilts:
        for seed in a.seeds:
            for mode in a.modes:
                log=a.output/f'{mode}-tilt{tilt:g}-seed{seed}.log'
                with log.open('w',encoding='utf-8') as stream:
                    subprocess.run([str(a.probe.resolve()),mode,str(seed),str(tilt)],stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=45)
                result=dict(mode=mode,seed=seed,tilt_deg=tilt,raw_file=str(log),**analyze(log))
                results.append(result)
                m=result['phases']['after']
                print(mode,seed,tilt,'z RMS',m['metrics']['ekf_z'].get('rms'),
                      'out vz RMS',m['metrics']['out_vz'].get('rms'),'resets',m['vz_resets'],flush=True)
    manifest=dict(scope='synthetic sensor EKF-library experiment, not SIH or flight',
                  binary_sha256=sha256(a.probe),network_namespace=namespace,physical_writes=0,
                  duration_s=120,height_removed_s=40,ev_start_s=20,
                  imu_hz=200,ev_hz=10,accel_sigma_mps2=.02,gyro_sigma_radps=.001,
                  ev_xy_sigma=.01,ev_xy_variance=.01,ev_z_variance=1e4,
                  vertical_truth=0.,position_truth=[0.,0.,0.],seeds=a.seeds,tilts=a.tilts,
                  modes=a.modes,raw_lidar_used=False,all_S01_S14_passed=False,
                  mission_acceptance_thresholds=None,runs=results)
    (a.output/'comparison.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__': main()
