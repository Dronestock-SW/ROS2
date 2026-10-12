"""Synthetic causal H80 response and overlapping-output correlation audit.

Exercises the pinned range solver only. It excludes the runtime B_TF admission,
ToF subset selection, radio clock mapping, multipath and FC fusion.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from drone_uwb.processing.solvers.h80 import fit_h80
from drone_uwb.processing.experiments.h80_b import BSettings


def run(config, *, frequency, seed, sigma, seconds=35.):
    anchors=np.asarray(config['anchors_xyz_m'],float)
    settings=BSettings(**config['B'])
    rng=np.random.default_rng(seed)
    # Four sequential anchor observations per 25 ms cycle. Ranges are never
    # reused as new measurements, and every fitted history is causal.
    t=np.arange(0.,seconds,.025/4)
    idx=np.arange(len(t))%4
    x=3.+.30*np.sin(2*np.pi*frequency*t) if frequency else np.full_like(t,3.)
    truth=np.column_stack((x,np.full_like(t,2.),np.full_like(t,1.)))
    ranges=np.linalg.norm(truth-anchors[idx],axis=1)+rng.normal(0.,sigma,len(t))
    output=[]
    for end in np.arange(2.,seconds,.025):
        keep=(t>=end-settings.window_s)&(t<=end)
        fit=fit_h80(anchors,idx[keep],t[keep]-end,ranges[keep],0.,settings)
        if fit.ok:
            output.append((end,fit.x0,3.+.30*np.sin(2*np.pi*frequency*end) if frequency else 3.))
    values=np.asarray(output)
    error=values[:,1]-values[:,2]
    result=dict(frequency_hz=frequency,seed=seed,range_noise_std_m=sigma,
        samples=len(output),window_s=settings.window_s,
        xy_axis_rms_error_m=float(np.sqrt(np.mean(error**2))),
        correlation_lag_25ms=float(np.corrcoef(error[:-1],error[1:])[0,1]) if sigma else None)
    if frequency:
        omega=2*np.pi*frequency
        basis=np.column_stack((np.sin(omega*values[:,0]),np.cos(omega*values[:,0]),np.ones(len(values))))
        a,b,_=np.linalg.lstsq(basis,values[:,1],rcond=None)[0]
        phase=np.arctan2(b,a)
        result.update(gain=float(np.hypot(a,b)/.30),phase_deg=float(np.rad2deg(phase)),
            equivalent_phase_delay_s=float(-phase/omega),
            delay_scope='frequency-specific sinusoidal phase; not transport or EKF2_EV_DELAY calibration')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    config=json.loads(args.config.read_text(encoding='utf-8'))
    results=[run(config,frequency=f,seed=11,sigma=0.) for f in (.1,.25,.5,1.)]
    results += [run(config,frequency=0.,seed=s,sigma=.08) for s in (11,22,33,44,55)]
    result=dict(schema=1,config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        solver_settings=asdict(BSettings(**config['B'])),results=results,
        scope=__doc__,physical_validation=False,settings_changed=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(results,indent=2))
