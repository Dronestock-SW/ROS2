"""Diagnostic 2D scan registration only; no calibrated odometry or FC output.

Assumes a rigid, static scan plane. Moving objects, tilt, unknown mounting,
per-ray motion and changing height can bias the result. Do not use as truth.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def points(scan, minimum, omit=None):
    r=np.asarray(scan['ranges'],float)
    a=scan['angle_min']+np.arange(len(r))*scan['angle_increment']
    keep=np.isfinite(r)&(r>=max(minimum,scan['range_min']))&(r<=min(11.,scan['range_max']))
    if omit is not None:
        keep &= (np.floor((a+np.pi)/(np.pi/2)).astype(int)%4)!=omit
    return np.column_stack([r[keep]*np.cos(a[keep]),r[keep]*np.sin(a[keep])])


def nearest(a,b):
    d=((a[:,None,:]-b[None,:,:])**2).sum(axis=2)
    idx=d.argmin(axis=1)
    return idx,np.sqrt(d[np.arange(len(a)),idx])


def register(current, reference):
    candidates=[]
    if min(len(current),len(reference))<30:raise ValueError('Insufficient scan points')
    for yaw in (-15.,-5.,0.,5.,15.):
        a=math.radians(yaw);rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]]);shift=np.zeros(2)
        for _ in range(40):
            transformed=current@rot.T+shift
            idx,dist=nearest(transformed,reference)
            keep=dist<min(.35,float(np.quantile(dist,.8)))
            if keep.sum()<30:break
            p=transformed[keep];q=reference[idx[keep]];pc=p.mean(axis=0);qc=q.mean(axis=0)
            u,_,vt=np.linalg.svd((p-pc).T@(q-qc));delta=vt.T@u.T
            if np.linalg.det(delta)<0:vt[-1]*=-1;delta=vt.T@u.T
            trans=qc-delta@pc;rot=delta@rot;shift=delta@shift+trans
            if np.linalg.norm(trans)<1e-5 and abs(math.atan2(delta[1,0],delta[0,0]))<1e-5:break
        transformed=current@rot.T+shift
        _,dist=nearest(transformed,reference);_,rev=nearest(reference,transformed)
        trimmed=dist[dist<=np.quantile(dist,.8)]
        score=float(np.sqrt(np.mean(trimmed**2)))
        candidates.append(dict(translation_m=shift.tolist(),yaw_deg=math.degrees(math.atan2(rot[1,0],rot[0,0])),
                               trimmed_rms_m=score,overlap_forward=float(np.mean(dist<.2)),
                               overlap_reverse=float(np.mean(rev<.2))))
    best=min(candidates,key=lambda c:c['trimmed_rms_m'])
    best['translation_norm_m']=float(np.linalg.norm(best['translation_m']))
    return best


def analyze(scans):
    times=np.array([s['t'] for s in scans])
    def at(t):return scans[int(np.argmin(abs(times-t)))]
    ref=at(times[0]+1)
    series=[]
    for t in np.arange(ref['t']+1,times[-1],5):
        target=at(t)
        series.append(dict(time_s=target['t'],reference_s=ref['t'],**register(points(target,.5),points(ref,.5))))
    sensitivity=[]
    for offset in (0.,2.,4.):
        ref=at(times[0]+offset);target=at(times[-1]-4+offset)
        for minimum in (.5,2.):
            sensitivity.append(dict(reference_s=ref['t'],time_s=target['t'],minimum_m=minimum,omit_sector=None,
                                    **register(points(target,minimum),points(ref,minimum))))
    ref=at(times[0]+2);target=at(times[-1]-2)
    for sector in range(4):
        sensitivity.append(dict(reference_s=ref['t'],time_s=target['t'],minimum_m=.5,omit_sector=sector,
                                **register(points(target,.5,sector),points(ref,.5,sector))))
    return dict(scope='OFFLINE_UNCALIBRATED_2D_REGISTRATION_NOT_POSITION_TRUTH',
                deskew=False,mount_confirmed=False,dynamic_object_rejection=False,
                series=series,endpoint_sensitivity=sensitivity)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scans',type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();result=analyze(json.loads(a.scans.read_text(encoding='utf-8')))
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))
