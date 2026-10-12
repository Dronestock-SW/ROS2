"""Audit rotating-beam latency, validity, PX4 admission/fusion and hover.

Statistics retain rejected scans and distinguish ground/takeoff from the
moving/hover window. Ground truth is read only after each trial completes.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from audit_hover10_sitl import analyze as hover
from audit_lidar_sitl_frontend import audit as frontend


def distribution(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return dict(n=0,minimum=None,maximum=None,mean=None,p95=None)
    return dict(n=len(a),minimum=float(a.min()),maximum=float(a.max()),
        mean=float(a.mean()),p95=float(np.quantile(a,.95)))


def analyze(root):
    h=hover(root);f=frontend(root)
    rows=[json.loads(v) for v in (root/'scan_observations.jsonl').read_text().splitlines()]
    mission=[json.loads(v) for v in (root/'mission.jsonl').read_text().splitlines()]
    active=[r['clock_s'] for r in mission if r['phase'] in ('MOVING','DWELL','RETURNING')]
    selected=[r for r in rows if active and active[0]<=r['clock_s']<=active[-1]]
    accepted=[r for r in selected if r['observation']['accepted']]
    timings=[r['observation']['timing'] for r in selected if 'timing' in r['observation']]
    result=dict(run=str(root),hover=h,frontend=f,active_scans=len(selected),
        active_accepted=len(accepted),active_reasons=dict(Counter(
            reason for r in selected for reason in r['observation']['reasons'])),
        active_withholding_fraction=1-len(accepted)/len(selected) if selected else None,
        process_ms=distribution([r['process_ms'] for r in selected]),
        clock_uncertainty_ms=distribution([r['clock_uncertainty_s']*1000 for r in timings]),
        imu_gap_ms=distribution([r['imu_max_gap_s']*1000 for r in timings]),
        first_ray_clock_error_ms=distribution([(r['observation']['timing']['first_ray_fc_s']-r['first_ray_fc_s'])*1000
            for r in selected if 'timing' in r['observation']]),
        scope='isolated simulation; capture-time gyro/clock alignment; no physical calibration')
    u=ULog(str(next(root.rglob('*.ulg'))))
    ds={d.name:d.data for d in u.data_list if d.multi_id==0}
    result['ulog_topics']=list(ds)
    if 'lidar_velocity2d' in ds:
        d=ds['lidar_velocity2d'];t=d['timestamp']/1e6
        mask=(t>=active[0])&(t<=active[-1]) if active else np.zeros(len(t),dtype=bool)
        result['px4_receipt_minus_midpoint_ms']=distribution((d['timestamp'][mask]-d['timestamp_sample'][mask])/1000)
        emitted=[r['observation'] for r in rows if r['observation']['accepted']]
        nominal=np.array([r['sample_midpoint_s'] for r in emitted])
        shifts=[]
        for index in np.where(mask)[0]:
            sample=d['timestamp_sample'][index]/1e6
            nearest=np.argmin(abs(nominal-sample)) if len(nominal) else None
            if nearest is not None and abs(nominal[nearest]-sample)<.05:
                velocity=emitted[nearest]['velocity_ne_m_s']
                if np.allclose(velocity,[d['velocity_ne[0]'][index],d['velocity_ne[1]'][index]],atol=1e-6):
                    shifts.append((sample-nominal[nearest])*1000)
        result['mavlink_time_conversion_shift_ms']=distribution(shifts)
    if 'lidar_velocity2d_status' in ds:
        d=ds['lidar_velocity2d_status'];t=d['timestamp']/1e6
        mask=(t>=active[0])&(t<=active[-1]) if active else np.zeros(len(t),dtype=bool)
        result['admission_reasons']=dict(Counter(map(int,d['reason'][mask])))
    if 'estimator_aid_src_aux_vel' in ds:
        d=ds['estimator_aid_src_aux_vel'];t=d['timestamp']/1e6
        mask=(t>=active[0])&(t<=active[-1]) if active else np.zeros(len(t),dtype=bool)
        result['auxiliary_fusion']=dict(reports=int(mask.sum()),fused=int(d['fused'][mask].sum()))
    if 'vehicle_local_position' in ds:
        d=ds['vehicle_local_position'];t=d['timestamp']/1e6
        mask=(t>=active[0])&(t<=active[-1]) if active else np.zeros(len(t),dtype=bool)
        result['resets_during_motion']={name:int(d[name][mask].max()-d[name][mask].min())
            for name in ('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter') if mask.any()}
    result['evidence_hashes']={name:hashlib.sha256((root/name).read_bytes()).hexdigest()
        for name in ('scan_observations.jsonl','mission.jsonl','telemetry.jsonl','manifest.json')}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    values=[analyze(r) for r in a.runs]
    a.output.write_text(json.dumps(values,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for r in values:
        print(json.dumps({k:v for k,v in r.items() if k not in ('hover','frontend','ulog_topics','evidence_hashes')}),flush=True)
