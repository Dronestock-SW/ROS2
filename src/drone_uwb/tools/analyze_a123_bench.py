"""Summarize original A123 records; position error requires a measured reference."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np


def read(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]


def analyze(root):
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    records=read(root/'uwb/raw/received.jsonl')
    cycles=[r for r in records if r['message'].get('type')=='uwb_raw_cycle']
    decisions=read(root/'uwb/processed/decisions.jsonl')
    obs=[r['observation'] for r in decisions if r.get('observation')]
    telemetry=read(root/'telemetry.jsonl')
    tof=[r['message'] for r in telemetry if r['topic']=='/mavros/downward_0']
    states=[r['message'] for r in telemetry if r['topic']=='/mavros/state']
    battery=[r['message'] for r in telemetry if r['topic']=='/mavros/battery']
    samples=[r['message'] for r in cycles]
    valid_tof=[r['range'] for r in tof if r['range'] is not None and r['min_range']<=r['range']<=r['max_range']]
    out=dict(capture=root.name,stage=manifest['stage'],raw_cycles=len(cycles),
        masks=dict(Counter(r['valid_mask'] for r in samples)),
        anchors={f'A{i+1}':dict(valid=sum(bool(r['valid_mask']&(1<<i)) for r in samples),
            failures=dict(Counter(r['failure'][i] for r in samples))) for i in range(4)},
        accepted_observations=len(obs),acceptance_fraction=len(obs)/len(cycles) if cycles else None,
        reasons=dict(Counter(r['reason'] for r in decisions)),
        maximum_accepted_gap_s=max((b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(obs,obs[1:])) if len(obs)>1 else None,
        tof_samples=len(tof),tof_valid_samples=len(valid_tof),
        tof_valid_median_m=float(np.median(valid_tof)) if valid_tof else None,
        tof_last_m=tof[-1]['range'] if tof else None,
        fc_state_samples=len(states),any_armed=any(r['armed'] for r in states) if states else None,
        battery_voltage_v=battery[-1]['voltage'] if battery else None,
        battery_remaining_pct=round(battery[-1]['percentage']*100,1) if battery and battery[-1]['percentage'] is not None else None,
        fc_output_enabled=False,reference_xyz_m=manifest.get('reference_xyz_m'))
    if obs:
        xy=np.asarray([[r['x'],r['y']] for r in obs])
        out['observed_xy_mean_m']=xy.mean(axis=0).tolist()
        out['observed_xy_std_m']=xy.std(axis=0).tolist()
        reference=manifest.get('reference_xyz_m')
        if reference is not None:
            error=np.linalg.norm(xy-np.asarray(reference[:2]),axis=1)
            out['reference_xy_error_m']=dict(rmse=float(np.sqrt(np.mean(error**2))),
                maximum=float(error.max()),p95=float(np.percentile(error,95)),count=len(error))
    (root/'analysis.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return out


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('capture',type=Path)
    args=parser.parse_args();print(json.dumps(analyze(args.capture),ensure_ascii=False,indent=2))
