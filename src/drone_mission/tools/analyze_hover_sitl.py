"""Explain hover timer interruptions from immutable SIH mission and telemetry logs."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def analyze(root):
    rows = [json.loads(s) for s in (root/'mission.jsonl').read_text(encoding='utf-8').splitlines()]
    telemetry = [json.loads(s) for s in (root/'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
    poses = {r['message']['time_boot_ms']: r['message'] for r in telemetry
             if r['message']['mavpackettype'] == 'LOCAL_POSITION_NED'}
    dwell = [r for r in rows if r.get('phase_before_tick',r['phase']) == 'DWELL']
    checks = []
    for r in dwell:
        p = poses.get(round(r['clock_s']*1000))
        if p is None:
            raise ValueError('missing exact LOCAL_POSITION_NED timestamp')
        target = r.get('target_before_tick',r.get('mission_target_xy', [2.9, 2.]))
        zgoal = r.get('height_before_tick_m')
        if zgoal is None:zgoal=r['goal']['altitude_enu_m']
        values = dict(xy=math.dist(r['pose_xy'], target),
                      speed=math.hypot(*r['velocity_xy']),
                      z=abs(-p['z']-zgoal), vz=abs(p['vz']))
        if r.get('snapshot'):
            values['uwb'] = math.dist(r['snapshot']['uwb_xy'], target)
        limits = dict(xy=.15, speed=.1, z=.15, vz=.1, uwb=.25)
        failed = [key for key, value in values.items() if value > limits[key]]
        checks.append(dict(t=r['clock_s'], values=values, failed=failed))
    longest = {}
    for name in ('xy', 'speed', 'z', 'vz', 'all_available'):
        begin = None; best = 0.
        for row in checks:
            ok = not row['failed'] if name == 'all_available' else name not in row['failed']
            if ok:
                if begin is None: begin = row['t']
                best = max(best, row['t']-begin)
            else: begin = None
        longest[name] = best
    def stats(values):
        a = np.asarray(values)
        return dict(p50=float(np.median(a)), p95=float(np.quantile(a,.95)), maximum=float(np.max(a))) if len(a) else None
    return dict(run=str(root), status=json.loads((root/'run_status.json').read_text()),
        samples=len(dwell), duration_s=dwell[-1]['clock_s']-dwell[0]['clock_s'] if dwell else 0.,
        failure_counts=dict(Counter(k for row in checks for k in row['failed'])),
        failure_combinations=dict(Counter(','.join(row['failed']) or 'none' for row in checks)),
        longest_continuous_s=longest,
        error_stats={k:stats([row['values'][k] for row in checks if k in row['values']]) for k in ('xy','speed','z','vz','uwb')},
        truth_xy_error_stats=stats([math.dist(r['truth_xy'],r.get('target_before_tick',r.get('mission_target_xy',[2.9,2.]))) for r in dwell]),
        truth_speed_stats=stats([math.hypot(*r['truth_velocity_xy']) for r in dwell]),
        velocity_estimation_error_stats=stats([math.dist(r['velocity_xy'],r['truth_velocity_xy']) for r in dwell]),
        horizontal_control_counts=dict(Counter((r['goal'] or {}).get('horizontal_control','position') for r in dwell)),
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'mission.jsonl',root/'telemetry.jsonl')},
        scope='Exact timestamp telemetry z/vz; legacy logs lack UWB and evaluate phase after tick. In legacy logs all_available omits UWB and the final transition tick.')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    results=[analyze(root) for root in args.runs]
    args.output.write_text(json.dumps(dict(results=results),indent=2)+'\n',encoding='utf-8')
    for row in results:
        print(json.dumps({k:row[k] for k in ('run','samples','failure_counts','longest_continuous_s','error_stats')}))
