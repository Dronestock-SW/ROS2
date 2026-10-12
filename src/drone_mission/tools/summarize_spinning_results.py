"""Compact measured evidence; retain input hashes and fixed-window denominators."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stability(root):
    rows=[json.loads(v) for v in (root/'mission.jsonl').read_text(encoding='utf-8').splitlines()]
    dwell=[r for r in rows if r.get('phase_before_tick')=='DWELL']
    result=dict(samples=len(dwell))
    if not dwell:return result
    flags={name:[] for name in ('xy','speed','z','vz','uwb')}
    curve=[]
    longest=0.;start=None;previous=None
    for row in dwell:
        s=row['snapshot'];t=row['clock_s'];goal=row['target_before_tick']
        values=dict(xy=math.dist(row['pose_xy'],goal),speed=math.hypot(*row['velocity_xy']),
            z=abs(s['altitude_enu_m']-row['height_before_tick_m']),
            vz=abs(s['vertical_speed_m_s']),uwb=math.dist(s['uwb_xy'],goal))
        limits=dict(xy=.15,speed=.1,z=.15,vz=.1,uwb=.25)
        for k,v in values.items():flags[k].append(v<=limits[k]+1e-6)
        valid=(all(v[-1] for v in flags.values()) and row['armed'] and row['landed']==2
            and row['mode']=='OFFBOARD')
        if not valid:start=None
        elif start is None or previous is None or t-previous>.12:start=t
        if start is not None:longest=max(longest,t-start)
        previous=t
        curve.append(dict(time_s=t-dwell[0]['clock_s'],fc_xy_m=values['xy'],
            truth_xy_m=math.dist(row['truth_xy'],goal),speed_m_s=values['speed'],vz_m_s=values['vz']))
    result.update(longest_continuous_s=longest,
        violation_samples={k:sum(not b for b in v) for k,v in flags.items()},curve=curve)
    return result


def summarize_audit(audit):
    h=audit['hover'];m=h['manifest']
    result={k:v for k,v in audit.items() if k not in ('hover','ulog_topics','evidence_hashes')}
    result.update(scenario=h['scenario'],seed=h['seed'],aided=m['lidar_aiding'],
        deskew=m['deskew_enabled'],status=h['status'],parameters=h['parameters'],
        first_10s_ulog=h.get('first_10s_ulog',[]),whole_dwell=h.get('whole_dwell'),
        mission_10s_complete=h['mission_10s_complete'],precision_hover_pass=h['precision_hover_pass'],
        stability=stability(Path(audit['run'])))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frontend',type=Path,required=True)
    p.add_argument('--audits',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    source=json.loads(a.frontend.read_text(encoding='utf-8'))
    frontend=[{k:v for k,v in r.items() if k!='rows'} for r in source['results']]
    runs=[summarize_audit(r) for path in a.audits for r in json.loads(path.read_text(encoding='utf-8'))]
    identifiers=[r['run'] for r in runs]
    if len(set(identifiers))!=len(identifiers):raise ValueError('duplicate_run_in_summary')
    value=dict(scope='synthetic sequential-ray sensor and isolated SIH; not physical validation',
        input_hashes={str(path):sha(path) for path in [a.frontend,*a.audits]},
        frontend_source_hashes=source['source_hashes'],frontend=frontend,runs=runs,
        terminal_counts=dict(Counter(r['status']['final_reason'] for r in runs)))
    a.output.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for r in runs:
        print(json.dumps(dict(run=r['run'],reason=r['status']['final_reason'],
            first=r['first_10s_ulog'],longest=r['stability'].get('longest_continuous_s'),
            wall_clearance_m=r['status'].get('min_wall_surface_clearance_m'))),flush=True)


if __name__=='__main__':main()
