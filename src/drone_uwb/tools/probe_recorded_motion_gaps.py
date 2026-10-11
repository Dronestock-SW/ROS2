"""Counterfactual gaps in previously accepted, measured UWB XY tracks.

This exercises only the motion requalification gate, not raw/TDMA/H80 or PX4.
Recorded coordinates and timestamps are unchanged; withheld observations are
synthetic gaps. No claim of independent position accuracy is possible here.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from drone_uwb.processing.motion_reacquisition import MotionReacquisition


def probe(path, gaps=(.2,.4,.6)):
    rows=[json.loads(s) for s in path.read_text(encoding='utf-8').splitlines()]
    rows=[r for r in rows if r.get('airborne') and r.get('replay_publish')
          and r.get('xyz_m') is not None]
    trials=[]
    for i,row in enumerate(rows):
        prefix=rows[max(0,i-24):i+1]
        for gap in gaps:
            checker=MotionReacquisition()
            for r in prefix:checker.observe(r['stamp_ns'],r['xy_m'])
            if checker._reference()[0] is None:continue
            start=row['stamp_ns'];last=start;future=[]
            for r in rows[i+1:]:
                dt=(r['stamp_ns']-last)/1e9
                if not 0<dt<=checker.max_gap_s:break
                last=r['stamp_ns']
                elapsed=(last-start)/1e9
                if elapsed>=gap:future.append(r)
                if elapsed>=gap+.25:break
            if not future or (future[-1]['stamp_ns']-future[0]['stamp_ns'])/1e9<checker.confirm_s:
                continue
            # Future samples are never used to train/check the prefix fit.
            for r in future:
                ready,e=checker.check(r['stamp_ns'],r['xy_m'],eligible=True,key='qualified_xy')
                if ready:break
            trials.append(dict(start_seq=row.get('seq'),last_seq=r.get('seq'),
                artificial_gap_s=gap,recovered=ready,evidence=e,
                displacement_m=math.dist(row['xy_m'],r['xy_m']),
                outside_old_near_radius=math.dist(row['xy_m'],r['xy_m'])>.1))
    return dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        scope='recorded XY, artificial missing interval; isolated gate; overlapping trials',
        trials=len(trials),recovered=sum(t['recovered'] for t in trials),
        recovered_outside_old_near_radius=sum(t['recovered'] and t['outside_old_near_radius'] for t in trials),
        reasons=dict(Counter(t['evidence']['reason'] for t in trials)),
        results=trials,independent_ground_truth=False,physical_validation=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('replays',type=Path,nargs='+')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();results=[probe(f) for f in args.replays]
    args.output.write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps([{k:v for k,v in r.items() if k!='results'} for r in results]))


if __name__=='__main__':main()
