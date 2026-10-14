"""Summarize already produced file/ROS replays against original decisions.

No ROS or FC access. Never relabel collector-order replay as original callback
execution. All timestamp comparisons use integer nanoseconds before subtraction.
"""
import argparse
from collections import Counter
import hashlib
import gzip
import json
from pathlib import Path

import numpy as np


def read_rows(path):
    if not path.exists() and path.with_suffix(path.suffix+'.gz').exists():
        path = path.with_suffix(path.suffix+'.gz')
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def stats(values):
    a = list(values)
    return (dict(count=len(a), median=float(np.median(a)), p95=float(np.percentile(a,95)),
                 p99=float(np.percentile(a,99)), max=float(max(a))) if a else dict(count=0))


def summarize(rows):
    focus = [r for r in rows if 17484 <= r.get('seq',-1) <= 17510]
    around = [r for r in rows if 17470 <= r.get('seq',-1) <= 17525]
    publish = lambda r: r.get('replay_publish', r.get('published', False))
    accepted = [r for r in around if publish(r)]
    before = next((r for r in reversed(accepted) if r['seq'] < 17484), None)
    after = next((r for r in accepted if r['seq'] > 17510), None)
    return dict(decisions=len(rows), reasons=dict(Counter(r.get('reason') for r in rows)),
        published=sum(publish(r) for r in rows),
        processing_ms=stats(r['processing_ms'] for r in rows if 'processing_ms' in r),
        focus_count=len(focus), focus_reasons=dict(Counter(r['reason'] for r in focus)),
        focus_published=[r['seq'] for r in focus if publish(r)],
        around_max_source_gap_s=max(((b['stamp_ns']-a['stamp_ns'])/1e9
            for a,b in zip(accepted,accepted[1:])),default=None),
        focus_bracketing_gap_s=((after['stamp_ns']-before['stamp_ns'])/1e9
            if before is not None and after is not None else None),
        focus_boundaries_observed=before is not None and after is not None,
        focus=[dict(seq=r['seq'],reason=r['reason'],published=publish(r),
                    height_selection=r.get('height_selection'),
                    sensor_snapshot=r.get('sensor_snapshot')) for r in focus])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path)
    p.add_argument('results',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args()
    recorded, tof, imu = [], [], []
    digest = hashlib.sha256()
    with (args.capture/'events.jsonl').open('rb') as src:
        for line in src:
            digest.update(line)
            row = json.loads(line)
            topic = row.get('topic')
            if topic == '/uwb/btf_decision':
                recorded.append(json.loads(row['data']['data']))
            elif topic == '/mavros/downward_0':
                tof.append(row)
            elif topic == '/mavros/imu/data':
                imu.append(row)
    output = dict(input_sha256=digest.hexdigest(), recorded=summarize(recorded), variants={},
        tof_full_source_interval_ms=stats((b['header_ns']-a['header_ns'])/1e6 for a,b in zip(tof,tof[1:])),
        imu_full_source_interval_ms=stats((b['header_ns']-a['header_ns'])/1e6 for a,b in zip(imu,imu[1:])),
        limitations=['Collector receipt order is not original B_TF callback order.',
            'A ROS replay changes scheduling and adds transport latency.',
            'Old 10 Hz data cannot validate new physical 50 Hz measurements.',
            'No FC fusion, control, physical drift or flight validation.'])
    for path in sorted(args.results.glob('offline-*.jsonl*')):
        rows = read_rows(path)
        name = path.name.removesuffix('.gz').removesuffix('.jsonl')
        output['variants'][name] = summarize(rows)
    for path in sorted(args.results.glob('*ros-process*/summary.json')):
        summary = json.loads(path.read_text())
        rows = read_rows(path.parent/'decisions.jsonl')
        output['variants'][path.parent.name] = dict(summarize(rows), runtime=summary)
    # Pure age gate sensitivity on the seven *recorded* ToF-stale cycles.
    stale = [r for r in recorded if 17484<=r.get('seq',-1)<=17510
             and r.get('height_selection',[{}])[-1].get('reason') == 'tof_stale']
    output['recorded_tof_age_gate_only'] = {str(ms): [r['seq'] for r in stale
        if r['height_selection'][-1]['tof_age_s']<=ms/1000] for ms in (100,120,150,200,300)}
    args.output.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:dict(focus_reasons=v['focus_reasons'],focus_published=v['focus_published'],
        around_max_source_gap_s=v['around_max_source_gap_s']) for k,v in output['variants'].items()}))


if __name__ == '__main__':
    main()
