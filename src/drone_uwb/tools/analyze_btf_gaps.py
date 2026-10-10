"""Attribute recorded B_TF gaps without replaying stale poses or approving flight."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path


def decode(row):
    data = row['data']
    return json.loads(data['data']) if row['type'] == 'std_msgs/msg/String' else data


def gap_stats(times):
    gaps = [(b-a)/1e9 for a,b in zip(times, times[1:])]
    return dict(samples=len(times), max_gap_s=max(gaps, default=None))


def analyze(capture):
    topic_times = defaultdict(list)
    decisions = []
    raw_cycles = []
    state = landed = None
    state_time = landed_time = -10**30
    air_segments = []
    in_air = False
    last_t = 0
    manifest_bytes = (capture/'manifest.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    digest = hashlib.sha256()
    with (capture/'events.jsonl').open('rb') as source:
        for line in source:
            digest.update(line)
            row = json.loads(line)
            t, topic = row['received_monotonic_ns'], row['topic']
            if t < last_t:
                raise ValueError('capture receipt clock reversed')
            last_t = t
            if topic not in ('/uwb/btf_decision', '/uwb/received', '/uwb/btf_pose',
                '/mavros/state','/mavros/extended_state','/mavros/local_position/odom',
                '/mavros/imu/data','/mavros/downward_0'):
                continue
            d = decode(row)
            header_age= row['received_ros_ns']-(row.get('header_ns') or 0)
            fresh=0<=header_age<=1_500_000_000
            if topic == '/mavros/state':
                state, state_time = d, t if fresh else -10**30
            elif topic == '/mavros/extended_state':
                landed, landed_time = d.get('landed_state'), t if fresh else -10**30
            airborne = (state is not None and state.get('armed') is True
                and state.get('connected') is True and landed == 2
                and 0 <= t-state_time <= 1_500_000_000 and 0 <= t-landed_time <= 1_500_000_000)
            if airborne and not in_air:
                air_segments.append([t,t])
            if airborne:
                air_segments[-1][1] = t
            in_air = airborne
            if not airborne:
                continue
            topic_times[topic].append(t)
            if topic == '/uwb/btf_decision':
                decisions.append((t,d))
            elif topic == '/uwb/received' and d.get('message',{}).get('type') == 'uwb_raw_cycle':
                raw_cycles.append((t,d['message']))
    intervals = []
    for start,end in air_segments:
        outputs = [t for t in topic_times['/uwb/btf_pose'] if start<=t<=end]
        edges = [start,*outputs,end]
        for a,b in zip(edges,edges[1:]):
            if b-a <= 250_000_000:
                continue
            ds = [d for t,d in decisions if a<t<b]
            raw = [(t,d) for t,d in raw_cycles if a<t<b]
            reasons = Counter(d.get('pose_blocked_reason') or d['reason'] for d in ds if not d.get('published'))
            scores = [d.get('models',{}).get('B_TF',{}).get('best_score_m') for d in ds]
            scores = [x for x in scores if isinstance(x,(float,int))]
            intervals.append(dict(start_offset_s=(a-manifest['started']['monotonic_ns'])/1e9,
                duration_s=(b-a)/1e9, boundary_censored=a==start or b==end,
                reasons=dict(reasons), raw_cycles=len(raw),
                raw_cycle_max_gap_s=gap_stats([t for t,_ in raw])['max_gap_s'],
                best_subset_rms_min_m=min(scores,default=None),
                published_decisions=sum(d.get('published') is True for d in ds)))
    return dict(capture=capture.name, manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
        events_sha256=digest.hexdigest(), scope='fresh ARM and IN_AIR; receipt times; not ground truth',
        air_duration_s=sum((b-a)/1e9 for a,b in air_segments), air_segments=len(air_segments),
        topics={k:gap_stats(v) for k,v in topic_times.items()},
        reasons=dict(Counter(d.get('pose_blocked_reason') or d['reason'] for _,d in decisions)),
        raw_anchor_missing=[sum(not usable_range(d,i) for _,d in raw_cycles) for i in range(4)],
        gaps_over_250ms=sorted(intervals,key=lambda x:x['duration_s'],reverse=True),
        timing_confirmed=False, fusion_verified=False)


def usable_range(cycle,index):
    ranges=cycle.get('raw_slant_m')
    if not isinstance(ranges,list) or len(ranges)!=4:return False
    value=ranges[index]
    return (type(value) in (int,float) and math.isfinite(value) and value>0
            and bool(cycle.get('valid_mask',0)&(1<<index)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('captures',nargs='+',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=[analyze(c) for c in args.captures]
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    for r in result:
        print(r['capture'],json.dumps({'reasons':r['reasons'],'largest_gaps':r['gaps_over_250ms'][:3]}))


if __name__=='__main__':
    main()
