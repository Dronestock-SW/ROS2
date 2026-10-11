"""Attribute recorded B_TF gaps without replaying stale poses or approving flight."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics


def decode(row):
    data = row['data']
    return json.loads(data['data']) if row['type'] == 'std_msgs/msg/String' else data


def gap_stats(times):
    gaps = [(b-a)/1e9 for a,b in zip(times, times[1:])]
    return dict(samples=len(times), max_gap_s=max(gaps, default=None))


def distribution(values):
    values=[v for v in values if type(v) in (int,float) and math.isfinite(v)]
    return dict(samples=len(values), minimum=min(values), median=statistics.median(values),
                maximum=max(values)) if values else dict(samples=0)


def replay_comparison(path):
    """Compare two publication gates on identical solver results, not truth.

    Segment boundaries are replay decision times; their gaps are censored and
    slightly shorter than the full capture's state/sensor boundaries.
    """
    segments=[]; active=False; digest=hashlib.sha256(); previous=0
    with path.open('rb') as src:
        for line in src:
            digest.update(line);row=json.loads(line);t=row['received_monotonic_ns']
            if t<previous:raise ValueError('replay receipt clock reversed')
            previous=t
            if row['airborne']:
                if not active:segments.append([])
                segments[-1].append(row)
            active=row['airborne']
    ds=[d for segment in segments for d in segment]; result={}
    for key in ('legacy_height_gate_publish','replay_publish'):
        intervals=[]
        for segment in segments:
            start,end=segment[0]['received_monotonic_ns'],segment[-1]['received_monotonic_ns']
            times=[d['received_monotonic_ns'] for d in segment if d.get(key) is True]
            for a,b in zip([start,*times],[*times,end]):
                if b-a>250_000_000:
                    intervals.append(dict(duration_s=(b-a)/1e9,
                        boundary_censored=a==start or b==end,
                        start_offset_s=(a-start)/1e9))
        result[key]=dict(published=sum(d.get(key) is True for d in ds),
            gaps_over_250ms=sorted(intervals,key=lambda d:d['duration_s'],reverse=True))
    added=[d for d in ds if d['replay_publish'] and not d['legacy_height_gate_publish']]
    quarantined=[d for d in ds if d['reason']=='observation_jump_quarantined']
    return dict(file=path.name,sha256=digest.hexdigest(),comparison=result,
        added_sources=dict(Counter(d['models']['B_TF']['source'] for d in added)),
        added_with_xyz=sum(d.get('xyz_m') is not None for d in added),
        removed_publications=sum(d['legacy_height_gate_publish'] and not d['replay_publish'] for d in ds),
        quarantine_prior_age_s=distribution([d.get('jump_check',{}).get('prior_age_s') for d in quarantined]),
        quarantine_raw_height_rms_m=distribution([d.get('raw_height_rms_m') for d in quarantined]),
        scope='collector-order replay, decision-time boundaries; no position accuracy or flight proof')


def analyze(capture):
    topic_times = defaultdict(list)
    decisions = []
    raw_cycles = []
    tof_rows=[]
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
            elif topic == '/mavros/downward_0':
                tof_rows.append((len(air_segments),row.get('header_ns') or 0,d))
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
        tof_source_interval_s=distribution([(b[1]-a[1])/1e9 for a,b in zip(tof_rows,tof_rows[1:]) if a[0]==b[0]]),
        tof_samples=len(tof_rows),
        tof_outside_sensor_limits=sum(not valid_tof(d) for _,_,d in tof_rows),
        height_selection_reasons=dict(Counter(s['reason'] for _,d in decisions for s in d.get('height_selection',[]))),
        gaps_over_250ms=sorted(intervals,key=lambda x:x['duration_s'],reverse=True),
        timing_confirmed=False, fusion_verified=False)


def valid_tof(data):
    try:
        r,lo,hi=(float(data[k]) for k in ('range','min_range','max_range'))
        return all(math.isfinite(v) for v in (r,lo,hi)) and 0<lo<=r<=hi
    except (KeyError,ValueError,TypeError):
        return False


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
    p.add_argument('--replay',nargs='+',type=Path,help='Optional paired publication-gate replay files')
    args=p.parse_args()
    result=[analyze(c) for c in args.captures]
    report=result if not args.replay else dict(captures=result,replays=[replay_comparison(f) for f in args.replay])
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    for r in result:
        print(r['capture'],json.dumps({'reasons':r['reasons'],'largest_gaps':r['gaps_over_250ms'][:3]}))


if __name__=='__main__':
    main()
