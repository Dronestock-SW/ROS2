"""Summarize an exact initial time window from a completed ROS capture.

Reads original records only. A 600-second comparison may be nested inside a
longer unchanged 1800-second stability run without adding a second recorder.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import numpy as np


def distribution(values):
    return dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100])))) if values else None


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('capture',type=Path)
    parser.add_argument('--seconds',type=float,default=600)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    meta=json.loads((args.capture/'capture.json').read_text(encoding='utf-8'))
    whole=json.loads((args.capture/'summary.json').read_text(encoding='utf-8'))
    if not 0<args.seconds<=meta['seconds']:raise RuntimeError('Window exceeds capture')
    start=meta['measurement_start_monotonic_ns'];end=start+round(args.seconds*1e9)
    raw=[];mapped=[];poses=[];status=[];ages=[];latencies=[];durations=[];uncertainties=[];delivery=[];source=[]
    counts=Counter();masks=Counter();failures=Counter();reasons=Counter();windows=Counter();pose_windows=Counter();last={}
    first_receiver=None
    with (args.capture/'events.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line);now=row['capture_monotonic_ns']
            if now<start:continue
            if now>=end:break
            topic,value=row['topic'],row['value'];counts[topic]+=1;last[topic]=value
            if topic=='/uwb/status' and first_receiver is None:first_receiver=value
            if topic=='/uwb/raw' and value.get('type')=='uwb_raw_cycle':
                raw.append((now,value));masks[value['valid_mask']]+=1;windows[(now-start)//1_000_000_000]+=1
                durations.append((value['cycle_end_us']-value['cycle_start_us'])/1000)
                for i,code in enumerate(value['failure']):failures[f'A{i+1}_{code}']+=1
            if topic=='/uwb/raw' and value.get('type')=='uwb_raw_status':status.append(value)
            if topic=='/uwb/received' and value.get('single_tag',{}).get('mapped'):
                mapped.append(now);timing=value['single_tag'];source.append(value['host_received_monotonic_ns'])
                observed=[t for t in timing['sample_host_monotonic_ns'] if t is not None]
                if observed:ages.append((now-max(observed))/1e6)
                delivery.append((now-value['host_received_monotonic_ns'])/1e6)
                latencies.append((value['host_received_monotonic_ns']-timing['cycle_end_host_monotonic_ns'])/1e6)
                uncertainties.append(timing['clock']['uncertainty_us'])
            if topic=='/uwb/btf_decision':
                reasons[value.get('reason','unknown')]+=1
                if value.get('published'):
                    poses.append(now);pose_windows[(now-start)//1_000_000_000]+=1
    def rate(times):return (len(times)-1)/((times[-1]-times[0])/1e9) if len(times)>1 else 0.
    def gaps(times):return distribution([(b-a)/1e6 for a,b in zip(times,times[1:])])
    missing=max_missing=duplicates=0
    for (_,a),(_,b) in zip(raw,raw[1:]):
        if (a['boot_id'],a['session'])!=(b['boot_id'],b['session']):continue
        delta=(b['seq']-a['seq'])&0xffffffff
        if 0<delta<0x80000000:missing+=delta-1;max_missing=max(max_missing,delta-1)
        else:duplicates+=1
    raw_times=[t for t,_ in raw];complete=math.floor(args.seconds)
    result=dict(schema=1,seconds=args.seconds,source_capture_seconds=meta['seconds'],
        measurement_start_monotonic_ns=start,measurement_end_monotonic_ns=end,
        source_events_sha256=whole['events_sha256'],counts=dict(counts),valid_masks=dict(masks),failure_codes=dict(failures),
        btf_reasons=dict(reasons),raw_received=len(raw),raw_rate_hz=rate(raw_times),
        valid_four_ratio=masks[15]/len(raw) if raw else 0.,mapped_received=len(mapped),mapped_rate_hz=rate(mapped),
        new_pose_received=len(poses),new_pose_rate_hz=rate(poses),
        raw_min_per_complete_second=min((windows[i] for i in range(complete)),default=0),
        pose_min_per_complete_second=min((pose_windows[i] for i in range(complete)),default=0),
        sequence_missing=missing,max_consecutive_missing=max_missing,raw_duplicate_or_reordered=duplicates,
        raw_interval_ms=gaps(raw_times),pose_interval_ms=gaps(poses),receiver_source_interval_ms=gaps(source),
        source_age_ms=distribution(ages),serial_latency_ms=distribution(latencies),cycle_duration_ms=distribution(durations),
        recorder_delivery_delay_ms=distribution(delivery),clock_uncertainty_us=distribution(uncertainties),
        first_raw=raw[0][1] if raw else None,last_raw=raw[-1][1] if raw else None,
        first_heartbeat=status[0] if status else None,last_heartbeat=status[-1] if status else None,
        free_heap_range=[min(s['free_heap'] for s in status),max(s['free_heap'] for s in status)] if status else None,
        boots=sorted(set(s['boot_id'] for s in status)),sessions=sorted(set(s['session'] for s in status)),
        first_receiver_status=first_receiver,last_status=last.get('/uwb/status'),last_btf_status=last.get('/uwb/btf_status'),
        ground=last.get('/mavros/state'),landed=last.get('/mavros/extended_state'),
        source_recording_errors=whole['recording_errors'],source_recording_queue_dropped=whole['counts'].get('recording_queue_dropped',0))
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('seconds','raw_received','raw_rate_hz','valid_four_ratio','sequence_missing','raw_interval_ms')}))


if __name__=='__main__':main()
