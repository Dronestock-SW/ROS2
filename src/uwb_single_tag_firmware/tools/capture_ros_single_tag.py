"""Record RAW, mapped events, B_TF decisions and FC ground state without port I/O."""
import argparse
from collections import Counter
import datetime
import hashlib
import json
import math
from pathlib import Path
import time
import queue
import threading

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State, ExtendedState
from std_msgs.msg import String


def distribution(values):
    if not values:return None
    return dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100]))))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--settle-seconds',type=float,default=5.0)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    counts=Counter();masks=Counter();failures=Counter();decisions=Counter()
    raw_times=[];pose_times=[];mapped_times=[];ages=[];latencies=[];durations=[];uncertainties=[];delivery=[];source_times=[]
    heartbeats=[];last={};sequences=[];pose_sources=set();raw_windows=Counter();pose_windows=Counter()
    first_status=None;first_raw=None;last_raw=None
    start=time.monotonic_ns()+round(args.settle_seconds*1e9);end=start+round(args.seconds*1e9)
    metadata=dict(schema=1,seconds=args.seconds,settle_seconds=args.settle_seconds,
        measurement_start_monotonic_ns=start,measurement_end_monotonic_ns=end,
        storage_location=str(args.output.resolve()),recording_queue_capacity=2048,
        capture_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    (args.output/'capture.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    rclpy.init();node=rclpy.create_node('single_tag_ground_capture')
    with (args.output/'events.jsonl').open('w',encoding='utf-8') as stream:
        write_queue=queue.Queue(maxsize=2048)
        write_error=[]
        write_error_count=0
        max_write_us=0
        def writer():
            nonlocal write_error_count,max_write_us
            while True:
                row=write_queue.get()
                if row is None:break
                started=time.monotonic_ns()
                try:stream.write(row)
                except OSError as exc:
                    write_error_count+=1
                    if len(write_error)<10:write_error.append(str(exc))
                max_write_us=max(max_write_us,(time.monotonic_ns()-started)//1000)
        worker=threading.Thread(target=writer,daemon=True);worker.start()
        def record(topic,value,encoded=None):
            nonlocal first_status,first_raw,last_raw
            now=time.monotonic_ns()
            if now>=end:return
            prefix=json.dumps(dict(topic=topic,capture_monotonic_ns=now,measurement_window=now>=start),separators=(',',':'))[:-1]
            encoded=encoded if encoded is not None else json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False)
            try:write_queue.put_nowait(prefix+',"value":'+encoded+'}\n')
            except queue.Full:counts['recording_queue_dropped']+=1
            if now<start:return
            counts[topic]+=1
            last[topic]=value
            if topic=='/uwb/status' and first_status is None:first_status=value
            window=(now-start)//1_000_000_000
            if topic=='/uwb/raw' and value.get('type')=='uwb_raw_cycle':
                if first_raw is None:first_raw=value
                last_raw=value
                raw_times.append(now);raw_windows[window]+=1;masks[value['valid_mask']]+=1
                durations.append((value['cycle_end_us']-value['cycle_start_us'])/1000)
                for i,code in enumerate(value['failure']):failures[f'A{i+1}_{code}']+=1
                sequences.append((value['boot_id'],value['session'],value['seq']))
            if topic=='/uwb/raw' and value.get('type')=='uwb_raw_status':heartbeats.append(value)
            if topic=='/uwb/received' and value.get('single_tag',{}).get('mapped'):
                mapped_times.append(now)
                meta=value['single_tag'];source=meta['sample_host_monotonic_ns']
                source_times.append(value['host_received_monotonic_ns'])
                delivery.append((now-value['host_received_monotonic_ns'])/1e6)
                observed=[x for x in source if x is not None]
                if observed:ages.append((now-max(observed))/1e6)
                latencies.append((value['host_received_monotonic_ns']-meta['cycle_end_host_monotonic_ns'])/1e6)
                uncertainties.append(meta['clock']['uncertainty_us'])
            if topic=='/uwb/btf_decision':
                decisions[value.get('reason','unknown')]+=1
                if value.get('published'):
                    key=(*value.get('source_identity',[]),value.get('seq'),value.get('time_us'))
                    if key in pose_sources:counts['duplicate_published_pose']+=1
                    pose_sources.add(key);pose_times.append(now);pose_windows[window]+=1
        def receive(topic):
            def callback(message):
                try:record(topic,json.loads(message.data),message.data)
                except (ValueError,TypeError,KeyError):counts['parse_error']+=1
            return callback
        subscriptions=[]
        for topic in ('/uwb/raw','/uwb/received','/uwb/status','/uwb/btf_decision','/uwb/btf_status'):
            subscriptions.append(node.create_subscription(String,topic,receive(topic),
                QoSProfile(depth=512,reliability=ReliabilityPolicy.BEST_EFFORT)))
        subscriptions.append(node.create_subscription(State,'/mavros/state',
            lambda m:record('/mavros/state',dict(connected=m.connected,armed=m.armed,mode=m.mode)),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(ExtendedState,'/mavros/extended_state',
            lambda m:record('/mavros/extended_state',dict(landed_state=m.landed_state)),qos_profile_sensor_data))
        try:
            while time.monotonic_ns()<end:rclpy.spin_once(node,timeout_sec=.1)
        finally:
            node.destroy_node();rclpy.shutdown()
            write_queue.put(None);worker.join()
    complete=max(0,math.floor(args.seconds))
    def rate(times):return (len(times)-1)/((times[-1]-times[0])/1e9) if len(times)>1 else 0.
    def gaps(times):return distribution([(b-a)/1e6 for a,b in zip(times,times[1:])])
    skip=0;max_skip=0
    for a,b in zip(sequences,sequences[1:]):
        if a[:2]==b[:2]:
            delta=(b[2]-a[2])&0xffffffff
            if 0<delta<0x80000000:skip+=delta-1;max_skip=max(max_skip,delta-1)
            else:counts['raw_duplicate_or_reordered']+=1
    events_hash=hashlib.sha256()
    with (args.output/'events.jsonl').open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):events_hash.update(chunk)
    result=dict(**metadata,counts=dict(counts),valid_masks=dict(masks),
        failure_codes=dict(failures),btf_reasons=dict(decisions),
        raw_received=len(raw_times),raw_rate_hz=rate(raw_times),valid_four_ratio=masks[15]/len(raw_times) if raw_times else 0.,
        mapped_received=len(mapped_times),mapped_rate_hz=rate(mapped_times),new_pose_received=len(pose_times),new_pose_rate_hz=rate(pose_times),
        complete_one_second_windows=complete,
        raw_min_per_complete_second=min((raw_windows[i] for i in range(complete)),default=0),
        pose_min_per_complete_second=min((pose_windows[i] for i in range(complete)),default=0),
        sequence_missing=skip,max_consecutive_missing=max_skip,
        raw_interval_ms=gaps(raw_times),pose_interval_ms=gaps(pose_times),source_age_ms=distribution(ages),
        serial_latency_ms=distribution(latencies),cycle_duration_ms=distribution(durations),
        recorder_delivery_delay_ms=distribution(delivery),receiver_source_interval_ms=gaps(source_times),
        first_receiver_status=first_status,first_raw=first_raw,last_raw=last_raw,recording_errors=write_error,
        recording_error_count=write_error_count,storage_max_write_us=max_write_us,
        clock_uncertainty_us=distribution(uncertainties),
        boots=sorted(set(x.get('boot_id') for x in heartbeats)),
        sessions=sorted(set(x.get('session') for x in heartbeats)),
        free_heap_range=[min(x['free_heap'] for x in heartbeats),max(x['free_heap'] for x in heartbeats)] if heartbeats else None,
        first_heartbeat=heartbeats[0] if heartbeats else None,last_heartbeat=heartbeats[-1] if heartbeats else None,
        last_status=last.get('/uwb/status'),last_btf_status=last.get('/uwb/btf_status'),
        ground=last.get('/mavros/state'),landed=last.get('/mavros/extended_state'),
        events_sha256=events_hash.hexdigest())
    (args.output/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    compact={k:result[k] for k in ('seconds','raw_received','raw_rate_hz','valid_four_ratio','mapped_rate_hz','new_pose_rate_hz','raw_min_per_complete_second','pose_min_per_complete_second','raw_interval_ms','clock_uncertainty_us','ground','landed')}
    print(json.dumps(compact),flush=True)


if __name__=='__main__':main()
