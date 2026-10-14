"""Offline resource and decision-timeline summaries for an isolated run."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

from btf_replay_measurements import distribution, read_jsonl


def cpu_series(samples, pid, ticks_per_second, tid=None):
    points = []
    for sample in samples:
        entry = sample['processes'].get(str(pid), {})
        record = entry.get('process') if tid is None else entry.get('threads',{}).get(str(tid))
        if record:
            points.append((sample['monotonic_ns'],record['starttime_ticks'],
                           record['utime_ticks']+record['stime_ticks']))
    intervals, cpu_seconds, wall_seconds = [], 0., 0.
    for a,b in zip(points,points[1:]):
        if a[1]!=b[1] or b[0]<=a[0] or b[2]<a[2]:
            continue
        wall = (b[0]-a[0])/1e9
        cpu = (b[2]-a[2])/ticks_per_second
        cpu_seconds += cpu
        wall_seconds += wall
        intervals.append(100*cpu/wall)
    return dict(samples=len(points),covered_seconds=wall_seconds,
        cpu_seconds=cpu_seconds,mean_percent_one_core=100*cpu_seconds/wall_seconds if wall_seconds else None,
        interval_percent_one_core=distribution(intervals))


def analyze(root):
    manifest = json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    sampler = json.loads((root/'sampler.json').read_text(encoding='utf-8'))
    base = json.loads((root/'summary.json').read_text(encoding='utf-8'))
    start,end = manifest['start_ns'],manifest['feed_end_ns']
    all_resources = read_jsonl(root/'resources.jsonl.gz')
    samples = [r for r in all_resources if start<=r['monotonic_ns']<=end]
    ticks = sampler['clock_ticks_per_second']
    process_metrics = {role:cpu_series(samples,pid,ticks) for role,pid in manifest['pids'].items()}
    subject = str(manifest['pids']['subject'])
    tids = sorted({tid for r in samples for tid in r['processes'].get(subject,{}).get('threads',{})})
    threads = {tid:cpu_series(samples,subject,ticks,tid=tid) for tid in tids}
    temperatures,frequencies,governors = defaultdict(list),defaultdict(list),defaultdict(set)
    for sample in samples:
        for path,value in sample['temperatures'].items():
            if value is not None:
                temperatures[path].append(float(value)/1000)
        for path,value in sample['cpu_frequencies'].items():
            if value is not None:
                frequencies[path].append(float(value)/1000)
        for path,value in sample['cpu_governors'].items():
            governors[path].add(value)
    decisions = []
    for row in read_jsonl(root/'observed.jsonl.gz'):
        if row['kind']=='decision':
            data = json.loads(row['text'])
            decisions.append(dict(receive_ns=row['receive_ns'],data=data))
    decisions.sort(key=lambda r:r['receive_ns'])
    episodes, current = [], None
    expiry_times = []
    for row in decisions:
        data = row['data']
        time_s = (row['receive_ns']-start)/1e9
        if data.get('reason')=='receiver_queue_expired':
            expiry_times.append(time_s)
        if data.get('reason')=='clock_unsynced':
            if current is None:
                current = dict(first_observed_s=time_s,last_observed_s=time_s,count=0,
                               previous_observed_expiry_s=expiry_times[-1] if expiry_times else None)
            current['last_observed_s'] = time_s
            current['count'] += 1
        elif current is not None:
            episodes.append(current)
            current = None
    if current is not None:
        episodes.append(current)
    for episode in episodes:
        episode['observed_event_span_s'] = episode['last_observed_s']-episode['first_observed_s']
    return dict(label=manifest['label'],phase=manifest['phase'],run_failures=base['run_failures'],
        source_sha=manifest['input_sha256'],revision=manifest['revision_label'],
        process_cpu=process_metrics,subject_thread_cpu=threads,
        main_thread_id=subject,
        background_cpu={str(pid):cpu_series(samples,pid,ticks) for pid in manifest['resource_sample_pids']
                        if pid not in manifest['pids'].values()},
        temperature_c={key:distribution(values) for key,values in temperatures.items()},
        temperature_sensor_types=sampler.get('temperature_sensor_types',{}),
        cpu_frequency_mhz={key:distribution(values) for key,values in frequencies.items()},
        cpu_governors={key:sorted(str(v) for v in values) for key,values in governors.items()},
        sampler_cost_ms=distribution([r['sample_cost_ns']/1e6 for r in samples]),
        unavailable_read_paths=dict(Counter(path for r in samples for path in r.get('unavailable_reads',{}))),
        system_minus_monotonic_ns=distribution([r['realtime_ns']-r['monotonic_ns'] for r in samples]),
        disk_free_mib=distribution([r['disk_free_bytes']/1024**2 for r in samples]),
        raw_to_pose_ms=base['raw_to_pose_ms'],processing_ms=base['processing_ms'],
        receiver_expiry_observed_s=expiry_times,clock_unsynced_observed_runs=episodes,
        limitations=[
            'CPU percentages use sampled tick deltas; thread identities other than the main TID are not attributed.',
            'Consecutive observed clock_unsynced events are proxies, not exact state transition intervals.',
            'Resource samples cover only their timestamps, not every moment between samples.',
            'No inference of GIL, DDS loss, thermal throttling, or a flight fix is made here.'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    result=analyze(args.run)
    (args.run/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(label=result['label'],phase=result['phase'],
        subject_cpu=result['process_cpu']['subject'],clock_episodes=len(result['clock_unsynced_observed_runs'])),indent=2))
