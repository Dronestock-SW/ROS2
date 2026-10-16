"""Offline identity and timing analysis for the isolated B_TF replay harness.

No ROS imports. Publish-call timestamps are not wire-delivery timestamps.
Ambiguous identities and negative intervals never enter latency percentiles.
"""
from collections import Counter, defaultdict
import gzip
import json
import math


def write_jsonl(path, rows):
    with gzip.open(path, 'wt', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')


def read_jsonl(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def distribution(values):
    values = sorted(values)
    def percentile(q):
        index = (len(values)-1)*q
        lower = int(index)
        upper = min(lower+1, len(values)-1)
        return values[lower]+(values[upper]-values[lower])*(index-lower)
    return dict(count=len(values), **({
        'p50': percentile(.5), 'p95': percentile(.95), 'p99': percentile(.99),
        'min': values[0], 'max': values[-1],
    } if values else {key: None for key in ('p50','p95','p99','min','max')}))


def gap_summary(times, start, end):
    if end < start:
        raise ValueError('reversed_observation_window')
    times = sorted(t for t in times if start <= t <= end)
    boundaries = [start, *times, end]
    return dict(count=len(times), window_ms=(end-start)/1e6,
        max_including_edges_ms=max(b-a for a,b in zip(boundaries,boundaries[1:]))/1e6,
        max_between_outputs_ms=max((b-a for a,b in zip(times,times[1:])), default=0)/1e6
            if len(times)>1 else None,
        first_output_after_start_ms=(times[0]-start)/1e6 if times else None,
        last_output_before_end_ms=(end-times[-1])/1e6 if times else None)


def epoch_key(data):
    names = ('boot_id', 'session', 'sf_seq', 'seq')
    return tuple(data[name] for name in names) if all(name in data for name in names) else None


def raw_key(data, decision=False):
    names = ('seq', 'source_cycle_end_us' if decision else 'cycle_end_us')
    return tuple(data[name] for name in names) if all(name in data for name in names) else None


def summarize(inputs, observed, *, start_ns, feed_end_ns, observation_end_ns):
    """Return aggregate metrics and an explicit per-observation linkage table."""
    if not start_ns <= feed_end_ns <= observation_end_ns:
        raise ValueError('invalid_run_boundaries')
    raw, epochs = defaultdict(list), defaultdict(list)
    returned = [r for r in inputs if r.get('publish_returned', False)]
    for row in returned:
        if row.get('type') == 'uwb_raw_cycle':
            raw[raw_key(row)].append(row)
        elif row.get('type') == 'uwb_tdma_epoch':
            epochs[epoch_key(row)].append(row)
    raw.pop(None, None)
    epochs.pop(None, None)
    decisions, poses, statuses, malformed = [], [], [], []
    for index, row in enumerate(observed):
        try:
            kind = row['kind']
            if kind in ('decision', 'status'):
                data = json.loads(row['text'])
                if not isinstance(data, dict):
                    raise ValueError('expected_json_object')
                record = dict(index=index, receive_ns=row['receive_ns'], data=data)
                (decisions if kind == 'decision' else statuses).append(record)
            elif kind == 'pose':
                if (len(row['xy_m']) != 2 or not all(math.isfinite(v) for v in row['xy_m'])
                        or type(row['stamp_ns']) is not int):
                    raise ValueError('invalid_pose_fields')
                poses.append(dict(row, index=index))
        except (ValueError, TypeError, KeyError) as exc:
            malformed.append(dict(index=index, error=str(exc)))
    published = defaultdict(list)
    decision_links, branch_times, raw_decision_ordinals = [], defaultdict(list), set()
    for decision in decisions:
        data = decision['data']
        matches = raw.get(raw_key(data, decision=True), [])
        record = dict(kind='decision', observed_index=decision['index'],
            receive_ns=decision['receive_ns'], reason=data.get('reason'),
            raw_match='unique' if len(matches)==1 else 'ambiguous' if matches else 'missing')
        if len(matches)==1:
            record['raw_ordinal'] = matches[0]['ordinal']
            raw_decision_ordinals.add(matches[0]['ordinal'])
        decision_links.append(record)
        if data.get('published'):
            published[data.get('stamp_ns')].append(decision)
        duration = data.get('processing_ms')
        if isinstance(duration, (int, float)) and math.isfinite(duration) and duration>=0:
            branch = json.dumps([data.get('reason'), bool(data.get('published')),
                                 data.get('pose_blocked_reason'), data.get('height_source')])
            branch_times[branch].append(duration)
    # Resolve both stamp and position. Repeated observations of one decision
    # remain ambiguous; do not silently pick the first/fastest received copy.
    candidates, usages = {}, Counter()
    for pose in poses:
        matches = []
        for decision in published.get(pose['stamp_ns'], []):
            xy = decision['data'].get('xy_m', [])
            if len(xy)==2 and all(abs(a-b)<1e-12 for a,b in zip(pose['xy_m'], xy)):
                matches.append(decision)
        candidates[pose['index']] = matches
        if len(matches)==1:
            usages[matches[0]['index']] += 1
    links, categories = [], Counter()
    latencies, epoch_latencies, pair_waits, call_times = [], [], [], []
    for pose in poses:
        link = dict(kind='pose', observed_index=pose['index'], receive_ns=pose['receive_ns'],
                    stamp_ns=pose['stamp_ns'])
        matches = candidates[pose['index']]
        if not start_ns <= pose['receive_ns'] <= observation_end_ns:
            category = 'outside_observation_window'
        elif len(matches) != 1:
            category = 'ambiguous_decision' if matches else 'no_matching_decision'
        elif usages[matches[0]['index']] != 1:
            category = 'multiple_poses_for_decision'
        else:
            decision = matches[0]
            data = decision['data']
            link['decision_observed_index'] = decision['index']
            source = raw.get(raw_key(data, decision=True), [])
            if len(source) != 1:
                category = 'ambiguous_raw' if source else 'no_matching_raw'
            else:
                source = source[0]
                latency_ns = pose['receive_ns']-source['before_ns']
                category = 'matched' if latency_ns >= 0 else 'negative_latency'
                link.update(raw_ordinal=source['ordinal'], raw_to_pose_ns=latency_ns)
                if latency_ns >= 0:
                    latencies.append(latency_ns/1e6)
                    call_times.append((source['after_ns']-source['before_ns'])/1e6)
                epoch = epochs.get(epoch_key(data.get('tdma', {})), [])
                link['epoch_match'] = 'unique' if len(epoch)==1 else 'ambiguous' if epoch else 'missing'
                if len(epoch)==1:
                    epoch = epoch[0]
                    epoch_ns = pose['receive_ns']-epoch['before_ns']
                    pair_ns = epoch['before_ns']-source['before_ns']
                    link.update(epoch_ordinal=epoch['ordinal'], epoch_to_pose_ns=epoch_ns,
                                raw_to_epoch_publish_ns=pair_ns)
                    if latency_ns >= 0 and epoch_ns >= 0 and pair_ns >= 0:
                        epoch_latencies.append(epoch_ns/1e6)
                        pair_waits.append(pair_ns/1e6)
                    else:
                        link['epoch_match'] = 'invalid_temporal_order'
        link['category'] = category
        categories[category] += 1
        links.append(link)
    all_processing = [v for values in branch_times.values() for v in values]
    reasons = Counter(d['data'].get('reason', 'unspecified') for d in decisions)
    last_status = max(statuses, key=lambda r:r['receive_ns'])['data'] if statuses else None
    summary = dict(
        scope='same-host publish-call-before to external observer callback entry; no FC',
        sent=dict(Counter(row['topic'] for row in returned)),
        attempted_count=len(inputs), publish_returned_count=len(returned),
        raw_input_count=sum(len(v) for v in raw.values()),
        raw_duplicate_occurrences=sum(max(0,len(v)-1) for v in raw.values()),
        epoch_duplicate_occurrences=sum(max(0,len(v)-1) for v in epochs.values()),
        decision_count=len(decisions), reasons=dict(reasons),
        pose_blocked_reasons=dict(Counter(d['data']['pose_blocked_reason'] for d in decisions
                                          if d['data'].get('pose_blocked_reason'))),
        clock_unsynced_details=dict(Counter(d['data'].get('clock',{}).get('reason','unspecified')
            for d in decisions if d['data'].get('reason')=='clock_unsynced')),
        decision_raw_mapping=dict(Counter(r['raw_match'] for r in decision_links)),
        raw_without_identified_decision=sum(len(v) for v in raw.values())-len(raw_decision_ordinals),
        published_decisions=sum(len(v) for v in published.values()),
        published_decisions_without_observed_pose=sum(1 for ds in published.values() for d in ds
                                                     if usages[d['index']]==0),
        observed_pose_count=len(poses), pose_mapping=dict(categories),
        raw_to_pose_ms=distribution(latencies), epoch_to_pose_ms=distribution(epoch_latencies),
        raw_to_epoch_publish_ms=distribution(pair_waits),
        matched_raw_publish_call_ms=distribution(call_times), processing_ms=distribution(all_processing),
        processing_by_branch_ms={key:distribution(values) for key,values in branch_times.items()},
        feeder_lateness_ms=distribution([(r['before_ns']-r['target_ns'])/1e6 for r in inputs]),
        source_age_at_raw_publish_ms=distribution([
            (r['before_ros_ns']-r['source_host_received_ros_ns']-r['ros_shift_ns'])/1e6
            for r in returned if r.get('type')=='uwb_raw_cycle' and 'before_ros_ns' in r]),
        source_age_at_uwb_publish_ms_by_type={kind:distribution([
            (r['before_ros_ns']-r['source_host_received_ros_ns']-r['ros_shift_ns'])/1e6
            for r in returned if r.get('type')==kind and 'before_ros_ns' in r])
            for kind in ('uwb_raw_cycle','uwb_tdma_epoch','uwb_raw_status')},
        publish_call_ms=distribution([(r['after_ns']-r['before_ns'])/1e6 for r in returned]),
        gaps_during_feed=gap_summary([p['receive_ns'] for p in poses],start_ns,feed_end_ns),
        gaps_with_drain=gap_summary([p['receive_ns'] for p in poses],start_ns,observation_end_ns),
        malformed_observed_records=malformed, last_status=last_status,
        limitations=[
            'Latency is conditional on a uniquely linked observed output; report missing outputs alongside it.',
            'No callback observation does not locate DDS loss, scheduling delay, or a shutdown backlog.',
            'Source age and processing_ms have different endpoints from external observed latency.',
            'Clock reason events are not exact clock-state transition timestamps.',
            'Collector receipt schedule is not the historical B_TF callback schedule.'])
    return summary, decision_links+links
