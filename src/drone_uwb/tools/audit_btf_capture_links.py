"""Read-only audit of recorded RAW -> decision -> pose identity, not replay."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path


def audit(path):
    raw, decisions, poses = [], [], []
    kinds, topics = Counter(), Counter()
    received_lags = defaultdict(list)
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for ordinal, line in enumerate(source, 1):
            digest.update(line)
            row = json.loads(line)
            topic = row.get('topic')
            topics[topic] += 1
            if topic == '/uwb/received':
                event = json.loads(row['data']['data'])
                message = event['message']
                kinds[message['type']] += 1
                received_lags[message['type']].append(
                    (row['received_ros_ns']-event['host_received_ros_ns'])/1e6)
                if message['type'] == 'uwb_raw_cycle':
                    raw.append(dict(ordinal=ordinal, seq=message['seq'],
                        cycle_end_us=message['cycle_end_us'],
                        collector_lag_ns=row['received_ros_ns']-event['host_received_ros_ns']))
            elif topic == '/uwb/btf_decision':
                decisions.append(json.loads(row['data']['data']))
            elif topic == '/uwb/btf_pose':
                header = row['data']['header']
                position = row['data']['pose']['pose']['position']
                poses.append(dict(stamp_ns=header['stamp']['sec']*10**9+header['stamp']['nanosec'],
                                  xy_m=[position['x'], position['y']]))
    raw_by_key, decision_by_stamp = defaultdict(list), defaultdict(list)
    for row in raw:
        raw_by_key[(row['seq'], row['cycle_end_us'])].append(row)
    mapping = Counter()
    for decision in decisions:
        key = (decision.get('seq'), decision.get('source_cycle_end_us'))
        matches = raw_by_key.get(key, [])
        mapping['unique_raw' if len(matches) == 1 else 'no_raw' if not matches else 'ambiguous_raw'] += 1
        if decision.get('published'):
            decision_by_stamp[decision['stamp_ns']].append(decision)
    pose_mapping = Counter()
    observed_stamps = Counter()
    for pose in poses:
        matches = decision_by_stamp.get(pose['stamp_ns'], [])
        observed_stamps[pose['stamp_ns']] += 1
        if len(matches) == 1:
            same = all(abs(a-b) < 1e-12 for a, b in zip(pose['xy_m'], matches[0]['xy_m']))
            pose_mapping['unique_stamp_and_xy' if same else 'xy_mismatch'] += 1
        else:
            pose_mapping['no_decision' if not matches else 'ambiguous_decision'] += 1
    lag = sorted(row['collector_lag_ns']/1e6 for row in raw)
    def percentile(values, q):
        if not values:
            return None
        index = (len(values)-1)*q
        lo, hi = int(index), min(int(index)+1, len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(index-lo)
    return dict(input_sha256=digest.hexdigest(), scope='recorded identity audit, no replay',
        topics=dict(topics), received_message_types=dict(kinds), raw_count=len(raw),
        raw_duplicate_keys=sum(len(v)-1 for v in raw_by_key.values()),
        decision_count=len(decisions), decision_to_raw=dict(mapping),
        decisions_without_raw_by_reason=dict(Counter(d.get('reason') for d in decisions
            if len(raw_by_key.get((d.get('seq'), d.get('source_cycle_end_us')), [])) != 1)),
        published_decisions=sum(bool(d.get('published')) for d in decisions),
        duplicate_published_stamp_keys=sum(len(v)-1 for v in decision_by_stamp.values()),
        pose_count=len(poses), pose_to_decision=dict(pose_mapping),
        duplicate_pose_stamp_keys=sum(v-1 for v in observed_stamps.values()),
        published_decisions_without_observed_pose=sum(len(v) for stamp,v in decision_by_stamp.items()
            if stamp not in observed_stamps),
        raw_collector_lag_ms=dict(p50=percentile(lag,.5),p95=percentile(lag,.95),
            p99=percentile(lag,.99),max=max(lag) if lag else None,
            over_150ms=sum(v>150 for v in lag)),
        received_collector_lag_by_type_ms={kind: dict(
            count=len(values), p50=percentile(sorted(values),.5),
            p95=percentile(sorted(values),.95), p99=percentile(sorted(values),.99),
            max=max(values), over_150ms=sum(v>150 for v in values),
            negative=sum(v<0 for v in values))
            for kind,values in received_lags.items()},
        limitations=['Collector receipt is not publisher or B_TF callback entry.',
            'Counts do not locate a loss in DDS; retained old messages or observer loss remain possible.',
            'Use run-local identity maps; uniqueness in this capture is not a protocol guarantee.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = json.dumps(audit(args.events), ensure_ascii=False, indent=2)+'\n'
    if args.output:
        args.output.write_text(result, encoding='utf-8')
    print(result)
