"""Offline receipt/header gap audit. Does not estimate or apply calibration."""
import argparse
import heapq
import json
from pathlib import Path


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def audit(lines, *, top=3):
    if not 1 <= top <= 20:
        raise ValueError('top_must_be_1_to_20')
    topics, previous, largest = {}, {}, {}
    offset_min = offset_max = None
    count = 0
    for count, line in enumerate(lines, 1):
        try:
            row = json.loads(line)
            topic = row['topic']
            mono, ros, header = row['received_monotonic_ns'], row['received_ros_ns'], row.get('header_ns')
            if not isinstance(topic, str) or not integer(mono) or not integer(ros):
                raise ValueError('invalid_topic_or_receipt_clock')
            if header is not None and not integer(header):
                raise ValueError('invalid_header_clock')
        except (ValueError, KeyError, TypeError) as exc:
            raise ValueError(f'invalid_capture_row:{count}') from exc
        offset = ros-mono
        offset_min = offset if offset_min is None else min(offset_min, offset)
        offset_max = offset if offset_max is None else max(offset_max, offset)
        s = topics.setdefault(topic, dict(count=0, max_receipt_gap_ns=0,
            receipt_clock_regressions=0, header_clock_regressions=0,
            future_header_count=0, missing_header_count=0,
            min_header_age_ns=None, max_header_age_ns=None))
        s['count'] += 1
        age = ros-header if header is not None and header > 0 else None
        if age is None:
            s['missing_header_count'] += 1
        else:
            s['future_header_count'] += int(age < 0)
            s['min_header_age_ns'] = age if s['min_header_age_ns'] is None else min(age,s['min_header_age_ns'])
            s['max_header_age_ns'] = age if s['max_header_age_ns'] is None else max(age,s['max_header_age_ns'])
        if topic in previous:
            pmono, pros, pheader = previous[topic]
            gap = mono-pmono
            progress = header-pheader if age is not None and pheader is not None and pheader>0 else None
            s['receipt_clock_regressions'] += int(gap < 0)
            s['header_clock_regressions'] += int(progress is not None and progress < 0)
            s['max_receipt_gap_ns'] = max(s['max_receipt_gap_ns'], gap)
            event = dict(before_monotonic_ns=pmono, after_monotonic_ns=mono,
                receipt_gap_ns=gap, header_progress_ns=progress,
                header_age_after_ns=age, receipt_clock_offset_change_ns=(ros-mono)-(pros-pmono))
            heap = largest.setdefault(topic, [])
            heapq.heappush(heap, (gap,count,event))
            if len(heap)>top:
                heapq.heappop(heap)
        previous[topic] = mono, ros, header
    for topic, s in topics.items():
        s['largest_gaps'] = [entry[2] for entry in sorted(largest.get(topic,[]),reverse=True)]
    return dict(schema=1, scope='offline_capture_transport_audit', rows=count, topics=topics,
        receipt_clock_offset_span_ns=offset_max-offset_min if count else None,
        alignment_confirmed=False, timing_confirmed=False, flight_authorized=False,
        limitation='Receipt gaps do not alone distinguish producer loss, DDS queuing, or recorder stalls. '
                   'Header age compares source headers to host ROS receipt time; it is not calibrated latency.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path,help='One capture directory from one host boot')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--top',type=int,default=3)
    args = parser.parse_args(argv)
    source = args.capture/'events.jsonl'
    if args.output.resolve() in {p.resolve() for p in args.capture.iterdir()}:
        parser.error('output_must_not_overwrite_capture_evidence')
    manifest = json.loads((args.capture/'manifest.json').read_text(encoding='utf-8'))
    with source.open(encoding='utf-8') as stream:
        result = audit(stream,top=args.top)
    result.update(boot_id=manifest.get('boot_id'),source_revision=manifest.get('source_revision'))
    with args.output.open('x',encoding='utf-8') as stream:
        stream.write(json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
