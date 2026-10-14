"""Separate historical input age from added replay scheduling and recording cost."""
import argparse
from collections import Counter
import json
from pathlib import Path

from btf_replay_probe import distribution


def analyze(capture_file, folders):
    receipt_by_input = {}
    source_ages = []
    for line in capture_file.open(encoding='utf-8'):
        row = json.loads(line)
        if row.get('topic') == '/uwb/received':
            event = json.loads(row['data']['data'])
            stamp = event['host_received_ros_ns']
            receipt_by_input[stamp] = row['received_ros_ns']
            source_ages.append((row['received_ros_ns']-stamp)/1e6)
    output = dict(input_collector_age_ms=distribution(source_ages),
        input_already_over_150ms=sum(v>150 for v in source_ages), runs={})
    for folder in folders:
        summary = json.loads((folder/'summary.json').read_text())
        shift = summary['ros_stamp_shift_ns']
        rows = [json.loads(line) for line in (folder/'decisions.jsonl').read_text().splitlines()]
        waits, introduced_expiry, historical_expiry = [], 0, 0
        native, focus = [], []
        for row in rows:
            original = row.get('input_received_ros_ns', 0)-shift
            if original in receipt_by_input:
                callback = row.get('callback_ros_ns', row.get('decision_stamp_ns'))
                waits.append((callback-shift-receipt_by_input[original])/1e6)
                if row.get('reason') == 'receiver_queue_expired':
                    if receipt_by_input[original]-original > 150_000_000:
                        historical_expiry += 1
                    else:
                        introduced_expiry += 1
            kernel = row.get('models',{}).get('B_TF',{}).get('native_call_ms')
            if kernel is not None:
                native.append(kernel)
            if 17475 <= row.get('seq', -1) <= 17520:
                focus.append(row)
        valid = [r for r in focus if r.get('published')]
        before_gap = next((r for r in reversed(rows) if r.get('published') and r.get('seq',10**9)<17484), None)
        after_gap = next((r for r in rows if r.get('published') and r.get('seq',-1)>17510), None)
        resources = json.loads(folder.with_name(folder.name+'-resources.json').read_text())
        samples = resources['samples']
        def counters(sample, key):
            return {line.split()[0]:int(line.split()[1]) for line in sample[key].splitlines()
                    if len(line.split()) == 2}
        first, last = counters(samples[0],'vmstat'), counters(samples[-1],'vmstat')
        cpu0 = [int(v) for v in samples[0]['stat'].splitlines()[0].split()[1:]]
        cpu1 = [int(v) for v in samples[-1]['stat'].splitlines()[0].split()[1:]]
        ticks = [b-a for a,b in zip(cpu0,cpu1)]
        total = sum(ticks[:8])
        output['runs'][str(folder)] = dict(
            callback_after_collector_ms=distribution(waits),
            native_adapter_ms=distribution(native),
            inherited_queue_expiry=historical_expiry,
            additional_queue_expiry=introduced_expiry,
            focus_reasons=dict(Counter(r['reason'] for r in focus)),
            focus_published=[r['seq'] for r in valid],
            focus_gap_boundaries_observed=before_gap is not None and after_gap is not None,
            focus_bracketing_gap_ms=((after_gap['stamp_ns']-before_gap['stamp_ns'])/1e6
                                    if before_gap is not None and after_gap is not None else None),
            focus_source_gap_ms=distribution([(b['stamp_ns']-a['stamp_ns'])/1e6
                                              for a,b in zip(valid,valid[1:])]),
            swap_in_pages=last['pswpin']-first['pswpin'],
            swap_out_pages=last['pswpout']-first['pswpout'],
            host_iowait_percent=100*ticks[4]/total if total else None,
            host_idle_percent=100*ticks[3]/total if total else None)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_file', type=Path)
    parser.add_argument('folders', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.capture_file, args.folders)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
