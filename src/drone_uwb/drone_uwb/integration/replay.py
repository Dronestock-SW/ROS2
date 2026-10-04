"""Replay immutable capture files through the same live observation checks."""
import argparse
from collections import Counter
import csv
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

from drone_uwb.processing.solvers.observations import Observation, Processor, Settings


def error_metrics(xy, reference):
    if not xy:
        return None
    values = np.array(xy)
    result = {'count': len(values), 'mean_xy_m': np.mean(values, axis=0).tolist(),
              'stddev_xy_m': np.std(values, axis=0).tolist()}
    if reference is not None:
        errors = np.linalg.norm(values - reference, axis=1)
        result.update(mean_error_m=float(np.mean(errors)), rmse_m=float(np.sqrt(np.mean(errors**2))),
                      p95_error_m=float(np.percentile(errors, 95)), max_error_m=float(max(errors)),
                      mean_error_vector_m=(np.mean(values, axis=0)-reference).tolist())
    return result


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path, help='received.jsonl, with host receive times')
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--settings', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', nargs=2, type=float)
    parser.add_argument('--source-mode', choices=('raw_ranges', 'tag_xy'))
    args = parser.parse_args()
    layout = json.loads(args.layout.read_text(encoding='utf-8'))
    configured = yaml.safe_load(args.settings.read_text(encoding='utf-8'))['uwb_node']['ros__parameters']
    settings = Settings(**{k: v for k, v in configured.items() if k in asdict(Settings())})
    if args.source_mode:
        settings.source_mode = args.source_mode
    processor = Processor(layout, settings)
    args.output.mkdir(parents=True, exist_ok=False)
    counts = Counter()
    accepted, candidates, accepted_times, cycle_times, tag_xy = [], [], [], [], []
    with args.capture.open(encoding='utf-8') as source, \
            (args.output/'decisions.jsonl').open('x', encoding='utf-8') as decisions, \
            (args.output/'accepted.csv').open('x', encoding='utf-8', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=list(Observation.__dataclass_fields__))
        writer.writeheader()
        for line in source:
            record = json.loads(line)
            msg, mono = record['message'], record['host_received_monotonic_ns']
            ros = record.get('host_received_ros_ns')
            if ros is None:
                ros = int(datetime.fromisoformat(record['host_received_utc']).timestamp()*1e9)
            decision = processor.process(msg, mono, ros)
            counts[decision.reason] += 1
            if msg.get('type') == 'uwb_raw_cycle':
                cycle_times.append(mono)
                if msg.get('raw_xy_valid') is True:
                    tag_xy.append([msg['raw_x_m'], msg['raw_y_m']])
            if 'candidate_xy_m' in decision.details:
                candidates.append(decision.details['candidate_xy_m'])
            if decision.observation is not None:
                obs = decision.observation
                accepted.append([obs.x, obs.y])
                accepted_times.append(obs.stamp_ns)
                writer.writerow(asdict(obs))
            decisions.write(json.dumps({'seq': msg.get('seq'), 'reason': decision.reason,
                                        'details': decision.details,
                                        'observation': asdict(decision.observation) if decision.observation else None},
                                       ensure_ascii=False, allow_nan=False)+'\n')
    duration = (cycle_times[-1]-cycle_times[0])/1e9 if len(cycle_times)>1 else 0
    gaps = np.diff(accepted_times)/1e9
    summary = {
        'source': str(args.capture), 'source_sha256': hashlib.sha256(args.capture.read_bytes()).hexdigest(),
        'layout': layout, 'settings': asdict(settings), 'reference_xy_m': args.reference,
        'counts': dict(counts), 'cycle_count': len(cycle_times), 'duration_between_cycles_s': duration,
        'raw_cycle_rate_hz': (len(cycle_times)-1)/duration if duration else None,
        'accepted_per_capture_second': len(accepted)/duration if duration else None,
        'accepted_fraction': len(accepted)/len(cycle_times) if cycle_times else None,
        'max_accepted_observation_gap_s': float(max(gaps)) if len(gaps) else None,
        'tag_xy_metrics': error_metrics(tag_xy, args.reference),
        'candidates_after_protocol_checks': error_metrics(candidates, args.reference),
        'accepted_xy_metrics': error_metrics(accepted, args.reference),
        'note': 'Observation rejection only; no bias calibration, prediction, or new ranging samples.',
    }
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in summary.items() if k not in ('layout','settings')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
