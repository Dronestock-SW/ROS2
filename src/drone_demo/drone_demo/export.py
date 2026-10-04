"""Export demo fixtures without starting ROS or accessing any hardware."""
import argparse
from collections import Counter
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

from .core import DemoRun, load_inputs


def write_demo(output, config, layout, uwb_settings):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    run = DemoRun(config, layout, uwb_settings)
    counts, errors, accepted_stamps = Counter(), [], []
    with (output / 'samples.jsonl').open('x', encoding='utf-8') as samples, \
            (output / 'poses.csv').open('x', encoding='utf-8', newline='') as poses:
        writer = csv.writer(poses)
        writer.writerow(['demo', 'time_s', 'truth_x_m', 'truth_y_m',
                         'target_x_m', 'target_y_m',
                         'uwb_x_m', 'uwb_y_m', 'decision'])
        for index in range(config.sample_count):
            sample = run.sample(index)
            samples.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + '\n')
            obs = sample['observation']
            writer.writerow([True, sample['time_s'], *sample['truth_xy_m'],
                             *sample['target_xy_m'], obs['x'] if obs else '',
                             obs['y'] if obs else '', sample['decision']])
            counts[sample['decision']] += 1
            if obs:
                accepted_stamps.append(obs['stamp_ns'])
                errors.append(math.dist([obs['x'], obs['y']],
                                        sample['observation_reference_xy_m']))
    gaps = [(b-a)/1e9 for a, b in zip(accepted_stamps, accepted_stamps[1:])]
    summary = {
        'demo': True, 'schema': 2, 'config': asdict(config), 'layout': layout,
        'observation_source': 'demo_xy',
        'display_variance_m2': uwb_settings.xy_stddev_m ** 2,
        'sample_count': config.sample_count,
        'counts': dict(counts),
        'synthetic_xy_rmse_m': math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
        'max_interval_between_accepted_s': max(gaps) if gaps else None,
        'last_truth_xy_m': sample['truth_xy_m'],
        'z_source': 'unobserved', 'z_m': None,
        'last_trajectory_phase': sample['trajectory_phase'],
        'timestamp_source': 'synthetic, fixed epoch; not a capture time',
        'limits': 'XY fixtures only; no RAW ranges, altitude, sensor capture, or flight validation.',
    }
    (output / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return summary


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='new directory; never overwritten')
    parser.add_argument('--config', default='')
    parser.add_argument('--scenario', choices=('stationary', 'move', 'gap'))
    parser.add_argument('--duration-s', type=float)
    args = parser.parse_args()
    try:
        inputs = load_inputs(args.config, scenario=args.scenario, duration_s=args.duration_s)
        summary = write_demo(args.output, *inputs)
    except (ValueError, TypeError, OSError) as exc:
        parser.exit(2, str(exc) + '\n')
    print(json.dumps({'output': str(args.output), 'demo': True,
                      'counts': summary['counts'],
                      'last_truth_xy_m': summary['last_truth_xy_m'],
                      'max_interval_between_accepted_s': summary['max_interval_between_accepted_s']},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
