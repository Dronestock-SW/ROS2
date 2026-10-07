"""Split PX4 sensor snapshots by physical IMU without aligning or filtering them."""
import argparse
from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np


def parse_query(record):
    command = record['command']
    if command not in ('listener sensor_accel -n 1', 'listener sensor_gyro -n 1'):
        return []
    sensor = 'accel' if 'sensor_accel' in command else 'gyro'
    text = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', record['text'])
    blocks = re.split(r'(?m)^Instance (\d+):\s*$', text)
    rows = []
    for i in range(1, len(blocks), 2):
        instance, block = int(blocks[i]), blocks[i+1]
        fields = dict(re.findall(r'(?m)^\s{4}([a-z_]+):\s*(.*?)\s*$', block))
        required = ('timestamp', 'timestamp_sample', 'device_id', 'x', 'y', 'z',
                    'temperature', 'error_count', 'clip_counter', 'samples')
        if any(name not in fields for name in required):
            raise ValueError(f'incomplete sensor fields in query {record["query"]}')
        row = {'query': record['query'], 'imu_instance': instance, 'sensor': sensor,
               'host_send_monotonic_ns': record['host_send_monotonic_ns'],
               'host_end_monotonic_ns': record['host_end_monotonic_ns']}
        for key in ('timestamp', 'timestamp_sample', 'device_id', 'error_count', 'samples'):
            row[key] = int(fields[key].split()[0])
        # Preserve PX4 console decimal text, including unavailable temperature "nan".
        for key in ('x', 'y', 'z', 'temperature'):
            row[key] = fields[key]
        row['clip_x'], row['clip_y'], row['clip_z'] = json.loads(fields['clip_counter'])
        row['unit'] = 'm/s^2' if sensor == 'accel' else 'rad/s'
        rows.append(row)
    if len(rows) != 2 or {row['imu_instance'] for row in rows} != {0, 1}:
        raise ValueError(f'expected exactly two IMU instances in query {record["query"]}')
    return rows


def summarize(rows):
    sample = np.array([r['timestamp_sample'] for r in rows], dtype=np.int64)
    gaps = np.diff(sample)/1e6
    xyz = np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
    if len(set(r['device_id'] for r in rows)) != 1:
        raise ValueError('sensor ID changed during capture')
    return {'count': len(rows), 'device_id': rows[0]['device_id'],
            'first_sample_us': int(sample[0]), 'last_sample_us': int(sample[-1]),
            'sample_span_s': float((sample[-1]-sample[0])/1e6),
            'snapshot_rate_hz': float((len(rows)-1)/((sample[-1]-sample[0])/1e6)),
            'max_snapshot_gap_ms': float(max(gaps)*1000),
            'duplicate_sample_times': int(sum(gaps == 0)),
            'backward_sample_times': int(sum(gaps < 0)),
            'mean_xyz': xyz.mean(axis=0).tolist(), 'stddev_xyz': xyz.std(axis=0).tolist(),
            'min_xyz': xyz.min(axis=0).tolist(), 'max_xyz': xyz.max(axis=0).tolist(),
            'error_count_first': rows[0]['error_count'], 'error_count_last': rows[-1]['error_count'],
            'observed_snapshots_with_clipping': sum(any(r[k] for k in ('clip_x','clip_y','clip_z')) for r in rows)}


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory
    queries = [json.loads(s) for s in (directory/'queries.jsonl').read_text(encoding='utf-8').splitlines()]
    captured = [q for q in queries if q['phase'] == 'capture']
    grouped = defaultdict(list)
    per_imu = defaultdict(list)
    for query in captured:
        if not query['prompt_received']:
            raise ValueError('incomplete shell response')
        for row in parse_query(query):
            grouped[(row['imu_instance'],row['sensor'])].append(row)
            per_imu[row['imu_instance']].append(row)
    if set(grouped) != {(0,'accel'),(0,'gyro'),(1,'accel'),(1,'gyro')}:
        raise ValueError('missing an IMU channel')
    summary = {'capture_window': json.loads((directory/'capture_window.json').read_text(encoding='utf-8')),
               'method': 'PX4 uORB sensor_accel / sensor_gyro console snapshots',
               'frame': 'FRD board: x forward, y right, z down',
               'unaltered_console_saved': 'console.raw',
               'native_rate_samples_all_captured': False,
               'software_filter_or_bias_correction_applied': False,
               'stationary_condition_confirmed_by_user': False,
               'queries': len(captured), 'sensors': {}}
    for (instance,sensor),rows in sorted(grouped.items()):
        summary['sensors'][f'imu{instance}_{sensor}'] = summarize(rows)
        with (directory/f'imu{instance}_{sensor}.csv').open('x',encoding='utf-8',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
    for instance,rows in sorted(per_imu.items()):
        rows.sort(key=lambda r:r['timestamp_sample'])
        with (directory/f'imu{instance}_raw.csv').open('x',encoding='utf-8',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
    (directory/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(12,8),sharex=True,layout='constrained')
    origin=min(r['timestamp_sample'] for rows in grouped.values() for r in rows)
    for col,sensor in enumerate(('accel','gyro')):
        for instance in (0,1):
            rows=grouped[(instance,sensor)]
            t=(np.array([r['timestamp_sample'] for r in rows])-origin)/1e6
            for axis,key in enumerate(('x','y','z')):
                axes[axis,col].plot(t,[float(r[key]) for r in rows],label=['BMI088 (instance 0)','ICM-42688-P (instance 1)'][instance],linewidth=.8)
                axes[axis,col].set_ylabel(f'{key}: '+('m/s²' if sensor=='accel' else 'rad/s'))
                axes[axis,col].grid(alpha=.25)
        axes[0,col].set_title('Acceleration' if sensor=='accel' else 'Angular velocity')
        axes[2,col].set_xlabel('PX4 sample time relative to first snapshot (s)')
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Two IMUs: 60-second raw topic snapshots (~5 Hz), board FRD axes')
    fig.savefig(directory/'imu_comparison.png',dpi=160)
    plt.close(fig)
    hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(directory.iterdir()) if f.is_file() and f.name!='sha256.json'}
    (directory/'sha256.json').write_text(json.dumps(hashes,indent=2)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
