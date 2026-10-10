import collections
import datetime
import glob
import hashlib
import json
import math
import pathlib
import statistics
import sys

start = int(sys.argv[1])
end = start + 60_000_000_000
reference = None if sys.argv[2] == 'unknown' else tuple(map(float, sys.argv[2:4]))
root = pathlib.Path('/home/arialhanho/.local/share/dronestock/manual-captures')
streams = collections.defaultdict(list)
cycles, tof, imu, poses, states, decisions = [], [], [], [], [], []
files, unreadable = [], []
selected_hash = hashlib.sha256()
selected_rows = selected_bytes = malformed = 0
for directory in sorted(root.glob('capture-*')):
    created = int(directory.name.split('-')[1])
    if created < start - 240_000_000_000 or created >= end:
        continue
    included = 0
    try:
        with (directory / 'events.jsonl').open('rb') as stream:
            for line in stream:
                try:
                    event = json.loads(line)
                except ValueError:
                    malformed += 1
                    continue
                stamp = event['received_ros_ns']
                if not start <= stamp < end:
                    continue
                selected_hash.update(line)
                selected_rows += 1
                selected_bytes += len(line)
                included += 1
                topic, data = event['topic'], event['data']
                streams[topic].append(stamp)
                if topic == '/uwb/received':
                    message = json.loads(data['data'])['message']
                    if message.get('type') == 'uwb_raw_cycle':
                        cycles.append((stamp, message))
                elif topic == '/mavros/downward_0':
                    tof.append((stamp, data))
                elif topic == '/mavros/imu/data':
                    q = data['orientation']
                    x, y, z, w = (q[k] for k in ('x', 'y', 'z', 'w'))
                    roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
                    pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
                    imu.append((roll, pitch))
                elif topic == '/uwb/btf_pose':
                    p = data['pose']['pose']['position']
                    poses.append((stamp, p['x'], p['y']))
                elif topic == '/mavros/state':
                    states.append(data)
                elif topic == '/uwb/btf_decision':
                    decisions.append(json.loads(data['data']))
        if included:
            files.append({'path': str(directory / 'events.jsonl'), 'selected_rows': included})
    except PermissionError:
        unreadable.append(str(directory))

def distribution(values):
    return {'n': len(values), 'mean': statistics.mean(values),
            'std': statistics.pstdev(values), 'median': statistics.median(values),
            'min': min(values), 'max': max(values)} if values else None

def xy_summary(xy):
    if not xy:
        return {'n': 0}
    xs, ys = zip(*xy)
    mean = (statistics.mean(xs), statistics.mean(ys))
    if reference is None:
        return {'n': len(xy), 'mean_xy_m': mean,
                'std_xy_m': (statistics.pstdev(xs), statistics.pstdev(ys)),
                'min_xy_m': (min(xs), min(ys)), 'max_xy_m': (max(xs), max(ys)),
                'accuracy_assessed': False}
    errors = sorted(math.hypot(x-reference[0], y-reference[1]) for x, y in xy)
    return {'n': len(xy), 'mean_xy_m': mean,
            'mean_position_error_m': math.dist(mean, reference),
            'rms_error_m': math.sqrt(statistics.mean(e*e for e in errors)),
            'p95_error_m': errors[math.ceil(.95*len(errors))-1],
            'std_xy_m': (statistics.pstdev(xs), statistics.pstdev(ys)),
            'min_xy_m': (min(xs), min(ys)), 'max_xy_m': (max(xs), max(ys))}

kst = datetime.timezone(datetime.timedelta(hours=9))
result = {
    'schema': 1,
    'window_kst': [datetime.datetime.fromtimestamp(t/1e9, kst).isoformat() for t in (start, end)],
    'start_ns': str(start), 'end_ns': str(end), 'duration_s': 60,
    'reference_xy_m': reference,
    'reference_note': '사용자에게 안내한 기준점에 배치했다는 전제. 독립 측량 오차 미평가.' if reference else '중앙 부근, 정확한 XY 미확인. 절대 위치 오차 평가 제외.',
    'evidence': {'files': files, 'selected_rows': selected_rows, 'selected_bytes': selected_bytes,
                 'selected_original_sha256': selected_hash.hexdigest(),
                 'unreadable': unreadable, 'malformed_file_lines': malformed},
    'streams': {},
}
for topic in ('/uwb/received', '/mavros/downward_0', '/mavros/imu/data', '/uwb/btf_pose', '/mavros/state'):
    stamps = sorted(streams[topic])
    gaps = [(b-a)/1e9 for a, b in zip(stamps, stamps[1:])]
    result['streams'][topic] = {'n': len(stamps), 'rate_hz': len(stamps)/60,
        'max_internal_gap_s': max(gaps, default=None),
        'first_last_offsets_s': [(stamps[0]-start)/1e9, (stamps[-1]-start)/1e9] if stamps else None}
complete = [m for _, m in cycles if m.get('valid_mask') == 15]
raw_xy = [(m['raw_x_m'], m['raw_y_m']) for _, m in cycles if m.get('raw_xy_valid')]
result['uwb'] = {'cycles': len(cycles), 'cycle_rate_hz': len(cycles)/60,
    'all_four_valid': len(complete), 'all_four_valid_percent': 100*len(complete)/len(cycles) if cycles else None,
    'mask_counts': dict(collections.Counter(m.get('valid_mask') for _, m in cycles)),
    'raw_xy': xy_summary(raw_xy), 'btf_xy': xy_summary([(x,y) for _,x,y in poses]),
    'btf_decision_count': len(decisions),
    'btf_decision_reasons': dict(collections.Counter(d.get('reason') for d in decisions))}
result['anchors'] = []
for i, anchor in enumerate(((0,0),(6.3,0),(0,4.6),(6.3,4.6))):
    v = [m['raw_slant_m'][i] for m in complete]
    minimum = math.dist(reference, anchor) if reference else None
    result['anchors'].append({'id': 'A'+str(i+1), 'complete_cycle_distance_m': distribution(v),
        'minimum_slant_from_xy_m': minimum,
        'mean_minus_minimum_m': statistics.mean(v)-minimum if v and minimum is not None else None})
result['tof'] = {'distance_m': distribution([d['range'] for _,d in tof]),
    'within_reported_range': sum(d['min_range']<=d['range']<=d['max_range'] for _,d in tof),
    'limits_m': [tof[-1][1]['min_range'],tof[-1][1]['max_range']] if tof else None}
result['imu'] = {'roll_deg': distribution([r for r,p in imu]), 'pitch_deg': distribution([p for r,p in imu])}
result['fc_states'] = sorted(set((d['connected'],d['armed'],d['mode']) for d in states))
result['calibration_changed'] = False
print(json.dumps(result, ensure_ascii=True))
