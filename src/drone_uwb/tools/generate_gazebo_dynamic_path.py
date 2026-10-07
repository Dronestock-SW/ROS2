"""Generate a declared algorithm-only XY path for repeated UWB comparison."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


def generate(spec):
    if (spec.get('schema') != 1 or spec.get('source') != 'synthetic_pose_fixture'
            or spec.get('clock_domain') != 'gazebo_sim_us'):
        raise ValueError('invalid_path_contract')
    rate, duration = spec['rate_hz'], spec['duration_s']
    if (type(rate) not in (int, float) or type(duration) not in (int, float)
            or not math.isfinite(rate) or not math.isfinite(duration)
            or rate <= 0 or duration <= 0 or rate*duration != int(rate*duration)):
        raise ValueError('invalid_path_timing')
    start = spec['start_xyz_m']
    if len(start) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in start):
        raise ValueError('invalid_path_origin')
    start_us = spec['start_time_us']
    if type(start_us) is not int or start_us < 0 or not spec.get('model'):
        raise ValueError('invalid_path_start')
    segments, previous_end = [], 0.
    for segment in spec['velocity_segments']:
        a, b = segment['start_s'], segment['end_s']
        vx, vy = segment['vx_m_s'], segment['vy_m_s']
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in (a, b, vx, vy))
                or not previous_end <= a < b <= duration or not segment.get('name')):
            raise ValueError('invalid_path_segment')
        segments.append((a, b, vx, vy))
        previous_end = b
    previous_time = None
    for k in range(int(rate*duration)):
        elapsed = k/rate
        stamp = start_us + round(elapsed*1e6)
        if previous_time is not None and stamp <= previous_time:
            raise ValueError('path_stamp_not_increasing')
        previous_time = stamp
        x, y, z = start
        for a, b, vx, vy in segments:
            active_s = max(0., min(elapsed-a, b-a))
            x += vx*active_s
            y += vy*active_s
        yield dict(source='synthetic_pose_fixture', clock_domain='gazebo_sim_us',
                   model=spec['model'], time_us=stamp,
                   position_xyz_m=[x, y, z], quaternion_wxyz=[1., 0., 0., 0.])


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    opts = parser.parse_args(args)
    spec_bytes = opts.spec.read_bytes()
    spec = json.loads(spec_bytes)
    rows = list(generate(spec))
    opts.output.mkdir(parents=True, exist_ok=False)
    (opts.output/'path_spec.json').write_bytes(spec_bytes)
    target = opts.output/'poses.jsonl'
    target.write_text(''.join(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n'
                              for row in rows), encoding='utf-8')
    manifest = dict(scope='synthetic_algorithm_path', output_count=len(rows),
                    path_spec_sha256=hashlib.sha256(spec_bytes).hexdigest(),
                    poses_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                    flight_valid=False, external_output_allowed=False)
    (opts.output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n',
                                             encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False))


if __name__ == '__main__':
    main()
