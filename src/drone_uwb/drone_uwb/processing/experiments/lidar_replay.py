"""Hash-checked, file-only replay of recorded horizontal LaserScan and gyro.

The original analyze.py is NOT included in Git; this module is a new comparator,
not a claim to reproduce its -267 degree result. No observation is published.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from .lidar_scanmatch import Config, KeyframeMatcher, compose, match, scan_cloud


def file_sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, np.ndarray)):
        return [json_safe(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


class GyroSeries:
    """Trapezoidal scan-Z gyro integration with explicit gap epochs.

    This is a planar angular consistency diagnostic, without bias calibration.
    PX4 orientation/position estimates are never used as independent LiDAR data.
    """
    def __init__(self, times, rates, imu_to_scan, max_gap_s=.03):
        self.times = np.asarray(times, dtype=float)
        rates = np.asarray(rates, dtype=float)
        transform = np.asarray(imu_to_scan, dtype=float)
        if transform.shape != (3, 3) or not np.isfinite(transform).all():
            raise ValueError('IMU-to-scan rotation must be a finite 3 by 3 matrix')
        if not np.allclose(transform @ transform.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(transform), 1., atol=1e-6):
            raise ValueError('IMU-to-scan rotation must be a proper rotation')
        if len(self.times) < 2 or rates.shape != (len(self.times), 3):
            raise ValueError('insufficient gyro samples')
        if not np.isfinite(self.times).all() or not np.all(np.diff(self.times) > 0.):
            raise ValueError('gyro timestamps must increase; do not silently sort clock jumps')
        self.max_gap_s = max_gap_s
        z = rates @ transform[2]
        self.z = z
        bad = ~np.isfinite(z)
        gaps = (np.diff(self.times) > max_gap_s) | bad[1:] | bad[:-1]
        increments = .5 * (z[1:] + z[:-1]) * np.diff(self.times)
        increments[gaps] = 0.
        self.integral = np.r_[0., np.cumsum(increments)]
        self.epochs = np.r_[0, np.cumsum(gaps)]
        self.valid = ~bad

    def at(self, stamp):
        if not np.isfinite(stamp):
            return None
        if stamp == self.times[-1] and self.valid[-1]:
            return int(self.epochs[-1]), float(self.integral[-1])
        right = int(np.searchsorted(self.times, stamp, side='right'))
        if right == 0 or right == len(self.times):
            return None
        left = right - 1
        if self.epochs[left] != self.epochs[right] or not (self.valid[left] and self.valid[right]):
            return None
        dt = stamp - self.times[left]
        slope = (self.z[right] - self.z[left]) / (self.times[right] - self.times[left])
        value = self.integral[left] + self.z[left] * dt + .5 * slope * dt * dt
        return int(self.epochs[left]), float(value)


def replay(events, expected_sha256, output, imu_rotation=None):
    actual = file_sha256(events)
    if actual.lower() != expected_sha256.lower():
        raise ValueError(f'raw SHA256 mismatch: expected {expected_sha256}, got {actual}')
    output.mkdir(parents=True, exist_ok=False)
    scans, imus, counts = [], [], Counter()
    with events.open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            topic = row.get('topic')
            counts[topic] += 1
            if topic in ('/scan', '/mavros/imu/data'):
                stamp = row.get('header_ns')
                if not isinstance(stamp, int) or stamp <= 0:
                    raise ValueError(f'missing sensor timestamp at line {line_number}')
                (scans if topic == '/scan' else imus).append((stamp, row['data'], line_number))
    if not scans:
        raise ValueError('no /scan in this capture')
    origin_ns = scans[0][0]
    series = None
    if imu_rotation is not None:
        series = GyroSeries([(r[0] - origin_ns) / 1e9 for r in imus],
                            [[r[1]['angular_velocity'][axis] for axis in ('x', 'y', 'z')] for r in imus], imu_rotation)
    tracker = KeyframeMatcher(require_imu=series is not None)
    previous = first = None
    previous_stamp = None
    imu_epoch = None
    chained = np.zeros(3)
    chain_yaw = 0.
    accepted, reasons = 0, Counter()
    beam_ratios = []
    with (output / 'scans.jsonl').open('x', encoding='utf-8') as stream:
        for index, (stamp, scan, line) in enumerate(scans):
            stamp_s = (stamp - origin_ns) / 1e9
            cloud = scan_cloud(scan['ranges'], scan['angle_min'], scan['angle_increment'], scan['range_min'], scan['range_max'])
            imu = series.at(stamp_s) if series is not None else None
            if imu is not None and imu_epoch is not None and imu[0] != imu_epoch:
                # Do not integrate motion across an IMU gap and claim continuity.
                tracker.reference = tracker.previous = None
            if imu is not None:
                imu_epoch = imu[0]
            observation = tracker.observe(cloud, stamp_s, imu_yaw_rad=imu[1] if imu is not None else None)
            if first is None:
                first = cloud
            reference = match(first, cloud)
            pair = None
            if previous is not None and 0 < stamp_s - previous_stamp < Config().max_gap_s:
                pair = match(previous, cloud)
                # A NEW software comparator; not a recreation of analyze.py.
                if pair.accepted and np.linalg.norm(pair.pose[:2]) <= Config().max_pair_translation_m and abs(pair.pose[2]) <= Config().max_pair_yaw_rad:
                    chained = compose(chained, pair.pose)
                    chain_yaw += pair.pose[2]
            accepted += int(observation.accepted)
            reasons.update(observation.reasons)
            scan_time = float(scan['scan_time'])
            ray_span = (len(scan['ranges']) - 1) * float(scan['time_increment'])
            ratio = ray_span / scan_time if scan_time > 0 else None
            if ratio is not None:
                beam_ratios.append(ratio)
            stream.write(json.dumps(json_safe(dict(index=index, source_line=line, stamp_ns=stamp,
                          fixed_reference=asdict(reference), sequential_pair=asdict(pair) if pair else None,
                          sequential_accepted_pose=chained, sequential_unwrapped_yaw_rad=chain_yaw,
                          candidate=asdict(observation), beam_span_to_scan_time=ratio,
                          imu_epoch=imu[0] if imu else None, deskew_applied=False)), allow_nan=False) + '\n')
            previous, previous_stamp = cloud, stamp_s
    summary = dict(schema=1, raw_path=str(events.resolve()), raw_sha256=actual, topics=dict(counts),
                   scans=len(scans), accepted_displacements=accepted, reasons=dict(reasons),
                   new_comparator_yaw_deg=float(np.rad2deg(chain_yaw)),
                   original_analyze_py_reproduced=False, original_minus267_reproduced=False,
                   imu_gate='planar scan-Z gyro diagnostic' if series else 'NOT_EVALUATED_NO_MOUNT_ROTATION',
                   imu_to_scan_rotation=imu_rotation, gyro_bias_calibrated=False,
                   header_epoch='unaltered first-ray stamp; optical beam timing not validated',
                   median_ray_span_to_scan_time=float(np.median(beam_ratios)) if beam_ratios else None,
                   config=asdict(Config()), deskew_applied=False, fc_output=False,
                   scope='file replay only; no independent position truth or flight validation')
    (output / 'summary.json').write_text(json.dumps(json_safe(summary), ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--events', type=Path, required=True)
    p.add_argument('--expected-sha256', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--imu-to-scan-rotation', type=Path,
                   help='JSON proper 3x3 rotation; explicitly supplied mount assumption, not inferred')
    args = p.parse_args()
    transform = json.loads(args.imu_to_scan_rotation.read_text(encoding='utf-8')) if args.imu_to_scan_rotation else None
    print(json.dumps(replay(args.events, args.expected_sha256, args.output, transform), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
