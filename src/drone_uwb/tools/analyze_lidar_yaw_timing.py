"""File-only yaw timing diagnostic. Never enables fusion or writes FC settings."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.spatial.transform import Rotation

from drone_uwb.processing.lidar_matching import Config, compose, match, ordered_cloud
from drone_uwb.processing.lidar_motion import sdk_points


def stamp(message):
    value = message['header']['stamp']
    return value['sec'] + value['nanosec'] / 1e9


def stats(values):
    v = np.asarray(values, dtype=float)
    return None if not len(v) else dict(count=len(v), minimum=float(v.min()),
        median=float(np.median(v)), p95=float(np.quantile(v, .95)), maximum=float(v.max()))


def fit_offsets(start, end, yaw, gyro_times, integral):
    """A profile, not a calibrated confidence interval; no constant-rate claim."""
    offsets = np.arange(-.25, .2501, .005)
    profile = []
    for offset in offsets:
        predicted = (np.interp(end + offset, gyro_times, integral)
                     - np.interp(start + offset, gyro_times, integral))
        residual = yaw - predicted
        # Bound occasional scan correspondence errors without hiding their count.
        clipped = np.minimum(residual ** 2, np.deg2rad(1.) ** 2)
        profile.append(dict(offset_s=float(offset), rms_deg=float(np.rad2deg(np.sqrt(np.mean(clipped)))),
            median_abs_deg=float(np.rad2deg(np.median(np.abs(residual))))))
    return min(profile, key=lambda p: p['rms_deg']), profile


def read_capture(path):
    counts = Counter()
    imus, scans, ranges, states, landed = [], [], [], [], []
    with path.open(encoding='utf-8') as source:
        for line in source:
            row = json.loads(line)
            topic, msg = row['topic'], row['message']
            counts[topic] += 1
            if topic == '/mavros/imu/data':
                imus.append((stamp(msg), [msg['angular_velocity'][k] for k in 'xyz'],
                    [msg['orientation'][k] for k in 'xyzw'], row['received_ros_ns'] / 1e9))
            elif topic == '/point_cloud':
                offsets = next(c['values'] for c in msg['channels'] if c['name'] == 'stamps')
                points, offsets, dropped = sdk_points([[p[k] for k in 'xyz'] for p in msg['points']], offsets)
                scans.append((stamp(msg), points, offsets, row['received_ros_ns'] / 1e9, dropped))
            elif topic == '/mavros/downward_0':
                ranges.append(msg['range'])
            elif topic == '/mavros/state':
                states.append(dict(armed=msg['armed'], connected=msg['connected'], mode=msg['mode']))
            elif topic == '/mavros/extended_state':
                landed.append(msg['landed_state'])
    return counts, imus, scans, ranges, states, landed


def deskew_profile(scans, origin, gt, integral, pairs, config):
    """Yaw-only ground diagnostic; use common pairs across the full profile."""
    offsets = np.arange(-.075, .0751, .005)
    mids = np.array([s[0] - origin + (s[2][0] + s[2][-1]) / 2 for s in scans])
    indices = [(int(np.argmin(abs(mids - p['start_s']))), int(np.argmin(abs(mids - p['end_s'])))) for p in pairs]
    residuals = np.full((len(offsets), len(pairs)), np.nan)
    for oi, delta in enumerate(offsets):
        clouds = {}
        for index in sorted(set(i for pair in indices for i in pair)):
            first, points, times, _, _ = scans[index]
            yaw = np.interp(first - origin + times + delta, gt, integral)
            yaw -= np.interp(mids[index] + delta, gt, integral)
            c, s = np.cos(yaw), np.sin(yaw)
            p = points[:, :2]
            projected = np.column_stack((c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]))
            ranges = np.linalg.norm(p, axis=1)
            clouds[index] = ordered_cloud(projected, (ranges >= .2) & (ranges <= 4.), config)
        for pi, (before, after) in enumerate(indices):
            forward = match(clouds[before], clouds[after], config=config)
            backward = match(clouds[after], clouds[before], config=config)
            reverse = compose(forward.pose, backward.pose)
            if (forward.accepted and backward.accepted and np.linalg.norm(reverse[:2]) <= .02
                    and abs(reverse[2]) <= np.deg2rad(.3)):
                predicted = np.interp(mids[after] + delta, gt, integral) - np.interp(mids[before] + delta, gt, integral)
                residuals[oi, pi] = forward.pose[2] - predicted
    common = np.isfinite(residuals).all(axis=0)
    result = dict(common_pairs=int(common.sum()), candidates_s=offsets.tolist(),
        accepted_pairs_per_candidate=np.isfinite(residuals).sum(axis=1).tolist(),
        scope='yaw_only_deskew_diagnostic_not_operational_configuration')
    if common.sum() < 10:
        return result
    matrix = residuals[:, common]
    scores = np.sqrt(np.mean(np.minimum(matrix ** 2, np.deg2rad(1.) ** 2), axis=1))
    best = int(np.argmin(scores))
    result.update(best_offset_s=float(offsets[best]), rms_deg=np.rad2deg(scores).tolist(),
        best_residual_deg=stats(np.rad2deg(matrix[best])),
        temporal_third_best_offsets_s=[float(offsets[np.argmin(np.mean(m ** 2, axis=1))])
            for m in np.array_split(matrix, 3, axis=1) if m.shape[1] >= 3])
    # Resample adjacent groups, not individual correlated scan pairs.
    rng = np.random.default_rng(20261013)
    blocks = np.array_split(np.arange(matrix.shape[1]), max(2, matrix.shape[1] // 5))
    bootstrap = []
    for _ in range(300):
        selected = np.concatenate([blocks[i] for i in rng.integers(len(blocks), size=len(blocks))])
        bootstrap.append(offsets[np.argmin(np.mean(np.minimum(matrix[:, selected] ** 2, np.deg2rad(1.) ** 2), axis=1))])
    result['diagnostic_block_bootstrap_95_s'] = np.quantile(bootstrap, [.025, .975]).tolist()
    result['bootstrap_is_not_systematic_error_bound'] = True
    return result


def analyze(path, *, deskew=False):
    counts, imus, scans, ranges, states, landed = read_capture(path)
    if len(imus) < 20 or len(scans) < 5:
        raise ValueError('insufficient_capture')
    origin = imus[0][0]
    gt = np.array([x[0] - origin for x in imus])
    gyro = np.array([x[1] for x in imus])
    monotonic = np.r_[True, np.diff(gt) > 0]
    duplicates = int((~monotonic).sum())
    gt, gyro = gt[monotonic], gyro[monotonic]
    integral = cumulative_trapezoid(gyro[:, 2], gt, initial=0)
    tilt = np.rad2deg(np.arccos(np.clip(Rotation.from_quat([x[2] for x in imus]).as_matrix()[:, 2, 2], -1, 1)))
    config = Config(spatial_support=True, estimate_range_bias=True, normal_half_window=4,
        convergence_translation_m=.0005, convergence_yaw_rad=.0002)
    pairs, reasons = [], Counter()
    previous = None
    for first, points, offsets, received, dropped in scans:
        mid = first - origin + (offsets[0] + offsets[-1]) / 2
        ranges_xy = np.linalg.norm(points[:, :2], axis=1)
        cloud = ordered_cloud(points[:, :2], (ranges_xy >= .2) & (ranges_xy <= 4.), config)
        if previous is not None:
            pmid, pc = previous
            # Keep only excited pairs bracketed for every candidate offset.
            left = np.searchsorted(gt, pmid - .25)
            right = np.searchsorted(gt, mid + .25, side='right')
            rates = gyro[left:right, 2]
            if .05 < mid - pmid < .2 and left > 0 and right < len(gt) and len(rates) and np.max(np.abs(rates)) > .06:
                gaps = np.diff(gt[max(0, left - 1):min(len(gt), right + 1)])
                if np.max(gaps) > .025:
                    reasons['imu_gap'] += 1
                else:
                    forward = match(pc, cloud, config=config)
                    backward = match(cloud, pc, config=config)
                    reverse = compose(forward.pose, backward.pose)
                    failure = list(forward.reasons) + ['reverse_' + r for r in backward.reasons]
                    if np.linalg.norm(reverse[:2]) > .02 or abs(reverse[2]) > np.deg2rad(.3):
                        failure.append('bidirectional_inconsistent')
                    if failure:
                        reasons.update(failure)
                    else:
                        pairs.append(dict(start_s=float(pmid), end_s=float(mid), yaw_rad=float(forward.pose[2]),
                            translation_m=forward.pose[:2].tolist(), residual_p90_m=forward.residual_p90_m,
                            reverse_yaw_deg=float(np.rad2deg(reverse[2]))))
        previous = mid, cloud
    result = dict(scope='GROUND_YAW_TIMING_DIAGNOSTIC_NOT_FLIGHT_APPROVAL',
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), topic_counts=dict(counts),
        all_states_disarmed=bool(states) and all(not x['armed'] for x in states),
        all_states_connected=bool(states) and all(x['connected'] for x in states),
        modes=sorted(set(x['mode'] for x in states)), landed_states=sorted(set(landed)),
        imu_nonmonotonic_samples=duplicates, imu_interval_s=stats(np.diff(gt)),
        imu_gaps_over_25ms=int((np.diff(gt) > .025).sum()), gyro_z_deg_s=stats(np.rad2deg(gyro[:, 2])),
        integrated_yaw_extent_deg=float(np.rad2deg(np.ptp(integral))), tilt_deg=stats(tilt), range_m=stats(ranges),
        scan_span_s=stats([s[2][-1] - s[2][0] for s in scans]),
        scan_end_receipt_age_s=stats([s[3] - s[0] - s[2][-1] for s in scans]),
        accepted_pairs=len(pairs), rejected_reasons=dict(reasons), pairs=pairs,
        timing_calibration_complete=False, fc_commands_sent=0, fc_parameters_written=0,
        limitations=['Raw rolling scans have scene-dependent effective observation times.',
                    'Offset profile is diagnostic, not an uncertainty certificate.',
                    'No independent translation ground truth or hover evidence.'])
    if len(pairs) >= 10:
        start = np.array([p['start_s'] for p in pairs])
        end = np.array([p['end_s'] for p in pairs])
        yaw = np.array([p['yaw_rad'] for p in pairs])
        best, profile = fit_offsets(start, end, yaw, gt, integral)
        prediction = np.interp(end, gt, integral) - np.interp(start, gt, integral)
        result.update(best_diagnostic_offset=best, offset_profile=profile,
            yaw_delta_correlation=float(np.corrcoef(yaw, prediction)[0, 1]),
            unshifted_error_deg=stats(np.rad2deg(yaw - prediction)),
            matched_yaw_delta_deg=stats(np.rad2deg(yaw)))
        splits = []
        for indices in np.array_split(np.arange(len(pairs)), 3):
            if len(indices) >= 5:
                splits.append(fit_offsets(start[indices], end[indices], yaw[indices], gt, integral)[0])
        result['temporal_third_best_offsets'] = splits
        if deskew:
            result['deskew_diagnostic'] = deskew_profile(scans, origin, gt, integral, pairs, config)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deskew', action='store_true')
    args = parser.parse_args()
    result = analyze(args.capture / 'topics.jsonl', deskew=args.deskew)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('pairs', 'offset_profile')}, allow_nan=False))


if __name__ == '__main__':
    main()
