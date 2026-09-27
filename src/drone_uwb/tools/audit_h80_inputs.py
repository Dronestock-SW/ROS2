"""Offline input audit and small A/H80 examples; no ROS or external output.

Real logs are inspected for fields, timing and matrix rank only. Synthetic
examples exercise existing Python solvers, not a reproduction of the ZIP binary.
"""
import argparse
from collections import Counter, deque
import csv
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from drone_uwb.preimu.h80 import fit_h80
from drone_uwb.preimu.rawxy import solve_raw_xy
from drone_uwb.preimu.settings import PreimuSettings


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return None
    return dict(n=int(values.size), min=float(values.min()),
                median=float(np.median(values)), p95=float(np.quantile(values, .95)),
                max=float(values.max()), rmse=float(np.sqrt(np.mean(values**2))))


def audit_raw(path, anchors):
    window = deque()
    counts = Counter()
    cycle_times, observations_per_window, spans = [], [], []
    last_t = None
    first_ready = None
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        event = json.loads(line)
        msg = event.get('message', {})
        counts[msg.get('type', 'unknown')] += 1
        if msg.get('event') == 'boot':
            window.clear()
            last_t = None
        if msg.get('type') != 'uwb_raw_cycle':
            continue
        t = msg['cycle_end_us'] / 1e6
        cycle_times.append(t)
        if last_t is not None and (t <= last_t or t - last_t > .15):
            window.clear()
            counts['window_resets'] += 1
        last_t = t
        ranges = msg.get('raw_slant_m', [])
        times = msg.get('sample_time_us', [])
        failures = msg.get('failure', [])
        if not (len(ranges) == len(times) == len(failures) == 4):
            counts['missing_required_arrays'] += 1
            continue
        for i in range(4):
            if not (msg.get('valid_mask', 0) & (1 << i)) or failures[i] != 'ok':
                counts['invalid_range_samples'] += 1
                continue
            if (not isinstance(ranges[i], (int, float)) or not np.isfinite(ranges[i])
                    or ranges[i] <= 0 or not isinstance(times[i], int)
                    or not msg['cycle_start_us'] <= times[i] <= msg['cycle_end_us']):
                counts['invalid_range_samples'] += 1
                continue
            window.append((times[i] / 1e6, i))
            counts['valid_range_samples'] += 1
        # Individual observations may arrive out of anchor-time order.
        window = deque(row for row in window if t - .8 <= row[0] <= t)
        observations_per_window.append(len(window))
        participating = Counter(i for _, i in window)
        if len(window) < 12 or any(participating[i] < 3 for i in range(4)):
            counts['insufficient_history_cycles'] += 1
            continue
        sample_s = np.array([s for s, _ in window])
        idx = np.array([i for _, i in window])
        u = (sample_s - t) / .8
        ax, ay = anchors[idx, 0], anchors[idx, 1]
        design = np.column_stack([-2*ax, -2*ay, -2*ax*u, -2*ay*u,
                                  np.ones_like(u), u, u*u])
        scale = np.max(np.abs(design), axis=0)
        if np.any(scale == 0) or np.linalg.matrix_rank(design / scale) < 7:
            counts['rank_deficient_cycles'] += 1
            continue
        counts['structurally_ready_cycles'] += 1
        spans.append(min(np.ptp(sample_s[idx == i]) for i in range(4)))
        if first_ready is None:
            first_ready = t - cycle_times[0]
    n = counts['uwb_raw_cycle']
    return dict(path=str(path), sha256=sha256(path), counts=dict(counts),
                cycle_gap_s=stats(np.diff(cycle_times)),
                duration_s=cycle_times[-1]-cycle_times[0] if n else None,
                samples_in_window=stats(observations_per_window),
                shortest_anchor_span_s=stats(spans), first_ready_after_s=first_ready,
                structurally_ready_fraction=counts['structurally_ready_cycles']/n if n else None,
                readiness_only=True, calibration_verified=False,
                position_accuracy_evaluated=False)


def sample_comparison(anchors):
    # These are synthetic fixture choices, never device calibration or run defaults.
    bias = np.array([-.08733, .41107, -.11148, .15125])
    settings = replace(PreimuSettings(), sigma_r_m=.04, raw_xy_max_rms_m=1e6,
                       raw_xy_region_margin_m=1e6, raw_xy_max_condition=1e6)
    cases = [('clean', 0.), ('noise', .04), ('spikes', .04), ('direction_change', .02)]
    results = []
    for name, noise in cases:
        rng = np.random.default_rng(7)
        history = deque()
        failures = Counter()
        errors_a, errors_h = [], []
        valid_a = valid_h = total = 0
        for k in range(320):
            t = k / 40.
            xy = np.array([.70, 1.80])
            if name == 'direction_change':
                xy[0] += .60 * min(t, 8.-t)
            z = 1.10 + .02*t
            ranges = np.linalg.norm(anchors - np.r_[xy, z], axis=1)
            raw = ranges + bias + rng.normal(0, noise, 4)
            if name == 'spikes' and k % 20 == 10:
                raw[1] += .60
            z_observed = z + rng.normal(0, .002 if noise else 0.)
            calibrated = raw - bias
            history.extend((t, i, r) for i, r in enumerate(calibrated))
            while history and history[0][0] < t - .8 - 1e-12:
                history.popleft()
            source_s = np.array([row[0] for row in history])
            idx = np.array([row[1] for row in history])
            rr = np.array([row[2] for row in history])
            a = solve_raw_xy(anchors, calibrated, [0, 1, 2, 3], z_observed, settings)
            h = fit_h80(anchors, idx, source_s, rr, t, settings)
            if t < .8:
                continue
            total += 1
            valid_a += int(a.valid)
            valid_h += int(h.ok)
            if not a.valid:
                failures['A:' + str(a.reason)] += 1
            if not h.ok:
                failures['H80:' + str(h.reason)] += 1
            if a.valid and h.ok:
                errors_a.append(np.linalg.norm(np.array([a.x, a.y])-xy))
                errors_h.append(np.linalg.norm(np.array([h.x0, h.y0])-xy))
        sa, sh = stats(errors_a), stats(errors_h)
        results.append(dict(case=name, uwb_noise_sigma_m=noise, evaluation_ticks=total,
                            paired_ticks=len(errors_a), A_error_m=sa, H80_error_m=sh,
                            A_coverage=valid_a/total, H80_coverage=valid_h/total,
                            failures=dict(failures),
                            rmse_reduction_pct=100*(1-sh['rmse']/sa['rmse'])
                            if sa and sh and sa['rmse'] > 1e-9 else None))
    clean = results[0]
    assert clean['A_error_m']['max'] < 1e-6
    assert clean['H80_error_m']['max'] < 1e-6
    short = fit_h80(anchors, np.arange(4), np.zeros(4), np.ones(4), 0., settings)
    assert not short.ok and short.reason == 'insufficient_anchors'
    return dict(scope='synthetic_examples_only', seed=7, duration_s=8., rate_hz=40.,
                simultaneous_ranges=True, warmup_excluded_s=.8,
                synthetic_z_formula='1.10 + 0.02*t', tof_noise_sigma_m=.002,
                synthetic_bias_m=bias.tolist(), extra_range_and_position_gates=False,
                A_residual_and_region_rejection_disabled_for_comparison=True,
                settings=dict(window_s=.8, sigma_r_m=.04, huber_m=.12, iterations=8),
                checks=['clean_A_and_H80_below_1um', 'missing_history_rejected'], cases=results)


def audit_ulog(path, output_dir):
    from pyulog import ULog
    ulog = ULog(str(path))
    inventory = [dict(topic=d.name, instance=d.multi_id,
                      count=len(d.data.get('timestamp', []))) for d in ulog.data_list]
    distance = ulog.get_dataset('distance_sensor').data
    attitude = ulog.get_dataset('vehicle_attitude').data
    matches = [(d.name, key) for d in ulog.data_list for key in d.data
               if any(s in (d.name+' '+key).lower()
                      for s in ['uwb', 'raw_slant', 'anchor_id', 'sample_time_us'])]
    for topic, fields in [
        ('distance_sensor', ['timestamp', 'current_distance', 'min_distance', 'max_distance',
                             'signal_quality', 'variance', 'orientation', 'device_id']),
        ('vehicle_attitude', ['timestamp', 'timestamp_sample', 'q[0]', 'q[1]', 'q[2]', 'q[3]'])]:
        data = ulog.get_dataset(topic).data
        with (output_dir / (topic+'.csv')).open('w', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(fields)
            writer.writerows(zip(*(data[f] for f in fields)))
    quality = distance['signal_quality']
    within = ((distance['current_distance'] >= distance['min_distance']) &
              (distance['current_distance'] <= distance['max_distance']))
    return dict(path=str(path), sha256=sha256(path),
                duration_s=(ulog.last_timestamp-ulog.start_timestamp)/1e6,
                dropout_count=len(ulog.dropouts), inventory=inventory,
                firmware=ulog.msg_info_dict.get('ver_sw'),
                raw_uwb_field_matches=matches,
                distance_m=stats(distance['current_distance']),
                distance_gap_s=stats(np.diff(distance['timestamp'].astype(np.int64))/1e6),
                distance_within_declared_limits=int(within.sum()),
                distance_quality_counts=dict(Counter(str(int(x)) for x in quality)),
                distance_variance_values=np.unique(distance['variance']).tolist(),
                distance_orientation_values=np.unique(distance['orientation']).tolist(),
                distance_device_ids=np.unique(distance['device_id']).tolist(),
                attitude_count=len(attitude['timestamp']),
                attitude_gap_s=stats(np.diff(attitude['timestamp_sample'].astype(np.int64))/1e6),
                tof_sensor_model_confirmed=False, z_used_by_pipeline=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--raw-log', type=Path, action='append', required=True)
    parser.add_argument('--ulog', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    layout = json.loads(args.layout.read_text(encoding='utf-8'))
    anchors = np.array(layout['anchors_xyz_m'], dtype=float)
    assert anchors.shape == (4, 3) and np.isfinite(anchors).all()
    assert np.allclose(anchors[:, 2], anchors[0, 2])
    assert layout['anchor_order'] == ['A1', 'A2', 'A3', 'A4']
    report = dict(schema='h80_readiness_audit_v1', script_sha256=sha256(__file__),
                  code_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                  layout_sha256=sha256(args.layout),
                  raw_logs=[audit_raw(path, anchors) for path in args.raw_log],
                  synthetic=sample_comparison(anchors))
    if args.ulog:
        report['ulog'] = audit_ulog(args.ulog, args.output_dir)
    from drone_uwb.preimu import h80, rawxy
    report['solver_sha256'] = dict(h80=sha256(h80.__file__), rawxy=sha256(rawxy.__file__))
    (args.output_dir/'summary.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('Audit saved:', args.output_dir/'summary.json')


if __name__ == '__main__':
    main()
