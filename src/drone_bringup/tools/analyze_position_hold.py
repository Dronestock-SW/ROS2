"""
Inspect PX4 Position intervals offline; never send flight commands.

Error means EKF local XY minus the logged controller target, not ground truth.
Mode IDs follow PX4 VehicleStatus (v1.17). Unknown IDs remain numeric.
Requires numpy and pyulog; matplotlib is optional with --plot.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


MODES = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION',
         4: 'AUTO_LOITER', 14: 'OFFBOARD', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND'}
SETTINGS = {'setpoint_max_age_s': 0.25, 'manual_max_age_s': 0.4,
            'state_max_age_s': 1.5, 'neutral_xy_stick_limit': 0.05,
            'fixed_target_span_limit_m': 0.001, 'candidate_min_coverage': 0.95}


def dataset(ulog, name, instance=0):
    """Return absent topics as missing data, never synthetic zero values."""
    try:
        return ulog.get_dataset(name, instance).data
    except (KeyError, IndexError, ValueError):
        return {}


def timestamps(data):
    values = np.asarray(data.get('timestamp', []), dtype=np.int64)
    if np.any(np.diff(values) < 0):
        raise ValueError('Non-monotonic topic timestamps; split the recording first')
    return values


def align(data, field, query_us, max_age_s):
    """Select only earlier samples within an age limit; missing remains NaN."""
    query_us = np.asarray(query_us, dtype=np.int64)
    result = np.full(query_us.shape, np.nan)
    source = timestamps(data)
    if field not in data or not len(source):
        return result
    index = np.searchsorted(source, query_us, side='right') - 1
    safe_index = np.maximum(index, 0)
    age = query_us - source[safe_index]
    valid = (index >= 0) & (age >= 0) & (age <= max_age_s * 1e6)
    result[valid] = np.asarray(data[field])[safe_index[valid]]
    return result


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {'n': int(len(values)), 'mean': float(np.mean(values)),
            'rms': float(np.sqrt(np.mean(values**2))),
            'p95': float(np.quantile(values, 0.95)), 'max': float(np.max(values))}


def changes(data, field, origin):
    if field not in data or not len(timestamps(data)):
        return []
    values = data[field]
    indices = np.r_[0, np.flatnonzero(values[1:] != values[:-1]) + 1]
    return [{'time_s': (int(data['timestamp'][i]) - origin) / 1e6,
             'value': float(values[i]), 'initial_sample': bool(i == 0)} for i in indices]


def mode_segments(status, origin, end):
    transitions = changes(status, 'nav_state', origin)
    result = []
    for i, row in enumerate(transitions):
        stop = transitions[i + 1]['time_s'] if i + 1 < len(transitions) else end
        start = max(0.0, row['time_s'])
        if stop <= start:
            continue
        mode = int(row['value'])
        result.append({'start_s': start, 'end_s': stop, 'duration_s': stop - start,
                       'nav_state': mode, 'mode': MODES.get(mode, str(mode))})
    return result


def aligned_samples(ulog):
    position = dataset(ulog, 'vehicle_local_position')
    time = timestamps(position)
    if not len(time):
        raise ValueError('vehicle_local_position is required')
    result = {'timestamp_us': time, 'time_s': (time - ulog.start_timestamp) / 1e6}
    for field in ('x', 'y', 'vx', 'vy', 'xy_valid', 'v_xy_valid',
                  'heading_good_for_control', 'xy_reset_counter',
                  'vxy_reset_counter', 'heading_reset_counter'):
        result[field] = np.asarray(position.get(field, np.full(len(time), np.nan)), dtype=float)
    sources = [
        ('vehicle_local_position_setpoint', {'x': 'target_x', 'y': 'target_y',
                                             'timestamp': 'target_timestamp_us'},
         SETTINGS['setpoint_max_age_s']),
        ('manual_control_setpoint', {'roll': 'roll', 'pitch': 'pitch',
                                     'valid': 'manual_valid'}, SETTINGS['manual_max_age_s']),
        ('vehicle_status', {'nav_state': 'nav_state', 'arming_state': 'arming_state'},
         SETTINGS['state_max_age_s']),
        ('vehicle_land_detected', {'landed': 'landed'}, SETTINGS['state_max_age_s']),
        ('vehicle_control_mode', {'flag_control_position_enabled': 'position_enabled'},
         SETTINGS['state_max_age_s']),
    ]
    for topic, fields, max_age in sources:
        for field, output in fields.items():
            result[output] = align(dataset(ulog, topic), field, time, max_age)
    result['error_m'] = np.hypot(result['x'] - result['target_x'],
                                 result['y'] - result['target_y'])
    result['speed_m_s'] = np.hypot(result['vx'], result['vy'])
    result['qualified'] = (
        (result['nav_state'] == 2) & (result['arming_state'] == 2)
        & (result['landed'] == 0) & (result['position_enabled'] == 1)
        & (result['xy_valid'] == 1) & (result['v_xy_valid'] == 1)
        & np.isfinite(result['error_m']) & np.isfinite(result['speed_m_s']))
    return result


def aid_summary(ulog, start, end):
    result = []
    selector = dataset(ulog, 'estimator_selector_status')
    for entry in ulog.data_list:
        if entry.name not in ('estimator_aid_src_optical_flow', 'estimator_aid_src_rng_hgt'):
            continue
        data = entry.data
        time = timestamps(data)
        relative = (time - ulog.start_timestamp) / 1e6
        primary = align(selector, 'primary_instance', time, SETTINGS['state_max_age_s'])
        mask = (relative >= start) & (relative < end) & (primary == entry.multi_id)
        ratios = [data[k][mask] for k in data if k == 'test_ratio' or k.startswith('test_ratio[')]
        result.append({'topic': entry.name, 'instance': entry.multi_id,
                       'selected_samples': int(mask.sum()),
                       'fused_samples': int(np.count_nonzero(data['fused'][mask])),
                       'rejected_samples': int(np.count_nonzero(
                           data['innovation_rejected'][mask])),
                       'test_ratio': stats(np.concatenate(ratios)) if ratios else None})
    return result


def summarize_window(samples, segment, origin):
    start, end = segment['start_s'], segment['end_s']
    inside = (samples['time_s'] >= start) & (samples['time_s'] < end)
    # A target from the preceding mode is not evidence for the current mode.
    mask = (inside & samples['qualified']
            & (samples['target_timestamp_us'] >= origin + start * 1e6))
    count = int(mask.sum())
    coverage = count / int(inside.sum()) if inside.any() else 0.0
    target = np.column_stack((samples['target_x'][mask], samples['target_y'][mask]))
    span = float(np.linalg.norm(np.ptp(target, axis=0))) if count else None
    manual_mask = inside & (samples['manual_valid'] == 1)
    sticks = np.maximum(np.abs(samples['roll']), np.abs(samples['pitch']))
    manual_stats = stats(sticks[manual_mask])
    reset_changes = {k: int(np.count_nonzero(np.diff(samples[k][inside])))
                     if np.isfinite(samples[k][inside]).all() else None
                     for k in ('xy_reset_counter', 'vxy_reset_counter', 'heading_reset_counter')}
    candidate = bool(count > 1 and span <= SETTINGS['fixed_target_span_limit_m']
                     and coverage >= SETTINGS['candidate_min_coverage']
                     and manual_stats and manual_stats['n'] == int(inside.sum())
                     and manual_stats['max'] <= SETTINGS['neutral_xy_stick_limit']
                     and all(v == 0 for v in reset_changes.values()))
    return {**segment, 'local_position_samples': int(inside.sum()),
            'qualified_samples': count, 'qualified_fraction': coverage,
            'error_m': stats(samples['error_m'][mask]),
            'speed_m_s': stats(samples['speed_m_s'][mask]),
            'target_median_xy_m': np.median(target, axis=0).tolist() if count else None,
            'target_span_m': span, 'xy_stick_abs': manual_stats,
            'reset_changes': reset_changes, 'position_hold_candidate': candidate,
            'qualification_is_not_flight_acceptance': True}


def make_plot(samples, windows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    xy, error, speed, sticks = axes.flat
    xy.plot(samples['x'], samples['y'], color='0.75', linewidth=1, label='All EKF samples')
    for i, window in enumerate(windows, 1):
        mask = ((samples['time_s'] >= window['start_s'])
                & (samples['time_s'] < window['end_s']) & samples['qualified'])
        color = f'C{i - 1}'
        xy.plot(samples['x'][mask], samples['y'][mask], color=color, label=f'Position {i}')
        if window['target_median_xy_m']:
            xy.scatter(*window['target_median_xy_m'], marker='x', color=color, s=80)
        error.plot(samples['time_s'][mask], samples['error_m'][mask] * 100, color=color)
        for axis in (error, speed, sticks):
            axis.axvspan(window['start_s'], window['end_s'], color=color, alpha=0.12)
    xy.set(xlabel='Local NED x (m)', ylabel='Local NED y (m)', title='EKF XY and hold targets (x)')
    xy.axis('equal')
    xy.legend()
    error.set(xlabel='Seconds from log header', ylabel='XY error (cm)',
              title='EKF-to-target error, not ground-truth accuracy')
    speed.plot(samples['time_s'], samples['speed_m_s'], linewidth=1)
    speed.set(xlabel='Seconds from log header', ylabel='Horizontal speed (m/s)')
    for field in ('roll', 'pitch'):
        sticks.plot(samples['time_s'], samples[field], label=field, linewidth=1)
    sticks.legend()
    sticks.set(xlabel='Seconds from log header', ylabel='Normalized horizontal stick input')
    for axis in axes.flat:
        axis.grid(alpha=0.25)
    fig.suptitle('PX4 Position hold baseline | shaded: POSCTL | offline log analysis')
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    from pyulog import ULog
    ulog = ULog(str(args.ulog))
    samples = aligned_samples(ulog)
    origin = ulog.start_timestamp
    duration = (ulog.last_timestamp - origin) / 1e6
    segments = mode_segments(dataset(ulog, 'vehicle_status'), origin, duration)
    windows = [summarize_window(samples, s, origin) for s in segments if s['nav_state'] == 2]
    for window in windows:
        window['selected_estimator_aid'] = aid_summary(ulog, window['start_s'], window['end_s'])
    transitions = {}
    for topic, fields in {
        'vehicle_status': ('arming_state', 'failsafe'),
        'vehicle_control_mode': ('flag_control_manual_enabled', 'flag_control_offboard_enabled'),
        'vehicle_land_detected': ('landed', 'ground_contact'),
        'vehicle_local_position': ('xy_valid', 'v_xy_valid', 'heading_good_for_control'),
        'estimator_selector_status': ('primary_instance',),
    }.items():
        transitions[topic] = {f: changes(dataset(ulog, topic), f, origin) for f in fields}
    for d in ulog.data_list:
        if d.name == 'estimator_status_flags':
            fields = ('cs_opt_flow', 'cs_rng_hgt', 'cs_ev_pos', 'cs_gnss_pos',
                      'cs_mag_hdg', 'cs_yaw_align', 'cs_mag_aligned_in_flight')
            transitions[f'{d.name}_{d.multi_id}'] = {f: changes(d.data, f, origin) for f in fields}
    prefixes = ('EKF2_EV', 'EKF2_GPS_CTRL', 'EKF2_OF', 'EKF2_RNG', 'EKF2_HGT_REF',
                'MPC_XY', 'MPC_ACC', 'MPC_JERK', 'MPC_POS_MODE', 'MPC_VEL_MANUAL',
                'COM_OF', 'COM_OBL', 'SENS_EN_PMW', 'SENS_EN_TF', 'SYS_HITL')
    report = {
        'schema': 'position_hold_audit_v1', 'file': args.ulog.name,
        'sha256': hashlib.sha256(args.ulog.read_bytes()).hexdigest(),
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'version': ulog.get_version_info_str(),
        'metadata': {k: ulog.msg_info_dict.get(k) for k in ('ver_hw', 'ver_sw', 'sys_os_name')},
        'origin_timestamp_us': origin, 'duration_s': duration,
        'logged_dropout_count': len(ulog.dropouts),
        'analysis_settings': SETTINGS, 'statistics_weighting': 'per_logged_sample',
        'ground_truth_available': False, 'sensor_models_confirmed': False,
        'mode_segments': segments, 'position_windows': windows, 'transitions': transitions,
        'parameters': {k: v for k, v in ulog.initial_parameters.items() if k.startswith(prefixes)},
        'parameter_changes': ulog.changed_parameters,
        'messages': [{'time_s': (m.timestamp - origin) / 1e6, 'message': m.message}
                     for m in ulog.logged_messages],
        'inventory': [{'topic': d.name, 'instance': d.multi_id,
                       'samples': len(timestamps(d.data)),
                       'logged_gap_s': stats(np.diff(timestamps(d.data)) / 1e6)}
                      for d in ulog.data_list],
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / 'summary.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    with (args.output_dir / 'samples.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(samples)
        writer.writerows(zip(*samples.values()))
    if args.plot:
        make_plot(samples, windows, args.output_dir / 'position_hold.png')
    print(args.output_dir / 'summary.json')


if __name__ == '__main__':
    main()
