"""Compare labelled ULogs offline. Estimated error is never ground-truth drift."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from analyze_position_hold import align, aligned_samples, dataset, mode_segments


def describe(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    return dict(n=len(a), mean=float(a.mean()), median=float(np.median(a)),
                rms=float(np.sqrt(np.mean(a*a))), p95_abs=float(np.quantile(abs(a), .95)),
                min=float(a.min()), max=float(a.max()))


def euler(q):
    w, x, y, z = q
    return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
            np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def analyze(path, settle_s=.3):
    from pyulog import ULog
    u = ULog(str(path))
    s = aligned_samples(u)
    t = s['timestamp_us']

    def at(topic, field, instance=0, age=.25):
        return align(dataset(u, topic, instance), field, t, age)

    primary = at('estimator_selector_status', 'primary_instance', age=1.5)

    def selected(topic, field):
        out = np.full(len(t), np.nan)
        for item in u.data_list:
            if item.name == topic:
                values = at(topic, field, item.multi_id,
                            age=1.5 if topic=='estimator_status_flags' else .6)
                mask = primary == item.multi_id
                out[mask] = values[mask]
        return out

    roll, pitch, yaw = euler([at('vehicle_attitude', f'q[{i}]') for i in range(4)])
    sr, sp, _ = euler([at('vehicle_attitude_setpoint', f'q_d[{i}]') for i in range(4)])
    values = {k: s[k] for k in ('error_m', 'speed_m_s')}
    # ULog topics are decimated (Range ~1Hz, Flow ~2Hz in these recordings).
    # These bounds align logged diagnostics, not live sensor freshness gates.
    values.update(range_m=at('distance_sensor', 'current_distance', age=1.2),
                  flow_quality=at('vehicle_optical_flow', 'quality', age=.6),
                  pitch_deg=np.degrees(pitch), pitch_target_deg=np.degrees(sp),
                  roll_error_deg=np.degrees(np.arctan2(np.sin(roll-sr), np.cos(roll-sr))),
                  pitch_error_deg=np.degrees(np.arctan2(np.sin(pitch-sp), np.cos(pitch-sp))),
                  forward_speed_mps=np.cos(yaw)*s['vx']+np.sin(yaw)*s['vy'],
                  right_speed_mps=-np.sin(yaw)*s['vx']+np.cos(yaw)*s['vy'],
                  battery_v=at('battery_status', 'voltage_v', age=1.),
                  primary_estimator=primary)
    for field in ('cs_opt_flow', 'cs_rng_hgt', 'cs_ev_pos', 'cs_inertial_dead_reckoning',
                  'fs_bad_optflow_x', 'fs_bad_optflow_y'):
        values[field] = selected('estimator_status_flags', field)
    for field in ('fused', 'innovation_rejected', 'test_ratio[0]', 'test_ratio[1]',
                  'innovation[0]', 'innovation[1]'):
        values['flow_'+field] = selected('estimator_aid_src_optical_flow', field)
    for i in range(4):
        values[f'motor_{i}'] = at('actuator_motors', f'control[{i}]')
        values[f'saturation_{i}'] = at('control_allocator_status', f'actuator_saturation[{i}]')
    neutral = ((s['manual_valid'] == 1) & (abs(s['roll']) <= .05) & (abs(s['pitch']) <= .05))
    windows = []
    for segment in mode_segments(dataset(u, 'vehicle_status'), u.start_timestamp,
                                 (u.last_timestamp-u.start_timestamp)/1e6):
        if segment['nav_state'] != 2:
            continue
        start, end = segment['start_s']+settle_s, segment['end_s']
        inside = (s['time_s'] >= start) & (s['time_s'] < end)
        mask = inside & s['qualified'] & neutral
        # Do not mix target/estimator resets into steady-hold statistics.
        reset = np.zeros(len(t), bool)
        for key in ('xy_reset_counter', 'vxy_reset_counter', 'heading_reset_counter'):
            x = s[key]
            reset |= np.r_[False, np.diff(x) != 0]
        resets = int(np.count_nonzero(reset & inside))
        target = np.column_stack((s['target_x'][mask], s['target_y'][mask]))
        span = float(np.linalg.norm(np.ptp(target, axis=0))) if len(target) else None
        windows.append(dict(mode_start_s=segment['start_s'], start_s=start, end_s=end,
            qualified_samples=int(mask.sum()), reset_events=resets, target_span_m=span,
            comparable_hold=bool(end-start >= 3. and mask.sum() > 10 and resets == 0 and span is not None and span <= .001),
            metrics={k: describe(v[mask]) for k, v in values.items()}))
    keys = ('SENS_FLOW_ROT', 'SENS_FLOW_SCALE', 'SENS_FLOW_RATE', 'MPC_THR_HOVER',
            'EKF2_OF_CTRL', 'EKF2_RNG_CTRL', 'EKF2_MAG_TYPE', 'EKF2_EV_CTRL')
    return dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        firmware=u.msg_info_dict.get('ver_sw'), parameters={k:u.initial_parameters.get(k) for k in keys},
        relevant_parameter_changes=[(float(ts), name, value) for ts,name,value in u.changed_parameters
                                    if name in keys], windows=windows)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log', action='append', required=True, help='operator-label=path')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    report = dict(scope='offline; operator labels; no independent XY or image ground truth',
                  settle_s=.3, minimum_comparable_hold_s=3.,
                  diagnostic_alignment_age_s={'range':1.2, 'flow_quality':.6,
                                               'aid_source':.6, 'estimator_flags':1.5},
                  illumination_cause_confirmed=False, logs={})
    for item in args.log:
        label, path = item.split('=', 1)
        if label in report['logs']:
            raise ValueError('duplicate label')
        report['logs'][label] = analyze(Path(path))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    for label, result in report['logs'].items():
        for w in result['windows']:
            if w['qualified_samples']:
                print(label, round(w['start_s'],2), round(w['end_s'],2),
                      {k:round(w['metrics'][k]['rms'],4) if w['metrics'][k] else None
                       for k in ('range_m','error_m','speed_m_s','pitch_error_deg','flow_quality','flow_fused')})


if __name__ == '__main__':
    main()
