"""Prepare native PX4 ULog sensor records for later UWB experiments.

No FC-to-host clock mapping, antenna-height estimate, synthetic UWB, or device
output is created. Native FRD-to-NED attitudes must pass a frame adapter first.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import write_json
from drone_uwb.processing.runner import json_line


def gap_metrics(timestamps):
    gaps = np.diff(np.asarray(timestamps, dtype=np.int64))/1e6
    if not len(gaps):
        return dict(interval_count=0)
    return dict(interval_count=len(gaps), median_s=float(np.median(gaps)),
                max_s=float(np.max(gaps)), nonpositive=int(np.sum(gaps <= 0)))


def range_records(data):
    output = []
    for i, t in enumerate(data['timestamp']):
        value, lower, upper = (float(data[k][i]) for k in ('current_distance', 'min_distance', 'max_distance'))
        quality, variance = int(data['signal_quality'][i]), float(data['variance'][i])
        reason = ('invalid_distance' if not np.isfinite([value, lower, upper]).all() or not lower <= value <= upper
                  else 'invalid_signal' if quality == 0 else 'ok')
        output.append(dict(kind='px4_distance_sensor', source='measured', clock_domain='px4_boot_us',
                           timestamp_us=int(t), timestamp_sample_us=None,
                           sample_time_known=False, distance_m=value if np.isfinite(value) else None,
                           min_distance_m=lower if np.isfinite(lower) else None,
                           max_distance_m=upper if np.isfinite(upper) else None,
                           sensor_type=int(data['type'][i]), orientation=int(data['orientation'][i]),
                           device_id=int(data['device_id'][i]), signal_quality=quality,
                           signal_quality_known=quality >= 0,
                           variance_m2=variance if np.isfinite(variance) and variance > 0 else None,
                           usable_raw=reason == 'ok', reason=reason,
                           z_antenna_m=None, external_output_allowed=False))
    return output


def attitude_records(data):
    output = []
    for i, t in enumerate(data['timestamp']):
        q = np.array([data[f'q[{k}]'][i] for k in range(4)], float)
        norm = float(np.linalg.norm(q))
        sample = int(data['timestamp_sample'][i])
        valid = bool(np.isfinite(q).all() and abs(norm-1) < 1e-3 and 0 <= sample <= int(t))
        output.append(dict(kind='px4_vehicle_attitude', source='measured', clock_domain='px4_boot_us',
                           timestamp_us=int(t), timestamp_sample_us=sample,
                           quaternion_wxyz=q.tolist() if np.isfinite(q).all() else None,
                           quaternion_norm=norm if np.isfinite(norm) else None,
                           rotation_convention='FRD_body_to_NED_earth',
                           quat_reset_counter=int(data['quat_reset_counter'][i]),
                           usable_raw=valid, reason='ok' if valid else 'invalid_attitude',
                           external_output_allowed=False))
    return output


def position_intervals(data, last_timestamp):
    """Observed POSCTL intervals; mode presence is not a hover accuracy metric."""
    times, states = data['timestamp'], data['nav_state']
    intervals, begin = [], None
    for t, state in zip(times, states):
        if state == 2 and begin is None:
            begin = int(t)
        if state != 2 and begin is not None:
            intervals.append(dict(start_us=begin, end_us=int(t), duration_s=(int(t)-begin)/1e6))
            begin = None
    if begin is not None:
        intervals.append(dict(start_us=begin, end_us=int(last_timestamp), duration_s=(int(last_timestamp)-begin)/1e6,
                              end_censored=True))
    return intervals


def summarize(distances, attitudes, status, last_timestamp):
    reasons = ['missing_synchronous_uwb_raw', 'fc_uwb_clock_mapping_unconfirmed',
               'sensor_mount_and_antenna_offsets_unconfirmed', 'warehouse_frame_alignment_unconfirmed',
               'distance_sensor_model_unconfirmed']
    return dict(distance_records=len(distances), usable_distance_records=sum(r['usable_raw'] for r in distances),
                distance_reasons=dict(Counter(r['reason'] for r in distances)),
                distance_record_gaps=gap_metrics([r['timestamp_us'] for r in distances]),
                distance_unknown_quality=sum(not r['signal_quality_known'] for r in distances),
                distance_unknown_variance=sum(r['variance_m2'] is None for r in distances),
                attitude_records=len(attitudes), usable_attitude_records=sum(r['usable_raw'] for r in attitudes),
                attitude_record_gaps=gap_metrics([r['timestamp_sample_us'] for r in attitudes]),
                nav_state_counts=dict(Counter(str(int(s)) for s in status['nav_state'])),
                observed_position_intervals=position_intervals(status, last_timestamp),
                ABC_flight_comparison_ready=False, blocked_reasons=reasons,
                antenna_height_calculated=False, position_accuracy_evaluated=False,
                external_output_allowed=False, flight_valid=False)


def run(ulog_path, output):
    from pyulog import ULog
    path, output = Path(ulog_path), Path(output)
    ulog = ULog(str(path))
    distance = ulog.get_dataset('distance_sensor').data
    attitude = ulog.get_dataset('vehicle_attitude').data
    status = ulog.get_dataset('vehicle_status').data
    distances, attitudes = range_records(distance), attitude_records(attitude)
    summary = summarize(distances, attitudes, status, ulog.last_timestamp)
    matches = [(d.name, key) for d in ulog.data_list for key in d.data
               if any(x in (d.name+' '+key).lower() for x in ('uwb', 'raw_slant', 'anchor_id', 'sample_time_us'))]
    if matches:
        raise ValueError('UWB_fields_present_review_native_schema_before_preparing_profile')
    names = ('EKF2_EV_CTRL', 'EKF2_OF_CTRL', 'EKF2_RNG_CTRL', 'EKF2_HGT_REF',
             'SENS_EN_TF02PRO', 'SENS_TFMINI_CFG', 'EKF2_RNG_POS_X', 'EKF2_RNG_POS_Y', 'EKF2_RNG_POS_Z')
    summary.update(input_file=str(path), input_sha256=digest(path),
                   duration_s=(ulog.last_timestamp-ulog.start_timestamp)/1e6,
                   firmware=ulog.msg_info_dict.get('ver_sw'), raw_uwb_field_matches=matches,
                   parameters={k: ulog.initial_parameters.get(k) for k in names},
                   sensor_model_note='Driver parameter configuration is evidence, not physical device identification.')
    output.mkdir(parents=True, exist_ok=False)
    records = sorted(distances+attitudes, key=lambda r: (r['timestamp_us'], r['kind']))
    (output/'native_sensor_events.jsonl').write_text(''.join(map(json_line, records)), encoding='utf-8')
    write_json(output/'readiness.json', summary)
    write_json(output/'manifest.json', dict(input_sha256=digest(path), code_sha256=digest(__file__),
                                          output_sha256={p.name: digest(p) for p in sorted(output.iterdir())}))
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    print(json.dumps(run(options.ulog, options.output), ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
