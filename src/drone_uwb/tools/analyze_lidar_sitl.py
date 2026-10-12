"""Analyze immutable SIH logs, separating received samples and fusion events."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from prepare_lidar_sitl import FIRMWARE_COMMIT
from lidar_ev_sitl import sha256


def metrics(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(n=len(a), rms=float(np.sqrt(np.mean(a*a))) if len(a) else None,
                p95=float(np.quantile(abs(a), .95)) if len(a) else None,
                maximum=float(np.max(abs(a))) if len(a) else None,
                median=float(np.median(a)) if len(a) else None)


def analyze(run):
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8'))
    status = json.loads((run / 'run_status.json').read_text(encoding='utf-8'))
    if status['errors']:
        raise ValueError(f'run failed: {status["errors"]}')
    files = list(run.rglob('*.ulg'))
    if len(files) != 1:
        raise ValueError('expected one complete ULog')
    u = ULog(str(files[0]))
    if u.msg_info_dict['ver_sw'] != FIRMWARE_COMMIT:
        raise ValueError('wrong firmware in ULog')
    data = {d.name: d.data for d in u.data_list if d.multi_id == 0}
    start = status['first_clock_us']

    def mask(d, lo, hi):
        return (d['timestamp'] >= start + lo * 1e6) & (d['timestamp'] < start + hi * 1e6)

    def phase(lo, hi):
        result = {}
        for label in ['ev_pos', 'ev_vel']:
            d = data.get('estimator_aid_src_' + label)
            if d is None:
                result[label] = dict(logged_updates=0, unique_samples=0, unique_aid_update_times=0)
                continue
            selected = mask(d, lo, hi)
            fused = selected & (d['time_last_fuse'] >= start + lo * 1e6) & (d['time_last_fuse'] < start + hi * 1e6) & (d['time_last_fuse'] > 0)
            result[label] = dict(logged_updates=int(selected.sum()),
                unique_samples=len(np.unique(d['timestamp_sample'][selected])),
                unique_aid_update_times=len(np.unique(d['time_last_fuse'][fused])),
                rejected_updates=int(d['innovation_rejected'][selected].sum()),
                observation_variance={str(i): metrics(d[f'observation_variance[{i}]'][selected])
                                      for i in range(3 if label == 'ev_vel' else 2)},
                test_ratio={str(i): metrics(d[f'test_ratio[{i}]'][selected])
                            for i in range(3 if label == 'ev_vel' else 2)})
        est, truth = data['vehicle_local_position'], data['vehicle_local_position_groundtruth']
        selected = mask(est, lo, hi) & (est['timestamp'] >= truth['timestamp'][0]) & (est['timestamp'] <= truth['timestamp'][-1])
        t = est['timestamp'][selected]
        # Simulator truth is read here only, never supplied to the observations.
        errors = {k: est[k][selected] - np.interp(t, truth['timestamp'], truth[k])
                  for k in ['x', 'y', 'z', 'vx', 'vy', 'vz']}
        result['error'] = dict(xy_m=metrics(np.hypot(errors['x'], errors['y'])),
                               vxy_mps=metrics(np.hypot(errors['vx'], errors['vy'])),
                               z_m=metrics(errors['z']), vz_mps=metrics(errors['vz']))
        result['resets'] = {k: dict(first=int(est[k][selected][0]), last=int(est[k][selected][-1]))
                            for k in ['xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter'] if selected.any()}
        return result

    odo = data.get('vehicle_visual_odometry')
    received = dict(logged=0)
    if odo is not None:
        # ULog logging can drop/decimate. These are LOGGED messages, not a
        # guaranteed count of every uORB publication or network packet.
        received = dict(logged=len(odo['timestamp']),
                        signed_receive_minus_sample_ms=metrics((odo['timestamp'].astype(float)-odo['timestamp_sample'].astype(float))/1e3),
                        unique_timestamps=len(np.unique(odo['timestamp_sample'])))
    flags = data.get('estimator_status_flags', {})
    sync = data.get('timesync_status', {})
    sync_result = {k: metrics(sync[k]) for k in ['round_trip_time','estimated_offset'] if k in sync}
    if 'round_trip_time' in sync:
        sync_result['logged_low_rtt_samples'] = int((sync['round_trip_time'] < 10000).sum())
    result = dict(manifest=manifest, status=status, ulog_sha256=sha256(files[0]),
                  firmware_commit=u.msg_info_dict['ver_sw'], received=received,
                  timesync=sync_result,
                  initial_parameters={k:v for k,v in u.initial_parameters.items() if k.startswith('EKF2_EV') or k in ['EKF2_BARO_CTRL','EKF2_RNG_CTRL','EKF2_GPS_CTRL']},
                  phase={name: phase(lo,hi) for name,lo,hi in [('normal',25,40),('fault',40,45),('recovery',50,120)]},
                  status_flags={k: dict(first=int(flags[k][0]), last=int(flags[k][-1]))
                                for k in ['cs_ev_pos','cs_ev_vel','cs_baro_hgt','cs_rng_hgt','cs_fake_hgt','cs_in_air'] if k in flags},
                  aid_count_semantics='Unique time_last_fuse includes measurement fusion AND reset-to-observation; inspect reset increments separately',
                  scope='stationary synthetic SIH transport diagnostic; no S01-S14 overall PASS')
    (run / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    args = p.parse_args()
    print(json.dumps(analyze(args.run), indent=2))
