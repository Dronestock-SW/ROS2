"""Read-only hash audit and delayed-EKF/output comparison of preserved SIH logs."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def metrics(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    result = dict(n=int(values.size), finite=int(finite.size))
    if finite.size:
        result.update(mean=float(finite.mean()), std=float(finite.std()),
                      rms=float(np.sqrt(np.mean(finite ** 2))),
                      p95_abs=float(np.quantile(np.abs(finite), .95)),
                      max_abs=float(np.max(np.abs(finite))), last=float(finite[-1]))
    return result


def hold(t, data, key):
    indices = np.searchsorted(data['timestamp'], t, side='right') - 1
    if (indices < 0).any():
        raise ValueError('zero-order hold requested before first sample')
    return data[key][indices]


def analyze(run):
    logs = list(run.rglob('*.ulg'))
    if len(logs) != 1:
        raise ValueError(f'{run}: expected exactly one ULog')
    u = ULog(str(logs[0]))
    if u.msg_info_dict['ver_sw'] != 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4':
        raise ValueError('wrong PX4 commit')
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    base = json.loads((run / 'run_status.json').read_text(encoding='utf-8'))['first_clock_us']
    s, l, truth = (d[x] for x in ['estimator_states', 'vehicle_local_position',
                                 'vehicle_local_position_groundtruth'])
    # This pinned revision uses quaternion 0:4, velocity 4:7, position 7:10.
    # state.pos.z is -AMSL, while local z = -(AMSL - local_origin_alt).
    # If ref_alt is NaN PX4 getEkfGlobalOriginAltitude() returns zero.
    phases = {}
    for name, lo, hi in [('before', 25, 40), ('after', 50, 120)]:
        si = (s['timestamp_sample'] >= base + lo*1e6) & (s['timestamp_sample'] < base + hi*1e6)
        li = (l['timestamp'] >= base + lo*1e6) & (l['timestamp'] < base + hi*1e6)
        t = s['timestamp_sample'][si]
        origin = hold(t, l, 'ref_alt')
        origin = np.where(np.isfinite(origin), origin, 0.)
        z = s['states[9]'][si].astype(float) + origin
        vz = s['states[6]'][si].astype(float)
        ztruth = np.interp(t, truth['timestamp'], truth['z'])
        vztruth = np.interp(t, truth['timestamp'], truth['vz'])
        # Align current output to the delayed EKF sample time for comparison.
        # This is logged-data interpolation, not an exact internal buffer read.
        zoutput = np.interp(t, l['timestamp'], l['z'])
        vzoutput = np.interp(t, l['timestamp'], l['vz'])
        deltas = np.r_[0, np.diff(l['vz_reset_counter'].astype(np.int64)) % 256]
        ev = d.get('estimator_aid_src_ev_vel')
        evi = None if ev is None else ((ev['timestamp'] >= base + lo*1e6) & (ev['timestamp'] < base + hi*1e6))
        phases[name] = dict(
            ekf_z_error_m=metrics(z-ztruth), ekf_vz_error_mps=metrics(vz-vztruth),
            output_z_error_m=metrics(l['z'][li]-np.interp(l['timestamp'][li], truth['timestamp'], truth['z'])),
            output_vz_error_mps=metrics(l['vz'][li]-np.interp(l['timestamp'][li], truth['timestamp'], truth['vz'])),
            aligned_output_minus_ekf_z_m=metrics(zoutput-z),
            aligned_output_minus_ekf_vz_mps=metrics(vzoutput-vz),
            ekf_vz_variance=metrics(s['covariances[5]'][si]),
            ekf_z_variance=metrics(s['covariances[8]'][si]),
            vz_resets=int(deltas[li].sum()),
            reset_delta_vz_sum=float(l['delta_vz'][li & (deltas > 0)].sum()),
            validity={k: dict(true=int(l[k][li].sum()), n=int(li.sum())) for k in ['z_valid', 'v_z_valid']},
            flags={k: np.unique(hold(l['timestamp'][li], d['estimator_status_flags'], k)).astype(int).tolist()
                   for k in ['cs_ev_vel','cs_fake_hgt','cs_baro_hgt','cs_rng_hgt','cs_in_air']},
            ev_vel_aid=None if ev is None else dict(n=int(evi.sum()), fused=int(ev['fused'][evi].sum()),
                rejected=int(ev['innovation_rejected'][evi].sum()),
                last_fuse_age_ms=metrics((ev['timestamp'][evi].astype(float)-ev['time_last_fuse'][evi])/1e3)))
    # Keep plotting data small; full-resolution metrics above retain all samples.
    indices = np.flatnonzero((s['timestamp_sample'] >= base) & (s['timestamp_sample'] < base+120e6))[::10]
    ts = s['timestamp_sample'][indices]
    origin = hold(ts, l, 'ref_alt')
    origin = np.where(np.isfinite(origin), origin, 0.)
    return dict(run=str(run), ulog_sha256=sha256(logs[0]), firmware=u.msg_info_dict['ver_sw'], phases=phases,
                trace=dict(time_s=((ts.astype(float)-base)/1e6).tolist(),
                           ekf_z=(s['states[9]'][indices].astype(float)+origin).tolist(),
                           ekf_vz=s['states[6]'][indices].astype(float).tolist(),
                           output_z=np.interp(ts,l['timestamp'],l['z']).tolist(),
                           output_vz=np.interp(ts,l['timestamp'],l['vz']).tolist()),
                state_mapping='states[6]=delayed NED vz; states[9]=-AMSL; covariance indices 5=vz,8=z',
                limitations='ULog does not expose all internal fuse timers or per-axis updates; output alignment is interpolation')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix', type=Path, required=True)
    p.add_argument('--prior-runs', type=Path, required=True)
    p.add_argument('--archive', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    prior = json.loads(a.prior_runs.read_text(encoding='utf-8'))['runs']
    audit = dict(ulogs=0, records=0, mismatches=[], prior_runs_sha256=sha256(a.prior_runs))
    for item in prior:
        run = a.matrix / Path(item['raw_location']).name
        expected = {item['status']['ulog_files'][0]:item['ulog_sha256'], **item['record_sha256']}
        for name, digest in expected.items():
            file = run / name
            actual = sha256(file) if file.is_file() else None
            if actual != digest:
                audit['mismatches'].append(dict(file=str(file), expected=digest, actual=actual))
            elif name.endswith('.ulg'):
                audit['ulogs'] += 1
            else:
                audit['records'] += 1
    if a.archive:
        digest = sha256(a.archive)
        audit['archive'] = dict(path=str(a.archive), bytes=a.archive.stat().st_size, sha256=digest,
                               matches=digest=='ba3882a9a2198f634c18aea395ae0dbd2ecaabbfacaa275f9c49016ab6e106b5')
    (a.output/'preservation_audit.json').write_text(json.dumps(audit,indent=2)+'\n',encoding='utf-8')
    if audit['mismatches'] or (a.archive and not audit['archive']['matches']):
        raise ValueError('raw evidence differs; stop before reanalysis')
    results = [analyze(a.matrix/f'{name}-seed{seed}')
               for name in ['vertical_aiding', 'vertical_aiding-position-only'] for seed in [11,22,33,44,55]]
    (a.output/'state_output_comparison.json').write_text(json.dumps(dict(runs=results),indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for r in results:
        x=r['phases']['after']
        print(Path(r['run']).name, json.dumps({k:x[k] for k in ['ekf_z_error_m','ekf_vz_error_mps','output_z_error_m','output_vz_error_mps','vz_resets']}),flush=True)


if __name__ == '__main__':
    main()
