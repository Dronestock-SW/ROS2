"""Verify preserved diagnostic traces and create reviewable metrics and figures.

Checks establish evidence integrity and causal-intervention isolation. They are
not vehicle acceptance thresholds. Existing logs and JSON files stay untouched.
"""
import argparse
import json
from pathlib import Path
import re

import numpy as np

from analyze_lidar_state_output import sha256
from run_lidar_vertical_probe import analyze


def samples(path):
    rows = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('SAMPLE '):
            payload = re.sub(r'(?<=:)-?nan(?=[,}])', 'NaN', line[7:])
            payload = re.sub(r'(?<=:)(-?)inf(?=[,}])', r'\1Infinity', payload)
            rows.append(json.loads(payload))
    if len(rows) != 12000:
        raise ValueError(f'incomplete trace: {path}')
    return rows


def compare(left, right, keys):
    if len(left) != len(right):
        raise ValueError('different sample count')
    for key in keys:
        a, b = (np.array([r[key] for r in rows]) for rows in (left, right))
        if not np.array_equal(a, b, equal_nan=True):
            raise ValueError(f'intervention isolation mismatch: {key}')
    return dict(samples=len(left), fields=list(keys), identical=True,
                precision='exact equality of printed values (9 significant digits)')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix', type=Path, required=True)
    p.add_argument('--stock-check', type=Path, required=True)
    p.add_argument('--state-output', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    prior = json.loads((a.matrix/'comparison.json').read_text(encoding='utf-8'))
    if len(prior['runs']) != 70:
        raise ValueError('expected the complete 7x5x2 matrix')
    results, traces = [], []
    for old in prior['runs']:
        path = a.matrix/Path(old['raw_file']).name
        if sha256(path) != old['raw_sha256']:
            raise ValueError(f'raw hash mismatch: {path}')
        row = {k: old[k] for k in ('mode', 'seed', 'tilt_deg')}
        row.update(raw_file=path.name, **analyze(path))
        trace = row.pop('trace')
        for phase, count in (('before', 1500), ('after', 7000)):
            if row['phases'][phase]['n'] != count:
                raise ValueError('phase sample denominator mismatch')
        if old['seed'] == 11:
            traces.append(dict(mode=old['mode'], tilt_deg=old['tilt_deg'], trace=trace))
        results.append(row)
    keys = ['t','delayed_us','ekf_z','ekf_vz','ekf_vx','ekf_vy','vz_var','z_var',
            'vz_reset','z_reset','internal_z_valid','internal_vz_valid','ev_vel',
            'fake_hgt','baro','aux_last_fuse','ev_last_fuse']
    pairs = []
    for tilt in (0, 20):
        for seed in (11, 22, 33, 44, 55):
            base = samples(a.matrix/f'stock-tilt{tilt}-seed{seed}.log')
            other = samples(a.matrix/f'no_output_reset-tilt{tilt}-seed{seed}.log')
            pairs.append(dict(tilt_deg=tilt, seed=seed, **compare(base, other, keys)))
    path = a.stock_check/'stock-tilt0-seed11.log'
    stock_manifest = json.loads((a.stock_check/'comparison.json').read_text(encoding='utf-8'))
    if sha256(path) != stock_manifest['runs'][0]['raw_sha256']:
        raise ValueError('stock check hash mismatch')
    instrumented = samples(a.matrix/'stock-tilt0-seed11.log')
    neutrality = compare(instrumented, samples(path), instrumented[0].keys())
    summary = {k: v for k, v in prior.items() if k != 'runs'}
    summary.update(runs=results, analysis_note=(
        'Recomputed from unchanged raw logs. Fusion updates count timestamp changes, '
        'not unique held values; the archived first JSON may include one held value.'),
        original_comparison_sha256=sha256(a.matrix/'comparison.json'))
    checks = dict(raw_hashes_verified=70, samples_per_run=12000,
                  total_samples=840000, phase_denominators=dict(before=1500,after=7000),
                  uninstrumented_stock_equivalence=neutrality,
                  no_output_reset_internal_equivalence=pairs,
                  acceptance_claim=False)
    for name, obj in [('probe_metrics.json', summary), ('verification.json', checks),
                      ('probe_trace_seed11.json', dict(runs=traces))]:
        (a.output/name).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n',encoding='utf-8')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
    fig, axes = plt.subplots(2, 2, figsize=(12,7), constrained_layout=True)
    for row, tilt in enumerate((0, 20)):
        for mode in ('stock','no_vreset','no_output_reset','position_only','aux2d'):
            trace = next(r['trace'] for r in traces if r['tilt_deg']==tilt and r['mode']==mode)
            style = '--' if mode == 'no_output_reset' else '-'
            for col, field in enumerate(('ekf_z','out_vz')):
                axes[row,col].plot(trace['t'],trace[field],style,label=mode,linewidth=1.2)
        for col, label in enumerate(('Internal EKF z (m, down)','Output vz (m/s, down)')):
            ax = axes[row,col]
            ax.axvline(40,color='black',linestyle=':',label='Height removed')
            ax.set(xlim=(20,120),xlabel='Simulation time (s)',ylabel=label,title=f'Pitch {tilt} deg, seed 11')
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Stationary synthetic EKF-library intervention; not SIH or flight')
    fig.savefig(a.output/'probe_state_output.png',dpi=170)
    plt.close(fig)

    old = json.loads(a.state_output.read_text(encoding='utf-8'))
    fig, axes = plt.subplots(2,1,figsize=(10,7),sharex=True,constrained_layout=True)
    for mode, style in [('vertical_aiding-seed11','-'),('vertical_aiding-position-only-seed11','--')]:
        trace=next(r['trace'] for r in old['runs'] if Path(r['run']).name==mode)
        label='3D EV' if 'position-only' not in mode else 'Position only'
        for index,(internal,output) in enumerate([('ekf_z','output_z'),('ekf_vz','output_vz')]):
            axes[index].plot(trace['time_s'],trace[internal],style,label=label+' internal',linewidth=1.5)
            axes[index].plot(trace['time_s'],trace[output],style,label=label+' output',alpha=.7,linewidth=.9)
    for ax,label in zip(axes,['z (m, down)','vz (m/s, down)']):
        ax.axvline(40,color='black',linestyle=':')
        ax.set(xlim=(20,120),ylabel=label)
        ax.legend(fontsize=8)
    axes[1].set_xlabel('Simulation time (s); output interpolated to delayed sample time')
    fig.suptitle('Preserved stationary DISARM SIH, seed 11; no new SIH run')
    fig.savefig(a.output/'preserved_sih_state_output.png',dpi=170)
    plt.close(fig)
    print(json.dumps(checks,indent=2),flush=True)


if __name__ == '__main__':
    main()
