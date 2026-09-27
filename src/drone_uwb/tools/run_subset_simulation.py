"""Run the existing six UWB simulation cases and plot their synthetic outputs.

No measured captures, ROS topics, flight controller or physics engine are used.
The unchanged benchmark_subsets.simulate owns the scenarios and model calls.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import platform

import numpy as np

from benchmark_subsets import simulate
from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import write_json
from drone_uwb.processing.runner import json_line


COLORS = dict(A='#687585', B='#00836b', C='#8c46b8', D='#dd7c16')
TITLES = dict(linear_clean='Linear motion / no noise', static_noise='Stationary XY / range noise',
              spikes='A2 spikes / +0.60 m', persistent_bias='A2 bias / +0.35 m from 2 to 6 s',
              direction_reversal='Direction reversal / turn at 4 s', gap='Missing 10 cycles / 0.25 s')


def positions(rows, model):
    return np.asarray([r['models'][model]['xy_m'] if r['models'][model]['ok']
                       else [np.nan, np.nan] for r in rows])


def describe_errors(summary, rows):
    """Bias and spread on each model's own valid outputs; keep paired metrics separate."""
    output = {}
    for case in summary['cases']:
        current = [r for r in rows if r['case'] == case['case']]
        truth = np.asarray([r['truth_xy_m'] for r in current])
        models = {}
        for model in COLORS:
            differences = positions(current, model)-truth
            differences = differences[np.isfinite(differences).all(axis=1)]
            models[model] = dict(count=len(differences),
                                 mean_error_xy_m=np.mean(differences, axis=0).tolist() if len(differences) else None,
                                 std_error_xy_m=np.std(differences, axis=0).tolist() if len(differences) else None)
        reasons = Counter()
        for row in current:
            reasons.update(row.get('D_pair_reasons', {}))
        output[case['case']] = dict(models=models, D_slot_reasons=dict(reasons))
    return output


def plot(summary, rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(6, 2, figsize=(13, 15), gridspec_kw={'width_ratios': [3, 1.3]})
    for (error_ax, coverage_ax), case in zip(axes, summary['cases']):
        current = [r for r in rows if r['case'] == case['case']]
        time = np.asarray([r['t_ref_s'] for r in current])
        truth = np.asarray([r['truth_xy_m'] for r in current])
        for model, color in COLORS.items():
            error = np.linalg.norm(positions(current, model)-truth, axis=1)*100
            error_ax.plot(time, error, color=color, lw=1, alpha=.85, label=model)
        if case['case'] == 'persistent_bias':
            error_ax.axvspan(2, 6, color='#e5cd9b', alpha=.25)
        elif case['case'] == 'direction_reversal':
            error_ax.axvline(4, color='#333333', ls=':', lw=1)
        elif case['case'] == 'gap':
            missing = [r['t_ref_s'] for r in current if r['models']['A']['reason'] == 'missing_cycle']
            error_ax.axvspan(min(missing)-.0125, max(missing)+.0125, color='#d9d9d9', alpha=.6)
        error_ax.set(title=TITLES[case['case']], xlabel='Simulation time (s)', ylabel='XY error (cm)', xlim=(0, 8))
        error_ax.grid(alpha=.2)
        if case['case'] == summary['cases'][0]['case']:
            error_ax.legend(ncol=4, loc='upper left')
        for i, model in enumerate(COLORS):
            value = case['models'][model]['coverage']*100
            coverage_ax.barh(i, value, color=COLORS[model], height=.65)
            coverage_ax.text(value+1, i, f'{value:.1f}%', va='center', fontsize=9)
        coverage_ax.set(yticks=range(4), yticklabels=list(COLORS), xlim=(0, 123),
                        xticks=[0, 50, 100], xlabel='Scheduled cycles (%)', title='Output coverage')
        coverage_ax.invert_yaxis()
        coverage_ax.grid(axis='x', alpha=.2)
    fig.suptitle('UWB algorithm simulation: A / B / C / D', fontsize=16)
    fig.text(.5, .009, '8 s per case, 40 Hz, seed 7. Gaps are missing outputs. Exact simulator heights; no flight physics.',
             ha='center', fontsize=10)
    fig.tight_layout(rect=[0, .025, 1, .97])
    for ext in ('png', 'pdf'):
        fig.savefig(output/('simulation_errors.'+ext), dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for column, name in enumerate(('linear_clean', 'direction_reversal')):
        current = [r for r in rows if r['case'] == name]
        time = [r['t_ref_s'] for r in current]
        truth = np.asarray([r['truth_xy_m'] for r in current])
        axes[0, column].plot(truth[:, 0], truth[:, 1], 'k--', lw=2, label='Truth', zorder=10)
        axes[1, column].plot(time, truth[:, 0], 'k--', lw=2, label='Truth', zorder=10)
        for model, color in COLORS.items():
            xy = positions(current, model)
            axes[0, column].plot(xy[:, 0], xy[:, 1], color=color, lw=.9, alpha=.8, label=model)
            axes[1, column].plot(time, xy[:, 0], color=color, lw=.9, alpha=.8, label=model)
        axes[0, column].set(title=TITLES[name], xlabel='X (m)', ylabel='Y (m)')
        axes[0, column].set_aspect('equal', adjustable='datalim')
        axes[1, column].set(xlabel='Simulation time (s)', ylabel='X position (m)', xlim=(0, 8))
        if name == 'direction_reversal':
            axes[1, column].axvline(4, color='#333333', ls=':', lw=1)
    for axis in axes.flat:
        axis.grid(alpha=.2)
    axes[0, 0].legend(ncol=5, fontsize=9)
    fig.suptitle('Known path and estimated positions', fontsize=16)
    fig.text(.5, .012, 'Missing estimates remain gaps; no previous position is substituted. A and C often overlap.',
             ha='center', fontsize=10)
    fig.tight_layout(rect=[0, .035, 1, .96])
    for ext in ('png', 'pdf'):
        fig.savefig(output/('simulation_paths.'+ext), dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--anchors', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8'))
    layout = json.loads(args.anchors.read_text(encoding='utf-8'))
    if layout['anchor_order'] != ['A1', 'A2', 'A3', 'A4'] or layout['units'] != 'm':
        raise ValueError('unsupported_anchor_layout')
    args.output.mkdir(parents=True, exist_ok=False)
    summary, rows = simulate(np.asarray(layout['anchors_xyz_m']), config)
    write_json(args.output/'synthetic_summary.json', summary)
    (args.output/'synthetic_results.jsonl').write_text(''.join(map(json_line, rows)), encoding='utf-8')
    write_json(args.output/'diagnostics.json', describe_errors(summary, rows))
    write_json(args.output/'model_settings.json', {m: config[m] for m in COLORS})
    write_json(args.output/'anchors.json', layout)
    write_json(args.output/'simulation_conditions.json', dict(
        duration_s=8, rate_hz=40, seed=7, sample_offsets_us=[4000, 8000, 12000, 16000], reference_offset_us=20000,
        z_m='1.1 + 0.02 * time_s', height_source='exact_simulator_per_sample',
        range_noise_sigma_m=dict(linear_clean=0, direction_reversal=.02, other=.03),
        injected_and_subtracted_bias_m=[-.14, .20, -.17, -.06],
        spike=dict(anchor='A2', additional_m=.60, cycle_rule='seq % 20 == 10'),
        persistent_bias=dict(anchor='A2', additional_m=.35, interval_s=[2, 6]),
        gap=dict(missing_seq_inclusive=[120, 129], scheduled_duration_s=.25),
        trajectories=dict(linear_clean='x=.7+.25*t; y=1.4+.07*t', gap='x=.7+.25*t; y=1.4+.07*t',
                          direction_reversal='x=.7+.6*min(t,8-t); y=1.8', other='x=.7; y=1.8'),
        physics_engine=False, tof_sensor_model=False, external_output_allowed=False))
    plot(summary, rows, args.output)
    package = Path(__file__).resolve().parents[1]/'drone_uwb'
    write_json(args.output/'manifest.json', dict(
        source='synthetic', config_sha256=digest(args.config), anchors_sha256=digest(args.anchors),
        scripts_sha256={p.name: digest(p) for p in (Path(__file__), Path(__file__).with_name('benchmark_subsets.py'),
                                                   Path(__file__).with_name('benchmark_h80_b.py'))},
        package_sha256={str(p.relative_to(package)): digest(p) for p in sorted(package.rglob('*.py'))},
        output_sha256={p.name: digest(p) for p in sorted(args.output.iterdir())},
        runtime=dict(python=platform.python_version(), numpy=np.__version__),
        external_output_allowed=False))
    print(json.dumps([dict(case=c['case'], **{m: dict(coverage=c['models'][m]['coverage'],
                                                       paired_A_rmse_m=c['paired_with_A'][m]['A']['rmse_m'],
                                                       paired_model_rmse_m=c['paired_with_A'][m]['candidate']['rmse_m'])
                                             for m in ('C', 'D')}) for c in summary['cases']], ensure_ascii=False))


if __name__ == '__main__':
    main()
