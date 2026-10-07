"""Repeatable synthetic A/B scenarios and static measured comparison plot.

Synthetic A receives exact simulator heights at the sample times. This checks
algorithm response, not ToF/attitude synchronization or measured flight latency.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from drone_uwb.acquisition.validation import Cycle
from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import BSettings, H80Window, write_json
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy
from drone_uwb.processing.runner import json_line


def trajectory(t, case):
    t = np.asarray(t)
    if case in ('linear_clean', 'gap'):
        x, y = .7+.25*t, 1.4+.07*t
    elif case == 'direction_reversal':
        x, y = .7+.6*np.minimum(t, 8.-t), 1.8+0*t
    else:
        x, y = .7+0*t, 1.8+0*t
    return np.stack([x, y, 1.1+.02*t], axis=-1)


def metrics(values):
    values = np.array(values)
    if not len(values):
        return dict(count=0)
    return dict(count=len(values), rmse_m=float(np.sqrt(np.mean(values**2))),
                p95_m=float(np.quantile(values, .95)), max_m=float(values.max()))


def simulate(anchors, config):
    cases, records = [], []
    for case in ('linear_clean', 'static_noise', 'spikes', 'persistent_bias', 'direction_reversal', 'gap'):
        rng = np.random.default_rng(7)
        bias = np.array([-.14, .20, -.17, -.06])
        window = H80Window(anchors, bias, BSettings(**config['B']))
        current, reversal_delay = [], None
        for k in range(320):
            start = 1_000_000+k*25000
            sample_us = [start+4000*i for i in range(1, 5)]
            t = (np.array(sample_us)-1_000_000)/1e6
            end = start+20000
            ref_t = (end-1_000_000)/1e6
            truth = trajectory(ref_t, case)[:2]
            row = dict(case=case, seq=k, t_ref_s=ref_t, truth_xy_m=truth.tolist(),
                       A_xy_m=None, B_xy_m=None, A_reason='missing_cycle', B_reason='missing_cycle')
            if not (case == 'gap' and 120 <= k < 130):
                positions = trajectory(t, case)
                raw = np.linalg.norm(positions-anchors, axis=1)+bias
                noise = 0. if case == 'linear_clean' else .02 if case == 'direction_reversal' else .03
                raw += rng.normal(0, noise, 4)
                if case == 'spikes' and k % 20 == 10:
                    raw[1] += .60
                if case == 'persistent_bias' and 2 <= ref_t <= 6:
                    raw[1] += .35
                cycle = Cycle(k, start, end, 15, raw, sample_us, ['ok']*4, list(range(4)))
                a = solve_uniform_xy(anchors, raw-bias, positions[:, 2], **config['A'])
                b = window.process(cycle, k+1)
                row.update(A_xy_m=a.xy_m if a.ok else None, B_xy_m=b['xy_m'],
                           A_reason=a.reason, B_reason=b['reason'], reset_reason=b['reset_reason'])
                if case == 'direction_reversal' and b['ok']:
                    row['B_vx_m_s'] = b['velocity_m_s'][0]
                    if ref_t >= 4 and reversal_delay is None and row['B_vx_m_s'] < 0:
                        reversal_delay = ref_t-4
            current.append(row)
        paired = [r for r in current if r['A_xy_m'] is not None and r['B_xy_m'] is not None]
        result = dict(case=case, scheduled_ticks=320,
                      A_coverage=sum(r['A_xy_m'] is not None for r in current)/320,
                      B_coverage=sum(r['B_xy_m'] is not None for r in current)/320,
                      paired_count=len(paired), paired={})
        for model in ('A', 'B'):
            errors = [np.linalg.norm(np.array(r[model+'_xy_m'])-r['truth_xy_m']) for r in paired]
            result['paired'][model] = metrics(errors)
        if case == 'direction_reversal':
            result['B_velocity_sign_change_after_turn_s'] = reversal_delay
            result['delay_definition'] = 'First fitted vx < 0 after t=4s; noisy scenario, not end-to-end transport latency.'
            near_turn = [r for r in paired if 4 <= r['t_ref_s'] <= 4.8]
            result['turn_window'] = {m: metrics([np.linalg.norm(np.array(r[m+'_xy_m'])-r['truth_xy_m'])
                                                for r in near_turn]) for m in ('A', 'B')}
        if case == 'gap':
            recovered = next(r for r in current if r['seq'] >= 130 and r['B_xy_m'] is not None)
            result['recovery_after_first_return_s'] = recovered['t_ref_s']-current[130]['t_ref_s']
        cases.append(result)
        records.extend(current)
    return dict(scope='synthetic_algorithm_check', seed=7, settings=config['B'], duration_s=8,
                scheduled_rate_hz=40, A_height_source='exact_simulator_height_per_range',
                noise_sigma_m=dict(linear_clean=0, direction_reversal=.02, other=.03),
                persistent_A2_bias_m=.35, spike_A2_m=.60,
                external_output_allowed=False, cases=cases), records


def plot(run_dir, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    results = [json.loads(line) for line in (run_dir/'results.jsonl').read_text(encoding='utf-8').splitlines()]
    summary = json.loads((run_dir/'summary.json').read_text(encoding='utf-8'))
    reference = np.array(summary['evaluation_reference_xyz_m'][:2])
    fig, (ax, cdf) = plt.subplots(1, 2, figsize=(12, 4), gridspec_kw={'width_ratios': [2, 1]})
    for model, color in [('A', '#52667a'), ('B', '#007f72')]:
        valid = [r for r in results if r['models'][model]['ok']]
        error = np.array([np.linalg.norm(np.array(r['models'][model]['xy_m'])-reference)*100 for r in valid])
        ax.plot([r['t_rel_s'] for r in valid], error, color=color, lw=.85, label=model)
        paired = [r for r in results if all(r['models'][m]['ok'] for m in ('A', 'B'))]
        err = sorted(np.linalg.norm(np.array(r['models'][model]['xy_m'])-reference)*100 for r in paired)
        cdf.plot(err, np.arange(1, len(err)+1)/len(err), color=color, label=model)
    lo, hi = summary['reported_excursion_seq_interval']
    span = [r['t_rel_s'] for r in results if r['seq'] is not None and lo <= r['seq'] <= hi]
    ax.axvspan(min(span), max(span), color='#ddba80', alpha=.3, label='Reported excursion span')
    ax.set(xlabel='Host receive elapsed time (s)', ylabel='XY reference error (cm)', title='Measured static capture: A vs B (H80)')
    cdf.set(xlabel='XY reference error (cm)', ylabel='Cumulative fraction', title='Paired ticks only')
    for axis in (ax, cdf):
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    fig.text(.5, .015, 'Manual reference; single-point candidate calibration. Not a flight accuracy guarantee.', ha='center', fontsize=9)
    fig.tight_layout(rect=[0, .055, 1, 1])
    for extension in ('png', 'pdf'):
        fig.savefig(output/('h80_b_comparison.'+extension), dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads((args.run_dir/'config.json').read_text(encoding='utf-8'))
    anchors = np.array(json.loads((args.run_dir/'anchors.json').read_text(encoding='utf-8'))['anchors_xyz_m'])
    summary, records = simulate(anchors, config)
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output/'synthetic_summary.json', summary)
    (args.output/'synthetic_results.jsonl').write_text(''.join(map(json_line, records)), encoding='utf-8')
    plot(args.run_dir, args.output)
    write_json(args.output/'benchmark_manifest.json', dict(script_sha256=digest(__file__),
                                                         config_sha256=digest(args.run_dir/'config.json'),
                                                         outputs={p.name: digest(p) for p in args.output.iterdir()}))
    print(json.dumps(summary['cases'], ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
