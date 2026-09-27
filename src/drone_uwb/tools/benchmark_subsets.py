"""A/B/C/D synthetic checks and static error/coverage figure, file output only."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from benchmark_h80_b import trajectory
from drone_uwb.acquisition.validation import Cycle
from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import BSettings, H80Window, write_json
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy
from drone_uwb.processing.intersections import DSettings, make_candidates as model_d
from drone_uwb.processing.triplets import make_candidates as model_c
from drone_uwb.processing.runner import json_line


def error_metrics(rows, model):
    errors = np.array([np.linalg.norm(np.array(r['models'][model]['xy_m'])-r['truth_xy_m']) for r in rows])
    if not len(errors):
        return dict(count=0)
    return dict(count=len(errors), rmse_m=float(np.sqrt(np.mean(errors**2))),
                p95_m=float(np.quantile(errors, .95)), max_m=float(np.max(errors)))


def simulate(anchors, config):
    all_rows, cases = [], []
    models = ('A', 'B', 'C', 'D')
    for case in ('linear_clean', 'static_noise', 'spikes', 'persistent_bias', 'direction_reversal', 'gap'):
        rng = np.random.default_rng(7)
        bias = np.array([-.14, .20, -.17, -.06])
        window = H80Window(anchors, bias, BSettings(**config['B']))
        records = []
        for k in range(320):
            start = 1_000_000+k*25000
            samples = [start+4000*i for i in range(1, 5)]
            t = (np.array(samples)-1_000_000)/1e6
            end, ref_t = start+20000, (start+20000-1_000_000)/1e6
            row = dict(case=case, seq=k, t_ref_s=ref_t, truth_xy_m=trajectory(ref_t, case)[:2].tolist(),
                       models={m: dict(ok=False, reason='missing_cycle', xy_m=None) for m in models})
            if not (case == 'gap' and 120 <= k < 130):
                pos = trajectory(t, case)
                raw = np.linalg.norm(pos-anchors, axis=1)+bias
                noise = 0. if case == 'linear_clean' else .02 if case == 'direction_reversal' else .03
                raw += rng.normal(0, noise, 4)
                if case == 'spikes' and k % 20 == 10:
                    raw[1] += .60
                if case == 'persistent_bias' and 2 <= ref_t <= 6:
                    raw[1] += .35
                cycle = Cycle(k, start, end, 15, raw, samples, ['ok']*4, list(range(4)))
                a = solve_uniform_xy(anchors, raw-bias, pos[:, 2], **config['A'])
                b = window.process(cycle, k+1)
                c = model_c(anchors, raw-bias, pos[:, 2], t_ref_us=end, settings=config['C'])
                d = model_d(anchors, raw-bias, pos[:, 2], t_ref_us=end, settings=DSettings(**config['D']))
                row['models']['A'] = dict(ok=a.ok, reason=a.reason, xy_m=a.xy_m)
                for m, f in [('B', b), ('C', c), ('D', d)]:
                    row['models'][m] = {key: f[key] for key in ('ok', 'reason', 'xy_m')}
                row['D_pair_reasons'] = dict(Counter(p['reason'] for p in d['pair_candidates']))
            records.append(row)
        case_summary = dict(case=case, scheduled_ticks=320, models={}, paired_with_A={})
        for model in models:
            valid = [r for r in records if r['models'][model]['ok']]
            case_summary['models'][model] = dict(error_metrics(valid, model), coverage=len(valid)/320,
                                                 reasons=dict(Counter(r['models'][model]['reason'] for r in records)))
            paired = [r for r in valid if r['models']['A']['ok']]
            case_summary['paired_with_A'][model] = dict(count=len(paired), A=error_metrics(paired, 'A'),
                                                       candidate=error_metrics(paired, model))
        common = [r for r in records if all(r['models'][m]['ok'] for m in models)]
        case_summary['common'] = dict(count=len(common), models={m: error_metrics(common, m) for m in models})
        cases.append(case_summary)
        all_rows.extend(records)
    return dict(scope='synthetic_model_comparison', seed=7, duration_s=8, rate_hz=40,
                height_source='exact_simulator_per_sample', sensor_mounting_validated=False,
                external_output_allowed=False, cases=cases), all_rows


def plot(run_dir, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summary = json.loads((run_dir/'summary.json').read_text(encoding='utf-8'))
    rows = [json.loads(s) for s in (run_dir/'results.jsonl').read_text(encoding='utf-8').splitlines()]
    ref = np.array(summary['evaluation_reference_xyz_m'][:2])
    fig, (ax, coverage) = plt.subplots(1, 2, figsize=(13, 4), gridspec_kw={'width_ratios': [3, 1]})
    colors = {'A': '#6d7886', 'B': '#007f72', 'C': '#794db0', 'D': '#cf7319'}
    t = [r['t_rel_s'] for r in rows]
    for m in ('A', 'B', 'C', 'D'):
        errors = [np.linalg.norm(np.array(r['models'][m]['xy_m'])-ref)*100 if r['models'][m]['ok'] else np.nan for r in rows]
        ax.plot(t, errors, lw=.8, alpha=.85, color=colors[m], label=m)
    lo, hi = summary['reported_excursion_seq_interval']
    span = [r['t_rel_s'] for r in rows if r['seq'] is not None and lo <= r['seq'] <= hi]
    ax.axvspan(min(span), max(span), color='#ccbb88', alpha=.2)
    ax.set(title='Measured static capture: A / B / C / D', xlabel='Host receive elapsed time (s)', ylabel='XY reference error (cm)')
    ax.legend(ncol=4)
    models = summary['groups']['all']['models']
    coverage.bar(list(colors), [models[m]['coverage']*100 for m in colors], color=list(colors.values()))
    for i, m in enumerate(colors):
        coverage.text(i, models[m]['coverage']*100+1, f'{models[m]["count"]}/2540', ha='center', fontsize=8)
    coverage.set(title='Output coverage', ylabel='Received cycles (%)', ylim=(0, 110))
    for axis in (ax, coverage):
        axis.grid(axis='y', alpha=.2)
    fig.text(.5, .02, 'Manual static reference. D rejects failed intersections; compare paired errors AND coverage.', ha='center', fontsize=9)
    fig.tight_layout(rect=[0, .06, 1, 1])
    for extension in ('png', 'pdf'):
        fig.savefig(output/('subsets_comparison.'+extension), dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads((args.run_dir/'config.json').read_text(encoding='utf-8'))
    anchors = np.array(json.loads((args.run_dir/'anchors.json').read_text(encoding='utf-8'))['anchors_xyz_m'])
    summary, rows = simulate(anchors, config)
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output/'synthetic_summary.json', summary)
    (args.output/'synthetic_results.jsonl').write_text(''.join(map(json_line, rows)), encoding='utf-8')
    plot(args.run_dir, args.output)
    write_json(args.output/'benchmark_manifest.json', dict(script_sha256=digest(__file__),
              trajectory_script_sha256=digest(Path(__file__).with_name('benchmark_h80_b.py')),
              outputs={p.name: digest(p) for p in sorted(args.output.iterdir())}))
    print(json.dumps([{c['case']: {m: {'count':v['count'], 'rmse_m':v.get('rmse_m')} for m,v in c['models'].items()}} for c in summary['cases']]))


if __name__ == '__main__':
    main()
