"""Offline A/B/C/D comparison with paired ticks, failures and candidate lineage."""
import argparse
from collections import Counter
import csv
from itertools import combinations
import json
from pathlib import Path
import platform
from time import perf_counter_ns

import numpy as np

from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import BSettings, calculate as calculate_ab, read_events, write_json
from drone_uwb.processing.experiments.static_a import position_metrics
from drone_uwb.processing.solvers.intersections import DSettings, make_candidates as model_d
from drone_uwb.processing.solvers.triplets import make_candidates as model_c
from drone_uwb.processing.runner import json_line, require_finite_json


def calculate(events, anchors, height_m, bias_m, config):
    events = list(events)
    results, timings, event_counts = calculate_ab(events, anchors, height_m, bias_m, config['A'], BSettings(**config['B']))
    by_line = {e['input_line']: e for e in events}
    models = ['A', 'B']+config['models']
    d_settings = DSettings(**config['D']) if 'D' in models else None
    for row, timing in zip(results, timings):
        cycle = by_line[row['input_line']]['cycle']
        row.update(model_ids=models, variant='same_input_model_comparison')
        for model in config['models']:
            fit = dict(ok=False, reason=row['reason'], xy_m=None, candidates=[])
            timing[model+'_ms'] = None
            if cycle is not None:
                if len(cycle.indices) != 4:
                    fit['reason'] = 'four_anchors_required'
                else:
                    corrected = cycle.raw-np.asarray(bias_m)
                    kwargs = dict(t_ref_us=cycle.end_us, obs_ids=[f'{row["input_line"]}:A{i+1}' for i in range(4)])
                    start = perf_counter_ns()
                    fit = (model_c(anchors, corrected, height_m, settings=config['C'], **kwargs) if model == 'C'
                           else model_d(anchors, corrected, height_m, settings=d_settings, **kwargs))
                    timing[model+'_ms'] = (perf_counter_ns()-start)/1e6
            row['models'][model] = fit
    return results, timings, event_counts


def evaluate(results, reference_xy, excursion_interval, models):
    ref = np.asarray(reference_xy, float)
    if ref.shape != (2,) or not np.isfinite(ref).all():
        raise ValueError('invalid_evaluation_reference')
    lo, hi = excursion_interval
    inside = lambda r: r['seq'] is not None and lo <= r['seq'] <= hi
    subsets = dict(all=results, reported_excursion_span=[r for r in results if inside(r)],
                   outside_reported_span=[r for r in results if not inside(r)],
                   full_B_window_elapsed=[r for r in results if r['models']['B'].get('full_window_elapsed')])
    def metrics(rows, model):
        return position_metrics([r['models'][model]['xy_m'] for r in rows], ref)
    output = {}
    for name, rows in subsets.items():
        group = dict(input_ticks=len(rows), models={}, pairwise={})
        for model in models:
            valid = [r for r in rows if r['models'][model]['ok']]
            group['models'][model] = dict(metrics(valid, model), coverage=len(valid)/len(rows) if rows else None,
                                         reasons=dict(Counter(r['models'][model]['reason'] for r in rows)))
        for one, two in combinations(models, 2):
            paired = [r for r in rows if r['models'][one]['ok'] and r['models'][two]['ok']]
            first, second = metrics(paired, one), metrics(paired, two)
            group['pairwise'][one+'_'+two] = dict(count=len(paired), models={one: first, two: second},
                                                rmse_reduction_fraction=1-second['rmse_m']/first['rmse_m']
                                                if paired and first['rmse_m'] > 1e-12 else None)
        common = [r for r in rows if all(r['models'][m]['ok'] for m in models)]
        group['all_model_common'] = dict(count=len(common), models={m: metrics(common, m) for m in models})
        output[name] = group
    return output


def run(config_path, root, output):
    root, output = Path(root), Path(output)
    config = json.loads(Path(config_path).read_text(encoding='utf-8'))
    require_finite_json(config)
    if config['models'] not in (['C'], ['C', 'D']):
        raise ValueError('unsupported_model_selection')
    if config['target_mode'] != 'reference_slant':
        raise ValueError('unsupported_B_variant')
    loaded = {}
    for name, entry in config['files'].items():
        path = root/entry['path']
        if digest(path) != entry['sha256']:
            raise ValueError('input_hash_mismatch:'+name)
        loaded[name] = path
    calibration = json.loads(loaded['locked_calibration'].read_text(encoding='utf-8'))
    if calibration['source_sha256'] != digest(loaded['calibration_raw']):
        raise ValueError('calibration_source_mismatch')
    if calibration['source_sha256'] == digest(loaded['evaluation_raw']):
        raise ValueError('calibration_evaluation_must_be_separate')
    layout = json.loads(loaded['anchors'].read_text(encoding='utf-8'))
    if (layout['anchor_order'] != ['A1', 'A2', 'A3', 'A4'] or layout['units'] != 'm'
            or layout['coordinate_frame'] != 'uwb_map'):
        raise ValueError('invalid_anchor_map')
    anchors = np.asarray(layout['anchors_xyz_m'], float)
    metadata = json.loads(loaded['evaluation_metadata'].read_text(encoding='utf-8'))
    ref = np.array([metadata['reference_position_m'][k] for k in ('x', 'y', 'z')], float)
    if not np.isfinite(ref).all():
        raise ValueError('invalid_evaluation_reference')
    annotation = json.loads(loaded['excursion_annotation'].read_text(encoding='utf-8'))
    interval = [min(e['first_seq'] for e in annotation['events']), max(e['last_seq'] for e in annotation['events'])]
    results, timings, events = calculate(read_events(loaded['evaluation_raw'], config['tag_id'], config['firmware']),
                                        anchors, ref[2], calibration['bias_m'], config)
    models = ['A', 'B']+config['models']
    ab_matches = None
    if 'prior_AB_results' in loaded:
        previous = [json.loads(s) for s in loaded['prior_AB_results'].read_text(encoding='utf-8').splitlines()]
        ab_matches = len(previous) == len(results) and all(
            old['input_line'] == new['input_line'] and all(old['models'][m] == new['models'][m] for m in ('A', 'B'))
            for old, new in zip(previous, results))
        if not ab_matches:
            raise ValueError('A_or_B_changed_from_locked_baseline')
    candidate_reasons = {m: dict(Counter(c['reason'] for r in results for c in r['models'][m]['candidates']))
                         for m in config['models']}
    pair_reasons = dict(Counter(p['reason'] for r in results
                               for p in r['models'].get('D', {}).get('pair_candidates', [])))
    summary = dict(scope='offline_static_subset_comparison', source='measured_uwb', models=models,
                   variants=dict(A='uniform_slant', B='reference_slant', C='uniform4', D='strict_uniform4'),
                   evaluation_reference_xyz_m=ref.tolist(), reference_independently_verified=False,
                   candidate_bias_m=calibration['bias_m'], height_source_C_D='manual_capture_reference',
                   groups=evaluate(results, ref[:2], interval, models), event_counts=events,
                   candidate_reasons=candidate_reasons, D_pair_reasons=pair_reasons,
                   prior_A_B_exactly_preserved=ab_matches, reported_excursion_seq_interval=interval,
                   extra_range_gate=False, extra_position_gate=False, radius_expansion=False,
                   external_output_allowed=False, flight_valid=False,
                   limitation='One reported static point, locked single-point bias. Development data, not independent flight validation.')
    timing_summary = {}
    for model in models:
        values = [r[model+'_ms'] for r in timings if r.get(model+'_ms') is not None]
        timing_summary[model] = (dict(count=len(values), median_ms=float(np.median(values)),
                                      p95_ms=float(np.quantile(values, .95)), max_ms=float(np.max(values)))
                                 if values else dict(count=0))
    output.mkdir(parents=True, exist_ok=False)
    for name, obj in [('summary.json', summary), ('config.json', config), ('anchors.json', layout),
                      ('calibration.json', calibration), ('timing_summary.json', timing_summary)]:
        write_json(output/name, obj)
    (output/'results.jsonl').write_text(''.join(map(json_line, results)), encoding='utf-8')
    (output/'timings.jsonl').write_text(''.join(map(json_line, timings)), encoding='utf-8')
    with (output/'positions.csv').open('x', encoding='utf-8', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['input_line', 'seq', 't_rel_s', 'model', 'ok', 'reason', 'x_m', 'y_m', 'error_m', 'all_model_common'])
        for row in results:
            common = all(row['models'][m]['ok'] for m in models)
            for model in models:
                fit = row['models'][model]
                xy = fit['xy_m'] if fit['ok'] else [None, None]
                error = float(np.linalg.norm(np.asarray(xy)-ref[:2])) if fit['ok'] else None
                writer.writerow([row['input_line'], row['seq'], row['t_rel_s'], model, fit['ok'], fit['reason'], *xy, error, common])
    package = Path(__file__).resolve().parents[2]
    manifest = dict(inputs=config['files'], config_sha256=digest(config_path),
                    code_sha256={str(p.relative_to(package)): digest(p) for p in sorted(package.rglob('*.py'))},
                    output_sha256={p.name: digest(p) for p in sorted(output.iterdir())},
                    runtime=dict(python=platform.python_version(), numpy=np.__version__),
                    deterministic_files=['results.jsonl', 'summary.json', 'positions.csv'], device_output=False)
    write_json(output/'manifest.json', manifest)
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    summary = run(options.config, options.root, options.output)
    print(json.dumps(summary['groups']['all']['models'], ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
