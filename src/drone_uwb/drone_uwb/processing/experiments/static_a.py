"""Offline A trial: estimate single-point bias on one capture, test another.

Capture-reference height is a manual measurement for this static experiment.
It is not a ToF sample. No live input, time fusion, ROS node, or device output.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import csv
import json
from pathlib import Path
import platform
import shutil

import numpy as np

from drone_uwb.contracts.protocol import InvalidSample, decode_line, integer
from drone_uwb.acquisition.validation import InputValidator, InvalidInput
from drone_uwb.processing.runner import json_line, require_finite_json
from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy


def read_capture(path, tag_id, firmware):
    """Check a whole file session before replay; status is metadata, not a live gate."""
    events, rows = [], []
    for number, line in enumerate(Path(path).read_bytes().splitlines(), 1):
        try:
            event = decode_line(line)
            require_finite_json(event)
            if not isinstance(event.get('message'), dict):
                raise InvalidSample('invalid_envelope')
            events.append((number, event))
        except InvalidSample as exc:
            rows.append(dict(input_line=number, event_type=None, reason=str(exc), cycle=None))
    validator = InputValidator(PreimuSettings(tag_id=tag_id), require_recent_status=False)
    statuses = [e['message'] for _, e in events if e['message'].get('type') == 'uwb_raw_status']
    if not statuses:
        raise ValueError('status_required')
    keys = ('firmware', 'clock_domain', 'anchor_order', 'anchor_count', 'anchor_layout_id')
    for status in statuses:
        validator.check_common(status)
        validator.on_status(status, 0)
        if status.get('firmware') != firmware or any(status.get(k) != statuses[0].get(k) for k in keys):
            raise ValueError('inconsistent_session_metadata')
        if status.get('range_bias_applied') is True:
            raise ValueError('already_bias_corrected_input')
    host_previous = None
    for number, event in events:
        msg = event['message']
        if msg.get('type') != 'uwb_raw_cycle':
            continue
        row = dict(input_line=number, event_type='uwb_raw_cycle', reason='ok',
                   seq=msg.get('seq') if integer(msg.get('seq')) else None, cycle=None)
        try:
            host = event.get('host_received_monotonic_ns')
            if not integer(host) or host < 0 or (host_previous is not None and host < host_previous):
                raise InvalidInput('invalid_host_time')
            host_previous = host
            validator.check_common(msg)
            row['cycle'] = validator.on_cycle(msg, host, 0)
        except InvalidInput as exc:
            row['reason'] = exc.reason
        rows.append(row)
    return sorted(rows, key=lambda r: r['input_line'])


def estimate_bias(rows, anchors, reference_xyz_m, minimum_samples=100):
    """Fit only calibration capture values; evaluation values cannot enter here."""
    anchors, reference = np.asarray(anchors, float), np.asarray(reference_xyz_m, float)
    if (anchors.shape != (4, 3) or reference.shape != (3,)
            or not np.isfinite(anchors).all() or not np.isfinite(reference).all()):
        raise ValueError('invalid_calibration_geometry')
    expected = np.linalg.norm(anchors-reference, axis=1)
    values = [[] for _ in range(4)]
    for row in rows:
        if row['cycle'] is not None:
            for i in row['cycle'].indices:
                values[i].append(float(row['cycle'].raw[i]))
    if any(len(v) < minimum_samples for v in values):
        raise ValueError('insufficient_calibration_samples')
    means = np.array([np.mean(v) for v in values])
    return dict(method='mean_raw_minus_reference_range', candidate_only=True,
                production_calibration_confirmed=False, reference_xyz_m=reference.tolist(),
                expected_slant_m=expected.tolist(), mean_raw_slant_m=means.tolist(),
                bias_m=(means-expected).tolist(), samples_per_anchor=[len(v) for v in values],
                raw_std_m=[float(np.std(v)) for v in values])


def calculate(rows, anchors, height_m, bias_m, solver_settings):
    """Solve two identical A models; their only difference is signed bias subtraction."""
    bias = np.asarray(bias_m, dtype=float)
    if bias.shape != (4,) or not np.isfinite(bias).all() or not np.isfinite(height_m):
        raise ValueError('invalid_trial_parameters')
    first = next((row['cycle'].start_us for row in rows if row['cycle'] is not None), None)
    output = []
    for row in rows:
        cycle = row['cycle']
        result = dict(model_id='A', variant_scope='static_reference_trial',
                      source='measured_uwb', input_line=row['input_line'], seq=row.get('seq'),
                      reason=row['reason'], z_source='capture_reference_measurement',
                      reference_height_m=height_m, external_output_allowed=False, flight_valid=False,
                      t_rel_s=None, raw_slant_m=None, cal_slant_m=None, fits={})
        if cycle is not None:
            result.update(t_rel_s=(cycle.start_us-first)/1e6,
                          source_report_time_us=cycle.sample_us,
                          raw_slant_m=[float(v) if np.isfinite(v) else None for v in cycle.raw])
            if len(cycle.indices) != 4:
                result['reason'] = 'four_anchors_required'
            else:
                corrected = cycle.raw-bias
                result['cal_slant_m'] = corrected.tolist()
                for name, ranges in [('without_bias', cycle.raw), ('with_candidate_bias', corrected)]:
                    result['fits'][name] = asdict(solve_uniform_xy(anchors, ranges, height_m,
                                                                 **solver_settings))
        output.append(result)
    return output


def position_metrics(points, reference_xy):
    if not points:
        return dict(count=0)
    xy, reference = np.asarray(points), np.asarray(reference_xy)
    difference = xy-reference
    errors = np.linalg.norm(difference, axis=1)
    return dict(count=len(xy), mean_xy_m=np.mean(xy, axis=0).tolist(),
                mean_difference_xy_m=np.mean(difference, axis=0).tolist(),
                std_xy_m=np.std(xy, axis=0).tolist(),
                rmse_m=float(np.sqrt(np.mean(errors**2))), median_m=float(np.median(errors)),
                p95_m=float(np.quantile(errors, .95)), max_m=float(np.max(errors)))


def evaluate(results, reference_xy, excursion_seq_interval):
    """Reference XY and old obstruction annotation are used only after solving."""
    reference = np.asarray(reference_xy, dtype=float)
    if reference.shape != (2,) or not np.isfinite(reference).all():
        raise ValueError('invalid_evaluation_reference')
    begin, end = excursion_seq_interval
    def inside(row):
        return row['seq'] is not None and begin <= row['seq'] <= end
    subsets = {'all': results, 'reported_excursion_span': [r for r in results if inside(r)],
               'outside_reported_span': [r for r in results if not inside(r)]}
    output = {}
    for name, rows in subsets.items():
        group = dict(input_records=len(rows), models={})
        for variant in ('without_bias', 'with_candidate_bias'):
            points = [r['fits'][variant]['xy_m'] for r in rows
                      if r['fits'].get(variant, {}).get('ok')]
            reasons = Counter(r['fits'].get(variant, {}).get('reason', r['reason']) for r in rows)
            group['models'][variant] = dict(position_metrics(points, reference), reasons=dict(reasons),
                                           coverage=len(points)/len(rows) if rows else None)
        paired = [r for r in rows if all(r['fits'].get(v, {}).get('ok')
                                        for v in ('without_bias', 'with_candidate_bias'))]
        if paired:
            before = position_metrics([r['fits']['without_bias']['xy_m'] for r in paired], reference)
            after = position_metrics([r['fits']['with_candidate_bias']['xy_m'] for r in paired], reference)
            group['paired'] = dict(count=len(paired), before_rmse_m=before['rmse_m'],
                                   after_rmse_m=after['rmse_m'],
                                   rmse_reduction_fraction=(1-after['rmse_m']/before['rmse_m'])
                                   if before['rmse_m'] > 0 else None)
        output[name] = group
    return output


def run(config_path, root, output):
    root, output = Path(root), Path(output)
    config = json.loads(Path(config_path).read_text(encoding='utf-8'))
    require_finite_json(config)
    loaded = {}
    for name, spec in config['files'].items():
        path = root/spec['path']
        if digest(path) != spec['sha256']:
            raise ValueError('input_hash_mismatch:'+name)
        loaded[name] = path
    if digest(loaded['calibration_raw']) == digest(loaded['evaluation_raw']):
        raise ValueError('calibration_evaluation_must_be_separate')
    layout = json.loads(loaded['anchors'].read_text(encoding='utf-8'))
    if (layout['anchor_order'] != ['A1', 'A2', 'A3', 'A4'] or layout['units'] != 'm'
            or layout['coordinate_frame'] != 'uwb_map'):
        raise ValueError('invalid_anchor_map')
    anchors = np.asarray(layout['anchors_xyz_m'], dtype=float)
    if anchors.shape != (4, 3) or not np.isfinite(anchors).all():
        raise ValueError('invalid_anchor_map')
    refs = {}
    for role in ('calibration', 'evaluation'):
        meta = json.loads(loaded[role+'_metadata'].read_text(encoding='utf-8'))
        refs[role] = [meta['reference_position_m'][k] for k in ('x', 'y', 'z')]
    calibration_rows = read_capture(loaded['calibration_raw'], config['tag_id'], config['firmware'])
    calibration = estimate_bias(calibration_rows, anchors, refs['calibration'],
                                config['minimum_calibration_samples'])
    calibration['source_sha256'] = digest(loaded['calibration_raw'])
    evaluation_rows = read_capture(loaded['evaluation_raw'], config['tag_id'], config['firmware'])
    results = calculate(evaluation_rows, anchors, refs['evaluation'][2], calibration['bias_m'], config['solver'])
    excursion = json.loads(loaded['excursion_annotation'].read_text(encoding='utf-8'))
    interval = [min(e['first_seq'] for e in excursion['events']), max(e['last_seq'] for e in excursion['events'])]
    metrics = evaluate(results, refs['evaluation'][:2], interval)
    summary = dict(model='A', scope='static_reference_trial', source='measured_uwb',
                   evaluation_reference_xyz_m=refs['evaluation'], reference_independently_verified=False,
                   height_source='manual_capture_reference; not ToF',
                   calibration_candidate=calibration, groups=metrics,
                   reported_excursion_seq_interval=interval,
                   calibration_cycle_records=sum(r['event_type'] == 'uwb_raw_cycle' for r in calibration_rows),
                   evaluation_cycle_records=sum(r['event_type'] == 'uwb_raw_cycle' for r in evaluation_rows),
                   calibration_validation_reasons=dict(Counter(r['reason'] for r in calibration_rows)),
                   production_calibration_confirmed=False, external_output_allowed=False, flight_valid=False,
                   limitation='Separate captures at the same reported point; no independent metrology, dynamic flight, or ToF validation.')
    output.mkdir(parents=True, exist_ok=False)
    for role in ('calibration_raw', 'evaluation_raw'):
        shutil.copyfile(loaded[role], output/(role+'.jsonl'))
    for name, obj in [('summary.json', summary), ('calibration.json', calibration),
                      ('config.json', config), ('anchors.json', layout)]:
        (output/name).write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (output/'results.jsonl').write_text(''.join(map(json_line, results)), encoding='utf-8')
    with (output/'positions.csv').open('x', encoding='utf-8', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['input_line', 'seq', 't_rel_s', 'variant', 'x_m', 'y_m',
                         'reference_difference_m', 'range_fit_rms_m', 'in_reported_span'])
        for row in results:
            for variant, fit in row['fits'].items():
                if fit['ok']:
                    xy = fit['xy_m']
                    error = float(np.linalg.norm(np.asarray(xy)-refs['evaluation'][:2]))
                    writer.writerow([row['input_line'], row['seq'], row['t_rel_s'], variant, *xy,
                                     error, fit['rms_m'], interval[0] <= row['seq'] <= interval[1]])
    package_root = Path(__file__).resolve().parents[2]
    manifest = dict(files=config['files'], code_sha256={str(p.relative_to(package_root)): digest(p)
                    for p in sorted(package_root.rglob('*.py'))},
                    output_sha256={p.name: digest(p) for p in sorted(output.iterdir()) if p.is_file()},
                    runtime=dict(python=platform.python_version(), numpy=np.__version__),
                    status_policy='whole_session_metadata_check_before_offline_replay',
                    data_origin='stored_serial_capture', device_output=False)
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    summary = run(options.config, options.root, options.output)
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
