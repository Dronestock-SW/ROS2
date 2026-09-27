"""File-only B (four-anchor H80) comparison against the unchanged A solver.

One received cycle is one evaluation tick. B uses only ranges already received;
metadata is preflighted across this offline session, like the original A trial.
CPU timings are separate from deterministic positions and decisions. No ROS I/O.
"""
import argparse
from collections import Counter, deque
from dataclasses import asdict, dataclass
import csv
import json
from pathlib import Path
import platform
from time import perf_counter_ns

import numpy as np

from drone_uwb.acquisition.validation import InputValidator, InvalidInput
from drone_uwb.contracts.protocol import InvalidSample, decode_line, integer
from drone_uwb.processing.h80 import fit_h80
from drone_uwb.processing.runner import json_line, require_finite_json
from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.static_a import position_metrics
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy


@dataclass(frozen=True)
class BSettings:
    window_s: float = .8
    min_samples_per_anchor: int = 3
    min_samples_total: int = 12
    required_anchor_count: int = 4
    sigma_r_m: float = .08
    huber_m: float = .12
    h80_irls_iterations: int = 8
    max_scaled_condition: float = 1e6
    reset_gap_s: float = .15
    max_extrapolation_s: float = .04
    max_range_m: float = 80.

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError('invalid_B_setting:'+name)
        for name in ('min_samples_per_anchor', 'min_samples_total', 'required_anchor_count', 'h80_irls_iterations'):
            if not isinstance(getattr(self, name), int):
                raise ValueError('integer_B_setting_required:'+name)
        if (self.required_anchor_count != 4 or self.min_samples_per_anchor < 3
                or self.min_samples_total < 4*self.min_samples_per_anchor
                or self.max_scaled_condition <= 1):
            raise ValueError('invalid_four_anchor_B_settings')


def read_events(path, tag_id, firmware):
    """Preserve bad input ticks, status/boot boundaries, and sequential validation.

    Session metadata may appear after the first cycle in these captures. Only
    metadata is preflighted; future range values never enter a calculation.
    """
    parsed = []
    for number, line in enumerate(Path(path).read_bytes().splitlines(), 1):
        try:
            event = decode_line(line)
            require_finite_json(event)
            if not isinstance(event.get('message'), dict):
                raise InvalidSample('invalid_envelope')
            parsed.append((number, event, None))
        except InvalidSample as exc:
            parsed.append((number, None, str(exc)))
    validator = InputValidator(PreimuSettings(tag_id=tag_id), require_recent_status=False)
    statuses = [e['message'] for _, e, _ in parsed if e and e['message'].get('type') == 'uwb_raw_status']
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
    validator.on_status(statuses[0], 0)
    previous_host = None
    for number, event, error in parsed:
        row = dict(input_line=number, seq=None, cycle=None, reset_reason=None, reason=error or 'ok', tick=True)
        if event is None:
            yield row
            continue
        msg = event['message']
        row['seq'] = msg.get('seq') if integer(msg.get('seq')) else None
        row['tick'] = msg.get('type') == 'uwb_raw_cycle'
        host = event.get('host_received_monotonic_ns')
        try:
            if not integer(host) or host < 0 or (previous_host is not None and host < previous_host):
                validator.disconnect()
                row['reset_reason'] = 'invalid_host_time'
                raise InvalidInput('invalid_host_time')
            previous_host = host
            validator.check_common(msg)
            if msg.get('event') == 'boot':
                validator.disconnect()
                row['reset_reason'] = 'boot'
            if msg.get('type') == 'uwb_raw_status':
                validator.on_status(msg, host)
                row['reason'] = 'status'
            elif row['tick']:
                end = msg.get('cycle_end_us')
                if integer(end) and validator.last_end_us is not None and end < validator.last_end_us:
                    validator.disconnect()
                    row['reset_reason'] = 'source_time_backwards'
                    raise InvalidInput('source_restart_wait_status')
                row['cycle'] = validator.on_cycle(msg, host, 0)
            else:
                row['reason'] = 'unsupported_type'
        except InvalidInput as exc:
            row['reason'] = exc.reason
            if exc.reset:
                row['reset_reason'] = row['reset_reason'] or exc.reason
        yield row


class H80Window:
    """Causal range history, independent of A positions and evaluation truth."""

    def __init__(self, anchors, bias_m, settings=None):
        self.settings = settings or BSettings()
        self.anchors, self.bias = np.asarray(anchors, float), np.asarray(bias_m, float)
        if (self.anchors.shape != (4, 3) or self.bias.shape != (4,)
                or not np.isfinite(self.anchors).all() or not np.isfinite(self.bias).all()):
            raise ValueError('invalid_B_geometry_or_bias')
        if np.ptp(self.anchors[:, 2]) > 1e-9:
            raise ValueError('reference_slant_requires_equal_anchor_heights')
        self.reset('startup')

    def reset(self, reason):
        self.history = deque()
        self.last_end_us = None
        self.first_end_us = None
        self.last_sample_us = [None]*4
        self.last_observed_us = None
        self.pending_reset = reason

    def process(self, cycle, input_line):
        cfg = self.settings
        end = cycle.end_us
        if self.last_end_us is not None:
            if end <= self.last_end_us:
                self.reset('source_time_backwards')
            elif end-self.last_end_us > round(cfg.reset_gap_s*1e6):
                self.reset('gap')
        self.last_end_us = end
        if self.first_end_us is None:
            self.first_end_us = end
        reset = self.pending_reset
        self.pending_reset = None
        fresh, excluded = [], {}
        for i in cycle.indices:
            r = float(cycle.raw[i]-self.bias[i])
            sample = cycle.sample_us[i]
            if not np.isfinite(r) or not 0 < r <= cfg.max_range_m:
                excluded[str(i)] = 'invalid_corrected_range'
            elif sample is None or not cycle.start_us <= sample <= end:
                excluded[str(i)] = 'invalid_sample_time'
            elif self.last_sample_us[i] is not None and sample <= self.last_sample_us[i]:
                excluded[str(i)] = 'duplicate_or_old_sample'
            else:
                fresh.append((sample, i, r, f'{input_line}:A{i+1}'))
                self.last_sample_us[i] = sample
        if (fresh and self.last_observed_us is not None
                and min(obs[0] for obs in fresh)-self.last_observed_us > round(cfg.reset_gap_s*1e6)):
            self.reset('observation_gap')
            reset = self.pending_reset
            self.pending_reset = None
            self.last_end_us = self.first_end_us = end
            for sample, i, _, _ in fresh:
                self.last_sample_us[i] = sample
        if fresh:
            self.last_observed_us = max(obs[0] for obs in fresh)
        self.history.extend(fresh)
        lower = end-round(cfg.window_s*1e6)
        self.history = deque(obs for obs in self.history if lower <= obs[0] <= end)
        counts = [sum(obs[1] == i for obs in self.history) for i in range(4)]
        latest = [max((obs[0] for obs in self.history if obs[1] == i), default=None) for i in range(4)]
        result = dict(ok=False, reason='insufficient_anchors', xy_m=None, velocity_m_s=None,
                      window_count=len(self.history), counts=counts, fit=None, reset_reason=reset,
                      fresh_observation_count=len(fresh), excluded=excluded,
                      obs_ids=[obs[3] for obs in self.history],
                      anchor_age_s=[(end-t)/1e6 if t is not None else None for t in latest],
                      segment_age_s=(end-self.first_end_us)/1e6,
                      full_window_elapsed=end-self.first_end_us >= round(cfg.window_s*1e6),
                      oldest_sample_us=min((obs[0] for obs in self.history), default=None),
                      newest_sample_us=max((obs[0] for obs in self.history), default=None))
        if not fresh:
            result['reason'] = 'no_fresh_observations'
            return result
        if end-max(obs[0] for obs in fresh) > round(cfg.max_extrapolation_s*1e6):
            result['reason'] = 'extrapolation_limit'
            return result
        if min(counts) < cfg.min_samples_per_anchor or len(self.history) < cfg.min_samples_total:
            return result
        idx = np.array([obs[1] for obs in self.history], dtype=int)
        # Relative times avoid subtraction of large source-clock floats.
        times = np.array([(obs[0]-end)/1e6 for obs in self.history])
        ranges = np.array([obs[2] for obs in self.history])
        fit = fit_h80(self.anchors, idx, times, ranges, 0., cfg)
        result.update(ok=fit.ok, reason=fit.reason, fit=asdict(fit))
        if fit.ok:
            result['xy_m'] = [fit.x0, fit.y0]
            result['velocity_m_s'] = [fit.x1/cfg.window_s, fit.y1/cfg.window_s]
        return result


def calculate(events, anchors, height_m, bias_m, a_settings, b_settings=None):
    window = H80Window(anchors, bias_m, b_settings)
    results, timings, event_counts = [], [], Counter()
    origin_host = None
    for event in events:
        event_counts[event['reason']] += 1
        if event['reset_reason']:
            window.reset(event['reset_reason'])
        if not event['tick']:
            continue
        cycle = event['cycle']
        a = dict(ok=False, reason=event['reason'], xy_m=None)
        b = dict(ok=False, reason=event['reason'], xy_m=None, reset_reason=None)
        cpu_a = cpu_b = None
        row = dict(model_ids=['A', 'B'], variant='reference_slant_four_anchor',
                   input_line=event['input_line'], seq=event['seq'], reason=event['reason'],
                   t_rel_s=None, source_ref_us=None, raw_slant_m=None, cal_slant_m=None,
                   external_output_allowed=False, flight_valid=False)
        if cycle is not None:
            if origin_host is None:
                origin_host = cycle.host_mono_ns
            corrected = cycle.raw-np.asarray(bias_m)
            row.update(t_rel_s=(cycle.host_mono_ns-origin_host)/1e9, source_ref_us=cycle.end_us,
                       sample_time_us=cycle.sample_us,
                       raw_slant_m=[float(v) if np.isfinite(v) else None for v in cycle.raw],
                       cal_slant_m=[float(v) if np.isfinite(v) else None for v in corrected])
            start = perf_counter_ns()
            if len(cycle.indices) == 4:
                a = asdict(solve_uniform_xy(anchors, corrected, height_m, **a_settings))
            else:
                a['reason'] = 'four_anchors_required'
            cpu_a = (perf_counter_ns()-start)/1e6
            start = perf_counter_ns()
            b = window.process(cycle, event['input_line'])
            cpu_b = (perf_counter_ns()-start)/1e6
            # Common geometric diagnostic only; it never feeds B's fit or a gate.
            b['current_frame_residuals_m'] = None
            b['current_frame_rms_m'] = None
            if b['ok'] and len(cycle.indices) == 4:
                position = np.array([*b['xy_m'], height_m])
                residual = np.linalg.norm(np.asarray(anchors)-position, axis=1)-corrected
                b['current_frame_residuals_m'] = residual.tolist()
                b['current_frame_rms_m'] = float(np.sqrt(np.mean(residual**2)))
        row['models'] = dict(A=a, B=b)
        results.append(row)
        timings.append(dict(input_line=event['input_line'], A_ms=cpu_a, B_ms=cpu_b))
    return results, timings, dict(event_counts)


def evaluate(results, reference_xy, excursion_interval):
    reference = np.asarray(reference_xy, float)
    if reference.shape != (2,) or not np.isfinite(reference).all():
        raise ValueError('invalid_evaluation_reference')
    lo, hi = excursion_interval
    inside = lambda row: row['seq'] is not None and lo <= row['seq'] <= hi
    groups = dict(all=results, reported_excursion_span=[r for r in results if inside(r)],
                  outside_reported_span=[r for r in results if not inside(r)],
                  full_window_elapsed=[r for r in results if r['models']['B'].get('full_window_elapsed')])
    summary = {}
    for name, rows in groups.items():
        group = dict(input_ticks=len(rows), models={})
        for model in ('A', 'B'):
            points = [r['models'][model]['xy_m'] for r in rows if r['models'][model]['ok']]
            group['models'][model] = dict(position_metrics(points, reference),
                                         coverage=len(points)/len(rows) if rows else None,
                                         reasons=dict(Counter(r['models'][model]['reason'] for r in rows)))
        paired = [r for r in rows if all(r['models'][m]['ok'] for m in ('A', 'B'))]
        metrics = {m: position_metrics([r['models'][m]['xy_m'] for r in paired], reference) for m in ('A', 'B')}
        group['paired'] = dict(count=len(paired), models=metrics, rmse_reduction_fraction=None)
        if paired and metrics['A']['rmse_m'] > 1e-12:
            group['paired']['rmse_reduction_fraction'] = 1-metrics['B']['rmse_m']/metrics['A']['rmse_m']
        summary[name] = group
    return summary


def timing_metrics(timings):
    output = {}
    for model in ('A', 'B'):
        values = [r[model+'_ms'] for r in timings if r[model+'_ms'] is not None]
        output[model] = dict(count=len(values))
        if values:
            output[model].update(median_ms=float(np.median(values)), p95_ms=float(np.quantile(values, .95)),
                                 max_ms=float(np.max(values)))
    return output


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def run(config_path, root, output):
    root, output = Path(root), Path(output)
    config = json.loads(Path(config_path).read_text(encoding='utf-8'))
    require_finite_json(config)
    if config['target_mode'] != 'reference_slant':
        raise ValueError('unsupported_B_variant')
    settings = BSettings(**config['B'])
    loaded = {}
    for name, spec in config['files'].items():
        path = root/spec['path']
        if digest(path) != spec['sha256']:
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
    reference = np.array([metadata['reference_position_m'][k] for k in ('x', 'y', 'z')], float)
    if not np.isfinite(reference).all():
        raise ValueError('invalid_evaluation_reference')
    annotation = json.loads(loaded['excursion_annotation'].read_text(encoding='utf-8'))
    interval = [min(e['first_seq'] for e in annotation['events']), max(e['last_seq'] for e in annotation['events'])]
    results, timings, event_counts = calculate(read_events(loaded['evaluation_raw'], config['tag_id'], config['firmware']),
                                              anchors, reference[2], calibration['bias_m'], config['A'], settings)
    groups = evaluate(results, reference[:2], interval)
    valid_b = [r for r in results if r['models']['B']['ok']]
    summary = dict(scope='offline_static_A_B_comparison', source='measured_uwb', target_mode='reference_slant',
                   evaluation_reference_xyz_m=reference.tolist(), reference_independently_verified=False,
                   A_height_source='manual_capture_reference', B_height_input='not_used_by_reference_slant',
                   locked_bias_m=calibration['bias_m'], settings=asdict(settings),
                   reported_excursion_seq_interval=interval, groups=groups, event_counts=event_counts,
                   first_B_input_line=valid_b[0]['input_line'] if valid_b else None,
                   first_B_after_s=valid_b[0]['t_rel_s'] if valid_b else None,
                   B_reset_counts=dict(Counter(r['models']['B']['reset_reason'] for r in results
                                              if r['models']['B'].get('reset_reason'))),
                   B_iteration_limit_outputs=sum(r['models']['B']['ok'] and not r['models']['B']['fit']['converged']
                                                 for r in results),
                   extra_range_gate=False, extra_position_gate=False, Q_S10=False,
                   external_output_allowed=False, flight_valid=False,
                   limitation='Same reported static point; single-point candidate calibration; no independent survey or dynamic flight validation.')
    output.mkdir(parents=True, exist_ok=False)
    for name, obj in [('summary.json', summary), ('config.json', config), ('anchors.json', layout),
                      ('calibration.json', calibration), ('timing_summary.json', timing_metrics(timings))]:
        write_json(output/name, obj)
    (output/'results.jsonl').write_text(''.join(map(json_line, results)), encoding='utf-8')
    (output/'timings.jsonl').write_text(''.join(map(json_line, timings)), encoding='utf-8')
    with (output/'positions.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['input_line', 'seq', 't_rel_s', 'source_ref_us', 'model', 'ok', 'reason',
                         'x_m', 'y_m', 'error_m', 'paired', 'in_reported_span'])
        for row in results:
            paired = all(row['models'][m]['ok'] for m in ('A', 'B'))
            for model, fit in row['models'].items():
                xy = fit['xy_m'] if fit['ok'] else [None, None]
                error = float(np.linalg.norm(np.asarray(xy)-reference[:2])) if fit['ok'] else None
                writer.writerow([row['input_line'], row['seq'], row['t_rel_s'], row['source_ref_us'], model,
                                 fit['ok'], fit['reason'], *xy, error, paired,
                                 row['seq'] is not None and interval[0] <= row['seq'] <= interval[1]])
    package = Path(__file__).resolve().parents[2]
    manifest = dict(files=config['files'], config_sha256=digest(config_path),
                    code_sha256={str(p.relative_to(package)): digest(p) for p in sorted(package.rglob('*.py'))},
                    output_sha256={p.name: digest(p) for p in sorted(output.iterdir()) if p.is_file()},
                    runtime=dict(python=platform.python_version(), numpy=np.__version__, machine=platform.machine()),
                    status_policy='whole_session_metadata_preflight; sequential_cycles_and_boot_resets',
                    evaluation_clock='cycle_end_us; plot uses host receive elapsed time',
                    source_timestamp_kind='Report_read_time_not_hardware_range_time',
                    deterministic_files=['results.jsonl', 'summary.json', 'positions.csv'],
                    device_output=False)
    write_json(output/'manifest.json', manifest)
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    summary = run(options.config, options.root, options.output)
    print(json.dumps(summary['groups'], ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
