"""Replay baseline A into files with causal ToF/attitude selection and provenance.

The existing pipeline remains a separate recorder. This module has no ROS, serial,
network, or PX4 output. Reference positions are read only by the evaluator.
"""
import argparse
from collections import Counter
from copy import deepcopy
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from drone_uwb.contracts.protocol import InvalidSample, decode_line, finite, integer
from drone_uwb.processing.clock import ClockMap
from drone_uwb.processing.height import tof_to_fc_height
from drone_uwb.acquisition.validation import InputValidator, InvalidInput
from drone_uwb.processing.runner import json_line, require_finite_json
from drone_uwb.processing.sensors import SensorInputs
from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation_wb(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class BaselineA:
    def __init__(self, config, layout):
        self.config = deepcopy(config)
        self.layout = deepcopy(layout)
        require_finite_json(config)
        self.anchors = np.asarray(layout['anchors_xyz_m'], dtype=float)
        if (self.anchors.shape != (4, 3) or not np.isfinite(self.anchors).all()
                or layout['anchor_order'] != ['A1', 'A2', 'A3', 'A4']
                or layout['units'] != 'm' or layout['coordinate_frame'] != config['frame_id']):
            raise ValueError('invalid_anchor_map')
        if config['source'] not in ('simulation', 'measured'):
            raise ValueError('invalid_source')
        for key in ('max_sensor_age_s', 'max_attitude_tof_skew_s'):
            if not finite(config[key]) or config[key] <= 0:
                raise ValueError('invalid_sensor_limits')
        bias = config.get('range_bias_m')
        if bias is not None and (not isinstance(bias, list) or len(bias) != 4
                                 or not all(finite(v) for v in bias)):
            raise ValueError('invalid_bias')
        self.settings = PreimuSettings(tag_id=config['tag_id'])
        self.validator = InputValidator(self.settings, require_recent_status=False)
        self.last_host_ns = None
        self.reset()

    def reset(self):
        self.validator.disconnect()
        self.clock = ClockMap(self.settings)
        self.sensors = SensorInputs(max_age_s=self.config['max_sensor_age_s'])

    def height_at(self, target_us):
        snap = self.sensors.snapshot(target_us)
        for name in ('tof_sample', 'attitude_sample'):
            if not snap[name]['available']:
                raise ValueError(name+'_unavailable')
            if snap[name]['sample']['source'] != self.config['source']:
                raise ValueError('sensor_source_mismatch')
        tof = snap['tof_sample']['sample']['message']
        stamp = tof['measurement_time_us']
        # Rotation is selected at the ToF measurement, not at the later UWB time.
        att = self.sensors.snapshot(stamp)['attitude_sample']
        if not att['available']:
            raise ValueError('attitude_at_tof_unavailable')
        msg = att['sample']['message']
        if stamp-msg['measurement_time_us'] > self.config['max_attitude_tof_skew_s']*1e6:
            raise ValueError('attitude_tof_skew')
        rotation = rotation_wb(msg['quaternion_wxyz'])
        geometry = self.config['tof_geometry']
        z_fc = tof_to_fc_height(tof['distance_m'], geometry['bias_m'], rotation,
                                geometry['axis_B'], geometry['lever_tof_B_m'],
                                geometry['ground_z_m'])
        z_ant = z_fc + float((rotation @ geometry['lever_uwb_B_m'])[2])
        return z_ant, dict(tof_time_us=stamp, attitude_time_us=msg['measurement_time_us'],
                           target_time_us=target_us, age_s=(target_us-stamp)/1e6)

    def process(self, event, input_line):
        message = event.get('message', {})
        kind = message.get('type') if isinstance(message, dict) else None
        result = dict(type='uwb_baseline_a_result', schema_version='uwb_experiment_v1',
                      model_id='A', variant_id='uniform_slant', input_line=input_line,
                      event_id=str(input_line), dataset_id=self.config['dataset_id'],
                      event_type=kind, source=event.get('source'), status='blocked',
                      reason='awaiting_input', t_ref_us=None, estimate=None,
                      last_estimate=None, fresh=False, held=False, predicted_only=False,
                      eligible_for_comparison=False, external_output_allowed=False,
                      flight_valid=False, diagnostics={})
        try:
            host_ns = event['host_received_monotonic_ns']
            if not integer(host_ns) or host_ns < 0 or not isinstance(message, dict):
                raise ValueError('invalid_event')
            if self.last_host_ns is not None and host_ns < self.last_host_ns:
                self.reset()
                raise ValueError('host_time_reversed')
            self.last_host_ns = host_ns
            if event['source'] != self.config['source']:
                self.reset()
                raise ValueError('input_source_mismatch')
            if kind in ('tof_sample', 'attitude_sample'):
                try:
                    self.sensors.add(message, event['source'], host_ns)
                except ValueError:
                    # A malformed new sample must not silently expose an older valid one.
                    self.sensors.rows[kind].clear()
                    raise
                result.update(status='pending', reason='sensor_recorded')
                return result
            if kind == 'pipeline_tick':
                result.update(status='pending', reason='no_new_range')
                return result
            if kind not in ('uwb_raw_status', 'uwb_raw_cycle'):
                raise ValueError('unsupported_type')
            self.validator.check_common(message)
            if kind == 'uwb_raw_status':
                if message.get('event') == 'boot':
                    self.reset()
                self.validator.on_status(message, host_ns)
                result.update(status='pending', reason='status_recorded')
                return result
            cycle = self.validator.on_cycle(message, host_ns, 0)
            self.clock.update(cycle.end_us, host_ns)
            result['diagnostics']['clock'] = dict(state=self.clock.state, alpha=self.clock.alpha,
                                                  method='receive_lower_envelope',
                                                  residual_p95_s=self.clock.residual_p95_s)
            if len(cycle.indices) != 4:
                raise ValueError('four_anchors_required')
            missing = []
            if self.config.get('range_bias_m') is None or self.config.get('calibration_confirmed') is not True:
                missing.append('calibration_unconfirmed')
            if self.config.get('tof_geometry') is None or self.config.get('geometry_confirmed') is not True:
                missing.append('geometry_unconfirmed')
            if missing:
                result['diagnostics']['missing_inputs'] = missing
                raise ValueError(missing[0])
            if not self.clock.ready:
                raise ValueError('clock_unsynced')
            times = [round(self.clock.host_s(t)*1e6) for t in cycle.sample_us]
            target = max(times)
            age_s = (host_ns/1000-target)/1e6
            result.update(t_ref_us=target, source_age_s=age_s)
            if not 0 <= age_s <= self.settings.max_queue_s:
                raise ValueError('source_time_out_of_bounds')
            heights = [self.height_at(t) for t in times]
            calibrated = cycle.raw-np.asarray(self.config['range_bias_m'])
            fit = solve_uniform_xy(self.anchors, calibrated, [z for z, _ in heights],
                                   **self.config['solver'])
            result['diagnostics'].update(raw_slant_m=cycle.raw.tolist(),
                cal_slant_m=calibrated.tolist(), z_antenna_m=[z for z, _ in heights],
                z_selection=[meta for _, meta in heights], source_time_host_us=times,
                frame_skew_s=(max(times)-min(times))/1e6, fit=asdict(fit))
            if not fit.ok:
                result.update(status='failed', reason=fit.reason)
                return result
            obs_ids = [f'{input_line}:A{i+1}' for i in range(4)]
            result.update(status='ok', reason='ok', fresh=True, eligible_for_comparison=True,
                          input_ids=obs_ids,
                          z_sample_ids=[str(meta['tof_time_us']) for _, meta in heights],
                          estimate=dict(candidate_id=f'A:{input_line}', xy_m=fit.xy_m,
                              frame_id=self.config['frame_id'], reference_point='uwb_antenna',
                              t_ref_us=target, used_obs_ids=obs_ids,
                              anchor_ids=self.layout['anchor_order'], cov_xy_m2=None,
                              covariance_kind='unknown', residuals_m=fit.residuals_m,
                              parent_ids=[], score=None))
        except InvalidInput as exc:
            if exc.reset:
                self.reset()
            result.update(status='rejected', reason=exc.reason)
        except (ValueError, KeyError, TypeError, np.linalg.LinAlgError) as exc:
            result.update(reason=str(exc))
        return result


def run(input_path, output, config, layout, reference=None):
    if digest(input_path) != config['input_sha256']:
        raise ValueError('input_hash_mismatch')
    if reference is not None:
        for key in ('input_sha256', 'dataset_id', 'frame_id', 'source'):
            if reference[key] != config[key]:
                raise ValueError('reference_mismatch:'+key)
        if (reference['reference_point'] != 'uwb_antenna'
                or reference['method'] != 'known_stationary_simulator_fixture'
                or config['source'] != 'simulation'):
            raise ValueError('unsupported_reference')
        xy = np.asarray(reference['xy_m'], dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('invalid_reference')
    output = Path(output)
    processor = BaselineA(config, layout)
    output.mkdir(parents=True, exist_ok=False)
    config_hash = hashlib.sha256(json_line(config).encode('utf-8')).hexdigest()
    counts, cycle_reasons = Counter(), Counter()
    positions = []
    position_rows = []
    with Path(input_path).open('rb') as incoming, (output/'input.jsonl').open('xb') as original, \
            (output/'results.jsonl').open('x', encoding='utf-8') as results:
        for index, line in enumerate(incoming, 1):
            original.write(line)
            try:
                event = decode_line(line)
                require_finite_json(event)
                # Evaluation-only fields are never passed into the processor.
                clean = {k: event[k] for k in ('source', 'host_received_monotonic_ns', 'message') if k in event}
                result = processor.process(clean, index)
            except InvalidSample as exc:
                result = dict(input_line=index, status='rejected', reason=str(exc), estimate=None,
                              event_type=None, fresh=False, external_output_allowed=False, flight_valid=False)
            result.update(run_id=output.name, config_hash=config_hash)
            counts[result['reason']] += 1
            if result['event_type'] == 'uwb_raw_cycle':
                cycle_reasons[result['reason']] += 1
            if result['estimate'] is not None:
                positions.append(result['estimate']['xy_m'])
                x, y = positions[-1]
                position_rows.append([index, result['t_ref_us'], x, y,
                                      result['diagnostics']['fit']['rms_m']])
            results.write(json_line(result))
    metrics = None
    if reference is not None and positions:
        errors = np.linalg.norm(np.asarray(positions)-np.asarray(reference['xy_m']), axis=1)
        metrics = dict(reference_source=reference['method'], samples=len(errors),
                       median_m=float(np.median(errors)), p95_m=float(np.quantile(errors, .95)),
                       max_m=float(errors.max()), rmse_m=float(np.sqrt(np.mean(errors**2))))
    summary = dict(schema_version='uwb_experiment_v1', model='A', source=config['source'],
                   input_sha256=config['input_sha256'], config_hash=config_hash,
                   event_count=sum(counts.values()), cycle_count=sum(cycle_reasons.values()),
                   reasons=dict(counts), cycle_reasons=dict(cycle_reasons),
                   positions=len(positions),
                   cycle_coverage=len(positions)/sum(cycle_reasons.values()) if cycle_reasons else None,
                   mean_xy_m=np.mean(positions, axis=0).tolist() if positions else None,
                   position_error=metrics, external_output_allowed=False, flight_valid=False,
                   scope='A_file_replay_only', reference=reference)
    artifacts = [('summary.json', summary), ('config.json', config), ('anchors.json', layout)]
    if reference is not None:
        artifacts.append(('reference.json', reference))
    for name, value in artifacts:
        (output/name).write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)+'\n',
                                 encoding='utf-8')
    with (output/'positions.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['input_line', 't_ref_us', 'x_m', 'y_m', 'range_fit_rms_m'])
        writer.writerows(position_rows)
    package_root = Path(__file__).resolve().parents[2]
    manifest = dict(code_sha256={str(p.relative_to(package_root)): digest(p)
                                for p in sorted(package_root.rglob('*.py'))},
                    artifacts_sha256={p.name: digest(p) for p in sorted(output.iterdir()) if p.is_file()},
                    clock_settings={key: getattr(processor.settings, key) for key in
                                    ('clock_min_samples', 'clock_window_s', 'clock_alpha_min_span_s',
                                     'clock_scale_tolerance', 'clock_residual_p95_max_s',
                                     'max_report_span_s', 'max_queue_s')})
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'output', 'config', 'anchors'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--reference', type=Path)
    options = parser.parse_args(args)
    config = json.loads(options.config.read_text(encoding='utf-8'))
    if digest(options.anchors) != config['layout_sha256']:
        parser.error('layout_hash_mismatch')
    layout = json.loads(options.anchors.read_text(encoding='utf-8'))
    reference = json.loads(options.reference.read_text(encoding='utf-8')) if options.reference else None
    result = run(options.input, options.output, config, layout, reference)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
