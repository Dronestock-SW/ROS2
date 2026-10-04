"""Replay captured simulator poses as virtual UWB for A/B/C/D/WLS.

This is a shadow experiment. All four ranges are generated simultaneously at
each recorded pose. Truth XY feeds only sensor generation and later evaluation.
The default diagnostic uses simulator-pose height. An optional sensor file
path uses causal ToF/IMU height with alignment and floor gates.
"""
import argparse
from collections import Counter
from dataclasses import asdict
from itertools import combinations
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

from drone_uwb.acquisition.validation import Cycle
from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.h80_b import BSettings, H80Window, write_json
from drone_uwb.processing.experiments.uniform_xy import solve_uniform_xy
from drone_uwb.processing.intersections import DSettings, make_candidates as model_d
from drone_uwb.processing.triplets import make_candidates as model_c
from drone_uwb.processing.weighted_xy import solve_weighted_xy
from drone_uwb.processing.runner import json_line, require_finite_json
from drone_uwb.processing.gazebo_geometry import tag_position
from drone_uwb.processing.ranges import RangeGate
from drone_uwb.processing.experiments.gazebo_height import GazeboSensorHeight


MODELS = ('A', 'B', 'C', 'D', 'WLS')


def validate_config(config):
    require_finite_json(config)
    for key in ('noise_sigma_m', 'wls_sigma_m', 'bias_m'):
        values = np.asarray(config[key], float)
        if values.shape != (4,) or not np.isfinite(values).all():
            raise ValueError('invalid_'+key)
        if key != 'bias_m' and np.any(values < 0 if key == 'noise_sigma_m' else values <= 0):
            raise ValueError('invalid_'+key)
    anchors = np.asarray(config['anchors_xyz_m'], float)
    if anchors.shape != (4, 3) or not np.isfinite(anchors).all():
        raise ValueError('invalid_anchors')
    if config['coordinate_frame'] != 'gazebo_world' or config['external_output_allowed'] is not False:
        raise ValueError('shadow_simulation_contract_required')
    if 'tag_offset_body_flu_m' in config:
        offset = np.asarray(config['tag_offset_body_flu_m'], float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError('invalid_tag_offset')
    for fault in config['faults']:
        if not 0 <= fault['start_s'] < fault['end_s']:
            raise ValueError('invalid_fault_interval')
        if set(fault) not in ({'start_s', 'end_s', 'drop_all'}, {'start_s', 'end_s', 'anchor_id', 'bias_m'}):
            raise ValueError('unsupported_fault')
        if 'anchor_id' in fault and fault['anchor_id'] not in ('A1', 'A2', 'A3', 'A4'):
            raise ValueError('invalid_fault_anchor')
        if 'drop_all' in fault and fault['drop_all'] is not True:
            raise ValueError('invalid_drop_flag')
    if 'range_gate' in config:
        gate = config['range_gate']
        if set(gate) != {'margin_m', 'speed_m_s', 'cluster_radius_m', 'confirm_count'}:
            raise ValueError('invalid_range_gate_settings')
        if (any(not isinstance(gate[k], (int, float)) or isinstance(gate[k], bool)
                or not np.isfinite(gate[k]) or gate[k] <= 0
                for k in ('margin_m', 'speed_m_s', 'cluster_radius_m'))
                or type(gate['confirm_count']) is not int or gate['confirm_count'] < 2):
            raise ValueError('invalid_range_gate_settings')
    return anchors


def compare_poses(poses, config, raw_cycles=None, height_provider=None):
    anchors = validate_config(config)
    if raw_cycles is not None and config['faults']:
        raise ValueError('recorded_raw_requires_unmodified_fault_schedule')
    raw_iterator = iter(raw_cycles) if raw_cycles is not None else None
    bias = np.asarray(config['bias_m'])
    covariance = np.diag(np.asarray(config['wls_sigma_m'])**2)
    rng = np.random.default_rng(config['seed'])
    window = H80Window(anchors, bias, BSettings(**config['B']))
    gate_config = config.get('range_gate')
    model_names = MODELS+('B_M03',) if gate_config else MODELS
    if gate_config:
        gate_settings = SimpleNamespace(max_range_m=80., range_gate_margin_m=gate_config['margin_m'],
                                        range_gate_speed_m_s=gate_config['speed_m_s'],
                                        range_reacquire_cluster_m=gate_config['cluster_radius_m'],
                                        range_reacquire_confirm=gate_config['confirm_count'])
        gates = [RangeGate(gate_settings) for _ in range(4)]
        gated_window = H80Window(anchors, bias, BSettings(**config['B']))
    first = previous = None
    source = model_name = None
    for seq, pose in enumerate(poses):
        require_finite_json(pose)
        xyz = np.asarray(pose['position_xyz_m'], float)
        stamp = pose['time_us']
        if (xyz.shape != (3,) or not np.isfinite(xyz).all() or not isinstance(stamp, int)
                or isinstance(stamp, bool) or stamp < 0 or pose['clock_domain'] != 'gazebo_sim_us'
                or pose['source'] not in ('gazebo', 'synthetic_pose_fixture')):
            raise ValueError('invalid_simulator_pose')
        if previous is not None and stamp <= previous:
            raise ValueError('non_increasing_simulator_time')
        if first is None:
            first, source, model_name = stamp, pose['source'], pose['model']
        if pose['source'] != source or pose['model'] != model_name:
            raise ValueError('mixed_pose_sources')
        if 'tag_offset_body_flu_m' in config:
            xyz = tag_position(pose, config['tag_offset_body_flu_m'])
        elapsed = (stamp-first)/1e6
        if raw_iterator is None:
            raw = np.linalg.norm(anchors-xyz, axis=1)+bias+rng.normal(0, config['noise_sigma_m'], 4)
            sample_times = [stamp]*4
        else:
            try:
                message = next(raw_iterator)
            except StopIteration as exc:
                raise ValueError('recorded_raw_count_mismatch') from exc
            if (message.get('schema') != 1 or message.get('source') != 'simulation'
                    or message.get('type') != 'sim_uwb_cycle'
                    or message.get('clock_domain') != 'gazebo_sim_us'
                    or message.get('seq') != seq or message.get('time_us') != stamp
                    or message.get('anchor_order') != ['A1', 'A2', 'A3', 'A4']
                    or message.get('range_bias_applied') is not False
                    or message.get('external_output_allowed') is not False):
                raise ValueError('recorded_raw_contract_mismatch')
            raw = np.asarray(message.get('raw_slant_m'), dtype=float)
            sample_times = message.get('sample_time_us')
            if (raw.shape != (4,) or not np.isfinite(raw).all() or sample_times != [stamp]*4
                    or message.get('valid_mask') != sum(1 << i for i, v in enumerate(raw) if 0 < v <= 80)):
                raise ValueError('recorded_raw_values_or_time_mismatch')
        active, dropped = [], False
        for fault in config['faults']:
            if fault['start_s'] <= elapsed < fault['end_s']:
                active.append(fault)
                if fault.get('drop_all'):
                    dropped = True
                else:
                    raw[int(fault['anchor_id'][1])-1] += fault['bias_m']
        height_m, height_meta = (float(xyz[2]), dict(reason='simulator_pose')) if height_provider is None \
                                else height_provider.height_at(stamp)
        row = dict(seq=seq, t_rel_s=elapsed, source=source, time_us=stamp,
                   truth_xyz_m=xyz.tolist(),
                   height_source='same_simulator_pose' if height_provider is None else 'gazebo_tof_imu',
                   height_m=height_m, height_selection=height_meta,
                   sample_time_us=sample_times, raw_slant_m=raw.tolist(), cal_slant_m=(raw-bias).tolist(),
                   raw_source='recorded_capture' if raw_iterator is not None else 'regenerated_from_pose',
                   active_faults=active, frame_dropped=dropped, external_output_allowed=False,
                   flight_valid=False, models={})
        if 'tag_offset_body_flu_m' in config:
            row['body_truth_xyz_m'] = pose['position_xyz_m']
            if height_provider is None:
                row['height_source'] = 'simulator_tag_pose_with_mount_offset'
        invalid_range = np.any((raw-bias <= 0) | (raw-bias > 80))
        if dropped or invalid_range:
            if dropped:
                row['raw_slant_m'] = row['cal_slant_m'] = [None]*4
            reason = 'injected_dropout' if dropped else 'invalid_corrected_range'
            row['models'] = {m: dict(ok=False, reason=reason, xy_m=None) for m in model_names}
        else:
            corrected = raw-bias
            cycle = Cycle(seq, stamp, stamp, 15, raw, [stamp]*4, ['ok']*4, list(range(4)))
            obs = [f'{seq}:A{i+1}' for i in range(4)]
            row['models']['B'] = window.process(cycle, seq)
            if height_m is None:
                for model in ('A', 'C', 'D', 'WLS'):
                    row['models'][model] = dict(ok=False, reason=height_meta['reason'], xy_m=None)
            else:
                row['models']['A'] = asdict(solve_uniform_xy(anchors, corrected, height_m, **config['solver']))
                row['models']['C'] = model_c(anchors, corrected, height_m, t_ref_us=stamp, obs_ids=obs, settings=config['solver'])
                row['models']['D'] = model_d(anchors, corrected, height_m, t_ref_us=stamp, obs_ids=obs, settings=DSettings(**config['D']))
                row['models']['WLS'] = solve_weighted_xy(anchors, corrected, height_m, covariance, **config['solver'])
            if gate_config:
                decisions = [gate.update(sample_times[i], float(corrected[i])) for i, gate in enumerate(gates)]
                accepted = [i for i, decision in enumerate(decisions) if decision.accepted]
                filtered_cycle = Cycle(seq, stamp, stamp, sum(1 << i for i in accepted), raw,
                                       sample_times, ['ok' if i in accepted else 'range_gate_pending'
                                                      for i in range(4)], accepted)
                row['range_gate_decisions'] = [dict(accepted=d.accepted, reason=d.reason,
                                                    pending_count=d.pending_count) for d in decisions]
                row['models']['B_M03'] = gated_window.process(filtered_cycle, seq)
        previous = stamp
        yield row
    if raw_iterator is not None and next(raw_iterator, None) is not None:
        raise ValueError('recorded_raw_count_mismatch')


def evaluate(rows, target_m=.07):
    if not np.isfinite(target_m) or target_m <= 0:
        raise ValueError('positive_finite_target_required')

    def metrics(selected, model):
        if not selected:
            return dict(count=0, target_m=target_m, at_or_above_target_count=0,
                        max_under_target_on_valid=None)
        error_xy = np.asarray([np.asarray(r['models'][model]['xy_m'])-np.asarray(r['truth_xyz_m'][:2]) for r in selected])
        distance = np.linalg.norm(error_xy, axis=1)
        return dict(count=len(selected), rmse_m=float(np.sqrt(np.mean(distance**2))),
                    median_m=float(np.median(distance)), p95_m=float(np.quantile(distance, .95)),
                    max_m=float(np.max(distance)), target_m=target_m,
                    at_or_above_target_count=int(np.count_nonzero(distance >= target_m)),
                    max_under_target_on_valid=bool(np.all(distance < target_m)),
                    mean_error_xy_m=np.mean(error_xy, axis=0).tolist(), std_error_xy_m=np.std(error_xy, axis=0).tolist())
    models = tuple(rows[0]['models']) if rows else MODELS
    result = dict(input_ticks=len(rows), models={}, pairwise={})
    for model in models:
        valid = [r for r in rows if r['models'][model]['ok']]
        result['models'][model] = dict(metrics(valid, model), coverage=len(valid)/len(rows) if rows else None,
                                       missing_count=len(rows)-len(valid),
                                       reasons=dict(Counter(r['models'][model]['reason'] for r in rows)))
    for one, two in combinations(models, 2):
        paired = [r for r in rows if r['models'][one]['ok'] and r['models'][two]['ok']]
        result['pairwise'][one+'_'+two] = dict(count=len(paired), models={m: metrics(paired, m) for m in (one, two)})
    return result


def run(input_path, config_path, output, raw_path=None, truth_path=None,
        tof_path=None, attitude_path=None, height_profile_path=None):
    input_path, config_path, output = map(Path, (input_path, config_path, output))
    config = json.loads(config_path.read_text(encoding='utf-8'))
    poses = [json.loads(line) for line in input_path.read_text(encoding='utf-8').splitlines()]
    if not poses:
        raise ValueError('no_simulator_poses')
    if (raw_path is None) != (truth_path is None):
        raise ValueError('recorded_raw_and_truth_required_together')
    sensor_paths = (tof_path, attitude_path, height_profile_path)
    if any(path is not None for path in sensor_paths) and not all(path is not None for path in sensor_paths):
        raise ValueError('tof_attitude_and_height_profile_required_together')
    height_provider = None
    if all(path is not None for path in sensor_paths):
        if 'tag_offset_body_flu_m' not in config:
            raise ValueError('sensor_height_requires_tag_mount')
        tof_rows = [json.loads(line) for line in Path(tof_path).read_text(encoding='utf-8').splitlines()]
        attitude_rows = [json.loads(line) for line in Path(attitude_path).read_text(encoding='utf-8').splitlines()]
        profile = json.loads(Path(height_profile_path).read_text(encoding='utf-8'))
        height_provider = GazeboSensorHeight(tof_rows, attitude_rows, profile,
                                             config['tag_offset_body_flu_m'])
    raw_cycles = None
    if raw_path is not None:
        raw_path, truth_path = Path(raw_path), Path(truth_path)
        raw_cycles = [json.loads(line) for line in raw_path.read_text(encoding='utf-8').splitlines()]
        truth_rows = [json.loads(line) for line in truth_path.read_text(encoding='utf-8').splitlines()]
        if len(truth_rows) != len(poses) or len(raw_cycles) != len(poses):
            raise ValueError('recorded_capture_count_mismatch')
        anchors = validate_config(config)
        for pose, truth in zip(poses, truth_rows):
            tag = tag_position(pose, config['tag_offset_body_flu_m'])
            expected = np.linalg.norm(anchors-tag, axis=1)
            if (truth.get('time_us') != pose['time_us']
                    or not np.allclose(truth.get('body_xyz_m'), pose['position_xyz_m'], rtol=0, atol=1e-9)
                    or not np.allclose(truth.get('tag_xyz_m'), tag, rtol=0, atol=1e-9)
                    or not np.allclose(truth.get('geometric_ranges_m'), expected, rtol=0, atol=1e-9)):
                raise ValueError('recorded_truth_pose_or_map_mismatch')
    results = list(compare_poses(poses, config, raw_cycles, height_provider))
    summary = dict(scope=('gazebo_sensor_UWB_shadow' if height_provider is not None
                          else 'simulator_pose_UWB_shadow'),
                   source=poses[0]['source'], evaluation=evaluate(results),
                   height_source=('gazebo_tof_imu' if height_provider is not None else
                                  'simulator_tag_pose_with_mount_offset' if 'tag_offset_body_flu_m' in config
                                  else 'same_simulator_pose'),
                   height_selection_reasons=dict(Counter(row['height_selection']['reason'] for row in results)),
                   ranges_sampled_simultaneously=True,
                   range_source='recorded_capture' if raw_cycles is not None else 'regenerated_from_pose',
                   weight_source='configured_expected_range_sigma; no_reference_based_adaptation',
                   external_output_allowed=False, flight_valid=False)
    output.mkdir(parents=True, exist_ok=False)
    (output/'poses.jsonl').write_bytes(input_path.read_bytes())
    if raw_cycles is not None:
        (output/'raw_ranges.jsonl').write_bytes(raw_path.read_bytes())
        (output/'truth.jsonl').write_bytes(truth_path.read_bytes())
    if height_provider is not None:
        for source_path, name in ((tof_path, 'tof.jsonl'), (attitude_path, 'attitude.jsonl'),
                                  (height_profile_path, 'height_profile.json')):
            (output/name).write_bytes(Path(source_path).read_bytes())
    (output/'results.jsonl').write_text(''.join(map(json_line, results)), encoding='utf-8')
    write_json(output/'config.json', config)
    write_json(output/'summary.json', summary)
    package = Path(__file__).resolve().parents[2]
    write_json(output/'manifest.json', dict(input_sha256=digest(input_path), config_sha256=digest(config_path),
              recorded_raw_sha256=digest(raw_path) if raw_cycles is not None else None,
              recorded_truth_sha256=digest(truth_path) if raw_cycles is not None else None,
              tof_sha256=digest(tof_path) if height_provider is not None else None,
              attitude_sha256=digest(attitude_path) if height_provider is not None else None,
              height_profile_sha256=digest(height_profile_path) if height_provider is not None else None,
              package_sha256={str(p.relative_to(package)): digest(p) for p in sorted(package.rglob('*.py'))},
              outputs_sha256={p.name: digest(p) for p in sorted(output.iterdir())},
              physics_run_verified=False, note='Pose source tag is recorded; it is not authentication of a real Gazebo run.'))
    return summary


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--raw', type=Path, help='Recorded Gazebo RAW JSONL; requires --truth')
    parser.add_argument('--truth', type=Path, help='Recorded Gazebo truth JSONL; requires --raw')
    parser.add_argument('--tof', type=Path, help='Gazebo ToF JSONL; requires --attitude and --height-profile')
    parser.add_argument('--attitude', type=Path, help='Gazebo IMU JSONL; requires --tof and --height-profile')
    parser.add_argument('--height-profile', type=Path, help='Mount and alignment gates for sensor height')
    options = parser.parse_args(args)
    result = run(options.input, options.config, options.output, options.raw, options.truth,
                 options.tof, options.attitude, options.height_profile)
    print(json.dumps(result['evaluation']['models'], ensure_ascii=False))


if __name__ == '__main__':
    main()
