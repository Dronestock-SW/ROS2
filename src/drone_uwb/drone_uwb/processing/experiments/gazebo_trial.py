"""Replay captured simulator poses as virtual UWB for A/B/C/D/WLS.

This is a shadow experiment. All four ranges are generated simultaneously at
each recorded pose. Truth XY feeds only sensor generation and later evaluation.
Height is supplied from the same simulator pose, not a ToF product model.
"""
import argparse
from collections import Counter
from dataclasses import asdict
from itertools import combinations
import json
from pathlib import Path
import sys

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
    return anchors


def compare_poses(poses, config):
    anchors = validate_config(config)
    bias = np.asarray(config['bias_m'])
    covariance = np.diag(np.asarray(config['wls_sigma_m'])**2)
    rng = np.random.default_rng(config['seed'])
    window = H80Window(anchors, bias, BSettings(**config['B']))
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
        raw = np.linalg.norm(anchors-xyz, axis=1)+bias+rng.normal(0, config['noise_sigma_m'], 4)
        active, dropped = [], False
        for fault in config['faults']:
            if fault['start_s'] <= elapsed < fault['end_s']:
                active.append(fault)
                if fault.get('drop_all'):
                    dropped = True
                else:
                    raw[int(fault['anchor_id'][1])-1] += fault['bias_m']
        row = dict(seq=seq, t_rel_s=elapsed, source=source, time_us=stamp,
                   truth_xyz_m=xyz.tolist(), height_source='same_simulator_pose', height_m=float(xyz[2]),
                   sample_time_us=[stamp]*4, raw_slant_m=raw.tolist(), cal_slant_m=(raw-bias).tolist(),
                   active_faults=active, frame_dropped=dropped, external_output_allowed=False,
                   flight_valid=False, models={})
        if 'tag_offset_body_flu_m' in config:
            row['body_truth_xyz_m'] = pose['position_xyz_m']
            row['height_source'] = 'simulator_tag_pose_with_mount_offset'
        invalid_range = np.any((raw-bias <= 0) | (raw-bias > 80))
        if dropped or invalid_range:
            if dropped:
                row['raw_slant_m'] = row['cal_slant_m'] = [None]*4
            reason = 'injected_dropout' if dropped else 'invalid_corrected_range'
            row['models'] = {m: dict(ok=False, reason=reason, xy_m=None) for m in MODELS}
        else:
            corrected = raw-bias
            cycle = Cycle(seq, stamp, stamp, 15, raw, [stamp]*4, ['ok']*4, list(range(4)))
            obs = [f'{seq}:A{i+1}' for i in range(4)]
            row['models']['A'] = asdict(solve_uniform_xy(anchors, corrected, xyz[2], **config['solver']))
            row['models']['B'] = window.process(cycle, seq)
            row['models']['C'] = model_c(anchors, corrected, xyz[2], t_ref_us=stamp, obs_ids=obs, settings=config['solver'])
            row['models']['D'] = model_d(anchors, corrected, xyz[2], t_ref_us=stamp, obs_ids=obs, settings=DSettings(**config['D']))
            row['models']['WLS'] = solve_weighted_xy(anchors, corrected, xyz[2], covariance, **config['solver'])
        previous = stamp
        yield row


def evaluate(rows):
    def metrics(selected, model):
        if not selected:
            return dict(count=0)
        error_xy = np.asarray([np.asarray(r['models'][model]['xy_m'])-np.asarray(r['truth_xyz_m'][:2]) for r in selected])
        distance = np.linalg.norm(error_xy, axis=1)
        return dict(count=len(selected), rmse_m=float(np.sqrt(np.mean(distance**2))),
                    p95_m=float(np.quantile(distance, .95)), max_m=float(np.max(distance)),
                    mean_error_xy_m=np.mean(error_xy, axis=0).tolist(), std_error_xy_m=np.std(error_xy, axis=0).tolist())
    result = dict(input_ticks=len(rows), models={}, pairwise={})
    for model in MODELS:
        valid = [r for r in rows if r['models'][model]['ok']]
        result['models'][model] = dict(metrics(valid, model), coverage=len(valid)/len(rows) if rows else None,
                                       reasons=dict(Counter(r['models'][model]['reason'] for r in rows)))
    for one, two in combinations(MODELS, 2):
        paired = [r for r in rows if r['models'][one]['ok'] and r['models'][two]['ok']]
        result['pairwise'][one+'_'+two] = dict(count=len(paired), models={m: metrics(paired, m) for m in (one, two)})
    return result


def run(input_path, config_path, output):
    input_path, config_path, output = map(Path, (input_path, config_path, output))
    config = json.loads(config_path.read_text(encoding='utf-8'))
    poses = [json.loads(line) for line in input_path.read_text(encoding='utf-8').splitlines()]
    if not poses:
        raise ValueError('no_simulator_poses')
    results = list(compare_poses(poses, config))
    summary = dict(scope='simulator_pose_UWB_shadow', source=poses[0]['source'], evaluation=evaluate(results),
                   height_source=('simulator_tag_pose_with_mount_offset' if 'tag_offset_body_flu_m' in config
                                  else 'same_simulator_pose'), ranges_sampled_simultaneously=True,
                   weight_source='configured_expected_range_sigma; no_reference_based_adaptation',
                   external_output_allowed=False, flight_valid=False)
    output.mkdir(parents=True, exist_ok=False)
    (output/'poses.jsonl').write_bytes(input_path.read_bytes())
    (output/'results.jsonl').write_text(''.join(map(json_line, results)), encoding='utf-8')
    write_json(output/'config.json', config)
    write_json(output/'summary.json', summary)
    package = Path(__file__).resolve().parents[2]
    write_json(output/'manifest.json', dict(input_sha256=digest(input_path), config_sha256=digest(config_path),
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
    options = parser.parse_args(args)
    result = run(options.input, options.config, options.output)
    print(json.dumps(result['evaluation']['models'], ensure_ascii=False))


if __name__ == '__main__':
    main()
