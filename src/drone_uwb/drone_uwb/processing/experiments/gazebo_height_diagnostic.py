"""Diagnose Gazebo ToF/IMU geometry against recorded pose truth, file-only.

This tool temporarily evaluates a candidate with the two confirmation flags
open in memory. It never edits the profile or authorizes navigation output.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np

from drone_uwb.processing.experiments.baseline_a import digest
from drone_uwb.processing.experiments.gazebo_height import GazeboSensorHeight
from drone_uwb.processing.experiments.h80_b import write_json
from drone_uwb.processing.geometry.gazebo_geometry import rotation_world_body, tag_position
from drone_uwb.processing.runner import json_line


def _metric(values):
    if not values:
        return dict(count=0, rmse_m=None, median_abs_m=None, p95_abs_m=None, max_abs_m=None)
    data = np.asarray(values, dtype=float)
    absolute = np.abs(data)
    return dict(count=len(data), rmse_m=float(np.sqrt(np.mean(data**2))),
                median_abs_m=float(np.median(absolute)),
                p95_abs_m=float(np.quantile(absolute, .95)),
                max_abs_m=float(np.max(absolute)))


def assess(poses, tof_rows, attitude_rows, profile, tag_offset_body_flu_m):
    """Return per-UWB-tick diagnostics; never return an authorized observation."""
    if not poses:
        raise ValueError('no_pose_samples')
    if profile.get('orientation_alignment_confirmed') is not False or profile.get('flat_floor_confirmed') is not False:
        raise ValueError('diagnostic_requires_closed_input_gates')
    candidate = dict(profile, orientation_alignment_confirmed=True, flat_floor_confirmed=True)
    provider = GazeboSensorHeight(tof_rows, attitude_rows, candidate, tag_offset_body_flu_m)
    tof_by_time = {row['time_us']: row for row in tof_rows}
    axis = np.asarray(profile['tof_axis_body_flu'], dtype=float)
    lever = np.asarray(profile['tof_lever_body_flu_m'], dtype=float)
    previous = None
    records, height_errors, range_residuals, tilt_degrees = [], [], [], []
    for pose in poses:
        stamp = pose.get('time_us')
        if (pose.get('clock_domain') != 'gazebo_sim_us'
                or pose.get('source') not in ('gazebo', 'synthetic_pose_fixture')
                or type(stamp) is not int or stamp < 0
                or previous is not None and stamp <= previous):
            raise ValueError('invalid_or_unordered_pose')
        previous = stamp
        body = np.asarray(pose['position_xyz_m'], dtype=float)
        if body.shape != (3,) or not np.isfinite(body).all():
            raise ValueError('invalid_pose_position')
        rotation = rotation_world_body(pose['quaternion_wxyz'])
        truth = tag_position(pose, tag_offset_body_flu_m)
        estimated, selection = provider.height_at(stamp)
        row = dict(time_us=stamp, selected=selection, candidate_height_m=estimated,
                   truth_tag_height_m=float(truth[2]),
                   height_error_m=(estimated-float(truth[2]) if estimated is not None else None),
                   tof_range_residual_m=None, truth_tilt_deg=None,
                   diagnostic_only=True, external_output_allowed=False)
        if estimated is not None:
            height_errors.append(row['height_error_m'])
            tof = tof_by_time[selection['tof_time_us']]
            beam_z = float((rotation@axis)[2])
            if beam_z < -1e-9:
                sensor_origin_z = float(body[2]+(rotation@lever)[2])
                expected_raw = ((profile['ground_z_m']-sensor_origin_z)/beam_z
                                + profile['tof_bias_m'])
                row['tof_range_residual_m'] = float(tof['distance_m']-expected_raw)
                range_residuals.append(row['tof_range_residual_m'])
                row['truth_tilt_deg'] = math.degrees(math.acos(np.clip(-beam_z, -1., 1.)))
                tilt_degrees.append(row['truth_tilt_deg'])
        records.append(row)
    summary = dict(scope='gazebo_sensor_geometry_diagnostic',
                   pose_source=poses[0]['source'], input_ticks=len(poses),
                   height_candidate=_metric(height_errors),
                   tof_range_residual=_metric(range_residuals),
                   tilt_range_deg=(dict(min=min(tilt_degrees), max=max(tilt_degrees))
                                   if tilt_degrees else None),
                   selection_reasons=dict(Counter(r['selected']['reason'] for r in records)),
                   input_gates_closed=True, profile_modified=False,
                   session_identity_verified=False, alignment_confirmed=False,
                   ground_plane_confirmed=False, eligible_for_navigation=False,
                   external_output_allowed=False, flight_valid=False)
    return records, summary


def run(poses_path, tof_path, attitude_path, profile_path, config_path, output):
    paths = [Path(p) for p in (poses_path, tof_path, attitude_path, profile_path, config_path)]
    output = Path(output)
    poses, tof, attitude = ([json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
                            for path in paths[:3])
    profile = json.loads(paths[3].read_text(encoding='utf-8'))
    config = json.loads(paths[4].read_text(encoding='utf-8'))
    records, summary = assess(poses, tof, attitude, profile, config['tag_offset_body_flu_m'])
    output.mkdir(parents=True, exist_ok=False)
    for path, name in zip(paths, ('poses.jsonl', 'tof.jsonl', 'attitude.jsonl',
                                  'height_profile.json', 'config.json')):
        (output/name).write_bytes(path.read_bytes())
    (output/'diagnostics.jsonl').write_text(''.join(json_line(row) for row in records), encoding='utf-8')
    write_json(output/'summary.json', summary)
    write_json(output/'manifest.json', dict(inputs_sha256={str(p): digest(p) for p in paths},
              outputs_sha256={p.name: digest(p) for p in sorted(output.iterdir())},
              note='Geometry diagnostic only; no profile confirmation or PX4 output.'))
    return summary


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('poses', 'tof', 'attitude', 'height-profile', 'config', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    options = parser.parse_args(args)
    result = run(options.poses, options.tof, options.attitude,
                 options.height_profile, options.config, options.output)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
