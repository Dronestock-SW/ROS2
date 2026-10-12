"""Reproducible ray-cast fixtures for the OFFLINE LiDAR experiment, not SITL."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import json
from pathlib import Path
import time

import numpy as np

from .lidar_scanmatch import Config, KeyframeMatcher, compose, inverse, match, scan_cloud, wrap

ROOM = np.array([[[-4, -3], [4, -3]], [[4, -3], [4, 3]],
                 [[4, 3], [-4, 3]], [[-4, 3], [-4, -3]],
                 [[-2, 1.4], [-.5, 1.8]], [[2.6, -.3], [2.6, 1.6]]], dtype=float)
CORRIDOR = np.array([[[-100., -2.], [100., -2.]], [[100., 2.], [-100., 2.]]])


def cast_scan(pose, rng, *, segments=ROOM, sigma_m=.003, beams=430, occlusion=False, phase_rad=0.):
    """Instantaneous 360 degree scan; independent range noise, no real driver model.

    The extra angular phase is intentionally NOT recorded in the reported
    angle_min: this is a synthetic angular-calibration perturbation, not proof
    of the YDLIDAR driver's behavior.
    """
    angles = -np.pi + np.arange(beams) * (2 * np.pi / beams)
    directions = np.column_stack((np.cos(angles + pose[2] + phase_rad), np.sin(angles + pose[2] + phase_rad)))
    edges = segments[:, 1] - segments[:, 0]
    relative = segments[:, 0] - np.asarray(pose[:2])
    denominator = directions[:, None, 0] * edges[None, :, 1] - directions[:, None, 1] * edges[None, :, 0]
    safe = np.where(abs(denominator) > 1e-12, denominator, np.nan)
    distance = (relative[:, 0] * edges[:, 1] - relative[:, 1] * edges[:, 0])[None, :] / safe
    fraction = (relative[None, :, 0] * directions[:, None, 1] - relative[None, :, 1] * directions[:, None, 0]) / safe
    good = (distance > .03) & (distance < 12.) & (fraction >= 0.) & (fraction <= 1.)
    ranges = np.min(np.where(good, distance, np.inf), axis=1)
    ranges += rng.normal(0., sigma_m, beams)
    if occlusion:
        ranges[30:270] = np.nan
    return ranges


def fixture_cloud(pose=(0., 0., 0.), seed=11, **kwargs):
    ranges = cast_scan(pose, np.random.default_rng(seed), **kwargs)
    return scan_cloud(ranges, -np.pi, 2 * np.pi / len(ranges), .03, 12.)


def trajectory(case, t):
    if case in ('static', 'corridor', 'occlusion', 'angular_perturbation'):
        return np.zeros(3)
    # Continuous X out/back then Y out/back. No truth is supplied to ICP.
    x = np.sin(np.pi * t / 20.) ** 2 if t <= 20 else 0.
    y = np.sin(np.pi * (t - 20.) / 20.) ** 2 if t > 20 else 0.
    if case == 'xy':
        return np.array([x, y, 0.])
    if case == 'yaw':
        return np.array([0., 0., np.deg2rad(30.) * np.sin(2 * np.pi * t / 40.)])
    return np.array([x, y, np.deg2rad(20.) * np.sin(2 * np.pi * t / 40.)])


def error_metrics(values):
    values = np.asarray(values)
    if len(values) == 0:
        return dict(n=0, rms=None, p95=None, maximum=None)
    return dict(n=len(values), rms=float(np.sqrt(np.mean(values ** 2))),
                p95=float(np.quantile(np.abs(values), .95)), maximum=float(np.max(np.abs(values))))


def run_case(case, seed, duration=40., config=Config()):
    rng = np.random.default_rng(seed)
    tracker = KeyframeMatcher(config)
    previous_cloud = None
    chained = np.zeros(3)
    chain_yaw = 0.
    previous_gyro_t = 0.
    previous_true_yaw = 0.
    gyro_yaw = 0.
    accepted = 0
    chain_accepted = 0
    keyframes = set()
    reasons = Counter()
    errors = {method: {'xy': [], 'yaw_deg': [], 'velocity': []} for method in ('pair', 'keyframe_imu')}
    timings = []
    for index, t in enumerate(np.arange(0., duration + .05, .1)):
        pose = trajectory(case, t)
        # A separately noisy simulated gyro increment (0.01 deg/s rate noise).
        dt = t - previous_gyro_t
        gyro_yaw += wrap(pose[2] - previous_true_yaw) + rng.normal(0., np.deg2rad(.01)) * dt
        previous_gyro_t, previous_true_yaw = t, pose[2]
        ranges = cast_scan(pose, rng, segments=CORRIDOR if case == 'corridor' else ROOM,
                           occlusion=case == 'occlusion' and 10 <= t < 15,
                           phase_rad=np.deg2rad(3.) if case == 'angular_perturbation' and 10 <= t < 15 else 0.)
        start = time.perf_counter()
        cloud = scan_cloud(ranges, -np.pi, 2 * np.pi / len(ranges), .03, 12., config)
        observation = tracker.observe(cloud, t, imu_yaw_rad=gyro_yaw)
        timings.append(time.perf_counter() - start)
        keyframes.add(observation.keyframe)
        reasons.update(observation.reasons)
        if previous_cloud is not None:
            pair = match(previous_cloud, cloud, config=config)
            # Only accepted consecutive pairs accumulate. Rejections are a
            # diagnostic hold, NOT a valid motion observation.
            if pair.accepted:
                chain_accepted += 1
                chained = compose(chained, pair.pose)
                chain_yaw += pair.pose[2]
            errors['pair']['xy'].append(float(np.linalg.norm(chained[:2] - pose[:2])))
            errors['pair']['yaw_deg'].append(float(np.rad2deg(wrap(chained[2] - pose[2]))))
        if observation.accepted:
            accepted += 1
            errors['keyframe_imu']['xy'].append(float(np.linalg.norm(observation.pose[:2] - pose[:2])))
            errors['keyframe_imu']['yaw_deg'].append(float(np.rad2deg(wrap(observation.pose[2] - pose[2]))))
            before = trajectory(case, t - .1)
            true_delta = compose(inverse(before), pose)
            errors['keyframe_imu']['velocity'].append(float(np.linalg.norm(observation.delta[:2] - true_delta[:2]) / .1))
        previous_cloud = cloud
    return dict(case=case, seed=seed, scans=index + 1, intervals=index,
                accepted=accepted, acceptance_fraction=accepted / index,
                pair_accepted=chain_accepted, pair_unwrapped_yaw_deg=float(np.rad2deg(chain_yaw)),
                keyframes=len(keyframes), reasons=dict(reasons),
                error={m: {k: error_metrics(v) for k, v in metrics.items()} for m, metrics in errors.items()},
                preprocess_bidirectional_s=error_metrics(timings),
                scope='synthetic planar instantaneous rays; not original capture, driver, deskew, or flight validation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[11, 22, 33, 44, 55])
    parser.add_argument('--cases', nargs='+', default=['static', 'xy', 'yaw', 'mixed', 'corridor', 'occlusion', 'angular_perturbation'])
    parser.add_argument('--strict-numerical-baseline', action='store_true',
                        help='Re-run the initial NEW comparator with 1e-5 m/rad convergence; not original analyze.py')
    args = parser.parse_args()
    config = replace(Config(), convergence_translation_m=1e-5, convergence_yaw_rad=1e-5) if args.strict_numerical_baseline else Config()
    results = []
    for case in args.cases:
        for seed in args.seeds:
            result = run_case(case, seed, config=config)
            results.append(result)
            print(json.dumps(dict(case=case, seed=seed, accepted=result['accepted'], intervals=result['intervals'])), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(schema=1, source='synthetic_raycast', config=asdict(config),
                                           original_baseline_reproduced=False, results=results), indent=2,
                                      ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
