"""Offline comparison of independent Gazebo truth, UWB and PX4 navigation.

No socket or flight API. Clock alignment must come from separate evidence,
never from fitting estimated positions to the answer being evaluated.
"""
import argparse
from bisect import bisect_left
from collections import Counter
from dataclasses import asdict, dataclass
import csv
import hashlib
import inspect
import json
import math
import platform
from pathlib import Path
import sys

import numpy as np

from drone_demo.flight_metrics import error_stats
from drone_demo.flight_fault_evaluation import fault_reports
from drone_uwb.integration.gazebo.gazebo_faults import RangeFaultPlan
from drone_uwb.integration.sitl.sitl_odometry_contract import SITLOdometrySettings
from drone_uwb.processing.geometry.gazebo_geometry import tag_position
from drone_uwb.processing.experiments.gazebo_trial import validate_config


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def xy(value):
    return isinstance(value, (list, tuple)) and len(value) == 2 and all(map(finite, value))


@dataclass(frozen=True)
class EvaluationPlan:
    start_sim_us: int
    end_sim_us: int
    model: str = 'dronestock_x500_0'
    max_truth_gap_s: float = .1
    max_px4_bracket_s: float = .1
    max_clock_uncertainty_us: int = 10000
    positioning_target_m: float = .07
    min_observation_coverage: float = .95
    max_observation_gap_s: float = .15
    max_observation_latency_s: float = .15
    arrival_enter_m: float = .15
    arrival_exit_m: float = .2
    arrival_speed_m_s: float = .1
    settle_s: float = 1.
    recovery_window_s: float = 2.
    recovery_settle_s: float = .5

    def __post_init__(self):
        if (type(self.start_sim_us) is not int or type(self.end_sim_us) is not int
                or not 0 <= self.start_sim_us < self.end_sim_us):
            raise ValueError('evaluation_interval_required')
        if not isinstance(self.model, str) or not self.model:
            raise ValueError('exact_model_required')
        values = asdict(self)
        for key in ('start_sim_us', 'end_sim_us', 'model'):
            values.pop(key)
        if any(not finite(value) or value <= 0 for value in values.values()):
            raise ValueError('positive_evaluation_limits_required')
        if self.min_observation_coverage > 1 or self.arrival_enter_m >= self.arrival_exit_m:
            raise ValueError('invalid_evaluation_limits')
        if self.recovery_settle_s >= self.recovery_window_s:
            raise ValueError('recovery_settle_must_fit_window')
        if any(not math.isfinite(v*1e6) or v*1e6 < 1
               for v in (self.recovery_window_s,self.recovery_settle_s)):
            raise ValueError('recovery_time_not_representable')


@dataclass(frozen=True)
class EvaluationClock:
    confirmed: bool = False
    px4_to_gazebo_offset_us: int = None
    uncertainty_us: int = None
    valid_start_px4_us: int = None
    valid_end_px4_us: int = None
    evidence: str = ''

    def map_time(self, stamp, plan):
        if self.confirmed is not True:
            raise ValueError('evaluation_clock_unconfirmed')
        if (any(type(v) is not int for v in (stamp, self.px4_to_gazebo_offset_us,
                    self.uncertainty_us, self.valid_start_px4_us, self.valid_end_px4_us))
                or not 0 <= self.uncertainty_us <= plan.max_clock_uncertainty_us
                or not 0 < self.valid_start_px4_us <= stamp <= self.valid_end_px4_us
                or not isinstance(self.evidence, str) or not self.evidence.strip()):
            raise ValueError('evaluation_clock_evidence_or_interval_invalid')
        return stamp+self.px4_to_gazebo_offset_us


def interpolate(stamp, samples, times, max_gap_s):
    index = bisect_left(times, stamp)
    if index < len(times) and times[index] == stamp:
        return samples[index][1], 0.
    if index == 0 or index == len(times):
        return None, None  # No extrapolation before/after the recorded flight.
    first, second = samples[index-1], samples[index]
    gap = (second[0]-first[0])/1e6
    if gap > max_gap_s:
        return None, gap
    ratio = (stamp-first[0])/(second[0]-first[0])
    return [a+(b-a)*ratio for a, b in zip(first[1], second[1])], gap


def px4_samples(events, clock, plan):
    samples, rejections = {}, Counter()
    previous, reset = None, None
    for event in events:
        if event.get('type') != 'mission_assessment':
            continue
        row = event.get('assessment', {})
        stamp, point = row.get('px4_sample_time_us'), row.get('position_map_xy_m')
        age = row.get('pose_age_s')
        if (row.get('position_source') != 'px4_ekf2' or row.get('reference_point') != 'px4_reference'
                or not xy(point) or not finite(age) or not 0 <= age <= .2):
            rejections['px4_position_missing_or_stale'] += 1
            continue
        try:
            mapped = clock.map_time(stamp, plan)
        except ValueError as exc:
            rejections[str(exc)] += 1
            continue
        counter = row.get('px4_reset_counter')
        if type(counter) is not int or not 0 <= counter <= 255:
            rejections['px4_reset_counter_missing'] += 1
            continue
        if (reset is not None and reset != counter) or (previous is not None and stamp < previous):
            raise ValueError('px4_reference_or_clock_reset_split_run_required')
        if mapped in samples and samples[mapped] != point:
            raise ValueError('conflicting_px4_sample')
        samples[mapped] = point
        previous, reset = stamp, counter
    return sorted(samples.items()), dict(rejections)


def leg_reports(events, rows, plan):
    commands = [e for e in events if e.get('type') == 'target_command_sent']
    reports = []
    for i, command in enumerate(commands):
        start, target = command.get('gazebo_time_us'), command.get('packet', {}).get('target_map_xy_m')
        if type(start) is not int or not xy(target):
            reports.append(dict(leg_index=command.get('leg_index'), status='missing_target_time_or_xy'))
            continue
        end = commands[i+1].get('gazebo_time_us') if i+1 < len(commands) else plan.end_sim_us
        if type(end) is not int or end <= start:
            raise ValueError('target_simulation_time_not_increasing')
        aborts = [e for e in events if e.get('type') == 'navigation_aborted'
                  and type(e.get('gazebo_time_us')) is int and start <= e['gazebo_time_us'] < end]
        if aborts:
            end = aborts[0]['gazebo_time_us']
        selected = [r for r in rows if start <= r['time_us'] < end]
        settling, first_arrival, previous_stamp = None, None, None
        inside = False
        overshoot = departures = 0
        states = []
        start_xy = selected[0]['truth_reference_xy_m'] if selected else None
        for row in selected:
            stamp, distance, speed = row['time_us'], row['truth_target_error_m'], row['truth_speed_m_s']
            continuous = previous_stamp is not None and (stamp-previous_stamp)/1e6 <= plan.max_truth_gap_s
            if not continuous:
                settling = None
            radius = plan.arrival_exit_m if settling is not None else plan.arrival_enter_m
            good = distance is not None and distance <= radius and speed is not None and speed <= plan.arrival_speed_m_s
            if good:
                if settling is None:
                    settling = stamp
            else:
                settling = None
            arrived = settling is not None and (stamp-settling)/1e6 >= plan.settle_s
            if arrived and first_arrival is None:
                first_arrival = stamp
            if distance is not None and first_arrival is not None:
                within = distance <= plan.arrival_exit_m
                departures += int(inside and not within)
                inside = within
            if start_xy is not None:
                vector = np.asarray(target)-np.asarray(start_xy)
                length = float(np.linalg.norm(vector))
                if length > 0:
                    along = float((np.asarray(row['truth_reference_xy_m'])-start_xy)@vector/length)
                    overshoot = max(overshoot, along-length)
            states.append((stamp, arrived))
            previous_stamp = stamp
        claims = [e for e in events if e.get('type') == 'leg_arrived'
                  and e.get('leg_index') == command.get('leg_index')]
        claim = claims[0].get('gazebo_time_us') if claims else None
        before = [state for state in states if type(claim) is int and state[0] <= claim]
        claim_verified = (bool(before) and (claim-before[-1][0])/1e6 <= plan.max_truth_gap_s
                          and before[-1][1]) if claims else None
        reports.append(dict(leg_index=command.get('leg_index'), target_map_xy_m=target,
            start_sim_us=start, end_sim_us=end, truth_rows=len(selected),
            first_truth_arrival_sim_us=first_arrival,
            truth_arrival_time_s=(first_arrival-start)/1e6 if first_arrival is not None else None,
            fc_claim_sim_us=claim, fc_arrival_confirmed_by_truth=claim_verified,
            overshoot_along_leg_m=overshoot if selected else None,
            departures_after_truth_arrival=departures,
            actual_tracking=error_stats([r['truth_target_error_m'] for r in selected]),
            estimated_tracking=error_stats([r['px4_target_error_m'] for r in selected]),
            status='aborted' if aborts else 'evaluated' if selected else 'no_truth_in_leg'))
    return reports


def evaluate(poses, observations, events, alignment, plan, clock):
    if not poses:
        raise ValueError('independent_truth_required')
    reference, previous, source = [], None, poses[0].get('source')
    if source not in ('gazebo', 'synthetic_pose_fixture'):
        raise ValueError('explicit_truth_source_required')
    for pose in poses:
        stamp = pose.get('time_us')
        if (type(stamp) is not int or stamp < 0 or previous is not None and stamp <= previous
                or pose.get('source') != source or pose.get('model') != plan.model
                or pose.get('clock_domain') != 'gazebo_sim_us'):
            raise ValueError('truth_model_source_or_time_mismatch')
        reference.append((stamp, tag_position(pose, alignment.px4_reference_offset_body_flu_m)[:2].tolist()))
        previous = stamp
    selected = [p for p in reference if plan.start_sim_us <= p[0] <= plan.end_sim_us]
    if not selected:
        raise ValueError('no_truth_in_evaluation_interval')
    px4, rejected = px4_samples(events, clock, plan)
    px4_times = [p[0] for p in px4]
    observed = {}
    for event in observations:
        if event.get('transmitted') is not True:
            continue
        stamp = event.get('time_us')
        candidate = event.get('candidate', {})
        point = candidate.get('reference_world_xyz_m')
        if (type(stamp) is not int or candidate.get('reference_point') != 'px4_reference'
                or not isinstance(point, (list, tuple)) or len(point) != 3 or not all(map(finite, point))):
            raise ValueError('invalid_transmitted_observation')
        if stamp in observed:
            raise ValueError('duplicate_transmitted_observation')
        observed[stamp] = event
    commands = [e for e in events if e.get('type') == 'target_command_sent'
                and type(e.get('gazebo_time_us')) is int]
    if any(b['gazebo_time_us'] <= a['gazebo_time_us'] for a,b in zip(commands, commands[1:])):
        raise ValueError('target_simulation_time_not_increasing')
    rows, available, latencies = [], [], []
    for index, (stamp, truth) in enumerate(selected):
        current = [e for e in commands if e['gazebo_time_us'] <= stamp]
        target = current[-1]['packet']['target_map_xy_m'] if current else None
        if target is not None and not xy(target):
            raise ValueError('invalid_target_xy')
        estimate, bracket = interpolate(stamp, px4, px4_times, plan.max_px4_bracket_s)
        observation = observed.get(stamp)
        obs_xy = observation['candidate']['reference_world_xyz_m'][:2] if observation else None
        latency = observation.get('sample_age_sim_s') if observation else None
        if observation:
            available.append(stamp)
            latencies.append(latency if finite(latency) and latency >= 0 else None)
        speed = None
        if index:
            last_stamp, last_truth = selected[index-1]
            dt = (stamp-last_stamp)/1e6
            if dt <= plan.max_truth_gap_s:
                speed = math.dist(truth, last_truth)/dt
        rows.append(dict(time_us=stamp, reference_point='px4_reference', truth_reference_xy_m=truth,
            uwb_reference_xy_m=obs_xy, px4_reference_xy_m=estimate, target_map_xy_m=target,
            uwb_position_error_m=math.dist(obs_xy, truth) if obs_xy else None,
            px4_position_error_m=math.dist(estimate, truth) if estimate else None,
            px4_target_error_m=math.dist(estimate, target) if estimate and target else None,
            truth_target_error_m=math.dist(truth, target) if target else None,
            truth_speed_m_s=speed, px4_interpolation_bracket_s=bracket,
            uwb_latency_sim_s=latency,
            missing_reasons=([name for name, missing in (
                ('no_transmitted_observation', observation is None),
                ('no_px4_position_at_truth_tick', estimate is None),
                ('truth_speed_unavailable', speed is None)) if missing]),
            missing_reason=('no_transmitted_observation' if not observation else None)))
    gaps = np.diff([plan.start_sim_us, *available, plan.end_sim_us])/1e6
    max_gap = float(np.max(gaps))
    truth_gaps = np.diff([plan.start_sim_us, *[r[0] for r in selected], plan.end_sim_us])/1e6
    truth_complete = (reference[0][0] <= plan.start_sim_us and reference[-1][0] >= plan.end_sim_us
                      and float(np.max(truth_gaps)) <= plan.max_truth_gap_s)
    coverage = len(available)/len(selected)
    uwb = error_stats([r['uwb_position_error_m'] for r in rows], plan.positioning_target_m)
    latency_known = bool(latencies) and all(v is not None for v in latencies)
    max_latency = max(latencies) if latency_known else None
    local_gate = (uwb['max_m'] is not None and uwb['max_m'] < plan.positioning_target_m
                  and coverage >= plan.min_observation_coverage and max_gap <= plan.max_observation_gap_s
                  and latency_known and max_latency <= plan.max_observation_latency_s and truth_complete)
    summary = dict(scope='offline_flight_evaluation', truth_source=source, reference_point='px4_reference',
        truth_rows=len(rows), truth_interval_complete=truth_complete,
        uwb_positioning=dict(uwb, coverage=coverage, max_gap_s=max_gap,
                            max_latency_sim_s=max_latency, this_interval_gate_passed=bool(local_gate)),
        px4_positioning=error_stats([r['px4_position_error_m'] for r in rows]),
        estimated_tracking=error_stats([r['px4_target_error_m'] for r in rows]),
        actual_tracking=error_stats([r['truth_target_error_m'] for r in rows]),
        px4_rejections=rejected, legs=leg_reports(events, rows, plan),
        interpolation='linear_px4_xy_at_truth_ticks_no_extrapolation',
        truth_speed_method='backward_reference_position_difference',
        flight_valid=False, fusion_verified=False, full_7cm_goal_achieved=False)
    return rows, summary


def run(navigation, ranges, plan_path, clock_path, output):
    navigation, ranges, plan_path, clock_path, output = map(Path,
        (navigation, ranges, plan_path, clock_path, output))
    inputs = dict(poses=ranges/'poses.jsonl', generator_raw=ranges/'raw_ranges.jsonl',
        generator_truth=ranges/'truth.jsonl', generator_config=ranges/'config.json',
        navigation_raw=navigation/'raw_ranges.jsonl', observations=navigation/'sitl_events.jsonl',
        navigation_events=navigation/'navigation_events.jsonl', alignment=navigation/'sitl_odometry.json',
        plan=plan_path, clock=clock_path)
    for name, filename in (('fault_plan','fault_plan.json'), ('range_emission','emission.jsonl'),
                           ('raw_before_fault','raw_before_fault.jsonl'),
                           ('range_manifest','manifest.json'), ('range_capture','capture.json')):
        path = ranges/filename
        if path.is_file():
            inputs[name] = path
    source_bytes = {name: path.read_bytes() for name,path in inputs.items()}
    def lines(name):
        return [json.loads(line) for line in source_bytes[name].decode('utf-8').splitlines() if line.strip()]
    # Preserve and compare the actual shared range messages, not just matching
    # timestamps from two simulator sessions which may both start at zero.
    generated = {}
    for row in lines('generator_raw'):
        key = (row['seq'], row['time_us'])
        if key in generated:
            raise ValueError('duplicate_generator_raw')
        generated[key] = row
    received = lines('navigation_raw')
    def payload(row):
        return {key: value for key,value in row.items() if key not in (
            'host_clock_domain', 'host_callback_start_monotonic_ns', 'host_callback_end_monotonic_ns')}
    if not received or any(generated.get((r['seq'], r['time_us'])) != payload(r) for r in received):
        raise ValueError('raw_session_pairing_mismatch')
    observations = lines('observations')
    received_keys = {(r['seq'],r['time_us']) for r in received}
    if any((r.get('seq'),r.get('time_us')) not in received_keys
           for r in observations if r.get('transmitted') is True):
        raise ValueError('transmitted_observation_without_received_raw')
    if 'range_emission' in source_bytes:
        dropped = {(r['seq'],r['time_us']) for r in lines('range_emission') if r.get('drop_requested') is True}
        if any((r['seq'],r['time_us']) in dropped for r in received):
            raise ValueError('raw_received_despite_recorded_drop')
    poses, truth = lines('poses'), lines('generator_truth')
    config = json.loads(source_bytes['generator_config'])
    anchors = validate_config(config)
    if not len(poses) == len(truth) == len(generated):
        raise ValueError('generator_pose_truth_raw_count_mismatch')
    for index, (pose, answer) in enumerate(zip(poses, truth)):
        tag = tag_position(pose, config['tag_offset_body_flu_m'])
        if ((index, pose['time_us']) not in generated or answer['time_us'] != pose['time_us']
                or not np.allclose(answer['body_xyz_m'], pose['position_xyz_m'], rtol=0, atol=1e-9)
                or not np.allclose(answer['tag_xyz_m'], tag, rtol=0, atol=1e-9)
                or not np.allclose(answer['geometric_ranges_m'], np.linalg.norm(anchors-tag,axis=1), rtol=0, atol=1e-9)):
            raise ValueError('generator_truth_geometry_mismatch')
    plan = EvaluationPlan(**json.loads(source_bytes['plan']))
    clock = EvaluationClock(**json.loads(source_bytes['clock']))
    alignment = SITLOdometrySettings(**json.loads(source_bytes['alignment']))
    rows, summary = evaluate(poses, observations, lines('navigation_events'), alignment, plan, clock)
    summary.update(fault_reports(rows, list(generated.values()), received,
        lines('raw_before_fault') if 'raw_before_fault' in source_bytes else None,
        lines('range_emission') if 'range_emission' in source_bytes else None,
        json.loads(source_bytes['fault_plan']) if 'fault_plan' in source_bytes else None,
        plan, (poses[0]['time_us'],poses[-1]['time_us'])))
    output.mkdir(parents=True, exist_ok=False)
    def save(name, value):
        (output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    save('metrics.json', summary)
    csv_rows = []
    for segment in summary['scenarios']:
        flat = {key:value for key,value in segment.items() if not isinstance(value,dict)}
        for metric in ('uwb_positioning','px4_positioning','estimated_tracking','actual_tracking'):
            flat.update({metric+'_'+key:value for key,value in segment[metric].items()})
        csv_rows.append(flat)
    with (output/'scenario_metrics.csv').open('x',encoding='utf-8',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(dict.fromkeys(key for row in csv_rows for key in row)))
        writer.writeheader(); writer.writerows(csv_rows)
    (output/'inputs').mkdir()
    index = {}
    for name,path in inputs.items():
        snapshot = Path('inputs')/(name+path.suffix)
        (output/snapshot).write_bytes(source_bytes[name])
        index[name] = dict(path=str(path.resolve()), snapshot=str(snapshot), bytes=len(source_bytes[name]),
                          sha256=hashlib.sha256(source_bytes[name]).hexdigest())
    save('input_index.json', index)
    sources = [Path(__file__), Path(inspect.getfile(error_stats)), Path(inspect.getfile(fault_reports)),
               Path(inspect.getfile(RangeFaultPlan)), Path(inspect.getfile(tag_position)),
               Path(inspect.getfile(validate_config)), Path(inspect.getfile(SITLOdometrySettings))]
    save('manifest.json', dict(scope='offline_flight_evaluation', run_id=output.name,
        parent_run_id=navigation.name, plan=asdict(plan), clock=asdict(clock),
        environment=dict(python=sys.version, numpy=np.__version__, platform=platform.platform()),
        alignment=asdict(alignment), source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        matched_raw_count=len(received), flight_valid=False, fusion_verified=False))
    (output/'results.jsonl').write_text(''.join(json.dumps(row, allow_nan=False)+'\n' for row in rows), encoding='utf-8')
    missing = [dict(time_us=row['time_us'], reasons=row['missing_reasons']) for row in rows if row['missing_reasons']]
    (output/'failures.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in missing), encoding='utf-8')
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation-run', type=Path, required=True)
    parser.add_argument('--range-run', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--clock', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    opts = parser.parse_args(args)
    sys.stdout.reconfigure(encoding='utf-8')
    summary = run(opts.navigation_run, opts.range_run, opts.plan, opts.clock, opts.output)
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
