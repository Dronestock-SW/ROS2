"""Known independent errors must not be mistaken for successful navigation."""
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

import pytest

from drone_demo.flight_evaluation import EvaluationClock, EvaluationPlan, evaluate, run
from drone_uwb.integration.sitl_odometry_contract import SITLOdometrySettings
from drone_uwb.processing.gazebo_geometry import VirtualRanges, tag_position


def fixture(px4_bias=.3):
    alignment = SITLOdometrySettings(px4_reference_offset_body_flu_m=(.14, 0., 0.))
    plan = EvaluationPlan(100_000_000, 103_000_000)
    clock = EvaluationClock(True, 90_000_000, 1000, 10_000_000, 13_000_000, 'synthetic test clock')
    poses, observed, events = [], [], []
    for i in range(61):
        stamp = plan.start_sim_us+i*50000
        pose = dict(source='synthetic_pose_fixture', model=plan.model, clock_domain='gazebo_sim_us',
            time_us=stamp, position_xyz_m=[2., 1., 1.],
            quaternion_wxyz=[math.sqrt(.5), 0., 0., math.sqrt(.5)])
        poses.append(pose)
        reference = tag_position(pose, alignment.px4_reference_offset_body_flu_m).tolist()
        observed.append(dict(transmitted=True, seq=i, time_us=stamp, sample_age_sim_s=.02,
            candidate=dict(reference_point='px4_reference',
                           reference_world_xyz_m=[reference[0]+.03, *reference[1:]])))
        events.append(dict(type='mission_assessment', gazebo_time_us=stamp, assessment=dict(
            px4_sample_time_us=stamp-90_000_000, position_map_xy_m=[reference[0]+px4_bias, reference[1]],
            position_source='px4_ekf2', reference_point='px4_reference', pose_age_s=.02,
            px4_reset_counter=0)))
    target = events[0]['assessment']['position_map_xy_m']
    events.insert(0, dict(type='target_command_sent', gazebo_time_us=plan.start_sim_us,
                         leg_index=0, packet=dict(target_map_xy_m=target)))
    events.append(dict(type='leg_arrived', gazebo_time_us=102_000_000, leg_index=0))
    return poses, observed, events, alignment, plan, clock


def test_separates_positioning_tracking_and_false_fc_arrival():
    rows, result = evaluate(*fixture())
    assert result['uwb_positioning']['max_m'] == pytest.approx(.03)
    assert result['px4_positioning']['max_m'] == pytest.approx(.3)
    assert result['estimated_tracking']['max_m'] == 0.
    assert result['actual_tracking']['max_m'] == pytest.approx(.3)
    assert result['legs'][0]['fc_arrival_confirmed_by_truth'] is False
    assert result['legs'][0]['first_truth_arrival_sim_us'] is None
    assert result['uwb_positioning']['this_interval_gate_passed']
    assert not result['flight_valid'] and not result['full_7cm_goal_achieved']
    # The FC offset rotates with truth attitude; it is not added to world x.
    assert rows[0]['truth_reference_xy_m'] == pytest.approx([2., 1.14])


def test_truth_settle_requires_new_samples_and_reports_actual_arrival():
    _, result = evaluate(*fixture(px4_bias=.03))
    leg = result['legs'][0]
    assert leg['fc_arrival_confirmed_by_truth']
    assert leg['truth_arrival_time_s'] == pytest.approx(1.05)
    assert leg['departures_after_truth_arrival'] == 0


def test_missing_observations_remain_missing_and_cannot_pass_by_filtering():
    args = list(fixture())
    del args[1][20:40]
    rows, result = evaluate(*args)
    assert result['uwb_positioning']['count'] == 41
    assert result['uwb_positioning']['missing_count'] == 20
    assert result['uwb_positioning']['max_gap_s'] == pytest.approx(1.05)
    assert result['uwb_positioning']['max_m'] == pytest.approx(.03)
    assert not result['uwb_positioning']['this_interval_gate_passed']
    assert rows[20]['uwb_position_error_m'] is None


@pytest.mark.parametrize('change', ['unconfirmed', 'excess_uncertainty', 'outside_interval'])
def test_unverified_clock_does_not_fabricate_zero_px4_error(change):
    args = list(fixture())
    args[-1] = replace(args[-1], **({'confirmed': False} if change == 'unconfirmed' else
        {'uncertainty_us': 10001} if change == 'excess_uncertainty' else {'valid_end_px4_us': 1}))
    _, result = evaluate(*args)
    assert result['px4_positioning']['count'] == 0
    assert result['px4_positioning']['max_m'] is None
    assert result['px4_rejections']
    assert result['uwb_positioning']['count'] == 61


def test_exact_target_threshold_is_a_failure_before_rounding():
    args = list(fixture())
    rows, result = evaluate(*args)
    exact = result['uwb_positioning']['max_m']
    args[-2] = replace(args[-2], positioning_target_m=exact)
    _, result = evaluate(*args)
    assert result['uwb_positioning']['at_or_above_target_count'] == 61
    assert not result['uwb_positioning']['this_interval_gate_passed']


def test_unknown_latency_or_truth_gap_prevents_interval_success():
    args = list(fixture())
    args[1][0].pop('sample_age_sim_s')
    _, result = evaluate(*args)
    assert result['uwb_positioning']['max_latency_sim_s'] is None
    assert not result['uwb_positioning']['this_interval_gate_passed']
    args = list(fixture())
    del args[0][20:40]
    _, result = evaluate(*args)
    assert not result['truth_interval_complete']
    assert not result['uwb_positioning']['this_interval_gate_passed']


def test_sparse_px4_is_not_extrapolated_or_interpolated_across_large_gaps():
    args = list(fixture())
    args[2] = [e for e in args[2] if e['type'] != 'mission_assessment'
               or 101_000_000 <= e['gazebo_time_us'] <= 102_000_000]
    rows, result = evaluate(*args)
    assert result['px4_positioning']['count'] == 21
    assert rows[0]['px4_position_error_m'] is None and rows[-1]['px4_position_error_m'] is None
    args = list(fixture())
    args[2] = [e for e in args[2] if e['type'] != 'mission_assessment'
               or not 101_000_000 < e['gazebo_time_us'] < 102_000_000]
    rows, result = evaluate(*args)
    assert result['px4_positioning']['count'] == 42
    assert rows[30]['px4_position_error_m'] is None
    assert rows[30]['px4_interpolation_bracket_s'] == 1.


def test_clock_reset_and_conflicting_duplicate_are_rejected():
    args = list(fixture())
    args[2][30]['assessment']['px4_reset_counter'] = 1
    with pytest.raises(ValueError, match='split_run_required'):
        evaluate(*args)
    args = list(fixture())
    duplicate = deepcopy(args[2][30])
    duplicate['assessment']['position_map_xy_m'][0] += 1
    args[2].insert(31, duplicate)
    with pytest.raises(ValueError, match='conflicting_px4_sample'):
        evaluate(*args)


def test_wrong_model_and_unknown_evaluation_interval_are_rejected():
    args = list(fixture())
    args[0][0]['model'] = 'other_drone'
    with pytest.raises(ValueError, match='truth_model_source_or_time'):
        evaluate(*args)
    with pytest.raises(ValueError, match='evaluation_interval_required'):
        EvaluationPlan(None, None)


def test_aborted_leg_is_not_reported_as_successfully_evaluated_route():
    args = list(fixture())
    args[2].append(dict(type='navigation_aborted', gazebo_time_us=100_500_000))
    _, result = evaluate(*args)
    assert result['legs'][0]['status'] == 'aborted'
    assert result['legs'][0]['fc_arrival_confirmed_by_truth'] is False


def recorded_fixture(tmp_path):
    poses, obs, events, alignment, plan, clock = fixture()
    nav, ranges = tmp_path/'nav', tmp_path/'range'
    nav.mkdir(); ranges.mkdir()
    config = json.loads((Path(__file__).parents[2]/'drone_uwb/config/gazebo_shadow.json').read_text(encoding='utf-8'))
    config['tag_offset_body_flu_m'] = [0., 0., .3]
    source = VirtualRanges(config)
    generated, truth = [], []
    for pose in poses:
        raw, answer = source.sample(pose)
        generated.append(raw); truth.append(answer)
    received = [dict(row, host_clock_domain='wsl_monotonic_ns',
        host_callback_start_monotonic_ns=123000, host_callback_end_monotonic_ns=124000) for row in generated]
    for folder, name, data in ((ranges,'poses.jsonl',poses), (ranges,'raw_ranges.jsonl',generated),
        (ranges,'truth.jsonl',truth), (nav,'raw_ranges.jsonl',received),
        (nav,'sitl_events.jsonl',obs), (nav,'navigation_events.jsonl',events)):
        (folder/name).write_text(''.join(json.dumps(row)+'\n' for row in data),encoding='utf-8')
    for path, data in ((ranges/'config.json', config), (nav/'sitl_odometry.json', asdict(alignment)),
                       (tmp_path/'plan.json',asdict(plan)), (tmp_path/'clock.json',asdict(clock))):
        path.write_text(json.dumps(data),encoding='utf-8')
    return nav, ranges, tmp_path/'plan.json', tmp_path/'clock.json'


def test_actual_record_formats_pair_host_decorated_raw_and_preserve_inputs(tmp_path):
    args = recorded_fixture(tmp_path)
    output = tmp_path/'evaluation'
    summary = run(*args,output)
    assert summary['uwb_positioning']['count'] == 61
    index = json.loads((output/'input_index.json').read_text(encoding='utf-8'))
    for item in index.values():
        assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() == item['sha256']
        assert hashlib.sha256((output/item['snapshot']).read_bytes()).hexdigest() == item['sha256']
    with pytest.raises(FileExistsError):
        run(*args,output)


def test_other_session_raw_and_corrupt_truth_fail_before_writing_output(tmp_path):
    args = recorded_fixture(tmp_path)
    path = args[0]/'raw_ranges.jsonl'
    rows = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    rows[0]['raw_slant_m'][0] += .1
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
    with pytest.raises(ValueError, match='raw_session_pairing_mismatch'):
        run(*args,tmp_path/'evaluation')
    assert not (tmp_path/'evaluation').exists()

    rows[0]['raw_slant_m'][0] -= .1
    # Restore exact JSON bytes from the original generated data, retaining
    # receipt metadata, then test an independently corrupted truth record.
    original = [json.loads(line) for line in (args[1]/'raw_ranges.jsonl').read_text(encoding='utf-8').splitlines()]
    rows[0]['raw_slant_m'] = original[0]['raw_slant_m']
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
    truth_path = args[1]/'truth.jsonl'
    truth = [json.loads(line) for line in truth_path.read_text(encoding='utf-8').splitlines()]
    truth[0]['tag_xyz_m'][0] += .1
    truth_path.write_text(''.join(json.dumps(row)+'\n' for row in truth),encoding='utf-8')
    with pytest.raises(ValueError, match='generator_truth_geometry_mismatch'):
        run(*args,tmp_path/'evaluation')
    assert not (tmp_path/'evaluation').exists()


def test_declared_drop_cannot_also_be_received_and_is_preserved_in_evaluation(tmp_path):
    args = recorded_fixture(tmp_path)
    emitted = [dict(seq=i,time_us=100_000_000+i*50000,drop_requested=i==10,published=i!=10)
               for i in range(61)]
    (args[1]/'emission.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in emitted),encoding='utf-8')
    with pytest.raises(ValueError,match='raw_received_despite_recorded_drop'):
        run(*args,tmp_path/'evaluation')
    raw_path = args[0]/'raw_ranges.jsonl'
    raw_lines = raw_path.read_text(encoding='utf-8').splitlines()
    raw_lines.pop(10)
    raw_path.write_text('\n'.join(raw_lines)+'\n',encoding='utf-8')
    with pytest.raises(ValueError,match='transmitted_observation_without_received_raw'):
        run(*args,tmp_path/'evaluation')
    obs_path = args[0]/'sitl_events.jsonl'
    obs_lines = obs_path.read_text(encoding='utf-8').splitlines()
    obs_lines.pop(10)
    obs_path.write_text('\n'.join(obs_lines)+'\n',encoding='utf-8')
    result = run(*args,tmp_path/'evaluation')
    assert result['uwb_positioning']['missing_count'] == 1
    index = json.loads((tmp_path/'evaluation/input_index.json').read_text(encoding='utf-8'))
    assert 'range_emission' in index
