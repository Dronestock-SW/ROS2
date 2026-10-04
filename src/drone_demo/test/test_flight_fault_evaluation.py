"""Fault segments must expose gaps, bad recovery and inconsistent evidence."""
import csv
import json

import pytest

from drone_demo.flight_evaluation import EvaluationPlan, run
from drone_demo.flight_fault_evaluation import fault_reports
from drone_uwb.integration.gazebo_faults import RangeFaultPlan
from test_flight_evaluation import recorded_fixture


def read_rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def write_rows(path, rows):
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')


def fault_fixture(tmp_path, kind='drop_all', second_event=None):
    args = recorded_fixture(tmp_path)
    nav, ranges, plan_path, _ = args
    event = dict(id='test_fault',type=kind,start_s=.5,end_s=1.)
    if kind == 'range_bias':
        event.update(anchor_id='A2',bias_m=.5)
    fault_plan = dict(schema=1,clock_domain='gazebo_sim_us',origin='first_recorded_pose',scenarios=[event])
    if second_event is not None:
        fault_plan['scenarios'].append(second_event)
    originals = read_rows(ranges/'raw_ranges.jsonl')
    schedule = RangeFaultPlan(fault_plan)
    modified, received, emissions = [], [], []
    drop_keys = set()
    for original in originals:
        raw, metadata = schedule.apply(original)
        drop = metadata['drop_requested']
        metadata.update(published=not drop,publish_attempted=not drop,
                        reason='scheduled_drop' if drop else 'published')
        modified.append(raw); emissions.append(metadata)
        if drop:
            drop_keys.add(raw['seq'])
        else:
            received.append(raw)
    write_rows(ranges/'raw_before_fault.jsonl', originals)
    write_rows(ranges/'raw_ranges.jsonl', modified)
    write_rows(ranges/'emission.jsonl', emissions)
    write_rows(nav/'raw_ranges.jsonl', received)
    write_rows(nav/'sitl_events.jsonl',[r for r in read_rows(nav/'sitl_events.jsonl') if r['seq'] not in drop_keys])
    (ranges/'fault_plan.json').write_text(json.dumps(fault_plan),encoding='utf-8')
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    plan.update(recovery_window_s=1.,recovery_settle_s=.2)
    plan_path.write_text(json.dumps(plan),encoding='utf-8')
    return args


def test_fault_partition_and_csv_preserve_the_failed_whole_interval(tmp_path):
    args = fault_fixture(tmp_path)
    result = run(*args,tmp_path/'evaluation')
    segments = result['scenarios']
    assert [s['kind'] for s in segments] == ['normal','fault','recovery','normal']
    assert [s['truth_rows'] for s in segments] == [10,10,20,21]
    assert sum(s['truth_rows'] for s in segments) == result['truth_rows'] == 61
    fault = segments[1]
    assert fault['planned_drop_count'] == 10
    assert fault['received_raw_count'] == 0
    assert fault['uwb_positioning']['count'] == 0
    assert fault['uwb_positioning']['max_m'] is None
    assert fault['uwb_positioning']['missing_count'] == 10
    assert not fault['uwb_positioning']['this_interval_gate_passed']
    assert result['uwb_positioning']['missing_count'] == 10
    assert not result['uwb_positioning']['this_interval_gate_passed']
    recovery = result['recoveries'][0]
    assert recovery['window_fully_recorded']
    assert recovery['first_fresh_observed_delay_s'] == 0.
    assert recovery['accuracy_settled_delay_s'] == pytest.approx(.2)
    assert recovery['observation_recovery_seen'] and not recovery['flight_recovery_verified']
    assert not result['flight_valid'] and not result['full_7cm_goal_achieved']
    with (tmp_path/'evaluation/scenario_metrics.csv').open(encoding='utf-8',newline='') as stream:
        csv_rows = list(csv.DictReader(stream))
    assert csv_rows[1]['uwb_positioning_max_m'] == ''
    assert csv_rows[1]['uwb_positioning_count'] == '0'
    assert len({r['scenario_id'] for r in read_rows(tmp_path/'evaluation/results.jsonl')}) == 4


def test_recovery_requires_fresh_accurate_continuous_samples(tmp_path):
    args = fault_fixture(tmp_path)
    path = args[0]/'sitl_events.jsonl'
    events = [r for r in read_rows(path) if r['seq'] not in set(range(20,26)) | {29}]
    next(r for r in events if r['seq']==28)['candidate']['reference_world_xyz_m'][0] += .2
    write_rows(path,events)
    result = run(*args,tmp_path/'evaluation')
    recovery = result['recoveries'][0]
    assert recovery['first_fresh_observed_delay_s'] == pytest.approx(.3)
    assert recovery['accuracy_settled_delay_s'] == pytest.approx(.7)
    assert result['scenarios'][2]['uwb_positioning']['at_or_above_target_count'] == 1
    assert result['scenarios'][2]['uwb_positioning']['missing_count'] == 7


def test_bias_replay_validates_actual_raw_and_preserves_original_values(tmp_path):
    args = fault_fixture(tmp_path,'range_bias')
    result = run(*args,tmp_path/'evaluation')
    segment = result['scenarios'][1]
    assert segment['fault_type'] == 'range_bias'
    assert segment['received_raw_count'] == 10 and segment['planned_drop_count'] == 0
    originals = read_rows(args[1]/'raw_before_fault.jsonl')
    raw = read_rows(args[1]/'raw_ranges.jsonl')
    assert raw[10]['raw_slant_m'][1]-originals[10]['raw_slant_m'][1] == pytest.approx(.5)
    # This fixture supplies arbitrary observation estimates. Its error values
    # do not assert that the position algorithm corrected this injected bias.
    originals[10]['raw_slant_m'][1] += .1
    write_rows(args[1]/'raw_before_fault.jsonl',originals)
    with pytest.raises(ValueError,match='modified_raw_mismatch'):
        run(*args,tmp_path/'bad_evaluation')
    assert not (tmp_path/'bad_evaluation').exists()


@pytest.mark.parametrize('damage',['id','published','missing_row','missing_originals'])
def test_inconsistent_fault_evidence_is_rejected(tmp_path,damage):
    args = fault_fixture(tmp_path)
    path = args[1]/'emission.jsonl'
    rows = read_rows(path)
    if damage=='id':
        rows[10]['active_fault_id']='wrong_event'
    elif damage=='published':
        rows[10]['published']=True
    elif damage=='missing_row':
        rows.pop(10)
    else:
        (args[1]/'raw_before_fault.jsonl').unlink()
    write_rows(path,rows)
    with pytest.raises(ValueError,match='fault_evaluation_'):
        run(*args,tmp_path/'evaluation')
    assert not (tmp_path/'evaluation').exists()


def test_no_recovery_is_missing_and_short_record_cannot_claim_full_window(tmp_path):
    args = fault_fixture(tmp_path)
    path = args[0]/'sitl_events.jsonl'
    write_rows(path,[r for r in read_rows(path) if not 20 <= r['seq'] < 40])
    result = run(*args,tmp_path/'evaluation')
    recovery = result['recoveries'][0]
    assert recovery['window_fully_recorded']
    assert recovery['first_fresh_observed_delay_s'] is None
    assert recovery['accuracy_settled_delay_s'] is None
    assert not recovery['observation_recovery_seen']
    plan = json.loads(args[2].read_text(encoding='utf-8'))
    plan['end_sim_us']=101_200_000
    args[2].write_text(json.dumps(plan),encoding='utf-8')
    shorter = run(*args,tmp_path/'short_evaluation')
    assert not shorter['recoveries'][0]['window_fully_recorded']


def test_fault_start_at_last_evaluation_tick_is_not_mislabeled_normal(tmp_path):
    args = fault_fixture(tmp_path)
    plan = json.loads(args[2].read_text(encoding='utf-8'))
    plan['end_sim_us']=100_500_000
    args[2].write_text(json.dumps(plan),encoding='utf-8')
    result = run(*args,tmp_path/'evaluation')
    assert [s['kind'] for s in result['scenarios']] == ['normal','fault']
    assert result['scenarios'][-1]['truth_rows'] == 1
    assert not result['scenarios'][-1]['truth_interval_complete']


def test_legacy_logs_are_unclassified_and_recovery_limit_must_be_predeclared(tmp_path):
    args = recorded_fixture(tmp_path)
    result = run(*args,tmp_path/'evaluation')
    assert result['partition_source'] == 'fault_plan_unavailable'
    assert len(result['scenarios']) == 1 and result['scenarios'][0]['kind']=='unclassified'
    assert result['recoveries'] == []
    with pytest.raises(ValueError,match='recovery_settle_must_fit_window'):
        EvaluationPlan(1,2,recovery_settle_s=2.,recovery_window_s=2.)


def test_missing_truth_inside_recovery_prevents_full_record_claim(tmp_path):
    args = fault_fixture(tmp_path)
    run(*args,tmp_path/'evaluation')
    rows = [r for r in read_rows(tmp_path/'evaluation/results.jsonl') if not 101_100_000 < r['time_us'] < 101_600_000]
    ranges=args[1]
    result=fault_reports(rows,read_rows(ranges/'raw_ranges.jsonl'),read_rows(args[0]/'raw_ranges.jsonl'),
        read_rows(ranges/'raw_before_fault.jsonl'),read_rows(ranges/'emission.jsonl'),
        json.loads((ranges/'fault_plan.json').read_text(encoding='utf-8')),
        EvaluationPlan(**json.loads(args[2].read_text(encoding='utf-8'))),(100_000_000,103_000_000))
    assert not result['recoveries'][0]['window_fully_recorded']


def test_next_fault_truncates_recovery_without_double_counting_truth(tmp_path):
    args = fault_fixture(tmp_path,second_event=dict(
        id='second_fault',type='drop_all',start_s=1.5,end_s=1.75))
    result = run(*args,tmp_path/'evaluation')
    segments = result['scenarios']
    assert [s['kind'] for s in segments] == ['normal','fault','recovery','fault','recovery','normal']
    assert sum(s['truth_rows'] for s in segments) == 61
    assert sum(s['planned_drop_count'] for s in segments) == 15
    first,second=result['recoveries']
    assert first['recovery_end_exclusive_sim_us'] == 101_500_000
    assert not first['window_fully_recorded']
    assert second['window_fully_recorded']


@pytest.mark.parametrize('window,settle',[(1e308,.5),(1.,1e-7)])
def test_recovery_window_respects_microsecond_resolution(window,settle):
    with pytest.raises(ValueError,match='recovery_time_not_representable'):
        EvaluationPlan(1,2,recovery_window_s=window,recovery_settle_s=settle)
