"""Partition a recorded trial using its declared faults, without hiding gaps."""
import math

from drone_demo.flight_metrics import error_stats
from drone_uwb.integration.gazebo.gazebo_faults import RangeFaultPlan


def fresh(row, plan):
    age = row.get('uwb_latency_sim_s')
    return (row.get('uwb_position_error_m') is not None
            and type(age) in (int, float) and math.isfinite(age)
            and 0 <= age <= plan.max_observation_latency_s)


def validate_fault_evidence(generated, originals, emissions, fault_plan):
    """Replay only the declared measurement fault to audit recorded provenance."""
    schedule = RangeFaultPlan(fault_plan)
    if originals is None or emissions is None or not len(generated) == len(originals) == len(emissions):
        raise ValueError('fault_evaluation_evidence_missing_or_count_mismatch')
    for raw, original, emitted in zip(generated, originals, emissions):
        expected_raw, expected = schedule.apply(original)
        if raw != expected_raw:
            raise ValueError('fault_evaluation_modified_raw_mismatch')
        if any(emitted.get(key) != value for key, value in expected.items()):
            raise ValueError('fault_evaluation_schedule_mismatch')
        published = emitted.get('published')
        if published is not None and type(published) is not bool:
            raise ValueError('fault_evaluation_publish_evidence_invalid')
        drop = expected['drop_requested']
        reason = ('scheduled_drop' if drop else 'published' if published is True
                  else 'publish_failed' if published is False else 'publish_exception')
        if ('published' not in emitted or type(emitted.get('drop_requested')) is not bool
                or emitted.get('publish_attempted') is not (not drop)
                or drop and published is not False or emitted.get('reason') != reason):
            raise ValueError('fault_evaluation_publish_evidence_invalid')
    return schedule


def interval_summary(rows, start, end, plan, truth_extent):
    available = [r for r in rows if r['uwb_position_error_m'] is not None]
    ages = [r['uwb_latency_sim_s'] for r in available]
    ages_known = bool(ages) and all(type(v) in (int, float) and math.isfinite(v) and v >= 0 for v in ages)
    coverage = len(available)/len(rows) if rows else None
    times = [start, *[r['time_us'] for r in available], end]
    max_gap = max((b-a)/1e6 for a,b in zip(times,times[1:]))
    truth_times = [start, *[r['time_us'] for r in rows], end]
    truth_complete = (bool(rows) and end > start and truth_extent[0] <= start
                      and truth_extent[1] >= end
                      and all((b-a)/1e6 <= plan.max_truth_gap_s for a,b in zip(truth_times,truth_times[1:])))
    stats = error_stats([r['uwb_position_error_m'] for r in rows], plan.positioning_target_m)
    max_latency = max(ages) if ages_known else None
    stats.update(coverage=coverage, max_gap_s=max_gap, max_latency_sim_s=max_latency,
        this_interval_gate_passed=bool(truth_complete and available
            and stats['max_m'] < plan.positioning_target_m
            and coverage >= plan.min_observation_coverage and max_gap <= plan.max_observation_gap_s
            and max_latency is not None and max_latency <= plan.max_observation_latency_s))
    return dict(truth_rows=len(rows), truth_interval_complete=truth_complete, uwb_positioning=stats,
        px4_positioning=error_stats([r['px4_position_error_m'] for r in rows]),
        estimated_tracking=error_stats([r['px4_target_error_m'] for r in rows]),
        actual_tracking=error_stats([r['truth_target_error_m'] for r in rows]))


def fault_reports(rows, generated, received, originals, emissions, fault_plan, plan, truth_extent):
    """Return a disjoint partition plus sample-time recovery diagnostics.

    Whole-interval metrics remain the responsibility of the caller. Recovery
    diagnostics do not certify FC receipt, control recovery, or mission resume.
    """
    if fault_plan is None:
        schedule = None
        intervals = []
    else:
        schedule = validate_fault_evidence(generated, originals, emissions, fault_plan)
        intervals = [(schedule.origin_us+a,schedule.origin_us+b,event)
                     for a,b,event in schedule.intervals]
    recovery_us = round(plan.recovery_window_s*1e6)
    begin, finish = plan.start_sim_us, plan.end_sim_us

    def label(stamp):
        for start,end,event in intervals:
            if start <= stamp < end:
                return 'fault', event['id'], event['type']
        for start,end,event in reversed(intervals):
            if end <= stamp < end+recovery_us:
                return 'recovery', event['id'], event['type']
        return ('normal' if schedule is not None else 'unclassified'), None, None

    cuts = sorted({begin, finish, *[t for start,end,_ in intervals
                                  for t in (start,end,end+recovery_us) if begin < t < finish]})
    spans = []
    for a,b in zip(cuts,cuts[1:]):
        kind = label(a)
        if spans and spans[-1][3] == kind:
            spans[-1] = spans[-1][0],b,False,kind
        else:
            spans.append((a,b,False,kind))
    if label(finish) == spans[-1][3]:
        a,b,_,kind = spans[-1]
        spans[-1] = a,b,True,kind
    else:
        # Preserve a fault beginning exactly at the inclusive final truth tick.
        spans.append((finish,finish,True,label(finish)))
    segments = []
    for index,(start,end,inclusive,kind) in enumerate(spans):
        contains = lambda stamp: start <= stamp < end or inclusive and stamp == end
        selected = [r for r in rows if contains(r['time_us'])]
        emitted = [r for r in emissions or [] if contains(r['time_us'])]
        segment_id = f'segment_{index:03d}'
        for row in selected:
            row.update(scenario_id=segment_id, scenario_kind=kind[0], fault_id=kind[1])
        segments.append(dict(scenario_id=segment_id, kind=kind[0], fault_id=kind[1], fault_type=kind[2],
            start_sim_us=start, end_sim_us=end, end_inclusive=inclusive,
            recorded_raw_count=sum(contains(r['time_us']) for r in generated),
            received_raw_count=sum(contains(r['time_us']) for r in received),
            planned_drop_count=sum(r.get('drop_requested') is True for r in emitted) if emissions is not None else None,
            publisher_success_count=sum(r.get('published') is True for r in emitted) if emissions is not None else None,
            publisher_failure_count=sum(r.get('reason') == 'publish_failed' for r in emitted) if emissions is not None else None,
            publisher_exception_count=sum(r.get('reason') == 'publish_exception' for r in emitted) if emissions is not None else None,
            **interval_summary(selected,start,end,plan,truth_extent)))
    assert sum(s['truth_rows'] for s in segments) == len(rows)

    recoveries = []
    for index,(start,end,event) in enumerate(intervals):
        next_start = intervals[index+1][0] if index+1 < len(intervals) else None
        desired_end = end+recovery_us
        limit = min(desired_end, next_start) if next_start is not None else desired_end
        selected = [r for r in rows if end <= r['time_us'] < limit]
        observed = next((r['time_us'] for r in selected if fresh(r,plan)), None)
        settling = previous = recovered = None
        for row in selected:
            stamp = row['time_us']
            continuous = (previous is not None and (stamp-previous)/1e6
                          <= min(plan.max_truth_gap_s,plan.max_observation_gap_s))
            if not continuous:
                settling = None
            if fresh(row,plan) and row['uwb_position_error_m'] < plan.positioning_target_m:
                settling = stamp if settling is None else settling
                if recovered is None and (stamp-settling)/1e6 >= plan.recovery_settle_s:
                    recovered = stamp
            else:
                settling = None
            previous = stamp
        truth_times = [end, *[r['time_us'] for r in selected], desired_end]
        complete = (bool(selected) and begin <= end and finish >= desired_end and truth_extent[0] <= end
                    and truth_extent[1] >= desired_end and limit == desired_end)
        complete = complete and all((b-a)/1e6 <= plan.max_truth_gap_s
                                    for a,b in zip(truth_times,truth_times[1:]))
        recoveries.append(dict(fault_id=event['id'], fault_type=event['type'],
            fault_start_sim_us=start, fault_end_sim_us=end,
            recovery_start_sim_us=end, recovery_end_exclusive_sim_us=limit,
            desired_recovery_end_sim_us=desired_end, window_fully_recorded=bool(complete),
            recovery_truth_rows=len(selected), first_fresh_observed_sample_us=observed,
            first_fresh_observed_delay_s=(observed-end)/1e6 if observed is not None else None,
            first_accuracy_settled_sample_us=recovered,
            accuracy_settled_delay_s=(recovered-end)/1e6 if recovered is not None else None,
            settle_s=plan.recovery_settle_s, observation_recovery_seen=recovered is not None,
            flight_recovery_verified=False))
    return dict(partition_source='validated_fault_plan' if schedule is not None else 'fault_plan_unavailable',
                scenarios=segments, recoveries=recoveries)
