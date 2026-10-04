"""Declared virtual-range faults; no physical RF model or flight commands."""
from copy import deepcopy
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


EMPTY_PLAN = dict(schema=1, clock_domain='gazebo_sim_us', origin='first_recorded_pose', scenarios=[])


class RangeFaultPlan:
    def __init__(self, plan=None):
        self.plan = deepcopy(EMPTY_PLAN if plan is None else plan)
        if (not isinstance(self.plan, dict) or set(self.plan) != set(EMPTY_PLAN)
                or type(self.plan['schema']) is not int or self.plan['schema'] != 1
                or self.plan['clock_domain'] != 'gazebo_sim_us'
                or self.plan['origin'] != 'first_recorded_pose'
                or not isinstance(self.plan['scenarios'], list)):
            raise ValueError('invalid_live_fault_plan')
        ids, intervals = set(), []
        for event in self.plan['scenarios']:
            if not isinstance(event, dict):
                raise ValueError('invalid_live_fault_event')
            kind = event.get('type')
            expected = {'id', 'type', 'start_s', 'end_s'}
            if kind == 'range_bias':
                expected |= {'anchor_id', 'bias_m'}
                if event.get('anchor_id') not in ('A1', 'A2', 'A3', 'A4') or not finite(event.get('bias_m')):
                    raise ValueError('invalid_range_bias_fault')
            elif kind != 'drop_all':
                raise ValueError('unsupported_live_fault_type')
            if (set(event) != expected or not isinstance(event['id'], str) or not event['id'].strip()
                    or event['id'] in ids or not finite(event['start_s']) or not finite(event['end_s'])
                    or not 0 <= event['start_s'] < event['end_s']):
                raise ValueError('invalid_fault_id_or_interval')
            if not math.isfinite(event['end_s']*1e6):
                raise ValueError('fault_interval_not_representable')
            start, end = round(event['start_s']*1e6), round(event['end_s']*1e6)
            if start >= end:
                raise ValueError('fault_interval_below_timestamp_resolution')
            ids.add(event['id'])
            intervals.append((start, end, event))
        self.intervals = sorted(intervals, key=lambda item: item[0])
        if any(first[1] > second[0] for first,second in zip(self.intervals, self.intervals[1:])):
            raise ValueError('overlapping_live_faults_require_separate_trials')
        self.origin_us = self.previous_us = None

    def apply(self, raw):
        stamp = raw.get('time_us')
        ranges = raw.get('raw_slant_m')
        if (type(stamp) is not int or stamp < 0 or raw.get('clock_domain') != 'gazebo_sim_us'
                or not isinstance(ranges, list) or len(ranges) != 4 or not all(map(finite, ranges))):
            raise ValueError('invalid_fault_input_cycle')
        if self.previous_us is not None and stamp <= self.previous_us:
            raise ValueError('fault_clock_not_increasing')
        if self.origin_us is None:
            self.origin_us = stamp
        elapsed = stamp-self.origin_us
        active = next((event for start,end,event in self.intervals if start <= elapsed < end), None)
        result = deepcopy(raw)
        delta = [0.]*4
        dropped = active is not None and active['type'] == 'drop_all'
        if active is not None and active['type'] == 'range_bias':
            index = int(active['anchor_id'][1])-1
            delta[index] = active['bias_m']
            result['raw_slant_m'][index] += delta[index]
            if not math.isfinite(result['raw_slant_m'][index]):
                raise ValueError('fault_created_nonfinite_range')
            result['valid_mask'] = sum(1 << i for i,v in enumerate(result['raw_slant_m']) if 0 < v <= 80.)
        self.previous_us = stamp
        record = dict(seq=raw['seq'], time_us=stamp, origin_sim_us=self.origin_us,
            elapsed_sim_s=elapsed/1e6, active_fault_id=active['id'] if active else None,
            applied_bias_m=delta, drop_requested=dropped, source='declared_simulation_fault')
        return result, record
