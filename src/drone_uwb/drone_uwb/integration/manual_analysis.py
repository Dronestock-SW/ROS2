"""Offline audit of a passive capture. No ROS, FC writes or calibration output."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

from drone_uwb.acquisition.tdma import TdmaGate
from drone_uwb.acquisition.validation import InputValidator, InvalidInput
from drone_uwb.contracts.protocol import InvalidSample
from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.timing.clock import ClockMap


class ClockReplay:
    """Replay only the RAW/TDMA/clock stage, not callback scheduling or geometry."""
    def __init__(self, tag_id, tdma_mode='required'):
        self.settings = PreimuSettings(tag_id=tag_id)
        self.gate = TdmaGate(tag_id, tdma_mode)
        self.validator = InputValidator(self.settings)
        self.clock = ClockMap(self.settings)
        self.counts, self.clock_reasons = Counter(), Counter()
        self.last_mono = self.last_offset = None
        self.first_pair = self.last_pair = self.worst = None

    def reset(self):
        self.gate.disconnect(); self.validator.disconnect(); self.clock.reset()

    def add(self, event):
        diagnostic = None
        try:
            mono, ros = event['host_received_monotonic_ns'], event['host_received_ros_ns']
            if any(type(t) is not int or t <= 0 for t in (mono, ros)):
                raise InvalidInput('invalid_host_time', reset=True)
            offset = ros-mono
            jumped = ((self.last_mono is not None and mono < self.last_mono)
                or (self.last_offset is not None and abs(offset-self.last_offset) > 250_000_000))
            self.last_mono, self.last_offset = mono, offset
            if jumped:
                self.reset(); raise InvalidInput('host_clock_jump')
            dispatch = self.gate.ingest(event['message'], mono, ros)
            if dispatch.reset_history:
                self.validator.reset(); self.clock.reset()
            if dispatch.clear_status: self.validator.disconnect()
            reason = dispatch.reason
            if dispatch.message is not None:
                msg, mono, ros = dispatch.message, dispatch.mono_ns, dispatch.ros_ns
                self.validator.check_common(msg)
                if msg['type']=='uwb_raw_status':
                    if msg.get('event')=='boot':
                        self.validator.disconnect(); self.clock.reset()
                    self.validator.on_status(msg, mono)
                    if msg.get('range_bias_applied') is True:
                        raise InvalidInput('already_bias_corrected_input', reset=True)
                else:
                    cycle = self.validator.on_cycle(msg, mono, ros)
                    if len(cycle.indices)!=4: raise InvalidInput('four_valid_ranges_required')
                    self.clock.update(cycle.end_us, mono)
                    diagnostic = self.clock.diagnostics()
                    self.clock_reasons[diagnostic['reason']] += 1
                    pair = dict(source_us=cycle.end_us, host_monotonic_ns=mono)
                    self.first_pair = self.first_pair or pair
                    self.last_pair = pair
                    if self.worst is None or abs(diagnostic['alpha']-1) > abs(self.worst['alpha']-1):
                        self.worst = dict(diagnostic, **pair)
                    reason = self.clock.state
        except InvalidInput as exc:
            reason = exc.reason
            if exc.reset: self.reset()
        except InvalidSample as exc:
            self.gate.discard_pending('invalid_message'); reason = str(exc)
        self.counts[reason] += 1
        return diagnostic

    def report(self):
        return dict(stage_counts=dict(self.counts), clock_reasons=dict(self.clock_reasons),
                    worst_rate=self.worst, first_pair=self.first_pair, last_pair=self.last_pair,
                    scope='RAW_TDMA_CLOCK_ONLY; B_TF callback scheduling and geometry not replayed',
                    fixed_delay_calibrated=False)


def analyze(rows, *, tag_id, tdma_mode='required'):
    replay = ClockReplay(tag_id, tdma_mode)
    topics, decisions = {}, defaultdict(Counter)
    transitions, clock_windows = [], {}
    state, state_mono, first = None, None, None
    tof_max = None
    raw_clock_first = raw_clock_last = None
    for row in rows:
        topic, mono = row['topic'], row['received_monotonic_ns']
        first = mono if first is None else first
        stat = topics.setdefault(topic, dict(count=0, max_gap_s=0., gaps_over_250ms=0, last_mono_ns=mono))
        gap = (mono-stat['last_mono_ns'])/1e9
        stat['count'] += 1
        stat['max_gap_s'] = max(stat['max_gap_s'], gap)
        stat['gaps_over_250ms'] += int(gap > .25)
        stat['last_mono_ns'] = mono
        data = row['data']
        if row['type']=='std_msgs/msg/String': data = json.loads(data['data'])
        if topic=='/mavros/state':
            current = {k:data.get(k) for k in ('connected','armed','mode')}
            if state != current: transitions.append(dict(time_s=(mono-first)/1e9, **current))
            state, state_mono = current, mono
        phase = ('armed' if state['armed'] else 'disarmed') if (state and state['connected']
            and state['armed'] in (True,False) and 0 <= mono-state_mono <= 1_500_000_000) else 'unknown'
        if topic=='/uwb/btf_decision':
            decisions['all'][data.get('reason','missing_reason')] += 1
            decisions[phase][data.get('reason','missing_reason')] += 1
        elif topic=='/uwb/received':
            raw_ns = data.get('host_received_monotonic_raw_ns')
            if raw_ns is not None:
                pair = (data['host_received_monotonic_ns'], raw_ns)
                raw_clock_first = raw_clock_first or pair; raw_clock_last = pair
            diagnostic = replay.add(data)
            if diagnostic:
                index = int((mono-first)/1e9)//10*10
                win = clock_windows.setdefault(index, dict(count=0, rejected=0, min_alpha=math.inf,
                    max_alpha=-math.inf, max_residual_p95_s=0., reasons=Counter()))
                win['count'] += 1; win['rejected'] += int(diagnostic['state']!='ok')
                win['min_alpha'] = min(win['min_alpha'],diagnostic['alpha'])
                win['max_alpha'] = max(win['max_alpha'],diagnostic['alpha'])
                win['max_residual_p95_s'] = max(win['max_residual_p95_s'],diagnostic['residual_p95_s'] or 0.)
                win['reasons'][diagnostic['reason']] += 1
        elif topic=='/mavros/downward_0':
            value = data.get('range')
            if type(value) in (int,float) and math.isfinite(value) and data['min_range'] <= value <= data['max_range']:
                tof_max = max(tof_max or value,value)
    clock_delta = None
    if raw_clock_first and raw_clock_last:
        clock_delta = ((raw_clock_last[0]-raw_clock_first[0])-(raw_clock_last[1]-raw_clock_first[1]))/1e9
    return dict(schema=1, scope='PASSIVE_CAPTURE_AUDIT_NOT_CALIBRATION', topics=topics,
        state_transitions=transitions, recorded_btf_reasons=dict(decisions),
        max_valid_downward_range_m=tof_max, clock_replay=replay.report(), clock_10s_windows=clock_windows,
        host_monotonic_minus_raw_elapsed_s=clock_delta, alignment_confirmed=False,
        timing_confirmed=False, flight_authorized=False, fc_commands_sent=0,
        limitation='PX4 position is not independent truth. Matching ULog and measured motion are still required.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--tdma-mode',choices=['required','auto','disabled'],default='required')
    args=parser.parse_args(argv)
    manifest=json.loads((args.capture/'manifest.json').read_text(encoding='utf-8'))
    evidence_names = ('events.jsonl', 'manifest.json', 'summary.json', 'markers.jsonl',
                      'mirror-source.json', 'mirror-status.json', 'mirror-ssh.log')
    if args.output.resolve() in [(args.capture/name).resolve() for name in evidence_names]:
        parser.error('analysis output must not replace capture evidence')
    digest=hashlib.sha256()
    def rows():
        with (args.capture/'events.jsonl').open('rb') as source:
            for line in source:
                digest.update(line)
                yield json.loads(line)
    result=analyze(rows(),tag_id=manifest['tag_id'],tdma_mode=args.tdma_mode)
    result.update(events_sha256=digest.hexdigest(), source_revision=manifest.get('source_revision'),
                  boot_id=manifest['boot_id'], capture_started=manifest['started'])
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(output=str(args.output),clock_replay=result['clock_replay'],
                         flight_authorized=False)))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
