"""Audit recorded RAW/TDMA transport. This does not assess position accuracy."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

from drone_uwb.acquisition.tdma import TdmaGate
from drone_uwb.acquisition.validation import InputValidator, InvalidInput
from drone_uwb.contracts.protocol import InvalidSample
from drone_uwb.processing.settings import PreimuSettings


def audit(events, tag_id, *, seconds=600, warmup_s=10):
    if tag_id not in ('5', '6') or type(seconds) is not int or seconds < 1 or warmup_s < 0:
        raise ValueError('invalid_audit_window')
    tdma = TdmaGate(tag_id, 'required')
    validator = InputValidator(PreimuSettings(tag_id=tag_id))
    accepted, rejected = [], Counter()
    first = last = None
    for event in events:
        try:
            mono, ros = event['host_received_monotonic_ns'], event['host_received_ros_ns']
            if type(mono) is not int or type(ros) is not int or mono <= 0 or ros <= 0:
                raise ValueError('invalid_envelope_time')
            first = mono if first is None else first
            last = mono
            result = tdma.ingest(event['message'], mono, ros)
            if result.reset_history:
                validator.reset()
            if result.clear_status:
                validator.disconnect()
            if result.reason == 'status':
                validator.on_status(result.message, result.mono_ns)
            elif result.reason == 'raw':
                cycle = validator.on_cycle(result.message, result.mono_ns, result.ros_ns)
                if len(cycle.indices) != 4:
                    raise ValueError('not_four_valid_ranges')
                accepted.append((result.mono_ns, cycle.seq, tuple(tdma.identity)))
            elif not result.reason.startswith('sideband_') and result.reason != 'awaiting_tdma':
                rejected[result.reason] += 1
        except (ValueError, KeyError, TypeError, InvalidInput, InvalidSample) as exc:
            rejected[str(exc)] += 1
    start = math.ceil(first/1e9)+warmup_s if first is not None else 0
    end = start+seconds
    counts = [0]*seconds
    pairs = [row for row in accepted if start*1e9 <= row[0] < end*1e9]
    for mono, _, _ in pairs:
        counts[int(mono//1_000_000_000)-start] += 1
    gaps, sessions = [], set()
    for i, (_, seq, session) in enumerate(pairs):
        sessions.add(session)
        if i and pairs[i-1][2] == session:
            gaps.append(((seq-pairs[i-1][1]) & 0xffffffff)-1)
    covered = first is not None and last >= end*1e9
    rate_ok = covered and min(counts) >= 35
    loss_ok = covered and len(sessions) == 1 and bool(gaps) and 0 <= max(gaps) <= 1
    return dict(scope='RECORDED_RAW_TDMA_TRANSPORT_ONLY', tag_id=tag_id,
        window_seconds=seconds, warmup_s=warmup_s, complete_window=covered,
        count=len(pairs), mean_hz=len(pairs)/seconds, min_1s_hz=min(counts),
        max_1s_hz=max(counts), windows_below_35hz=sum(v < 35 for v in counts),
        max_consecutive_missing=max(gaps) if gaps else None, session_count=len(sessions),
        rate_pass=rate_ok, consecutive_loss_pass=loss_ok, transport_pass=rate_ok and loss_ok,
        rejection_counts=dict(rejected), pending_pair_counts=dict(tdma.counts),
        btf_accuracy_verified=False, fusion_verified=False, flight_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--received', type=Path, required=True)
    parser.add_argument('--tag-id', required=True, choices=('5', '6'))
    parser.add_argument('--seconds', type=int, default=600)
    parser.add_argument('--warmup-s', type=int, default=10)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    # Fail on damaged envelopes rather than silently omit a recording gap.
    with args.received.open(encoding='utf-8') as handle:
        result = audit((json.loads(line) for line in handle), args.tag_id,
                       seconds=args.seconds, warmup_s=args.warmup_s)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
