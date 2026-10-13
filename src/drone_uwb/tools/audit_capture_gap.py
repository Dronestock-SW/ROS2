"""Read-only audit of a B_TF rejection interval in a measured capture.

Uses recorded decisions, not collector-order reconstruction, for causal claims.
Requires numpy and the local drone_uwb package. Optional plotting: matplotlib.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from drone_uwb.processing.observation_guard import ObservationGuard


def read_rows(path):
    return [json.loads(line) for line in path.open(encoding='utf-8') if line.strip()]


def payload(row):
    data = row['data']
    return json.loads(data['data']) if 'data' in data else data


def stats(values):
    values = list(values)
    if not values:
        return {'count': 0}
    return dict(count=len(values), min=float(np.min(values)),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                max=float(np.max(values)))


def compact(d):
    b = d.get('models', {}).get('B', {})
    tf = d.get('models', {}).get('B_TF', {})
    return dict(seq=d['seq'], stamp_ns=d['stamp_ns'], reason=d['reason'],
                published=d['published'], b4_fit_rms_m=b.get('fit', {}).get('rms_m'),
                raw_pair_residual_m=d.get('raw_pair_residual_max_m'),
                raw_slant_m=d.get('raw_slant_m'), height_m=d.get('height_m'),
                height_selection=d.get('height_selection'),
                b3_best_excluded_anchor=tf.get('best_excluded_anchor'),
                b3_best_physical_range_rms_m=tf.get('best_score_m'),
                b4_height_check=tf.get('b4_height_check'),
                processing_ms=d.get('processing_ms'), queue_age_s=d.get('queue_age_s'))


def audit(path, start, end, probe_dir=None):
    rows = read_rows(path)
    decisions = {payload(r)['seq']: payload(r) for r in rows
                 if r.get('topic') == '/uwb/btf_decision' and 'seq' in payload(r)}
    rejected = [decisions[s] for s in range(start, end+1)]
    before, after = decisions[start-1], decisions[end+1]
    if not before['published'] or not after['published'] or any(d['published'] for d in rejected):
        raise ValueError('Expected one complete consecutive rejected interval')
    poses = [r for r in rows if r.get('topic') == '/uwb/btf_pose']
    pose_by_stamp = {r['header_ns']: r for r in poses}
    p0, p1 = pose_by_stamp[before['stamp_ns']], pose_by_stamp[after['stamp_ns']]
    lo, hi = p0['received_monotonic_ns'], p1['received_monotonic_ns']
    in_gap = [r for r in rows if r.get('topic') and lo < r['received_monotonic_ns'] < hi]
    raw = [payload(r) for r in in_gap if r['topic'] == '/uwb/received'
           and payload(r)['message']['type'] == 'uwb_raw_cycle']
    raw_times = [d['message']['cycle_end_us'] / 1e6 for d in raw]
    tof = [r for r in rows if r.get('topic') == '/mavros/downward_0']
    local_tof = [r for r in tof if lo-1e9 <= r['received_monotonic_ns'] <= hi+1e9]
    stale = [d for d in rejected if d['reason'] == 'measured_height_unavailable']
    bad_attitude = [d for d in stale if d['height_selection'][-1]['reason'] == 'attitude_stale']
    attitude_examples = []
    for tof_stamp in sorted({d['height_selection'][-1]['tof_stamp_ns'] for d in bad_attitude}):
        near = [r for r in rows if r.get('topic') == '/mavros/imu/data'
                and abs(r['header_ns']-tof_stamp) <= 25_000_000]
        attitude_examples.append(dict(tof_stamp_ns=tof_stamp,
            selected_imu_stamps_ns=sorted({d['height_selection'][-1]['imu_stamp_ns'] for d in bad_attitude
                                          if d['height_selection'][-1]['tof_stamp_ns'] == tof_stamp}),
            nearby_collector_imu=[dict(stamp_ns=r['header_ns'], received_monotonic_ns=r['received_monotonic_ns'],
                delta_from_tof_ms=(r['header_ns']-tof_stamp)/1e6) for r in near]))
    scores = [d['models']['B_TF']['best_score_m'] for d in rejected
              if d['reason'] == 'no_height_consistent_subset']
    guard = ObservationGuard()
    guard_rows = []
    for p in poses:
        position = p['data']['pose']['pose']['position']
        reason = guard.check([position['x'], position['y']], p['header_ns'])
        if after['stamp_ns'] <= p['header_ns'] <= decisions[end+12]['stamp_ns']:
            guard_rows.append(dict(stamp_ns=p['header_ns'], reason=reason))
    first_ready = next(r for r in guard_rows if r['reason'] == 'ready')
    bridge = [payload(r) for r in rows if r.get('topic') == '/uwb/local_odometry_status']
    old_bridge = next(d for d in reversed(bridge) if d['last_input_stamp_ns'] == before['stamp_ns'])
    new_bridge = next(d for d in bridge if d['last_input_stamp_ns'] >= first_ready['stamp_ns'])
    changes = []
    for seq in range(start-12, start):
        d, prev = decisions[seq], decisions[seq-1]
        delta = np.array(d['raw_slant_m'])-np.array(prev['raw_slant_m'])
        if abs(delta[0]) >= .20:
            changes.append(dict(seq=seq, source_dt_s=(d['time_us']-prev['time_us'])/1e6,
                                range_delta_m=delta.tolist()))
    result = dict(schema=1, input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        source_receipt=next((r for r in rows if r.get('type') == 'source_receipt'), None),
        scope='Recorded B_TF decisions; pose guard replay is separately labelled',
        first_rejected_seq=start, last_rejected_seq=end,
        rejected_count=len(rejected), reasons=dict(Counter(d['reason'] for d in rejected)),
        source_pose_gap_s=(after['stamp_ns']-before['stamp_ns'])/1e9,
        collector_pose_gap_s=(hi-lo)/1e9,
        collector_gap_topics=dict(Counter(r['topic'] for r in in_gap)),
        raw_cycle_count=len(raw), raw_cycle_source_intervals_s=stats(np.diff(raw_times)),
        rejected_processing_ms=stats(d['processing_ms'] for d in rejected),
        rejected_queue_age_s=stats(d['queue_age_s'] for d in rejected),
        unavailable_latest_height_reasons=dict(Counter(d['height_selection'][-1]['reason'] for d in stale)),
        attitude_stale_examples=attitude_examples,
        b3_best_physical_range_rms_m=stats(scores),
        b3_best_excluded_anchor=dict(Counter(str(d['models']['B_TF'].get('best_excluded_anchor'))
            for d in rejected if d['reason'] == 'no_height_consistent_subset')),
        tof_context_source_intervals_s=stats(np.diff([r['header_ns'] for r in local_tof])/1e9),
        tof_context_receipt_intervals_s=stats(np.diff([r['received_monotonic_ns'] for r in local_tof])/1e9),
        a1_steps_over_20cm_before_rejection=changes,
        recorded_bridge_recovery_counter_before=old_bridge['counts']['observation_recovering'],
        recorded_bridge_recovery_counter_after=new_bridge['counts']['observation_recovering'],
        recorded_last_bridge_send_monotonic_ns=old_bridge['last_send_monotonic_ns'],
        guard_replay=dict(first_ready_stamp_ns=first_ready['stamp_ns'],
            withheld_count=sum(r['reason'] != 'ready' for r in guard_rows),
            added_source_delay_s=(first_ready['stamp_ns']-after['stamp_ns'])/1e9,
            resulting_source_gap_s=(first_ready['stamp_ns']-before['stamp_ns'])/1e9,
            rows=guard_rows),
        before=compact(before), after=compact(after), rejected=[compact(d) for d in rejected],
        limitations=['Collector receipt is not node callback execution time.',
            'Guard replay checks source timing; recorded counters independently corroborate it.',
            'Per-anchor ranging discontinuity does not identify its RF or physical cause.',
            'Range consistency and publication rate do not establish position accuracy.'])
    if probe_dir:
        probe = {}
        for name in ['baseline', 'quarantined']:
            ds = [d for d in read_rows(probe_dir / (name+'.jsonl')) if start-9 <= d.get('seq', -1) <= end+10]
            accepted = [d for d in ds if d['replay_publish']]
            probe[name] = dict(published=len(accepted),
                max_source_gap_s=float(np.diff([d['stamp_ns'] for d in accepted]).max()/1e9))
            if name == 'baseline':
                errs = [abs(d['models']['B']['fit']['rms_m'] - decisions[d['seq']]['models']['B']['fit']['rms_m'])
                        for d in ds if 'fit' in d.get('models', {}).get('B', {})]
                probe['baseline_b4_max_abs_rms_difference_m'] = max(errs)
        probe['scope'] = ('Collector-order offline probe; exact B4 comparison only. '
                          'Height callback ordering can differ from the live decisions.')
        result['history_admission_probe'] = probe
    return result, decisions


def plot(result, decisions, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    start, end = result['first_rejected_seq'], result['last_rejected_seq']
    ds = [decisions[s] for s in range(start-14, end+11)]
    seq = [d['seq'] for d in ds]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    ranges = np.array([d['raw_slant_m'] for d in ds])
    for i in range(4):
        axes[0].plot(seq, ranges[:,i]-ranges[0,i], '.-', ms=3, label=f'A{i+1}')
    axes[0].set(ylabel='Range change (m)', title='Recorded measurements and decisions around the B_TF outage')
    axes[1].plot(seq, [d['models']['B']['fit']['rms_m'] for d in ds], label='B4 fit RMS')
    axes[1].plot(seq, [d['models']['B_TF'].get('best_score_m', float('nan')) for d in ds],
                 '.-', label='Best B3 physical-range RMS')
    axes[1].axhline(.06, color='#b42318', ls='--', label='Recorded limit = 0.06 m')
    axes[1].set(ylabel='Residual (m)')
    axes[2].step(seq, [int(d['published']) for d in ds], where='mid', label='Published B_TF pose')
    axes[2].step(seq, [int(d['height_m'] is not None) for d in ds], where='mid', label='Measured height available')
    axes[2].set(ylabel='Available', yticks=[0,1], ylim=(-.1,1.15), xlabel='UWB cycle sequence (about 25 ms per cycle)')
    for ax in axes:
        ax.axvspan(start-.5, end+.5, color='#f3c64e', alpha=.22)
        ax.grid(alpha=.2)
        ax.legend(loc='upper left', fontsize=8)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', type=Path)
    parser.add_argument('--first-rejected', type=int, required=True)
    parser.add_argument('--last-rejected', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--probe-dir', type=Path)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    for output in [args.output, args.plot]:
        if output and output.resolve() == args.events.resolve():
            parser.error('Output must not overwrite the input capture')
    result, decisions = audit(args.events, args.first_rejected, args.last_rejected, args.probe_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    if args.plot:
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        plot(result, decisions, args.plot)
    print(json.dumps({k:v for k,v in result.items() if k not in ['rejected','source_receipt','before','after']},
                     ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
