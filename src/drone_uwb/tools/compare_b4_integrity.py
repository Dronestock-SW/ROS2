"""Compare aligned file replays; include censored/all-rejected output gaps.

This is analysis only. It opens files, not ROS, serial, or flight-control clients.
Publication counts are not correctness labels without independent position truth.
"""
import argparse
from collections import Counter
import hashlib
from itertools import zip_longest
import json
import math
from pathlib import Path


def gaps(rows, field):
    if not rows:
        return dict(published=0, longest_complete_s=None, longest_including_boundary_s=None)
    times = [t for t, flags in rows if flags[field]]
    closed = [b-a for a, b in zip(times, times[1:])]
    bounded = [b-a for a, b in zip([rows[0][0], *times], [*times, rows[-1][0]])]
    return dict(published=len(times), duration_s=rows[-1][0]-rows[0][0],
                longest_complete_s=max(closed, default=None),
                longest_including_boundary_s=max(bounded),
                all_rejected=not times)


def compare(before, after, *, start=None, end=None):
    segments = []; active = False; before_counts = Counter(); after_counts = Counter()
    sources = Counter(); geometry = Counter(); removed = added = 0
    changed_upstream = 0; delta = []; b4_delta = []; current = None
    hashes = [hashlib.sha256(), hashlib.sha256()]
    with before.open('rb') as old_file, after.open('rb') as new_file:
        for old_line, new_line in zip_longest(old_file, new_file):
            if old_line is None or new_line is None:
                raise ValueError('replay decision count mismatch')
            hashes[0].update(old_line); hashes[1].update(new_line)
            old, new = json.loads(old_line), json.loads(new_line)
            if any(old.get(k) != new.get(k) for k in ('received_monotonic_ns', 'seq', 'time_us', 'stamp_ns', 'airborne')):
                raise ValueError('unaligned replay identity')
            t = new['received_monotonic_ns']/1e9
            included = bool(new['airborne'] and (start is None or t >= start) and (end is None or t <= end))
            if not included:
                active = False
                continue
            if not active:
                current = []; segments.append(current)
            active = True
            a, b = old['replay_publish'], new['replay_publish']
            current.append((t, (a, b)))
            before_counts[old['reason']] += 1; after_counts[new['reason']] += 1
            removed += int(a and not b); added += int(b and not a)
            changed_upstream += int(any(old.get(k) != new.get(k) for k in
                ('raw_slant_m', 'raw_xy_m', 'stamp_ns', 'jump_check', 'replay_guard_prior'))
                or old.get('models', {}).get('B') != new.get('models', {}).get('B'))
            candidate = new.get('models', {}).get('B_TF', {})
            integrity = candidate.get('b4_height_check', {})
            if integrity:
                geometry[integrity['state']] += 1
            if b:
                sources[candidate['source']] += 1
            if a and b:
                distance = math.dist(old['xy_m'], new['xy_m']); delta.append(distance)
                if candidate.get('source') == 'unchanged_B4':
                    b4_delta.append(distance)
    def summary(field):
        parts = [gaps(s, field) for s in segments]
        return dict(published=sum(p['published'] for p in parts), segments=parts,
            longest_complete_s=max((p['longest_complete_s'] for p in parts if p['longest_complete_s'] is not None), default=None),
            longest_including_boundary_s=max((p['longest_including_boundary_s'] for p in parts), default=None))
    return dict(scope='aligned file replay, no independent movement truth or flight validation',
                before_sha256=hashes[0].hexdigest(), after_sha256=hashes[1].hexdigest(),
                before=summary(0), after=summary(1), removed=removed, added=added,
                before_reasons=dict(before_counts), after_reasons=dict(after_counts),
                after_sources=dict(sources), b4_height_states=dict(geometry),
                upstream_changed=changed_upstream, shared_xy_delta_max_m=max(delta, default=None),
                shared_B4_xy_delta_max_m=max(b4_delta, default=None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path); parser.add_argument('after', type=Path)
    parser.add_argument('--start', type=float); parser.add_argument('--end', type=float)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.before, args.after, start=args.start, end=args.end)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('before', 'after', 'removed', 'added', 'upstream_changed')}))


if __name__ == '__main__':
    main()
