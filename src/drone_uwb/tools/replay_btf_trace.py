"""File-only B_TF replay using internal sensor ingress and UWB callback order.

No ROS, sockets or aircraft connection. A source root and native library are
explicit, so old/new code can run in separate interpreters on identical inputs.
Recorded transport delays and invalid measurements are not repaired by replay.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import tarfile


def run(args):
    sys.path.insert(0, str(args.package_root.resolve()))
    from drone_uwb.processing.measured_btf import MeasuredBtf, height_independent_b4
    rows = []; config = None; corrupt = []
    with tarfile.open(args.trace) as archive:
        for member in archive.getmembers():
            if not member.isfile():
                continue
            session = None
            for ordinal, line in enumerate(archive.extractfile(member), 1):
                try:
                    item = json.loads(line)
                except ValueError:
                    corrupt.append([member.name, ordinal]); continue
                if item.get('type') == 'context':
                    session = item['session']
                    if session == args.session:
                        config = item['config']
                    continue
                if session == args.session and item.get('stream') == 'inputs':
                    rows.append(item)
    if config is None:
        raise ValueError('trace_session_not_found')
    config['native_btf_library'] = str(args.library.resolve())
    processor = MeasuredBtf(config)
    rows.sort(key=lambda r:r['enqueued_monotonic_ns'])
    decisions = []; counts = Counter(); published = []; ground = False
    for item in rows:
        row = item['row']; kind = row.get('type'); mono = item['enqueued_monotonic_ns']
        if kind in ('imu', 'tof'):
            processor.height.add(kind, row)
        elif kind == 'timesync' and row.get('clock_changed'):
            for buffer in processor.height.samples.values():
                buffer.clear()
            processor.reset_models()
        elif kind == 'sensor_reset':
            if args.legacy or row['reason'] != 'clock_not_ready_at_snapshot':
                for buffer in processor.height.samples.values():
                    buffer.clear()
        elif kind == 'fc_landed':
            ground = row['landed_state'] == 1
        elif kind == 'uwb':
            event = row['event']; now = row['callback_ros_ns']
            if args.legacy and not row['clock_sync_ready']:
                for buffer in processor.height.samples.values():
                    buffer.clear()
            processor.height.ground_reference = (dict(height_m=config['ground_antenna_height_m'],
                selection_stamp_ns=now, ground_state_stamp_ns=now)
                if ground and row['clock_sync_ready'] and config.get('ground_antenna_height_m') else None)
            result = processor.process(event)
            if result['reason'] in ('status','awaiting_tdma') or result['reason'].startswith('sideband_'):
                continue
            publish = bool(result['ok'] and 0 <= now-result['stamp_ns'] <= config['max_output_age_s']*1e9
                and (result.get('xyz_m') is not None or height_independent_b4(result)))
            result.update(replay_publish=publish, received_monotonic_ns=mono, callback_ros_ns=now)
            decisions.append(result)
            if args.start_ros_ns <= now <= args.end_ros_ns:
                counts[result['reason']] += 1
                counts['published'] += int(publish)
                if publish:
                    published.append(mono)
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output/'decisions.jsonl').open('w', encoding='utf-8') as f:
        for r in decisions:
            f.write(json.dumps(r, allow_nan=False)+'\n')
    result = dict(counts=dict(counts), max_complete_gap_s=max(
        ((b-a)/1e9 for a,b in zip(published,published[1:])), default=None),
        scope='internal trace callback-order numerical replay; no physical flight or position truth',
        legacy=args.legacy, inputs=len(rows), corrupt_lines=corrupt,
        trace_sha256=hashlib.sha256(args.trace.read_bytes()).hexdigest(),
        library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256((args.package_root/'drone_uwb/processing/measured_btf.py').read_bytes()).hexdigest())
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace', type=Path, required=True)
    p.add_argument('--session', required=True)
    p.add_argument('--package-root', type=Path, required=True)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--start-ros-ns', type=int, required=True)
    p.add_argument('--end-ros-ns', type=int, required=True)
    p.add_argument('--legacy', action='store_true')
    run(p.parse_args())
