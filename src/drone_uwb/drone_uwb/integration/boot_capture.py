"""Persistent rotating ROS capture supervisor. Never connects to the FC.

Only complete, continuously observed DISARM ground chunks may expire.
Armed, uncertain, partial and interrupted chunks are never auto-deleted.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

from drone_uwb.integration.manual_capture import atomic_json, boot_id, clock_record


def ground_only(manifest, summary):
    if (manifest.get('scope') != 'manual_flight_receive_only'
            or not summary.get('stopped') or summary.get('error')
            or not summary.get('writer_drained')):
        return False
    proof = summary.get('ground_observation', {})
    return (proof.get('all_ground') is True and proof.get('all_disarmed') is True
            and proof.get('all_connected') is True and proof.get('all_fresh') is True
            and proof.get('state_count', 0) >= 10 and proof.get('landed_count', 0) >= 10
            and proof.get('max_state_gap_s', 99) <= 1.5
            and proof.get('max_landed_gap_s', 99) <= 1.5
            and proof.get('first_state_delay_s', 99) <= 1.5
            and proof.get('first_landed_delay_s', 99) <= 1.5
            and proof.get('last_state_age_s', 99) <= 1.5
            and proof.get('last_landed_age_s', 99) <= 1.5)


def prune_idle(root, keep=3):
    root = Path(root).resolve()
    candidates = []
    chunks = []
    for path in root.glob('capture-*'):
        if path.is_symlink() or not path.is_dir() or path.resolve().parent != root:
            continue
        chunks.append(path)
        try:
            manifest = json.loads((path/'manifest.json').read_text())
            summary = json.loads((path/'summary.json').read_text())
            if ground_only(manifest, summary):
                candidates.append((manifest['started']['wall_ns'], path))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    eligible = {p for _,p in candidates}
    ordered = sorted(chunks)
    # Keep the immediately preceding ground context of every flight/uncertain
    # chunk as well, even after many later idle chunks have accumulated.
    context = {ordered[i-1] for i,p in enumerate(ordered) if i and p not in eligible}
    candidates = [(t,p) for t,p in candidates if p not in context]
    removed = []
    for _, path in sorted(candidates)[:max(0, len(candidates)-keep)]:
        # Scope and resolved containment were verified; unlink only this chunk.
        shutil.rmtree(path)
        removed.append(path.name)
    return removed


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--tag', choices=['A','B'], default='B')
    p.add_argument('--config', type=Path, action='append', default=[])
    p.add_argument('--seconds', type=int, default=180)
    p.add_argument('--max-mib', type=int, default=256)
    p.add_argument('--reserve-mib', type=int, default=1024)
    p.add_argument('--keep-idle', type=int, default=3)
    p.add_argument('--source-revision', default='unspecified')
    p.add_argument('--with-lidar', action='store_true')
    args = p.parse_args(argv)
    if not (10 <= args.seconds <= 3600 and 1 <= args.max_mib <= 1024
            and args.reserve_mib >= 512 and args.keep_idle >= 3):
        p.error('invalid capture capacity or retention')
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    # Prevent a second service/manual invocation from racing retention/recording.
    import fcntl
    lock = (root/'supervisor.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    stopped = []
    child = None
    def stop(sig, frame):
        stopped.append(sig)
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGTERM)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    status = dict(scope='manual_capture_boot', boot_id=boot_id(), flight_authorized=False,
                  fc_parameter_writes=False, fc_flight_commands=False)
    try:
        while not stopped:
            removed = prune_idle(root, args.keep_idle)
            free = shutil.disk_usage(root).free
            status.update(updated=clock_record(), removed_idle=removed, free_bytes=free,
                          active=False, capture=None)
            if free < (args.reserve_mib+args.max_mib)*1024*1024:
                status['reason'] = 'storage_reserve_waiting_no_flight_deletion'
                atomic_json(root/'status.json', status)
                time.sleep(5)
                continue
            directory = root/('capture-'+str(time.time_ns())+'-'+uuid.uuid4().hex[:8])
            command = [sys.executable, '-m', 'drone_uwb.integration.ros.manual_capture',
                'record', '--tag', args.tag, '--output', str(directory), '--seconds', str(args.seconds),
                '--max-mib', str(args.max_mib), '--reserve-mib', str(args.reserve_mib),
                '--source-revision', args.source_revision, '--note', 'Boot sensor-only capture; no mission or EV bridge.']
            for config in args.config:
                command.extend(['--config', str(config)])
            if args.with_lidar:
                command.append('--with-lidar')
            child = subprocess.Popen(command)
            status.update(active=True, reason='collecting', capture=str(directory), pid=child.pid)
            atomic_json(root/'status.json', status)
            while child.poll() is None:
                time.sleep(1)
                status['updated'] = clock_record()
                # Persist flushed sensor data periodically; abrupt power loss may
                # still lose the current writer buffer / last checkpoint.
                if int(time.monotonic()) % 10 == 0 and (directory/'events.jsonl').exists():
                    with (directory/'events.jsonl').open('rb') as stream:
                        os.fsync(stream.fileno())
                atomic_json(root/'status.json', status)
            status.update(active=False, returncode=child.returncode, updated=clock_record())
            atomic_json(root/'status.json', status)
            if child.returncode:
                time.sleep(5)
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            child.wait(timeout=15)
        status.update(active=False, reason='supervisor_stopped', updated=clock_record())
        atomic_json(root/'status.json', status)
        lock.close()


if __name__ == '__main__':
    main()
