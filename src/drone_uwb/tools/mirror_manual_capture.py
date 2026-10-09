#!/usr/bin/env python3
"""PC-side SSH mirror of an existing capture. Read remote files only; no FC IO.

Reconnects at the local byte offset. A completed file is verified by SHA256.
No key/password is accepted on the command line; use existing SSH key access.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import time


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--remote-dir', required=True)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--retry-seconds', type=int, default=120)
    args = parser.parse_args()
    if args.host.startswith('-') or not args.remote_dir.startswith('/') or args.pid <= 1 or not 0 <= args.retry_seconds <= 600:
        parser.error('Require an SSH host, absolute remote directory, PID >1 and retry 0..600s')
    args.output.mkdir(parents=True, exist_ok=True)
    identity = args.output/'mirror-source.json'
    source = dict(host=args.host, remote_dir=args.remote_dir, pid=args.pid)
    if (args.output/'events.jsonl').exists() and not identity.exists():
        raise ValueError('existing_events_have_no_mirror_identity')
    if identity.exists() and json.loads(identity.read_text(encoding='utf-8')) != source:
        raise ValueError('refusing_to_mix_capture_sessions')
    identity.write_text(json.dumps(source), encoding='utf-8')
    ssh = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
           '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', args.host]
    errors = (args.output/'mirror-ssh.log').open('ab')
    events = args.output/'events.jsonl'
    remote_file = args.remote_dir.rstrip('/')+'/events.jsonl'
    status = dict(complete=False, sha256_verified=False, fc_commands_sent=0)

    def remote(command):
        return subprocess.run(ssh+[command], stdout=subprocess.PIPE, stderr=errors, timeout=30)

    def metadata():
        for name in ('manifest.json', 'summary.json', 'markers.jsonl'):
            result = remote('cat -- '+shlex.quote(args.remote_dir.rstrip('/')+'/'+name))
            if result.returncode == 0:
                # Never replace a good JSON checkpoint with a partial transfer.
                if name.endswith('.json'):
                    json.loads(result.stdout)
                if name == 'manifest.json' and (args.output/name).exists():
                    prior = json.loads((args.output/name).read_bytes())
                    current = json.loads(result.stdout)
                    if (prior['boot_id'], prior['started']) != (current['boot_id'], current['started']):
                        raise ValueError('remote_capture_identity_changed')
                temporary = args.output/(name+'.tmp')
                temporary.write_bytes(result.stdout)
                temporary.replace(args.output/name)

    def checkpoint():
        status.update(updated_utc=datetime.now(timezone.utc).isoformat(),
                      copied_bytes=events.stat().st_size if events.exists() else 0)
        temporary = args.output/'mirror-status.tmp'
        temporary.write_text(json.dumps(status, indent=2)+'\n', encoding='utf-8')
        temporary.replace(args.output/'mirror-status.json')

    disconnected_since = None
    try:
        while True:
            try:
                metadata()
                offset = events.stat().st_size if events.exists() else 0
                command = f'tail --pid={args.pid} --sleep-interval=0.2 -c +{offset+1} -f -- '+shlex.quote(remote_file)
                with subprocess.Popen(ssh+[command], stdout=subprocess.PIPE, stderr=errors) as process:
                    try:
                        with events.open('ab', buffering=0) as output:
                            while True:
                                block = process.stdout.read(64*1024)
                                if not block:
                                    break
                                output.write(block)
                                disconnected_since = None
                                checkpoint()
                    except BaseException:
                        process.terminate()
                        raise
                    code = process.wait()
                metadata()
                summary_path = args.output/'summary.json'
                summary = json.loads(summary_path.read_text(encoding='utf-8')) if summary_path.exists() else {}
                if code == 0 and summary.get('stopped'):
                    result = remote('sha256sum -- '+shlex.quote(remote_file))
                    expected = result.stdout.decode('ascii').split()[0] if result.returncode == 0 else None
                    actual = digest(events)
                    status.update(sha256=actual, sha256_verified=actual == expected,
                                  complete=actual == expected, source_recording_error=summary.get('error'))
                    checkpoint()
                    return 0 if status['complete'] else 1
                status['last_error'] = 'ssh_ended_or_source_not_finalized'
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                status['last_error'] = type(exc).__name__+':'+str(exc)
            disconnected_since = disconnected_since or time.monotonic()
            checkpoint()
            if time.monotonic()-disconnected_since >= args.retry_seconds:
                return 1
            time.sleep(2)
    finally:
        checkpoint()
        errors.close()


if __name__ == '__main__':
    raise SystemExit(main())
