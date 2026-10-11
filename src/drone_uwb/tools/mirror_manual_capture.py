#!/usr/bin/env python3
"""Bounded SSH snapshots of a passive capture. No FC IO or remote writes.

Resume by byte offset; detect reboot, PID reuse, stale recorder and truncation.
A completed archive requires the source to stop and its whole SHA256 to match.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
from pathlib import Path
import shlex
import subprocess
import time


CHUNK_BYTES = 4*1024*1024
BOUNDARY_BYTES = 64*1024
# Every invocation exits. Never tail --pid on an ID reused after reboot.
REMOTE_SNAPSHOT = r'''
import hashlib, json, pathlib, sys, time
directory, offset, pid = pathlib.Path(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
try:
    manifest = json.loads((directory/'manifest.json').read_bytes())
    summary = json.loads((directory/'summary.json').read_bytes())
    marker_path = directory/'markers.jsonl'
    markers = marker_path.read_text(encoding='utf-8') if marker_path.exists() else ''
    if len(markers.encode('utf-8')) > 4*1024*1024:
        raise ValueError('marker_file_too_large')
    if not summary.get('stopped'):
        boot = pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if boot != manifest['boot_id']:
            raise ValueError('source_host_rebooted')
        process = manifest.get('process', {})
        if process.get('pid', pid) != pid:
            raise ValueError('source_pid_mismatch')
        stat = pathlib.Path('/proc', str(pid), 'stat').read_text()
        if process.get('start_ticks') is not None and stat.rsplit(')',1)[1].split()[19] != process['start_ticks']:
            raise ValueError('source_pid_reused')
        age = (time.monotonic_ns()-summary['updated']['monotonic_ns'])/1e9
        if not 0 <= age <= 15:
            raise ValueError('source_checkpoint_stale')
    with (directory/'events.jsonl').open('rb') as stream:
        stream.seek(0, 2)
        size = stream.tell()
        if offset > size:
            raise ValueError('source_file_shrank')
        stream.seek(0)
        first = stream.read(min(offset, 65536))
        stream.seek(max(0, offset-65536))
        boundary = stream.read(min(offset, 65536))
        stream.seek(offset)
        block = stream.read(min(4*1024*1024, size-offset))
        whole = None
        if summary.get('stopped') and offset+len(block) == size:
            stream.seek(0)
            h = hashlib.sha256()
            for part in iter(lambda: stream.read(1024*1024), b''):
                h.update(part)
            whole = h.hexdigest()
    header = dict(manifest=manifest, summary=summary, markers=markers,
        source_bytes=size, offset=offset, count=len(block),
        first_sha256=hashlib.sha256(first).hexdigest(),
        boundary_sha256=hashlib.sha256(boundary).hexdigest(),
        block_sha256=hashlib.sha256(block).hexdigest(), sha256=whole)
except (OSError, ValueError, KeyError) as exc:
    header, block = dict(error=type(exc).__name__+':'+str(exc)), b''
sys.stdout.buffer.write(json.dumps(header, separators=(',',':')).encode('utf-8')+b'\n'+block)
'''


class SourceError(ValueError):
    """Known source/session failure; reconnecting must not hide it."""


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_write(path, data):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_bytes(data)
    temporary.replace(path)


def accept_snapshot(output, payload):
    """Validate the complete response before appending any bytes."""
    header, separator, block = payload.partition(b'\n')
    if not separator:
        raise ValueError('incomplete_snapshot_header')
    info = json.loads(header)
    if info.get('error'):
        raise SourceError(info['error'])
    events = output/'events.jsonl'
    offset = events.stat().st_size if events.exists() else 0
    if (info['offset'] != offset or not 0 <= info['count'] <= CHUNK_BYTES
            or info['count'] != len(block) or offset+len(block) > info['source_bytes']
            or hashlib.sha256(block).hexdigest() != info['block_sha256']):
        raise ValueError('incomplete_or_invalid_snapshot_block')
    current = info['manifest']
    prior_path = output/'manifest.json'
    if prior_path.exists():
        prior = json.loads(prior_path.read_bytes())
        if any(prior.get(k) != current.get(k) for k in ('boot_id', 'started', 'process')):
            raise SourceError('remote_capture_identity_changed')
    first = boundary = b''
    if offset:
        with events.open('rb') as stream:
            first = stream.read(min(offset, BOUNDARY_BYTES))
            stream.seek(max(0, offset-BOUNDARY_BYTES))
            boundary = stream.read(min(offset, BOUNDARY_BYTES))
    if (hashlib.sha256(first).hexdigest() != info['first_sha256']
            or hashlib.sha256(boundary).hexdigest() != info['boundary_sha256']):
        raise SourceError('resume_prefix_mismatch')
    for line in info['markers'].splitlines():
        json.loads(line)
    for name, data in [('manifest.json', current), ('summary.json', info['summary'])]:
        atomic_write(output/name, json.dumps(data, ensure_ascii=False, indent=2).encode('utf-8'))
    atomic_write(output/'markers.jsonl', info['markers'].encode('utf-8'))
    with events.open('ab') as stream:
        stream.write(block)
    return info


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--connect-address', help='Verified LAN IP; retain original host-key identity')
    parser.add_argument('--remote-dir', required=True)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--retry-seconds', type=int, default=120)
    args = parser.parse_args(argv)
    if args.host.startswith('-') or not args.remote_dir.startswith('/') or args.pid <= 1 or not 0 <= args.retry_seconds <= 600:
        parser.error('Require an SSH host, absolute remote directory, PID >1 and retry 0..600s')
    args.output.mkdir(parents=True, exist_ok=True)
    identity = args.output/'mirror-source.json'
    source = dict(host=args.host, remote_dir=args.remote_dir, pid=args.pid)
    if (args.output/'events.jsonl').exists() and not identity.exists():
        raise ValueError('existing_events_have_no_mirror_identity')
    if identity.exists() and json.loads(identity.read_text(encoding='utf-8')) != source:
        raise ValueError('refusing_to_mix_capture_sessions')
    atomic_write(identity, json.dumps(source).encode('utf-8'))
    ssh = ['ssh', '-C', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
           '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2']
    if args.connect_address:
        address = str(ipaddress.ip_address(args.connect_address))
        ssh += ['-o', 'Hostname='+address, '-o', 'HostKeyAlias='+args.host.rsplit('@',1)[-1],
                '-o', 'CheckHostIP=no']
    ssh.append(args.host)
    events = args.output/'events.jsonl'
    status = dict(complete=False, sha256_verified=False, fc_commands_sent=0,
                  connect_address=args.connect_address)

    def checkpoint():
        status.update(updated_utc=datetime.now(timezone.utc).isoformat(),
                      copied_bytes=events.stat().st_size if events.exists() else 0)
        atomic_write(args.output/'mirror-status.json', (json.dumps(status, indent=2)+'\n').encode('utf-8'))

    disconnected_since = None
    try:
        with (args.output/'mirror-ssh.log').open('ab') as errors:
            while True:
                try:
                    offset = events.stat().st_size if events.exists() else 0
                    command = shlex.join(['python3', '-c', REMOTE_SNAPSHOT,
                                          args.remote_dir, str(offset), str(args.pid)])
                    result = subprocess.run(ssh+[command], stdout=subprocess.PIPE, stderr=errors, timeout=30)
                    if result.returncode:
                        raise OSError('ssh_exit_'+str(result.returncode))
                    info = accept_snapshot(args.output, result.stdout)
                    disconnected_since = None
                    status.pop('last_error', None)
                    status.update(source_bytes=info['source_bytes'],
                                  backlog_bytes=info['source_bytes']-offset-info['count'],
                                  source_recording_error=info['summary'].get('error'))
                    if info['summary'].get('stopped') and not status['backlog_bytes']:
                        actual = digest(events)
                        status.update(sha256=actual, sha256_verified=actual == info['sha256'],
                                      complete=actual == info['sha256'])
                        if not status['complete']:
                            status['last_error'] = 'final_sha256_mismatch'
                        checkpoint()
                        return 0 if status['complete'] else 1
                    checkpoint()
                    if not status['backlog_bytes']:
                        time.sleep(.5)
                    continue
                except SourceError as exc:
                    status['last_error'] = str(exc)
                    return 1
                except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
                    status['last_error'] = type(exc).__name__+':'+str(exc)
                disconnected_since = disconnected_since or time.monotonic()
                checkpoint()
                if time.monotonic()-disconnected_since >= args.retry_seconds:
                    return 1
                time.sleep(2)
    finally:
        checkpoint()


if __name__ == '__main__':
    raise SystemExit(main())
