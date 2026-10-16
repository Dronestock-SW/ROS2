"""ON/OFF/OFF/ON comparison on one fixed C++ configuration, isolated ROS only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from btf_replay_probe import distribution


def resources():
    result = {}
    for name in ('stat', 'vmstat', 'meminfo', 'loadavg', 'pressure/cpu', 'pressure/io', 'pressure/memory'):
        try:
            result[name] = Path('/proc/'+name).read_text()
        except OSError as exc:
            # Jetson kernels may omit PSI. Missing metrics are not zero pressure.
            result[name] = dict(unavailable=type(exc).__name__)
    return result


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(folder):
    summary = json.loads((folder/'summary.json').read_text())
    probe = json.loads((folder/'probe.json').read_text())
    rows = [json.loads(line) for line in (folder/'decisions.jsonl').read_text().splitlines()]
    poses = [r for r in rows if r.get('published')]
    source_stamps = [r['stamp_ns'] for r in poses]
    out = dict(trace=summary['trace_mode'], input_sha256=summary['input_sha256'],
        config_sha256=summary['config_sha256'], sent=summary['sent'],
        published=len(poses), decisions=len(rows), reasons=summary['reasons'],
        max_feeder_lateness_ms=summary['max_feeder_lateness_ms'],
        feeder_lateness_over_5ms=summary['feeder_lateness_over_5ms'],
        receiver_age_ms=distribution([r['receiver_age_ns']/1e6 for r in rows if 'receiver_age_ns' in r]),
        processing_ms=distribution([r['processing_ms'] for r in rows if 'processing_ms' in r]),
        output_source_gap_ms=distribution([(b-a)/1e6 for a,b in zip(source_stamps,source_stamps[1:])]),
        probe=probe, counts=(summary['last_status'] or {}).get('counts'),
        trace_status=(summary['last_status'] or {}).get('trace'),
        numerical_backend=(summary['last_status'] or {}).get('numerical_backend'),
        failures=summary['failures'], child_exit_code=summary['child_exit_code'])
    if probe['dropped']:
        out['failures'].append('probe_capacity_exceeded')
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--events-file', required=True)
    parser.add_argument('--config-file', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '99' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires ROS_DOMAIN_ID=99 and ROS_LOCALHOST_ONLY=1')
    config = json.loads(args.config_file.read_text())
    if config.get('numerical_backend') != 'cpp':
        parser.error('comparison requires numerical_backend=cpp')
    args.output.mkdir(parents=True, exist_ok=False)
    package = Path(__file__).resolve().parents[1]/'drone_uwb'
    source_hashes = {str(p.relative_to(package)):sha(p) for p in sorted(package.rglob('*.py'))}
    tool_hashes = {p.name:sha(p) for p in
                  (Path(__file__), Path(__file__).with_name('btf_replay_probe.py'),
                   Path(__file__).with_name('replay_btf_ros_capture.py'))}
    input_hash = sha(args.capture/args.events_file)
    config_hash, library_hash = sha(args.config_file), sha(config['native_btf_library'])
    runs = []
    for index, mode in enumerate(('on', 'off', 'off', 'on'), 1):
        if (sha(args.capture/args.events_file) != input_hash or sha(args.config_file) != config_hash
                or sha(config['native_btf_library']) != library_hash):
            raise RuntimeError('comparison_input_or_binary_changed')
        if any(sha(package/name) != digest for name,digest in source_hashes.items()):
            raise RuntimeError('comparison_source_changed')
        folder = args.output/f'{index}-{mode}'
        command = [sys.executable, str(Path(__file__).with_name('replay_btf_ros_capture.py')),
            str(args.capture), '--events-file', args.events_file, '--config-file', str(args.config_file),
            '--trace-mode', mode, '--probe', '--output', str(folder)]
        before = subprocess.check_output(['ps','-eo','user,pid,stat,comm,args'], text=True)
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True)
        samples = []
        try:
            while process.poll() is None:
                samples.append(dict(monotonic_ns=time.monotonic_ns(), **resources()))
                time.sleep(1)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log = process.communicate()[0]
        (args.output/f'{index}-{mode}.log').write_text(log, encoding='utf-8')
        (args.output/f'{index}-{mode}-resources.json').write_text(
            json.dumps(dict(processes_before=before, samples=samples)), encoding='utf-8')
        if process.returncode:
            raise RuntimeError(f'replay_failed:{folder}:exit={process.returncode}')
        runs.append(summarize(folder))
        print(json.dumps(dict(run=index, trace=mode, published=runs[-1]['published'],
                             timing=runs[-1]['processing_ms'])), flush=True)
    result = dict(schema=1, scope='partial/original capture as explicitly named; collector receipt replay, no FC',
        input_sha256=input_hash, config_file_sha256=config_hash, library_sha256=library_hash,
        source_hashes=source_hashes, tool_hashes=tool_hashes, order=['on','off','off','on'], runs=runs)
    (args.output/'comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
