"""Start an observed or explicitly enabled native-hover session. No boot commands."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['FLIGHT', 'SITL'], default='FLIGHT')
    parser.add_argument('--state-dir', type=Path, default=Path('/tmp/dronestock-native-hover'))
    parser.add_argument('--binary', type=Path, default=WORKSPACE/'install/sangwon_ai_replay/bin/sangwon_native_hover')
    parser.add_argument('--start-mavros', action='store_true')
    parser.add_argument('--fcu-url')
    parser.add_argument('--port', type=int, default=8350)
    parser.add_argument('--enable-output', action='store_true')
    parser.add_argument('--rc-handoff-verified', action='store_true', help='Only after a documented physical RC handoff test')
    parser.add_argument('--evaluate-rc-in-flight', action='store_true', help='Operator-selected first-hover evaluation; does not claim physical RC verification')
    args = parser.parse_args()
    if args.rc_handoff_verified and args.evaluate_rc_in_flight:
        parser.error('Choose verified evidence or a pending first-flight evaluation, not both')
    if args.profile == 'FLIGHT' and args.enable_output and not (args.rc_handoff_verified or args.evaluate_rc_in_flight):
        parser.error('FLIGHT output requires RC handoff evidence or an explicitly selected first-hover evaluation')
    domain = '173' if args.profile == 'SITL' else '2'
    if args.profile == 'SITL':
        if args.fcu_url and args.fcu_url != 'udp://127.0.0.1:14540@127.0.0.1:14580':
            parser.error('SITL uses the fixed loopback UDP endpoint; hardware serial is forbidden')
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = domain
    state = args.state_dir.resolve()
    if len(str(state/'hover.sock').encode()) >= 108:
        parser.error('state directory too long for Unix IPC')
    state.mkdir(parents=True, exist_ok=True, mode=0o700)
    if not args.binary.is_file():
        parser.error('native hover binary missing; run colcon build first')
    if args.start_mavros and args.profile == 'FLIGHT':
        result = subprocess.run(['fuser', '/dev/pixhawk'], capture_output=True)
        if result.returncode == 0:
            parser.error('/dev/pixhawk is already owned; reuse its MAVROS or stop its owner explicitly')
        if args.fcu_url and args.fcu_url != '/dev/pixhawk:921600':
            parser.error('FLIGHT serial endpoint must be /dev/pixhawk:921600')
    config = state/'config.json'
    config.write_text(json.dumps(dict(profile=args.profile, ros_domain_id=int(domain),
        output_enabled=args.enable_output, rc_auto_mode_handoff_verified=args.rc_handoff_verified,
        evaluate_rc_in_flight=args.evaluate_rc_in_flight,
        takeoff_height_m=1.3, state_dir=str(state)), indent=2)+'\n', encoding='utf-8')
    processes, logs = [], []

    def spawn(name, command):
        log = (state/(name+'.log')).open('ab')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        processes.append(process)
        return process

    def stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        if args.start_mavros:
            url = args.fcu_url or ('udp://127.0.0.1:14540@127.0.0.1:14580' if args.profile == 'SITL' else '/dev/pixhawk:921600')
            spawn('mavros', ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '-r', '__ns:=/mavros',
                '--params-file', str(ROOT/'config/mavros.native_hover.yaml'),
                '-p', 'fcu_url:='+url, '-p', 'gcs_url:=""', '-p', 'tgt_system:=1', '-p', 'tgt_component:=1', '-p', 'fcu_protocol:=v2.0'])
        spawn('controller', [str(args.binary), '--config', str(config)])
        spawn('web', [sys.executable, str(ROOT/'ops/native_hover_web.py'), '--socket', str(state/'hover.sock'), '--port', str(args.port)])
        (state/'pids.json').write_text(json.dumps({'launcher': os.getpid(), 'children': [p.pid for p in processes]}), encoding='utf-8')
        print(f'{args.profile} domain {domain}; output={args.enable_output}; http://127.0.0.1:{args.port}', flush=True)
        print(f'logs: {state}; START is a separate operator request', flush=True)
        while all(p.poll() is None for p in processes):
            time.sleep(.2)
        return 2
    except KeyboardInterrupt:
        return 0
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        for log in logs:
            log.close()


if __name__ == '__main__':
    sys.exit(main())
