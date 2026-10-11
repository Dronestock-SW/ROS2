#!/usr/bin/env python3
"""Launch the FINAL field stack after sourcing ROS. Never submits START/ARM.

Private configuration is required; an old boot alignment is rejected.
Run under the same OS user as MAVROS and the B_TF observations.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('config', 'alignment', 'site-config', 'mission-store', 'state-dir', 'ledger'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--check-only', action='store_true')
    a = p.parse_args()
    for path in (a.config, a.alignment, a.site_config, a.mission_store, a.ledger):
        if not path.is_file():
            p.error('Required private file missing: ' + str(path))
    cfg = json.loads(a.config.read_text(encoding='utf-8'))
    alignment = json.loads(a.alignment.read_text(encoding='utf-8'))['settings']
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if alignment['host_boot_id'] != boot or cfg['bridge_host_boot_id'] != boot:
        p.error('New boot: stationary heading/alignment review is required')
    for key in ('enu_yaw_deg', 'map_y_axis_sign', 'enu_offset_x_m', 'enu_offset_y_m'):
        if cfg[key] != alignment[key]:
            p.error('Observation/mission transform mismatch: ' + key)
    if cfg.get('command_frame') != 'local_xy' or cfg.get('execution_profile') != 'anchor_trial':
        p.error('Expected explicit local_xy anchor_trial configuration')
    if not os.environ.get('AMENT_PREFIX_PATH'):
        p.error('Source ROS and the workspace install/setup.bash first')
    manifest = a.state_dir / 'processes.json'
    if manifest.exists():
        for row in json.loads(manifest.read_text()):
            if row.get('boot_id') == boot and Path('/proc', str(row['pid'])).exists():
                p.error('Existing session is running; inspect it before restarting')
    for port in (8001, 8002):
        with socket.socket() as sock:
            try:
                sock.bind(('127.0.0.1', port))
            except OSError:
                p.error('Port already in use: ' + str(port))
    if a.check_only:
        print('Configuration precheck passed; FC readiness not tested; no processes started')
        return
    root = Path(__file__).resolve().parents[3]
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID='2', ROS_LOCALHOST_ONLY='1', MAVLINK20='1',
               DRONESTOCK_SERVER_URL='http://127.0.0.1:8001',
               DRONESTOCK_WS_URL='ws://127.0.0.1:8002/ws/drones/6/',
               DRONESTOCK_DRONE_ID='6', DRONESTOCK_MISSION_FORWARDING='true',
               DRONESTOCK_STATE_DIR=str(a.state_dir / 'telemetry'))
    env['PYTHONPATH'] = ':'.join(str(root / 'src' / x) for x in
                               ('drone_mission', 'drone_uwb', 'drone_platform_link')) + ':' + env.get('PYTHONPATH', '')
    a.state_dir.mkdir(parents=True, exist_ok=True)
    code = ('import rclpy\nfrom drone_mission.node import FlightNode\n'
            'rclpy.init()\nn=FlightNode()\nn.streams_requested=True\nrclpy.spin(n)')
    commands = [
        ('web', ['-c', 'from drone_mission.local_web import main; main()',
                 '--config', str(a.config), '--site-config', str(a.site_config),
                 '--mission-store', str(a.mission_store)]),
        ('uwb_bridge', ['-m', 'drone_uwb.integration.ros.local_odometry_bridge',
                        '--config', str(a.alignment), '--same-user-dds']),
        ('mission', ['-c', code, '--ros-args', '-p', 'execute:=true', '-p',
                     'config_file:=' + str(a.config), '-p',
                     'ledger_file:=' + str(a.ledger), '-p',
                     'record_directory:=' + str(a.state_dir / ('events-' + str(time.time_ns())))]),
        ('telemetry', ['-m', 'drone_platform_link.runtime']),
    ]
    children, rows = [], []
    try:
        for name, args in commands:
            with (a.state_dir / (name + '.log')).open('ab') as log:
                child = subprocess.Popen([sys.executable, *args], env=env,
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
            children.append(child)
            rows.append(dict(name=name, pid=child.pid, pgid=child.pid, boot_id=boot))
            manifest.write_text(json.dumps(rows, indent=2), encoding='utf-8')
            time.sleep(1)
            if any(c.poll() is not None for c in children):
                raise RuntimeError('A process exited; inspect session logs')
    except BaseException:
        import signal
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        raise
    print('Session started; verify /local/status. Fresh operator START is still required.')


if __name__ == '__main__':
    main()
