"""Ground-only ESP32 backup; run as the observation service owner on Jetson."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

SERVICE = 'dronestock-manual-observe.service'
EXPECTED_MAC = 'e8:3d:c1:e3:48:2c'


def ground_snapshot(timeout=12.0):
    import rclpy
    from mavros_msgs.msg import State, ExtendedState
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('esp32_backup_ground_guard')
    latest = {}
    node.create_subscription(State, '/mavros/state',
        lambda msg: latest.update(state=(time.monotonic(), msg)), qos_profile_sensor_data)
    node.create_subscription(ExtendedState, '/mavros/extended_state',
        lambda msg: latest.update(extended=(time.monotonic(), msg)), qos_profile_sensor_data)
    end = time.monotonic() + timeout
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(latest) == 2:
                ts, state = latest['state']
                te, extended = latest['extended']
                if max(time.monotonic()-ts, time.monotonic()-te) < 2:
                    result = {'connected': state.connected, 'armed': state.armed,
                              'mode': state.mode, 'landed_state': extended.landed_state,
                              'checked_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
                    if not state.connected or state.armed or extended.landed_state != 1:
                        raise RuntimeError('Fresh FC state does not permit ground-only reset: '+str(result))
                    return result
        raise RuntimeError('No fresh FC ground state; no service/device changes made')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    os.umask(0o077)
    root = Path.home() / 'esp32-single-tag-20261015'
    out = root / 'backup' / datetime.datetime.now().strftime('tag-%Y%m%d-%H%M%S')
    out.mkdir(parents=True)
    result = {'phase': 'pre_backup', 'ground': ground_snapshot()}

    def run(args, name, timeout=60):
        completed = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        output = completed.stdout + completed.stderr
        (out / name).write_text(output, encoding='utf-8')
        if completed.returncode:
            raise RuntimeError(f'{name} exited {completed.returncode}: {output[-1500:]}')
        return output

    active = subprocess.run(['systemctl', '--user', 'is-active', '--quiet', SERVICE]).returncode == 0
    result['was_active'] = active
    run(['systemctl', '--user', 'show', SERVICE, '-p', 'ActiveState', '-p', 'SubState',
         '-p', 'MainPID', '-p', 'ExecStart', '-p', 'FragmentPath'], 'service-before.txt')
    run(['systemctl', '--user', 'cat', SERVICE], 'service-unit.txt')
    result['port_realpath'] = os.path.realpath('/dev/uwb')
    if not result['port_realpath'].startswith('/dev/ttyUSB'):
        raise RuntimeError('Refuse unexpected UWB port')
    python = str(root / 'venv/bin/python')
    run([python, '-m', 'esptool', 'version'], 'esptool-version.txt')
    timer = 'esp32-backup-restore-' + str(os.getpid())
    if active:
        run(['systemd-run', '--user', '--unit='+timer, '--on-active=15m',
             '/usr/bin/systemctl', '--user', 'start', SERVICE], 'recovery-timer.txt')
    try:
        run(['systemctl', '--user', 'stop', SERVICE], 'service-stop.txt')
        base = [python, '-m', 'esptool', '--chip', 'esp32s3', '--port', '/dev/uwb',
                '--connect-attempts', '2']
        chip = run(base+['--baud', '115200', '--no-stub', 'flash_id'], 'flash-id.txt')
        if EXPECTED_MAC not in chip.lower() or '8MB' not in chip:
            raise RuntimeError('Tag MAC or flash size does not match intended Tag B')
        security = run(base+['--baud', '115200', '--no-stub', 'get_security_info'], 'security.txt')
        if 'Secure Boot: Disabled' not in security or 'Flash Encryption: Disabled' not in security:
            raise RuntimeError('Security state needs review before full backup')
        flash = out / 'full-flash-before.bin'
        run(base+['--baud', '921600', 'read_flash', '--no-progress', '0', 'ALL', str(flash)],
            'read-flash.txt', timeout=360)
        if flash.stat().st_size != 8388608:
            raise RuntimeError('Incomplete flash backup')
        result.update(phase='backup_read_complete', bytes=flash.stat().st_size,
            sha256=hashlib.sha256(flash.read_bytes()).hexdigest(), mac=EXPECTED_MAC)
    finally:
        if active:
            run(['systemctl', '--user', 'start', SERVICE], 'service-start.txt')
        result['service_active_after'] = subprocess.run(
            ['systemctl', '--user', 'is-active', '--quiet', SERVICE]).returncode == 0
        (out / 'result.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        print(json.dumps({'backup_dir': str(out), **result}), flush=True)
        print('KEEP recovery timer until fresh heartbeat has been checked: '+timer, flush=True)


if __name__ == '__main__':
    main()
