"""Private-network PX4 SIH ground diagnostics for standard EV transport.

This is a stationary/disarmed transport and estimator diagnostic. It does NOT
claim the dynamic, optical-flow, LiDAR-scene, or closed-loop S01-S14 gates.
No physical endpoint, ROS graph, or serial device is opened. Every run retains
the exact PX4 identity, stimulus log, PX4 console and unmodified ULog.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import socket
import subprocess
import sys
import threading
import time

import numpy as np

from prepare_lidar_sitl import FIRMWARE_COMMIT

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


CASES = ['nominal', 'nan_vz', 'invalid_velocity_frame', 'lidar_gap', 'uwb_gap',
         'reset', 'quality', 'duplicate', 'reverse_time', 'clock_jump', 'delay',
         'alternating', 'stale_velocity', 'vertical_aiding']


def require_isolation():
    if os.name != 'posix' or not Path('/proc/self/ns/net').exists():
        raise RuntimeError('Linux private network namespace required')
    host_namespace = os.environ.get('LIDAR_SITL_PARENT_NETNS')
    if not host_namespace or os.readlink('/proc/self/ns/net') == host_namespace:
        raise RuntimeError('A private network namespace is required; host network forbidden')
    # sysfs can retain the parent mount's namespace view after unshare. Query
    # this process's network interfaces instead of the inherited sysfs mount.
    interfaces = {name for _, name in socket.if_nameindex()}
    if interfaces != {'lo'}:
        raise RuntimeError(f'Only loopback may exist: {interfaces}')
    subprocess.run(['ip', 'link', 'set', 'lo', 'up'], check=True)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def covariance(x, y, z):
    value = [0.] * 21
    for index, variance in zip([0, 6, 11, 15, 18, 20], [x, y, z, 1e6, 1e6, 1e6]):
        value[index] = variance
    return value


def run(args):
    require_isolation()
    root = args.px4_root.resolve()
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if head != FIRMWARE_COMMIT:
        raise ValueError(f'wrong PX4 commit: {head}')
    # No tracked source or estimator edits are permitted for this diagnostic.
    if subprocess.check_output(['git', '-C', str(root), 'diff', 'HEAD', '--name-only'], text=True).strip():
        raise ValueError('PX4 tracked source differs from the required commit')
    board = 'evbench' if args.speed_factor > 1 else 'lidar'
    build = root / f'build/px4_sitl_{board}'
    binary = build / 'bin/px4'
    if not binary.is_file():
        raise FileNotFoundError(binary)
    if args.speed_factor > 1:
        commands = json.loads((build / 'compile_commands.json').read_text(encoding='utf-8'))
        sih_command = next(c['command'] for c in commands if c['file'].endswith('/simulator_sih/sih.cpp'))
        if '-DENABLE_LOCKSTEP_SCHEDULER' not in sih_command:
            raise ValueError('accelerated run requires the effective SIH lockstep compilation')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'rootfs').mkdir()
    rng = np.random.default_rng(args.seed)
    manifest = dict(schema=1, firmware_commit=head, binary_sha256=sha256(binary),
                    board_sha256=sha256(root / f'boards/px4/sitl/{board}.px4board'),
                    speed_factor=args.speed_factor,
                    instance=args.instance, mavlink_udp_port=18570 + args.instance,
                    source_clock='shared_sim_clock' if args.speed_factor > 1 else 'host_monotonic',
                    sih_sensor_seed=1234, position_only=args.position_only,
                    scenario=args.scenario, seed=args.seed, duration_s=args.duration,
                    noise_mode=args.noise_mode, z_variance=args.z_variance,
                    gap_s=args.gap, scope='stationary disarmed SIH diagnostic; synthetic independent EV samples',
                    network_namespace=os.readlink('/proc/self/ns/net'), physical_writes=0,
                    raw_lidar_used=False, closed_loop=False, mission_acceptance='NOT_DEFINED_BY_USER')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    master, slave = pty.openpty()
    stopped = threading.Event()
    console = (output / 'px4.log').open('wb')
    connection = None
    proc = None
    stats = dict(odometry_sent=0, position_invalid_sent=0, velocity_invalid_sent=0,
                 timesync_responses=0, heartbeats=0, errors=[])

    def pump():
        while not stopped.is_set():
            if select.select([master], [], [], .05)[0]:
                try:
                    console.write(os.read(master, 65536))
                    console.flush()
                except OSError:
                    return

    def shell(command):
        os.write(master, (command + '\n').encode('utf-8'))

    def send_ev(stamp_us, position, velocity, quality, reset, frame):
        connection.mav.odometry_send(stamp_us, mavutil.mavlink.MAV_FRAME_LOCAL_NED, frame,
                                     *position, [1., 0., 0., 0.], *velocity, 0., 0., 0.,
                                     covariance(.01, .01, 1e6), covariance(.01, .01, args.z_variance),
                                     reset_counter=reset, estimator_type=mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
                                     quality=quality)

    env = dict(os.environ, PX4_SIM_MODEL='sihsim_quadx', PX4_SIMULATOR='sihsim',
               PX4_SIM_SPEED_FACTOR=str(args.speed_factor),
               PX4_PARAM_EKF2_GPS_CTRL='0', PX4_PARAM_EKF2_EV_CTRL='1' if args.position_only else '5',
               PX4_PARAM_EKF2_EV_NOISE_MD=str(args.noise_mode), PX4_PARAM_EKF2_EV_DELAY='0',
               PX4_PARAM_EKF2_EVV_NOISE='0.1', PX4_PARAM_EKF2_EVP_NOISE='0.1',
               PX4_PARAM_EKF2_EV_QMIN='1', PX4_PARAM_EKF2_EV_POS_X='0',
               PX4_PARAM_EKF2_EV_POS_Y='0', PX4_PARAM_EKF2_EV_POS_Z='0',
               PX4_PARAM_SDLOG_MODE='1', PX4_PARAM_SDLOG_PROFILE='1',
               ROS_DOMAIN_ID='99', ROS_LOCALHOST_ONLY='1')
    inputs = (output / 'inputs.jsonl').open('w', encoding='utf-8')
    telemetry = (output / 'telemetry.jsonl').open('w', encoding='utf-8')
    try:
        proc = subprocess.Popen([str(binary), str(build / 'etc'), '-i', str(args.instance), '-w', str(output / 'rootfs')],
                                stdin=slave, stdout=slave, stderr=slave, cwd=output, env=env, start_new_session=True)
        os.close(slave)
        thread = threading.Thread(target=pump, daemon=True)
        thread.start()
        port = 18570 + args.instance
        connection = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=245, source_component=191)
        start_wall = time.monotonic()
        last_hb = last_send = last_probe = -float('inf')
        next_send_sim = None
        baseline_time = None
        sequence = 0
        first_clock = None
        clock = 0
        stream_requested = False
        snapshot_sent = False
        reset = 0
        finished = False
        while time.monotonic() - start_wall < args.duration + 45:
            now = time.monotonic()
            if proc.poll() is not None:
                raise RuntimeError(f'PX4 exited {proc.returncode}')
            if now - last_hb > 1.:
                connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 4)
                last_hb = now
            message = connection.recv_match(blocking=False)
            if message is not None:
                kind = message.get_type()
                if kind == 'HEARTBEAT':
                    stats['heartbeats'] += 1
                    if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        raise RuntimeError('unexpected armed state in ground diagnostic')
                    if not stream_requested:
                        shell(f'mavlink stream -r 50 -s LOCAL_POSITION_NED -u {port}')
                        shell(f'mavlink stream -r 50 -s TIMESYNC -u {port}')
                        shell('ver all')
                        shell('param show EKF2_EV*')
                        stream_requested = True
                if kind == 'TIMESYNC' and message.tc1 == 0:
                    # Accelerated diagnostics share the simulator clock. Echo
                    # its request-time sample; link delay is measured in ULog.
                    sender_ns = message.ts1 if args.speed_factor > 1 else time.monotonic_ns()
                    connection.mav.timesync_send(sender_ns, message.ts1)
                    stats['timesync_responses'] += 1
                # At this PX4 revision HIL_STATE_QUATERNION.time_usec is zero.
                # LOCAL_POSITION_NED carries the advancing PX4 clock; only its
                # timestamp is used to schedule synthetic observations.
                if kind == 'LOCAL_POSITION_NED' and message.time_boot_ms > 0:
                    clock = int(message.time_boot_ms) * 1000
                    if first_clock is None:
                        first_clock = clock
                        baseline_time = now
                        next_send_sim = first_clock + 20_000_000
                if kind in ('STATUSTEXT', 'HIL_STATE_QUATERNION', 'LOCAL_POSITION_NED', 'ODOMETRY', 'ESTIMATOR_STATUS', 'TIMESYNC'):
                    telemetry.write(json.dumps(dict(received_monotonic_s=now, message=message.to_dict()), allow_nan=True) + '\n')
            if first_clock is None:
                if message is None:
                    time.sleep(.0002)
                continue
            elapsed = (clock - first_clock) / 1e6
            if elapsed >= args.duration:
                finished = True
                break
            # 0-20 s align clocks/EKF. Common coherent 10 Hz samples thereafter.
            due = clock >= next_send_sim if args.speed_factor > 1 else now - last_send >= .1
            if elapsed < 20. or not due:
                if message is None:
                    time.sleep(.0002)
                continue
            stamp = clock if args.speed_factor > 1 else time.monotonic_ns() // 1000
            if args.scenario in ('duplicate', 'reverse_time', 'clock_jump') and 40 <= elapsed < 45:
                if first_fault_stamp is None:
                    first_fault_stamp = stamp
                if args.scenario == 'duplicate':
                    stamp = first_fault_stamp
                elif args.scenario == 'reverse_time':
                    stamp = first_fault_stamp - int((elapsed - 40) * 1e6)
                else:
                    stamp -= 10_000_000
            else:
                first_fault_stamp = None
            if args.scenario == 'delay' and 40 <= elapsed < 45:
                stamp -= int(args.delay_ms * 1000)
            position = [float(rng.normal(0., .01)), float(rng.normal(0., .01)), 0.]
            velocity = [float(rng.normal(0., .01)), float(rng.normal(0., .01)), 0.]
            quality, frame = 100, mavutil.mavlink.MAV_FRAME_BODY_FRD
            in_fault = 40 <= elapsed < 40 + args.gap
            if args.scenario == 'nan_vz' and in_fault:
                velocity[2] = float('nan')
            if args.scenario == 'invalid_velocity_frame' and in_fault:
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            if args.scenario == 'lidar_gap' and in_fault:
                velocity = [float('nan')] * 3
            if args.scenario == 'uwb_gap' and in_fault:
                position = [float('nan')] * 3
            if args.scenario == 'alternating':
                if sequence % 2:
                    velocity = [float('nan')] * 3
                else:
                    position = [float('nan')] * 3
            if args.scenario == 'quality' and in_fault:
                quality = 0
            if args.scenario == 'reset' and elapsed >= 40:
                reset = 1
            if args.scenario == 'stale_velocity' and in_fault:
                # Explicit negative control: send the old sample as though new.
                velocity = stale_velocity
            else:
                stale_velocity = velocity.copy()
            if args.scenario == 'vertical_aiding' and elapsed >= 40 and not snapshot_sent:
                shell('param set EKF2_BARO_CTRL 0')
                shell('param set EKF2_RNG_CTRL 0')
                snapshot_sent = True
            send_ev(stamp, position, velocity, quality, reset, frame)
            stats['odometry_sent'] += 1
            stats['position_invalid_sent'] += int(not np.isfinite(position).all())
            stats['velocity_invalid_sent'] += int(not np.isfinite(velocity).all())
            inputs.write(json.dumps(dict(elapsed_s=elapsed, sequence=sequence, stamp_us=stamp, publish_clock_us=clock,
                                          position=position, velocity=velocity, quality=quality,
                                          reset_counter=reset, velocity_frame=frame), allow_nan=True) + '\n')
            last_send = now
            # Skip missed slots; never burst-relabel old observations as new.
            next_send_sim += (max(0, (clock - next_send_sim) // 100_000) + 1) * 100_000
            sequence += 1
            if now - last_probe > 10:
                shell('listener estimator_aid_src_ev_vel -n 1')
                shell('listener estimator_aid_src_ev_pos -n 1')
                shell('listener vehicle_local_position -n 1')
                last_probe = now
        if not finished:
            raise TimeoutError('requested simulated duration not reached')
        stats.update(simulated_duration_s=elapsed, wall_duration_s=time.monotonic() - start_wall,
                     first_clock_us=first_clock, final_clock_us=clock)
        shell('logger stop')
        time.sleep(1.)
        shell('shutdown')
        proc.wait(timeout=8)
    except Exception as exc:
        stats['errors'].append(repr(exc))
        raise
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        stopped.set()
        if 'thread' in locals():
            thread.join(timeout=1)
        if connection is not None:
            connection.close()
        console.close()
        inputs.close()
        telemetry.close()
        os.close(master)
        stats['ulog_files'] = [str(p.relative_to(output)) for p in output.rglob('*.ulg')]
        (output / 'run_status.json').write_text(json.dumps(stats, indent=2), encoding='utf-8')
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=CASES, default='nominal')
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--noise-mode', type=int, choices=[0, 1], default=0)
    parser.add_argument('--z-variance', type=float, default=1e4)
    parser.add_argument('--gap', type=float, default=5.)
    parser.add_argument('--delay-ms', type=float, default=100.)
    parser.add_argument('--speed-factor', type=float, default=1.)
    parser.add_argument('--position-only', action='store_true')
    parser.add_argument('--instance', type=int, default=0)
    args = parser.parse_args()
    if args.duration < 25 or not math.isfinite(args.duration) or not 0 < args.z_variance <= 1e6 or not 1 <= args.speed_factor <= 10:
        parser.error('invalid duration or z variance')
    if not 0 <= args.instance <= 100:
        parser.error('instance must be between 0 and 100')
    if not os.environ.get('LIDAR_SITL_PARENT_NETNS'):
        # Compare against our own parent namespace: reading /proc/1/ns/net is
        # intentionally restricted inside an unprivileged user namespace.
        env = dict(os.environ, LIDAR_SITL_PARENT_NETNS=os.readlink('/proc/self/ns/net'))
        # Keep the current uid: fake uid 0 triggers PX4's mlockall path, which
        # can exhaust a user namespace's locked-memory allowance on thread start.
        result = subprocess.run(['unshare', '-Unc', '--keep-caps', sys.executable,
                                 str(Path(__file__).resolve()), *sys.argv[1:]], env=env)
        raise SystemExit(result.returncode)
    print(json.dumps(run(args), indent=2), flush=True)


if __name__ == '__main__':
    main()
