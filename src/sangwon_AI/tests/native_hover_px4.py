"""PX4 v1.17 SIH-as-SITL plus actual MAVROS and C++ hover; no physical FC endpoint."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('hover_web', ROOT/'ops/native_hover_web.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-root', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--position-source', choices=['gnss', 'ideal-ev'], default='gnss',
                        help='ideal-ev feeds SIH ground truth into PX4 EKF; never into the controller')
    parser.add_argument('--scenario', choices=['nominal', 'operator_cancel', 'rc_stick'], default='nominal')
    parser.add_argument('--local-origin', action='store_true',
                        help='SITL only: disable GNSS at boot and initialize the site datum from simulated truth')
    args = parser.parse_args()
    if args.local_origin and args.position_source != 'ideal-ev':
        parser.error('--local-origin requires ideal-ev; it is not a substitute for position aiding')
    assert os.environ.get('ROS_DOMAIN_ID') == '173' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil
    import rclpy
    from mavros_msgs.msg import RCIn
    from rclpy.qos import qos_profile_sensor_data
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    state = output/'controller' if args.local_origin else Path('/tmp/px4-native-hover')
    state.mkdir(mode=0o700, exist_ok=True)
    assert len(str(state/'hover.sock')) < 108
    px4_root = args.px4_root.resolve()
    px4_binary = px4_root/'build/px4_sitl_hover/bin/px4'
    work = px4_root/'build/px4_sitl_hover/rootfs'
    if args.local_origin:
        work = output/'rootfs'
        work.mkdir(exist_ok=False)
    assert px4_binary.is_file()
    master, slave = pty.openpty()
    px4_log = (output/'px4.log').open('wb')
    stopped = threading.Event()

    def pump():
        while not stopped.is_set():
            readable, _, _ = select.select([master], [], [], .1)
            if readable:
                try:
                    px4_log.write(os.read(master, 65536))
                    px4_log.flush()
                except OSError:
                    return

    thread = threading.Thread(target=pump)
    processes, logs = [], []
    connection, node, operator_thread, rc_thread = None, None, None, None

    def spawn(name, command, **kwargs):
        log = (output/(name+'.log')).open('wb')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True, **kwargs)
        processes.append(process)
        return process

    def shell(command):
        os.write(master, (command+'\n').encode())
        time.sleep(.08)

    try:
        env = dict(os.environ, PX4_SIM_MODEL='sihsim_quadx', PX4_SIMULATOR='sihsim')
        command = [str(px4_binary)]
        if args.local_origin:
            # Apply before EKF startup, not after a GNSS reference already exists.
            env['PX4_PARAM_EKF2_GPS_CTRL'] = '0'
            command += [str(px4_root/'build/px4_sitl_hover/etc'), '-w', str(work)]
        px4 = subprocess.Popen(command, cwd=work, stdin=slave, stdout=slave, stderr=slave, env=env, start_new_session=True)
        processes.append(px4)
        os.close(slave)
        thread.start()
        time.sleep(5)
        assert px4.poll() is None, 'PX4 startup failed'
        # These writes target the new software simulator, never /dev/pixhawk.
        values = {'MIS_TAKEOFF_ALT': 1.3, 'COM_TAKEOFF_ACT': 0, 'RC_MAP_ROLL': 1,
            'RC_MAP_PITCH': 2, 'RC_MAP_THROTTLE': 3, 'RC_MAP_YAW': 4,
            'RC_MAP_FLTMODE': 5, 'RC_MAP_KILL_SW': 7, 'RC_MAP_ARM_SW': 8,
            'COM_RC_OVERRIDE': 3, 'COM_RC_IN_MODE': 1}
        # Explicit simulator sensor profile. Production gates and FC params stay unchanged.
        values.update(EKF2_EV_CTRL=5 if args.position_source=='ideal-ev' else 0,
                      EKF2_EVP_NOISE=.02, EKF2_EVV_NOISE=.02)
        if args.local_origin:
            values['EKF2_GPS_CTRL'] = 0
        for name, value in values.items():
            shell(f'param set {name} {value}')
        connection = mavutil.mavlink_connection('udpout:127.0.0.1:18570', source_system=255, source_component=190)
        connection.mav.heartbeat_send(6, 8, 0, 0, 4)
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            shell('mavlink status')
            shell('commander status')
            shell('listener vehicle_status 1')
            raise AssertionError('No simulator heartbeat; inspect px4.log')
        assert heartbeat.get_srcSystem()==1 and heartbeat.get_srcComponent()==1, heartbeat
        if args.position_source=='ideal-ev':
            shell('mavlink stream -r 25 -s HIL_STATE_QUATERNION -u 18570')
        yaml = (ROOT/'config/mavros.native_hover.yaml').read_text(encoding='utf-8')
        yaml = yaml.replace(', rc_io', '')  # RC is an explicitly synthetic operator input here.
        config = output/'mavros.yaml'
        config.write_text(yaml, encoding='utf-8')
        spawn('mavros', ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '-r', '__ns:=/mavros',
            '--params-file', str(config), '-p', 'fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14580',
            '-p', 'gcs_url:=""', '-p', 'tgt_system:=1', '-p', 'tgt_component:=1', '-p', 'fcu_protocol:=v2.0'])
        hover_config = output/'hover.json'
        hover_config.write_text(json.dumps(dict(profile='SITL', ros_domain_id=173, output_enabled=True,
            takeoff_height_m=1.3, state_dir=str(state))), encoding='utf-8')
        spawn('controller', [str(args.binary.resolve()), '--config', str(hover_config)])
        rclpy.init()
        node = rclpy.create_node('sih_synthetic_operator_rc')
        rc = node.create_publisher(RCIn, '/mavros/rc/in', qos_profile_sensor_data)
        previous_phase, last_mode_request = None, 0
        operator_stick = threading.Event()
        sensor_stats = {'ideal_ev_messages': 0}
        truth_file = (output/'simulated_truth.jsonl').open('w', encoding='utf-8')
        logs.append(truth_file)
        def rc_input():
            previous = time.monotonic()
            sensor_stats['max_rc_publish_gap_s'] = 0
            while not stopped.is_set():
                now = time.monotonic()
                sensor_stats['max_rc_publish_gap_s'] = max(sensor_stats['max_rc_publish_gap_s'], now-previous)
                previous = now
                channels = [1500]*8
                if operator_stick.is_set():
                    channels[0] = 2000
                msg = RCIn(channels=channels, rssi=255)
                msg.header.stamp = node.get_clock().now().to_msg()
                rc.publish(msg)
                stopped.wait(.04)
        rc_thread = threading.Thread(target=rc_input)
        rc_thread.start()
        # Status IPC can wait for a slow consumer. Keep the operator input independent.
        def operator_input():
            last_heartbeat = 0
            origin = None
            covariance = [0.0]*21
            for i in (0, 6, 11, 15, 18, 20):
                covariance[i] = .02**2
            while not stopped.is_set():
                now = time.monotonic()
                if now-last_heartbeat>1:
                    connection.mav.heartbeat_send(6, 8, 0, 0, 4)
                    last_heartbeat = now
                connection.mav.manual_control_send(1, 600 if operator_stick.is_set() else 0, 0, 500, 0, 0)
                # A continuously busy GCS socket must not starve heartbeats or RC.
                until = time.monotonic()+.02
                while time.monotonic()<until:
                    packet = connection.recv_match(blocking=False)
                    if packet is None:
                        break
                    if packet.get_type()!='HIL_STATE_QUATERNION' or args.position_source!='ideal-ev':
                        continue
                    if origin is None:
                        origin = (packet.lat*1e-7, packet.lon*1e-7, packet.alt*.001)
                        sensor_stats['origin'] = origin
                        if args.local_origin:
                            # Keep the simulator's regional magnetic reference consistent.
                            # This initializes a datum; GNSS fusion remains disabled.
                            connection.mav.set_gps_global_origin_send(
                                1, packet.lat, packet.lon, packet.alt, time.time_ns()//1000)
                            sensor_stats['local_reference'] = dict(
                                kind='simulated_site_datum', latitude_deg=origin[0],
                                longitude_deg=origin[1], altitude_m=origin[2],
                                measured_geographic_fix=False)
                    north = math.radians(packet.lat*1e-7-origin[0])*6371000
                    east = math.radians(packet.lon*1e-7-origin[1])*6371000*math.cos(math.radians(origin[0]))
                    down = origin[2]-packet.alt*.001
                    connection.mav.odometry_send(time.time_ns()//1000, 1, 1, north, east, down,
                        packet.attitude_quaternion, packet.vx*.01, packet.vy*.01, packet.vz*.01,
                        packet.rollspeed, packet.pitchspeed, packet.yawspeed,
                        covariance, covariance, 0, 2, 100)
                    sensor_stats['ideal_ev_messages'] += 1
                    truth_file.write(json.dumps(dict(monotonic_s=now, ned_m=[north,east,down]))+'\n')
                    truth_file.flush()
                stopped.wait(.04)
        operator_thread = threading.Thread(target=operator_input)
        operator_thread.start()
        samples = (output/'observations.jsonl').open('w', encoding='utf-8')
        logs.append(samples)

        def wait(predicate, timeout):
            nonlocal previous_phase, last_mode_request
            end = time.monotonic()+timeout
            status = None
            while time.monotonic() < end:
                assert all(p.poll() is None for p in processes), 'test process exited'
                now = time.monotonic()
                rclpy.spin_once(node, timeout_sec=.001)
                try:
                    status = web.exchange(state/'hover.sock', {'method': 'status'})['status']
                    if not status['armed'] and status['phase']=='IDLE' and status['mode']!='POSCTL' and now-last_mode_request>3:
                        shell('commander mode posctl')
                        last_mode_request=now
                    samples.write(json.dumps({'monotonic_s': now, 'status': status})+'\n')
                    samples.flush()
                    if previous_phase != status['phase']:
                        previous_phase = status['phase']
                        print(status['phase'], status['start_blocker'], status['reason'], flush=True)
                    if predicate(status):
                        return status
                except OSError:
                    pass
                time.sleep(.04)
            raise AssertionError(('PX4 trial timeout', status))

        warmup_end = time.monotonic()+30
        ready = wait(lambda s: not s['start_blocker'] and time.monotonic()>=warmup_end, 90)
        started = web.exchange(state/'hover.sock', dict(method='start', session=ready['session'],
            confirm='TAKEOFF_HOVER_2S_LAND'))
        assert started['ok'], started
        if args.scenario!='nominal':
            airborne = wait(lambda s: s['phase']=='HOVER' or s['phase'] in ('FAILED','RELEASED'), 50)
            assert airborne['phase']=='HOVER', airborne
            if args.scenario=='operator_cancel':
                assert web.exchange(state/'hover.sock', dict(method='land', session=ready['session']))['ok']
            else:
                operator_stick.set()
        result = wait(lambda s: s['phase'] in ('COMPLETE', 'CANCELLED', 'FAILED', 'RELEASED'), 50)
        summary = dict(actual_px4=True, px4_source_commit='d6f12ad1c4f70ad3230afd7d86e971421e02fef4',
            simulator='SIH-as-SITL quadx', physical_flight=False, synthetic_rc_input=True,
            position_source=args.position_source, scenario=args.scenario, sensor_stats=sensor_stats,
            local_reference=sensor_stats.get('local_reference'),
            simulation_parameters=values, status=result)
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
        if args.scenario=='rc_stick':
            assert result['phase']=='RELEASED' and result['mode']=='POSCTL', result
            assert result['reason'] in ('RC_POSITION_CONFIRMED','PX4_MODE_CHANGED'), result
            # The simulated operator lands after receiving control. The companion stays released.
            operator_stick.clear()
            centered = time.monotonic()+.8
            wait(lambda s: time.monotonic()>=centered, 3)
            shell('commander mode auto:land')
            final = wait(lambda s: s['landed'] and not s['armed'], 35)
            summary['operator_landing'] = final
            assert final['phase']=='RELEASED' and not final['landing_verified'], final
            (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
        else:
            expected = 'CANCELLED' if args.scenario=='operator_cancel' else 'COMPLETE'
            assert result['phase']==expected and result['landing_verified'], result
            assert result['outcome']==('CANCELLED' if args.scenario=='operator_cancel' else 'PASS'), result
        print(f'PASS PX4 SIH-as-SITL {args.scenario} ({args.position_source})', flush=True)
    finally:
        stopped.set()
        if operator_thread:
            operator_thread.join(timeout=3)
        if rc_thread:
            rc_thread.join(timeout=3)
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
        for p in processes:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=3)
        if thread.is_alive():
            thread.join(timeout=3)
        px4_log.close()
        os.close(master)
        if node:
            node.destroy_node()
            rclpy.shutdown()
        if connection:
            connection.close()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
