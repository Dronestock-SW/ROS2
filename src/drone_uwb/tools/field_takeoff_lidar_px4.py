"""HTTP takeoff -> production C++ core -> actual PX4 SIH dynamics -> POSCTL.

Private loopback namespace only. Observations are synthetic SIH truth plus
declared noise, never physical LiDAR or UWB accuracy evidence. ROS/MAVROS are
not exercised here; their transport is checked separately on the Jetson.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
from http.server import ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen

import numpy as np
from lidar2d_independent_sih import require_isolation, sha256, FIRMWARE_COMMIT

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
for package in ('drone_mission', 'drone_uwb'):
    sys.path.insert(0, str(ROOT/package))
from drone_mission.contracts import Settings
from drone_mission.local_web import LocalPlatform
from drone_uwb.integration.lidar_velocity import VelocityReadinessWindow


class Core:
    def __init__(self, binary, settings):
        self.process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.out = self.call(method='configure', settings=settings)

    def call(self, **value):
        self.process.stdin.write(json.dumps(value, allow_nan=False).encode()+b'\n')
        self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 2.)[0]:
            raise TimeoutError('C++ execution core did not respond')
        reply = json.loads(self.process.stdout.readline())
        if not reply['ok']:
            raise ValueError(reply)
        return reply['payload']

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=3)
        self.process.stdout.close()


def run(args):
    require_isolation()
    runtime, output = args.runtime.resolve(), args.output.resolve()
    artifact = json.loads((runtime/'artifact.json').read_text(encoding='utf-8'))
    if artifact['base_commit'] != FIRMWARE_COMMIT or artifact['protocol_version'] != 2:
        raise ValueError('Pinned independent LiDAR firmware required')
    for name, key in [('bin/px4', 'binary_sha256'), ('px4_lidar2d.patch', 'patch_sha256')]:
        if sha256(runtime/name) != artifact[key]:
            raise ValueError('Runtime identity mismatch: '+name)
    output.mkdir(parents=True, exist_ok=False)
    (output/'rootfs').mkdir()
    cfg = Settings(command_frame='local_xy', mission_ai='cpp_bt', execution_profile='anchor_trial',
        full_mission=True, execute=True, layout_confirmed=True, sensor_mount_confirmed=True,
        takeoff_settings_confirmed=True, start_mode='takeoff_hold', takeoff_backend='local_position',
        takeoff_lidar_required=True, takeoff_lidar_fusion_ref='SYNTHETIC_R3_EKF_QUALIFICATION_ONLY',
        drone_id='6', expected_mis_takeoff_alt_m=1.7, speed_m_s=.25,
        enu_offset_x_m=-2., enu_offset_y_m=-2., takeoff_timeout_s=45.)
    manifest = dict(scope='SYNTHETIC_HTTP_CPP_CORE_ACTUAL_PX4_SIH_DYNAMICS',
        physical_flight=False, ros_mavros_exercised=False, actual_lidar_scene=False,
        artifact=artifact, core_sha256=sha256(args.core), script_sha256=sha256(Path(__file__)),
        settings=asdict(cfg), seed=args.seed, instance=args.instance,
        scenario=args.scenario, network_namespace=os.readlink('/proc/self/ns/net'),
        source_clock='shared_sim_clock', lidar_noise_std_m_s=.01,
        lidar_reported_std_m_s=args.lidar_reported_std, uwb_noise_std_m=.01,
        height_reference=args.height_reference, mag_type=args.mag_type, imu_profile=args.imu_profile,
        range_source=args.range_source,
        output_tau_s=args.output_tau, range_noise_model=args.range_noise_model,
        velocity_feedback_cutoff_hz=args.velocity_feedback_cutoff,
        synthetic_range_mount_z_ned_m=-.15 if args.range_source=='mavlink' else 0.,
        criteria=dict(max_takeoff_xy_m=.25, max_hold_xy_m=.15, hold_duration_s=20.,
                      max_hold_height_error_m=.15, max_hold_speed_m_s=.1,
                      uwb_gap_during_hold_s=10.))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    platform = LocalPlatform('6', cfg.layout_id, settings=cfg, mission_file=output/'missions.sqlite3')
    platform.site['ceiling_height_m'] = 3.
    server = ThreadingHTTPServer(('127.0.0.1', 0), platform.http_handler())
    web_thread = threading.Thread(target=server.serve_forever, daemon=True)
    web_thread.start()
    endpoint = 'http://127.0.0.1:'+str(server.server_port)
    core = Core(args.core, asdict(cfg))
    master, slave = pty.openpty()
    console = (output/'px4.log').open('wb')
    stop = threading.Event()
    records = (output/'observations.jsonl').open('w', encoding='utf-8')
    events, rows, commands = [], [], []
    rng = np.random.default_rng(args.seed)
    state, px4, connection = {}, None, None
    summary = dict(goal_achieved=False, errors=[], physical_flight=False)
    def pump():
        while not stop.is_set():
            if select.select([master], [], [], .05)[0]:
                try:
                    console.write(os.read(master, 65536)); console.flush()
                except OSError:
                    return
    def shell(command):
        os.write(master, (command+'\n').encode())
    def post(action):
        value = dict(action=action, client_request_id='sih-'+action+'-'+str(args.seed),
                     operator_session_id='synthetic-core-'+str(args.instance))
        request = Request(endpoint+'/local/command', data=json.dumps(value).encode(),
                          headers={'Content-Type': 'application/json'}, method='POST')
        with urlopen(request, timeout=2) as response:
            payload = json.load(response)
        events.append(dict(type='http_request', action=action, response=payload, clock_us=clock))
        return dict(action='land' if action=='takeoff_land' else action,
                    request_id=payload['control_request_id'], requested_at=payload['control_requested_at'],
                    requested_s=datetime.fromisoformat(payload['control_requested_at']).timestamp(),
                    ceiling_height_m=3.)
    def mode_request(mode):
        connection.mav.command_long_send(target_system, 1, 176, 0, 1, mode, 0, 0, 0, 0, 0)
    def odometry(component, estimator, stamp, position, velocity):
        connection.mav.srcComponent = component
        pc, vc = [math.nan]*21, [math.nan]*21
        pc[0] = pc[6] = .01**2; pc[1] = 0.
        vc[0] = vc[6] = args.lidar_reported_std**2; vc[1] = 0.
        connection.mav.odometry_send(stamp, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, *position, [math.nan]*4,
            *velocity, math.nan, math.nan, math.nan, pc, vc,
            reset_counter=0, estimator_type=estimator, quality=100)
        connection.mav.srcComponent = 190
    env = dict(os.environ, PX4_SIM_MODEL='sihsim_quadx', PX4_SIMULATOR='sihsim', PX4_SIM_SPEED_FACTOR='1',
        PX4_PARAM_EKF2_GPS_CTRL='0', PX4_PARAM_EKF2_EV_CTRL='1', PX4_PARAM_EKF2_EV_DELAY='0',
        PX4_PARAM_EKF2_EV_NOISE_MD='0', PX4_PARAM_EKF2_LID2_EN='1', PX4_PARAM_EKF2_LID2_AGE='180',
        PX4_PARAM_EKF2_AVEL_DELAY='0', PX4_PARAM_EKF2_MAG_TYPE=str(args.mag_type),
        PX4_PARAM_EKF2_RNG_CTRL='2', PX4_PARAM_EKF2_BARO_CTRL='1', PX4_PARAM_EKF2_HGT_REF=str(args.height_reference),
        PX4_PARAM_SIH_DISTSNSR_OVR='-1', PX4_PARAM_COM_RC_IN_MODE='1',
        PX4_PARAM_COM_RC_OVERRIDE='3', PX4_PARAM_COM_ARM_WO_GPS='1',
        PX4_PARAM_MIS_TAKEOFF_ALT='1.7', PX4_PARAM_COM_TAKEOFF_ACT='0',
        PX4_PARAM_MPC_Z_VEL_MAX_UP='.5', PX4_PARAM_MPC_Z_VEL_MAX_DN='.5',
        PX4_PARAM_MPC_TKO_SPEED='.5', PX4_PARAM_MPC_LAND_SPEED='.5',
        PX4_PARAM_SDLOG_MODE='1', PX4_PARAM_SDLOG_PROFILE='1')
    env.update(PX4_PARAM_EKF2_TAU_VEL=str(args.output_tau),PX4_PARAM_EKF2_TAU_POS=str(args.output_tau))
    env['PX4_PARAM_MPC_VEL_LP']=str(args.velocity_feedback_cutoff)
    if args.range_noise_model=='synthetic_quantized':
        # Declared centimeter quantization, with a conservative 1 cm floor;
        # no distance-dependent noise was added by this synthetic source.
        env.update(PX4_PARAM_EKF2_RNG_NOISE='.01',PX4_PARAM_EKF2_RNG_SFE='0')
    if args.imu_profile=='sih_noisy':
        # SIH reconstruct_sensors_signals uses armed per-axis stddev up to
        # 1.7 m/s^2 and 0.14 rad/s. This is an isolated sensitivity profile,
        # never a physical-aircraft parameter recommendation.
        env.update(PX4_PARAM_EKF2_ACC_NOISE='1.7', PX4_PARAM_EKF2_GYR_NOISE='.14')
    if args.range_source=='mavlink':
        # SIH's point-mass origin is on the floor. Its built-in ground range
        # can be slightly negative. Model a declared 15 cm sensor offset so
        # ground observations are valid; PX4 removes this known lever arm.
        # fabs(override)>=10000 disables SIH publication. An out-of-range but
        # still publishing first sensor would prevent EKF2 selecting MAVLink.
        env.update(PX4_PARAM_SIH_DISTSNSR_OVR='10000',PX4_PARAM_EKF2_RNG_POS_Z='-.15')
    try:
        px4 = subprocess.Popen([str(runtime/'bin/px4'), str(runtime/'etc'), '-i', str(args.instance),
            '-w', str(output/'rootfs')], stdin=slave, stdout=slave, stderr=slave,
            cwd=output, env=env, start_new_session=True)
        os.close(slave)
        thread = threading.Thread(target=pump, daemon=True); thread.start()
        port = 18570+args.instance
        connection = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=255, source_component=190)
        started = time.monotonic(); clock = 0; first_clock = None; origin = None
        target_system = None; launch_yaw = None
        last_hb = last_manual = last_mode = last_tick = last_lidar = last_uwb = -1e9
        last_progress = last_range = -1e9
        readiness=VelocityReadinessWindow()
        stream_requested = False; request = None; pending = None; hold_start = None
        launch_truth = None; arm_clock = None; last_phase = None; hover_verified = False
        last_uwb_wall = None; last_lidar_wall = None; lidar_start = None
        latest_uwb = [2., 2.]; uwb_count = 0; latest_lidar = [0., 0.]
        while time.monotonic()-started < 190:
            now = time.monotonic(); wall = time.time()
            if px4.poll() is not None:
                raise RuntimeError('PX4 exited before test completion')
            if now-last_hb >= .5:
                connection.mav.heartbeat_send(6, 8, 0, 0, 4); last_hb = now
            if target_system is not None and now-last_manual >= .05:
                connection.mav.manual_control_send(target_system, 0, 0, 500, 0, 0); last_manual = now
            for _ in range(100):
                message = connection.recv_match(blocking=False)
                if message is None:
                    break
                kind = message.get_type()
                state[kind] = (message, now)
                if kind=='HEARTBEAT' and not stream_requested:
                    target_system = message.get_srcSystem()
                    for name, rate in [('HIL_STATE_QUATERNION', 50), ('LOCAL_POSITION_NED', 50),
                        ('ATTITUDE', 30), ('EXTENDED_SYS_STATE', 20), ('ESTIMATOR_STATUS', 20),
                        ('POSITION_TARGET_LOCAL_NED', 20), ('TIMESYNC', 50)]:
                        shell(f'mavlink stream -r {rate} -s {name} -u {port}')
                    shell('ver all'); stream_requested = True
                if kind=='TIMESYNC' and message.tc1==0:
                    connection.mav.timesync_send(message.ts1, message.ts1)
                if kind=='LOCAL_POSITION_NED' and message.time_boot_ms>0:
                    clock = int(message.time_boot_ms)*1000
                    if first_clock is None:
                        first_clock = clock
                if kind=='HIL_STATE_QUATERNION' and origin is None:
                    origin = [message.lat/1e7, message.lon/1e7, message.alt*.001]
                if kind=='COMMAND_ACK':
                    events.append(dict(type='command_ack', command=message.command, result=message.result, clock_us=clock))
                    if pending and message.command==pending['command']:
                        core.call(method='ack', token=pending['token'], accepted=message.result==0, now_s=now)
                        pending = None
                if kind=='STATUSTEXT':
                    events.append(dict(type='statustext', text=message.text, clock_us=clock))
            if first_clock is None or origin is None:
                time.sleep(.002); continue
            elapsed = (clock-first_clock)/1e6
            h = state['HIL_STATE_QUATERNION'][0]
            truth = [math.radians(h.lon/1e7-origin[1])*6371000*math.cos(math.radians(origin[0])),
                     math.radians(h.lat/1e7-origin[0])*6371000, h.alt*.001-origin[2]]
            if args.range_source=='mavlink' and clock-last_range>=20000:
                # Synthetic downward range, not a height observation from LiDAR.
                q=h.attitude_quaternion
                cos_tilt=1.-2.*(q[1]**2+q[2]**2)
                distance=(truth[2]+.15)/max(cos_tilt,.1)
                connection.mav.distance_sensor_send(clock//1000,5,600,max(0,round(distance*100)),
                    0,42,25,1,signal_quality=100)
                last_range=clock
            hb = state.get('HEARTBEAT', (None,))[0]
            armed = hb is not None and bool(hb.base_mode&128)
            if armed and arm_clock is None:
                arm_clock = clock
            uwb_present = args.scenario=='nominal' or arm_clock is None or (hold_start is not None and now-hold_start>=10.)
            if elapsed>=8 and clock-last_lidar>=100000:
                latest_lidar = [h.vx*.01+float(rng.normal(0,.01)), h.vy*.01+float(rng.normal(0,.01))]
                odometry(198, mavutil.mavlink.MAV_ESTIMATOR_TYPE_LIDAR, clock, [math.nan]*3,
                         [*latest_lidar, math.nan])
                start=clock-100000 if last_lidar<0 else int(last_lidar)
                readiness.observe(dict(interval_start_ns=start*1000,interval_end_ns=clock*1000,
                    representative_stamp_ns=clock*1000,reset_counter=0,velocity_ne_m_s=latest_lidar))
                last_lidar = clock; last_lidar_wall = now
                if lidar_start is None: lidar_start = now
                records.write(json.dumps(dict(kind='lidar2d', clock_us=clock, velocity_ne=latest_lidar,
                    truth_xy=truth[:2], source='synthetic_SIH_truth_plus_declared_noise'))+'\n')
            if elapsed>=8 and uwb_present and clock-last_uwb>=100000:
                n, e = truth[1]+float(rng.normal(0,.01)), truth[0]+float(rng.normal(0,.01))
                odometry(196, mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION, clock, [n,e,0.], [math.nan]*3)
                latest_uwb = [e+2., n+2.]; last_uwb = clock; last_uwb_wall = now; uwb_count += 1
            if not all(k in state for k in ['HEARTBEAT','LOCAL_POSITION_NED','ATTITUDE','EXTENDED_SYS_STATE','ESTIMATOR_STATUS']):
                time.sleep(.002); continue
            # Decode PX4 main mode directly. pymavlink's legacy decoder treats
            # OFFBOARD as an AUTO submode and reports current PX4 as UNKNOWN.
            main_mode, sub_mode = (hb.custom_mode>>16)&255, (hb.custom_mode>>24)&255
            mode = {1:'MANUAL',2:'ALTCTL',3:'POSCTL',6:'OFFBOARD',7:'STABILIZED',
                    5:'ACRO'}.get(main_mode,'UNKNOWN')
            if main_mode==4:
                mode = {3:'AUTO.LOITER',4:'AUTO.MISSION',5:'AUTO.RTL',6:'AUTO.LAND'}.get(sub_mode,'AUTO.UNKNOWN')
            if request is None and elapsed>15 and mode!='POSCTL' and now-last_mode>1.:
                mode_request(3); last_mode = now
            if now-last_tick<.05:
                time.sleep(.002); continue
            last_tick = now
            p, att, est = state['LOCAL_POSITION_NED'][0], state['ATTITUDE'][0], state['ESTIMATOR_STATUS'][0]
            age = lambda key: now-state[key][1]
            uwb_age = 1e6 if last_uwb_wall is None else now-last_uwb_wall
            lidar_ready = last_lidar_wall is not None and now-last_lidar_wall<=.18 and lidar_start is not None and now-lidar_start>=2.
            window=readiness.status()
            s = dict(connected=True, armed=armed, landed=state['EXTENDED_SYS_STATE'][0].landed_state,
                mode=mode, state_age_s=age('HEARTBEAT'), landed_age_s=age('EXTENDED_SYS_STATE'),
                estimator_age_s=age('ESTIMATOR_STATUS'), estimator_valid=bool(est.flags&256 and est.flags&8 and est.flags&2),
                vertical_estimator_valid=bool(est.flags&4 and est.flags&32), pose_age_s=age('LOCAL_POSITION_NED'),
                xy=[p.y+2., p.x+2.], velocity_xy=[p.vy,p.vx], altitude_enu_m=-p.z,
                vertical_speed_m_s=-p.vz, yaw_deg=90.-math.degrees(att.yaw), uwb_xy=latest_uwb,
                uwb_age_s=uwb_age, bridge_ready=uwb_count>0, alignment_matches=True,
                bridge_observation_age_s=uwb_age, bridge_age_s=0., bridge_published=uwb_count,
                battery=.9, battery_age_s=0., feedback_age_s=age('POSITION_TARGET_LOCAL_NED') if 'POSITION_TARGET_LOCAL_NED' in state else 1e6,
                local_setpoint_ready=True, command_services_ready=True, ground_gate_ready=True,
                takeoff_param_age_s=0., takeoff_alt_m=1.7, max_up_m_s=.5, max_up_age_s=0.,
                rc_override=3, rc_required=False, web_age_s=0., request_persisted=request is not None,
                lidar_ready=lidar_ready, lidar_gate='ready' if lidar_ready else 'synthetic_lidar_unavailable',
                lidar_observation_age_s=1e6 if last_lidar_wall is None else now-last_lidar_wall,
                lidar_status_age_s=0.,
                lidar_stationary_window_ready=lidar_ready and window['ready'],
                lidar_speed_m_s=window.get('speed_m_s'))
            core.out = core.call(method='tick', state=s, request=request, now_s=now, wall_s=wall)
            phase = core.out['phase']
            if elapsed-last_progress>=5.:
                print(json.dumps(dict(type='progress',elapsed_s=elapsed,phase=phase,
                    mode=mode,flags=est.flags,blocker=core.out['native_takeoff']['blocker'],
                    estimated_xy=s['xy'],estimated_height=-p.z,truth_xyz=truth)),flush=True)
                last_progress=elapsed
            with platform.lock:
                platform.telemetry = dict(core.out, flight_state=phase, fc_connected=True, fc_armed=armed,
                    fc_landed=s['landed'], fc_mode=mode, operator_session_id='synthetic-core-'+str(args.instance))
                platform.telemetry_received_s = now
            if request is None and elapsed>20 and platform.can_takeoff():
                request = post('takeoff'); launch_truth = truth[:2]; launch_yaw = att.yaw
            for action in core.out['actions']:
                kind = action['kind']; command = 400 if kind=='arm' else 21 if kind=='land' else 176
                pending = dict(token=action['token'], command=command)
                core.call(method='dispatched', token=action['token'], now_s=now)
                commands.append(dict(kind=kind, clock_us=clock, elapsed_s=elapsed))
                if kind in ('offboard_mode','position_mode'): mode_request(6 if kind=='offboard_mode' else 3)
                elif kind=='arm': connection.mav.command_long_send(target_system,1,400,0,1,0,0,0,0,0,0)
                elif kind=='land': connection.mav.command_long_send(target_system,1,21,0,0,0,0,math.nan,math.nan,math.nan,math.nan)
                else: raise ValueError('Unexpected command: '+kind)
            if core.out['streaming']:
                goal = core.out['goal_xy_m']
                connection.mav.set_position_target_local_ned_send(clock//1000,target_system,1,1,2552,
                    goal[1]-2.,goal[0]-2.,-core.out['height_reference_m'],0,0,0,0,0,0,launch_yaw,0)
            if phase!=last_phase:
                event = dict(type='transition', phase=phase, reason=core.out['reason'], mode=mode,
                             elapsed_s=elapsed, truth_xyz=truth, estimated_xy=s['xy'], height=-p.z)
                events.append(event); print(json.dumps(event), flush=True); last_phase = phase
            if phase=='HOLD_READY' and hold_start is None: hold_start = now
            displacement = None if launch_truth is None else math.dist(truth[:2],launch_truth)
            rows.append(dict(elapsed_s=elapsed, clock_us=clock, phase=phase, mode=mode, armed=armed,
                truth_xyz=truth, launch_displacement_m=displacement, estimated_xy=s['xy'], estimated_height_m=-p.z,
                height_reference_m=core.out['height_reference_m'], truth_speed_xy_m_s=math.hypot(h.vx,h.vy)*.01,
                uwb_present=uwb_present, uwb_age_s=uwb_age, flags=est.flags,
                heartbeat_base_mode=hb.base_mode, heartbeat_custom_mode=hb.custom_mode,
                native_takeoff=core.out.get('native_takeoff')))
            if hold_start is not None and now-hold_start>=25. and not hover_verified:
                if phase!='HOLD_READY' or mode!='POSCTL':
                    raise AssertionError('POSITION hold ended before goal duration')
                hover_verified = True; request = post('takeoff_land')
            if phase in ('END','FAILED','UNCONFIRMED','PILOT_OVERRIDE'):
                if hover_verified and not armed and s['landed']==1 and core.out['failure']=='operator_cancelled':
                    break
                raise AssertionError(dict(unexpected_terminal=core.out))
        else:
            raise TimeoutError(dict(last_state=core.out, elapsed_s=elapsed, mode=mode, estimator_flags=est.flags))
        hold = [r for r in rows if r['phase']=='HOLD_READY']
        airborne = [r for r in rows if r['armed'] and r['phase'] not in ('LANDING','FAILED')]
        hold_duration = hold[-1]['elapsed_s']-hold[0]['elapsed_s'] if hold else 0.
        hold_gap = [r for r in hold if not r['uwb_present']]
        metrics = dict(hold_duration_s=hold_duration,
            max_takeoff_xy_m=max(r['launch_displacement_m'] for r in airborne),
            max_hold_xy_m=max(r['launch_displacement_m'] for r in hold),
            max_hold_height_error_m=max(abs(r['estimated_height_m']-r['height_reference_m']) for r in hold),
            max_hold_truth_speed_m_s=max(r['truth_speed_xy_m_s'] for r in hold),
            uwb_absent_hold_samples=len(hold_gap), disarmed_after_web_land=not armed)
        failures = []
        for metric, limit in [('max_takeoff_xy_m',.25),('max_hold_xy_m',.15),
                              ('max_hold_height_error_m',.15),('max_hold_truth_speed_m_s',.1)]:
            if metrics[metric]>limit: failures.append(metric)
        if hold_duration<20: failures.append('hold_duration')
        if args.scenario=='uwb_gap' and len(hold_gap)<150: failures.append('uwb_gap_not_exercised')
        summary.update(goal_achieved=not failures, failures=failures, metrics=metrics, final=core.out)
        shell('logger stop'); time.sleep(1.); shell('shutdown'); px4.wait(timeout=8)
    except Exception as error:
        summary['errors'].append(repr(error)); raise
    finally:
        if px4 is not None and px4.poll() is None:
            os.killpg(px4.pid, signal.SIGTERM)
            try: px4.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(px4.pid, signal.SIGKILL); px4.wait(timeout=3)
        stop.set()
        if 'thread' in locals(): thread.join(timeout=2)
        if connection is not None: connection.close()
        core.close(); server.shutdown(); server.server_close(); web_thread.join(2); platform.close()
        console.close(); records.close(); os.close(master)
        summary.update(commands=commands, ulogs=[str(p.relative_to(output)) for p in output.rglob('*.ulg')])
        for name, value in [('summary.json',summary),('events.json',events),('trace.json',rows)]:
            (output/name).write_text(json.dumps(value, indent=2)+'\n',encoding='utf-8')
    if not summary['goal_achieved']: raise AssertionError(summary['failures'])
    print(json.dumps(summary['metrics']), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime',type=Path,required=True)
    parser.add_argument('--core',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=['nominal','uwb_gap'],default='uwb_gap')
    parser.add_argument('--seed',type=int,default=11)
    parser.add_argument('--lidar-reported-std',type=float,default=.015,
        help='Synthetic covariance only; covers 0.01 noise and MAVLink cm/s quantization')
    parser.add_argument('--height-reference',type=int,choices=[0,2],default=2,
        help='Isolated SIH only: 0 barometer, 2 downward rangefinder')
    parser.add_argument('--mag-type',type=int,choices=[0,6],default=0,
        help='Isolated SIH only: 0 automatic magnetic fusion, 6 initialization only')
    parser.add_argument('--imu-profile',choices=['stock','sih_noisy'],default='stock')
    parser.add_argument('--range-source',choices=['builtin','mavlink'],default='builtin')
    parser.add_argument('--range-noise-model',choices=['stock','synthetic_quantized'],default='stock')
    parser.add_argument('--output-tau',type=float,default=.25)
    parser.add_argument('--velocity-feedback-cutoff',type=float,default=0.)
    parser.add_argument('--instance',type=int,default=86)
    args = parser.parse_args()
    if not 0<=args.instance<=100: parser.error('invalid instance')
    if not .01<=args.lidar_reported_std<=.5: parser.error('invalid synthetic observation uncertainty')
    if not .1<=args.output_tau<=1.: parser.error('invalid output predictor time constant')
    if not 0<=args.velocity_feedback_cutoff<=10.: parser.error('invalid velocity feedback cutoff')
    if not os.environ.get('LIDAR_SITL_PARENT_NETNS'):
        env=dict(os.environ,LIDAR_SITL_PARENT_NETNS=os.readlink('/proc/self/ns/net'))
        result=subprocess.run(['unshare','-Unc','--keep-caps',sys.executable,str(Path(__file__).resolve()),*sys.argv[1:]],env=env)
        raise SystemExit(result.returncode)
    run(args)


if __name__=='__main__': main()
