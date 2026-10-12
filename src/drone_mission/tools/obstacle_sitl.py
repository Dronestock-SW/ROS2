"""Closed-loop, private-network SIH experiment of the mission XY command path.

Synthetic UWB position and virtual sensors use simulator truth. LiDAR uses
either declared synthetic velocity or noisy rays through the actual matcher.
The patched firmware can fuse separate LiDAR2D velocity. This is not physical
RAW replay. No ROS graph, hardware device or
external network is available inside the required network namespace.
"""
import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

from drone_mission.contracts import Settings, LAYOUT
from drone_mission.guarded_local_xy import GuardedLocalXYMission
from drone_mission.horizontal_obstacles import GuardConfig, HorizontalGuard, scan_obstacles, ordered_obstacles
from drone_mission.local_command import local_command_fields
from drone_mission.local_xy import LocalXYMission
from drone_mission.session import Snapshot
from drone_uwb.processing.gazebo_geometry import rotation_world_body
from drone_uwb.processing.experiments.lidar_sitl_observation import (
    VirtualLidar,PlanarScanOdometry,enu_flu_rotation)
from drone_uwb.processing.experiments.lidar_spinning import (
    PoseTimeline, SpinningLidar, SpinningScanOdometry)
from drone_uwb.processing.experiments.lidar_timing import TimingError

FIRMWARE = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'
EXPECTED_BINARY = '6f8b77f94edde450dd5851619027c877ef9d51b4fae1c78284dea00e582d04d4'


def px4_mode(heartbeat):
    """Decode this firmware's px4_custom_mode.h, independently of old SDK maps."""
    if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        return 'UNKNOWN'
    main=(heartbeat.custom_mode>>16)&255
    sub=(heartbeat.custom_mode>>24)&255
    if main==4:return {2:'AUTO.TAKEOFF',3:'AUTO.LOITER',6:'AUTO.LAND'}.get(sub,'AUTO.OTHER')
    return {1:'MANUAL',2:'ALTCTL',3:'POSCTL',6:'OFFBOARD',7:'STABILIZED'}.get(main,'UNKNOWN')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def isolation():
    parent = os.environ.get('OBSTACLE_SITL_PARENT_NETNS')
    if not parent or os.readlink('/proc/self/ns/net') == parent:
        raise RuntimeError('private network namespace required')
    if {name for _, name in socket.if_nameindex()} != {'lo'}:
        raise RuntimeError('non-loopback network interface forbidden')
    subprocess.run(['ip','link','set','lo','up'],check=True)


def scan_box(xy, wall_x):
    angles = np.linspace(-math.pi,math.pi,721)
    distances=[]
    for angle in angles:
        c,s=math.cos(angle),math.sin(angle)
        tx=(wall_x-xy[0])/c if c>1e-9 else (0.-xy[0])/c if c<-1e-9 else 1e9
        ty=(5.-xy[1])/s if s>1e-9 else (0.-xy[1])/s if s<-1e-9 else 1e9
        distances.append(max(0.,min(tx,ty)))
    return dict(angle_min=-math.pi,angle_max=math.pi,angle_increment=2*math.pi/720,
                range_min=.02,range_max=12.,ranges=distances)


def covariance(xy):
    result=[0.]*21
    for index,value in zip((0,6,11,15,18,20),(xy,xy,1e6,1e6,1e6,1e6)):
        result[index]=value
    return result


def run(args):
    isolation()
    allowed_tuning = {'MPC_XY_P': (.5, 4.), 'MPC_XY_VEL_P_ACC': (1., 6.),
        'MPC_XY_VEL_I_ACC': (.1, 2.), 'MPC_XY_VEL_D_ACC': (0., .5),
        'MPC_VEL_LP': (0., 10.), 'EKF2_HGT_REF': (0., 2.),
        'EKF2_RNG_CTRL': (1., 2.), 'EKF2_RNG_NOISE': (.03, .2),
        'EKF2_TAU_POS': (.1, .5), 'EKF2_EVP_NOISE': (.03, .3), 'MPC_XY_VEL_MAX': (.25, .25),
        'EKF2_RNG_SFE': (0., .05), 'EKF2_ACC_NOISE': (.1, 1.), 'EKF2_GYR_NOISE': (.015, .1),
        'EKF2_LID2_AGE': (150., 250.)}
    allowed_tuning.update(EKF2_OF_N_MIN=(.05,.15),EKF2_OF_N_MAX=(.05,.5),EKF2_OF_DELAY=(0.,20.))
    tuning = json.loads(args.sitl_parameters.read_text(encoding='utf-8')) if args.sitl_parameters else {}
    if (not isinstance(tuning,dict) or any(k not in allowed_tuning or type(v) not in (int,float)
            or not math.isfinite(v) or not allowed_tuning[k][0] <= v <= allowed_tuning[k][1]
            for k,v in tuning.items())):
        raise ValueError('bounded_simulator_parameter_profile_required')
    if tuning.get('EKF2_HGT_REF',0) not in (0,2) or tuning.get('EKF2_RNG_CTRL',1) not in (1,2):
        raise ValueError('explicit_simulator_height_reference_required')
    native_gain_bound=max(1.,tuning.get('MPC_XY_P',1.))
    runtime=args.runtime.resolve();binary=runtime/'bin/px4'
    expected_binary = EXPECTED_BINARY
    artifact = None
    if args.patched_runtime:
        artifact = json.loads((runtime/'artifact.json').read_text(encoding='utf-8'))
        if (artifact.get('base_commit') != FIRMWARE or artifact.get('kind') != 'lidar2d_experiment'
                or sha(runtime/'px4_lidar2d.patch') != artifact.get('patch_sha256')
                or sha(runtime/'lidar2d.px4board') != artifact.get('board_sha256')):
            raise RuntimeError('unrecognized patched simulator artifact')
        expected_binary = artifact['binary_sha256']
    if args.lidar2d and not args.patched_runtime:
        raise RuntimeError('dedicated LiDAR2D firmware required')
    spinning = args.lidar_source=='spinning'
    if (spinning or args.sensor_capture_time) and (not artifact or not artifact.get('simulation_truth_stream')):
        raise RuntimeError('timestamped_spinning_simulator_required')
    if (runtime/'source_commit.txt').read_text().strip()!=FIRMWARE or sha(binary)!=expected_binary:
        raise RuntimeError('unrecognized simulator artifact')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    (output/'rootfs').mkdir()
    cfg=Settings(command_frame='local_xy',execution_profile='anchor_trial',full_mission=True,
        execute=True,layout_confirmed=True,sensor_mount_confirmed=True,takeoff_settings_confirmed=True,
        expected_ekf2_mag_type=0,expected_mis_takeoff_alt_m=1.7,speed_m_s=.25,
        enu_offset_x_m=-2.,enu_offset_y_m=-2.,bounds_xy_m=(.3,4.7,.3,4.7),
        leg_timeout_s=25.,takeoff_timeout_s=35.,takeoff_settle_s=2.)
    guard_cfg=GuardConfig(radius_m=.20,clearance_m=.50,brake_accel_m_s2=.5,
        command_latency_s=.20,max_age_s=.25,max_speed_m_s=.25,range_error_m=.015,
        pose_error_m=.04,obstacle_speed_bound_m_s=0.,velocity_error_m_s=.35)
    guard=HorizontalGuard(guard_cfg,source_id='synthetic_lidar',clock_id='px4_sim',epoch='run')
    mission=(LocalXYMission(cfg) if args.baseline else GuardedLocalXYMission(cfg,guard=guard,
        proportional_gain_s_inv=1.,resume_clear_s=.5,max_hold_s=3.,
        hover_position_hold=args.dwell_control=='position',native_position_gain_bound_s_inv=native_gain_bound,
        native_speed_limit_m_s=tuning.get('MPC_XY_VEL_MAX')))
    manifest=dict(schema=1,firmware_commit=FIRMWARE,binary_sha256=sha(binary),
        board_sha256=sha(runtime/('lidar2d.px4board' if args.patched_runtime else 'evbench.px4board')),script_sha256=sha(Path(__file__)),
        settings=asdict(cfg),guard=asdict(guard_cfg),scenario=args.scenario,seed=args.seed,
        baseline=args.baseline,instance=args.instance,speed_factor=args.speed_factor,
        dwell_control=args.dwell_control,fusion_horizon_ms=args.fusion_horizon_ms,
        zero_unused_airspeed_delay=args.zero_unused_airspeed_delay,
        velocity_output_tau_s=args.velocity_output_tau_s,
        sitl_parameters=tuning,hover_duration_s=args.hover_duration_s,
        lidar_source=args.lidar_source,
        deskew_enabled=not args.no_deskew, spatial_support_enabled=not args.no_spatial_support,
        sensor_capture_time=args.sensor_capture_time,
        lidar_velocity_intervals=args.lidar_velocity_intervals,
        simulated_lidar=dict(beams=430,range_noise_m=.01,scan_bias_noise_m=.003,
            origin_quantization_bound_m=0. if spinning else .012,zero_imu_lever_arm=True,
            instantaneous_scan=not spinning, sequential_beams=spinning,
            first_ray_header=True,beam_time_increment_s=.1/430,
            lidar_clock_offset_s=.037,lidar_clock_drift_ppm=35.),
        scan_algorithm_sha256=sha(Path(sys.modules[PlanarScanOdometry.__module__].__file__)),
        scan_matcher_sha256=sha(Path(sys.modules[PlanarScanOdometry.__module__].__file__).with_name('lidar_scanmatch.py')),
        spinning_source_sha256={name:sha(Path(sys.modules[SpinningScanOdometry.__module__].__file__).with_name(name))
            for name in ('lidar_spinning.py','lidar_timing.py')},
        native_position_gain_bound_s_inv=native_gain_bound,
        source_sha256={name:sha(Path(__file__).parent.parent/'drone_mission'/name)
            for name in ('guarded_local_xy.py','local_xy.py','horizontal_obstacles.py','local_command.py')},
        scope='closed-loop mission command test with synthetic UWB and virtual horizontal obstacles',
        lidar_aiding=args.lidar2d,lidar_transmitted=args.patched_runtime,raw_lidar_used=False,physical_writes=0,ros_used=False,
        patched_firmware=artifact,
        lidar_observation=('10 Hz sequential noisy rays -> aligned capture-time IMU deskew -> spatial/temporal robust matcher -> interval midpoint NE velocity'
            if spinning else
            '10 Hz virtual noisy ranges -> actual robust keyframe matcher -> midpoint NE velocity; FC attitude and measured range only'
            if args.lidar_source=='scan_matcher' else
            '10 Hz truth-derived instantaneous NE velocity plus independent seeded 0.02 m/s noise; not raw scan-matcher output'),
        random_seed_scope='seed applies to synthetic UWB; seed+10000 independently to synthetic LiDAR. SIH sensor RNG is not overridden.',
        synthetic_optical_flow=args.synthetic_flow,
        obstacle_transform='Synthetic ranges use truth; placement uses same-time FC estimated XY. No truth position is supplied to the guard.',
        run8_core_alignment=dict(EKF2_MAG_TYPE=0,EKF2_HGT_REF=0,EKF2_EV_CTRL=1,
            EKF2_EV_DELAY=0,EKF2_GPS_CTRL=0,MIS_TAKEOFF_ALT=1.7,
            missing_physical_modalities=[] if args.synthetic_flow else ['optical_flow','downward_range'],
            note='Not a full run8 reproduction. Synthetic sensor mounting is zero.'),
        time_contract=('EV phase-decimated to <=40 Hz with HIL capture timestamp; flow trapezoidal integrals with capture timestamp'
            if args.sensor_capture_time else
            'EV sampled per new HIL truth message; latest PX4 local-position clock; timestamp uncertainty <= 40 ms required'),
        network_namespace=os.readlink('/proc/self/ns/net'),
        predeclared=dict(min_wall_surface_clearance_m=.50,body_radius_m=.20,
            stale_scan_zero_command_within_s=.30,require_actual_arm=True,
            require_airborne=True,require_offboard=True,require_feedback=True,
            note='braking criteria apply after the mission reaches MOVING; no 3D collision guarantee'))
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    source_copy=output/'source_snapshot';source_copy.mkdir()
    for p in [Path(__file__),Path(sys.modules[PlanarScanOdometry.__module__].__file__),
            Path(sys.modules[PlanarScanOdometry.__module__].__file__).with_name('lidar_scanmatch.py'),
            Path(sys.modules[SpinningScanOdometry.__module__].__file__),
            Path(sys.modules[SpinningScanOdometry.__module__].__file__).with_name('lidar_timing.py'),
            *[Path(__file__).parent.parent/'drone_mission'/name for name in manifest['source_sha256']]]:
        shutil.copyfile(p,source_copy/p.name)
    rng=np.random.default_rng(args.seed)
    lidar_rng=np.random.default_rng(args.seed+10000)
    virtual_lidar=VirtualLidar(args.seed+20000)
    scan_odometry=PlanarScanOdometry(simulation_covariance_model=True,
        velocity_intervals=args.lidar_velocity_intervals,simulation_position_quantum_m=.012)
    spin_sensor = SpinningLidar(args.seed+20000)
    spin_tracker = SpinningScanOdometry(simulation_covariance_model=True,
        velocity_intervals=args.lidar_velocity_intervals, simulation_position_quantum_m=0.,
        deskew=not args.no_deskew, spatial_support=not args.no_spatial_support)
    truth_timeline = PoseTimeline()
    master,slave=pty.openpty();stop=threading.Event()
    console=(output/'px4.log').open('wb')
    log=(output/'telemetry.jsonl').open('w',encoding='utf-8')
    records=(output/'mission.jsonl').open('w',encoding='utf-8')
    scan_records=(output/'scan_observations.jsonl').open('w',encoding='utf-8')
    proc=connection=None
    status=dict(errors=[],physical_writes=0,armed_seen=False,airborne_seen=False,
                offboard_seen=False,moving_seen=False,feedback_count=0,ev_samples=0,
                flow_samples=0,flow_invalid_samples=0,range_samples=0)
    status['lidar_samples']=0

    def shell(value):os.write(master,(value+'\n').encode('utf-8'))
    def pump():
        while not stop.is_set():
            if select.select([master],[],[],.05)[0]:
                try:console.write(os.read(master,65536));console.flush()
                except OSError:return

    env=dict(os.environ,PX4_SIM_MODEL='sihsim_quadx',PX4_SIMULATOR='sihsim',
        PX4_SIM_SPEED_FACTOR=str(args.speed_factor),PX4_PARAM_EKF2_GPS_CTRL='0',
        PX4_PARAM_EKF2_EV_CTRL='1',PX4_PARAM_EKF2_EV_DELAY='0',PX4_PARAM_EKF2_EV_NOISE_MD='0',
        PX4_PARAM_EKF2_EVP_NOISE='0.1',PX4_PARAM_EKF2_MAG_TYPE='0',PX4_PARAM_EKF2_HGT_REF='0',
        PX4_PARAM_COM_RC_IN_MODE='4',PX4_PARAM_COM_RC_OVERRIDE='3',PX4_PARAM_NAV_DLL_ACT='0',
        PX4_PARAM_COM_RCL_EXCEPT='4',PX4_PARAM_MIS_TAKEOFF_ALT='1.7',PX4_PARAM_MIS_TKO_LAND_REQ='0',
        PX4_PARAM_SDLOG_MODE='1',PX4_PARAM_SDLOG_PROFILE='1',ROS_DOMAIN_ID='97',ROS_LOCALHOST_ONLY='1')
    if args.patched_runtime:
        env.update(PX4_PARAM_EKF2_LID2_EN='1' if args.lidar2d else '0',PX4_PARAM_EKF2_AVEL_DELAY='0')
    # A simulator-only ablation. Real sensor latency has not been calibrated;
    # this value is never written to hardware or an operational config.
    if args.fusion_horizon_ms is not None:
        env['PX4_PARAM_EKF2_DELAY_MAX']=str(args.fusion_horizon_ms)
    if args.zero_unused_airspeed_delay:
        # SIH quadrotor has no airspeed input. The compiled airspeed default
        # otherwise raises DELAY_MAX to 100 ms, even when it is not fused.
        env['PX4_PARAM_EKF2_ASP_DELAY']='0'
    if args.velocity_output_tau_s is not None:
        env['PX4_PARAM_EKF2_TAU_VEL']=str(args.velocity_output_tau_s)
    for name,value in tuning.items():
        env['PX4_PARAM_'+name]=str(value)
    try:
        proc=subprocess.Popen([str(binary),str(runtime/'etc'),'-i',str(args.instance),
            '-w',str(output/'rootfs')],stdin=slave,stdout=slave,stderr=slave,cwd=output,env=env,start_new_session=True)
        os.close(slave);thread=threading.Thread(target=pump,daemon=True);thread.start()
        port=18570+args.instance
        connection=mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}',source_system=244,source_component=191)
        samples={};params={};clock=0;first_clock=None;last_hb=-1.;last_tick=-1.;last_scan=-1.
        streams=False;truth_origin=None;truth_xy=None;truth_seq=0;last_ev_truth=-1;last_ev_clock=0
        last_truth_clock=0;last_pose_clock=0;uwb_xy=None;start_request=None;moving_start=None
        sequence=0;pending_command=None;mode_request_s=-1.;start_wall=time.monotonic()
        truth_alt0=None;previous_flow_clock=None;last_flow=None
        previous_flow_rate=None;previous_flow_angular=None
        integrated_flow=np.zeros(2);integrated_angular=np.zeros(3);integrated_flow_dt=0.
        next_uwb_clock=0.;source_time=0.
        truth_velocity=None;last_lidar_clock=-1.;last_lidar_sample=0.;lidar_reset=0;lidar_reset_done=False
        lidar_reset_offset=0
        last_lidar_truth=-1;next_lidar_clock=0.
        next_spin_first=None;last_clock_exchange=-1.;spin_truth_stamp=0.
        pose_history=deque(maxlen=300);spin_guard_pending=None
        min_clearance=math.inf;max_speed=0.;fault_start=None;zero_after_fault=None
        wall_x=3.25 if args.scenario=='wall' else 5.
        while time.monotonic()-start_wall < args.duration/args.speed_factor+60.:
            if proc.poll() is not None:raise RuntimeError('PX4 exited')
            host_now=time.monotonic()
            if host_now-last_hb>1.:
                connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,4)
                last_hb=host_now
            msg=connection.recv_match(blocking=False)
            if msg is not None:
                kind=msg.get_type()
                if kind=='LOCAL_POSITION_NED' and msg.time_boot_ms>0:
                    clock=msg.time_boot_ms/1000.;last_pose_clock=clock
                    pose_history.append((clock,msg.y+2.,msg.x+2.))
                    if first_clock is None:first_clock=clock
                samples[kind]=(msg,clock)
                if kind in ('LOCAL_POSITION_NED','HIL_STATE_QUATERNION','HEARTBEAT','ESTIMATOR_STATUS',
                            'POSITION_TARGET_LOCAL_NED','COMMAND_ACK','PARAM_VALUE','STATUSTEXT',
                            'HIGHRES_IMU','ATTITUDE_QUATERNION','ODOMETRY'):
                    log.write(json.dumps(dict(clock_s=clock,host_monotonic_s=host_now,message=msg.to_dict()),allow_nan=True)+'\n')
                if kind=='HEARTBEAT':
                    connection.target_system=msg.get_srcSystem();connection.target_component=1
                    status['armed_seen'] |= bool(msg.base_mode & 128)
                    status['offboard_seen'] |= px4_mode(msg)=='OFFBOARD'
                    if not streams:
                        for name,rate in [('LOCAL_POSITION_NED',50),('HIL_STATE_QUATERNION',100 if spinning else 40),
                            ('POSITION_TARGET_LOCAL_NED',50),('ESTIMATOR_STATUS',10),('EXTENDED_SYS_STATE',10),
                            ('BATTERY_STATUS',2),('TIMESYNC',50),('ATTITUDE_QUATERNION',100 if spinning else 50),
                            ('HIGHRES_IMU',200 if spinning else 0),('DISTANCE_SENSOR',20)]:
                            shell(f'mavlink stream -r {rate} -s {name} -u {port}')
                        shell('ver all');shell('param show EKF2_*');streams=True
                        for name in set(('COM_RC_OVERRIDE','EKF2_MAG_TYPE','MIS_TAKEOFF_ALT','MPC_XY_P','EKF2_DELAY_MAX','EKF2_ASP_DELAY','EKF2_TAU_VEL')) | set(tuning):
                            connection.mav.param_request_read_send(connection.target_system,1,name.encode(),-1)
                if kind=='PARAM_VALUE':
                    # This pinned PX4 uses MAVLink's byte-wise parameter
                    # encoding; an INT32 is not a numeric float conversion.
                    if msg.param_type==mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                        params[msg.param_id]=struct.unpack('<i',struct.pack('<f',msg.param_value))[0]
                    elif msg.param_type==mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
                        params[msg.param_id]=msg.param_value
                if kind=='TIMESYNC' and msg.tc1==0:
                    connection.mav.timesync_send(msg.ts1,msg.ts1)
                imu_fault = args.scenario=='imu_gap' and moving_start is not None and 1.<=clock-moving_start<3.
                if spinning and kind=='HIGHRES_IMU' and msg.time_usec>0 and not imu_fault:
                    spin_tracker.imu.add_gyro(msg.time_usec*1e-6, [msg.xgyro,-msg.ygyro,-msg.zgyro])
                if spinning and kind=='ATTITUDE_QUATERNION' and msg.time_boot_ms>0:
                    spin_tracker.imu.add_attitude(msg.time_boot_ms*.001,
                        enu_flu_rotation(rotation_world_body([msg.q1,msg.q2,msg.q3,msg.q4])))
                if spinning and kind=='ODOMETRY' and msg.estimator_type==mavutil.mavlink.MAV_ESTIMATOR_TYPE_NAIVE:
                    spin_truth_stamp=msg.time_usec*1e-6
                    truth_timeline.add(spin_truth_stamp, [msg.y+2.,msg.x+2.,-msg.z],
                        enu_flu_rotation(rotation_world_body(msg.q)), [msg.vy,msg.vx,-msg.vz])
                    if next_spin_first is None:
                        next_spin_first=math.ceil((spin_truth_stamp+3.)*10)/10.
                    clock_fault = args.scenario=='clock_gap' and moving_start is not None and 1.<=clock-moving_start<3.
                    if spin_truth_stamp-last_clock_exchange>=.019 and not clock_fault:
                        spin_tracker.clock.add_exchange(*spin_sensor.exchange(spin_truth_stamp))
                        last_clock_exchange=spin_truth_stamp
                if kind=='COMMAND_ACK' and pending_command and msg.command==pending_command[0]:
                    mission.command_result(pending_command[1],msg.result==0,clock)
                    pending_command=None
                if kind=='HIL_STATE_QUATERNION' and clock:
                    source_time=msg.time_usec*1e-6 if args.sensor_capture_time and msg.time_usec>0 else clock
                    if truth_origin is None:truth_origin=(msg.lat/1e7,msg.lon/1e7)
                    north=math.radians(msg.lat/1e7-truth_origin[0])*6371000.
                    east=math.radians(msg.lon/1e7-truth_origin[1])*6371000.*math.cos(math.radians(truth_origin[0]))
                    truth_xy=(east+2.,north+2.);truth_seq+=1;last_truth_clock=source_time
                    truth_velocity=np.array([msg.vx,msg.vy])/100.
                    if truth_alt0 is None:truth_alt0=msg.alt/1000.
                    height=msg.alt/1000.-truth_alt0
                    if args.synthetic_flow:
                        rotation=rotation_world_body(msg.attitude_quaternion)
                        body_velocity=rotation.T@np.array([msg.vx,msg.vy,msg.vz])/100.
                        angular=np.array([msg.rollspeed,msg.pitchspeed,msg.yawspeed])
                        ray=height/rotation[2,2] if rotation[2,2]>.7 else -1.
                        if previous_flow_clock is not None and .005<=source_time-previous_flow_clock<=.1:
                            dt=source_time-previous_flow_clock
                            # Pinned EKF2 negates both MAVLink pixel and gyro
                            # integrals. Its compensated prediction is [vy/r,-vx/r].
                            flow_valid=.10<=ray<=5.
                            pixel_rate=(np.array([-body_velocity[1]/ray,body_velocity[0]/ray])+angular[:2]
                                        if flow_valid else np.zeros(2))
                            if args.sensor_capture_time:
                                previous=previous_flow_rate if previous_flow_rate is not None else pixel_rate
                                previous_angular=previous_flow_angular if previous_flow_angular is not None else angular
                                integrated_flow+=(previous+pixel_rate)*(.5*dt)
                                integrated_angular+=(previous_angular+angular)*(.5*dt)
                                integrated_flow_dt+=dt
                            else:
                                integrated_flow=pixel_rate*dt;integrated_angular=angular*dt;integrated_flow_dt=dt
                            send_flow=not args.sensor_capture_time or integrated_flow_dt>=.018
                            if send_flow:
                                connection.mav.optical_flow_rad_send(round(source_time*1e6),0,round(integrated_flow_dt*1e6),
                                    *integrated_flow,*integrated_angular,20,255 if flow_valid else 0,0,float(ray) if flow_valid else -1.)
                                status['flow_samples']+=1
                                status['flow_invalid_samples']+=int(not flow_valid)
                                integrated_flow=np.zeros(2);integrated_angular=np.zeros(3);integrated_flow_dt=0.
                            # Advertise the synthetic sensor while DISARM so
                            # Sensors initializes VehicleOpticalFlow before ARM.
                            # Invalid ground range remains unknown, quality 0.
                            if flow_valid and send_flow:
                                connection.mav.distance_sensor_send(round(source_time*1000),10,500,round(ray*100),
                                    0,42,mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,1)
                                status['range_samples']+=1
                            last_flow=clock
                            previous_flow_rate=pixel_rate;previous_flow_angular=angular
                        previous_flow_clock=source_time
                if kind=='POSITION_TARGET_LOCAL_NED':
                    ns=round(clock*1e9)
                    if msg.type_mask & 31 == 3 and hasattr(mission,'applied_local_velocity'):
                        ok=mission.applied_local_velocity((msg.vy,msg.vx),-msg.z,ns)
                    else:ok=mission.applied_local_target((msg.y+2.,msg.x+2.),-msg.z,ns)
                    status['feedback_count']+=int(ok)
            if first_clock is None:
                if msg is None:time.sleep(.0003)
                continue
            elapsed=clock-first_clock
            if elapsed>=args.duration:break
            active_fault=moving_start is not None and 1.<=clock-moving_start<3.
            if (truth_xy and truth_seq!=last_ev_truth and elapsed>3. and clock-last_pose_clock<=.04
                    and (not args.sensor_capture_time or source_time>=next_uwb_clock)
                    and not (active_fault and args.scenario=='uwb_gap')):
                # No repeated scan/pose is relabelled. Ground-truth transport
                # quantization and bounded clock association are documented.
                noise=rng.normal(0.,.01,2);uwb_xy=(truth_xy[0]+float(noise[0]),truth_xy[1]+float(noise[1]))
                if active_fault and args.scenario=='uwb_jump':uwb_xy=(uwb_xy[0]+1.2,uwb_xy[1])
                connection.mav.odometry_send(round((source_time if args.sensor_capture_time else clock)*1e6),mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,uwb_xy[1]-2.,uwb_xy[0]-2.,0.,[math.nan]*4,
                    math.nan,math.nan,math.nan,math.nan,math.nan,math.nan,
                    covariance(.03**2),covariance(1e6),reset_counter=0,quality=100,
                    estimator_type=mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION)
                last_ev_truth=truth_seq;last_ev_clock=clock;status['ev_samples']+=1
                if args.sensor_capture_time:
                    next_uwb_clock=max(next_uwb_clock+.025,source_time+.001)
            spin_ready = spinning and next_spin_first is not None and spin_truth_stamp>=next_spin_first+.105
            if (spin_ready and spin_truth_stamp<next_spin_first+.14
                    and (not spin_tracker.imu.gyros or spin_tracker.imu.gyros[-1][0]<next_spin_first+.101)):
                # Give a trailing captured IMU packet time to arrive. The
                # fixed 140 ms acquisition deadline still exposes real gaps.
                spin_ready=False
            if (args.patched_runtime and truth_velocity is not None and elapsed>3.
                    and ((not spinning and clock>=next_lidar_clock and truth_seq!=last_lidar_truth
                        and clock-last_truth_clock<=.04) or spin_ready)):
                sample_clock=clock
                value=truth_velocity+lidar_rng.normal(0.,.02,2)
                frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED
                cov=covariance(.03**2);cov[1]=.0002
                accepted=True
                if spinning:
                    observation=dict(accepted=False,reasons=['missing_measured_floor_range'])
                    spin_tracker.last_obstacle_cloud=None
                    scan=None
                    started=time.perf_counter()
                    try:
                        person = None
                        missing = None
                        if args.scenario=='person' and moving_start is not None:
                            def person(times):
                                return np.column_stack((np.full(len(times),3.7),
                                    2.+.8*np.sin((times-moving_start)*.8))), .30
                        if args.scenario=='partial_occlusion' and active_fault:
                            missing=(-math.pi/2, -.15)
                        scan=spin_sensor.observe(truth_timeline, first_ray_fc_s=next_spin_first,
                            wall_x_m=wall_x, pillars=((.55,.55,.15),(4.45,4.4,.18)),
                            person=person,missing_sector=missing)
                        distance=samples.get('DISTANCE_SENSOR')
                        attitude=samples.get('ATTITUDE_QUATERNION')
                        if distance and attitude and clock-distance[1]<=.15:
                            a,d=attitude[0],distance[0]
                            fc_rotation=enu_flu_rotation(rotation_world_body([a.q1,a.q2,a.q3,a.q4]))
                            if d.orientation==mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270 and d.min_distance<=d.current_distance<=d.max_distance:
                                observation=spin_tracker.observe_scan(scan,
                                    now_local_s=spin_sensor.local_time(max(spin_truth_stamp,clock)),
                                    height_above_floor_m=d.current_distance*.01*fc_rotation[2,2])
                    except TimingError as error:
                        observation=dict(accepted=False,reasons=[str(error)])
                        spin_tracker.matcher.previous=None;spin_tracker.history.clear()
                    accepted=observation['accepted']
                    spin_guard_pending=spin_tracker.last_obstacle_cloud
                    if spin_guard_pending is None:
                        guard.frame=None;guard.input_reason='spinning_scan_or_imu_unavailable'
                    if accepted:
                        sample_clock=observation['sample_midpoint_s']
                        value=np.array(observation['velocity_ne_m_s'])
                        c=np.array(observation['covariance_ne_m2_s2'])
                        cov=covariance(c[0,0]);cov[1]=c[0,1];cov[6]=c[1,1]
                        lidar_reset=(observation['reset_counter']+lidar_reset_offset)%256
                    scan_records.write(json.dumps(dict(clock_s=clock,
                        ranges_m=[float(v) if np.isfinite(v) else None for v in scan.ranges] if scan else None,
                        first_ray_fc_s=next_spin_first,first_ray_lidar_s=scan.first_ray_s if scan else None,
                        time_increment_s=scan.time_increment_s if scan else None,
                        process_ms=(time.perf_counter()-started)*1000.,observation=observation,
                        truth_used_by='virtual_sensor_and_offline_evaluator_only'),allow_nan=False)+'\n')
                    next_spin_first+=.1
                if args.lidar_source=='scan_matcher':
                    truth_msg=samples['HIL_STATE_QUATERNION'][0]
                    ranges=virtual_lidar.observe(position_enu_m=[*truth_xy,truth_msg.alt/1000.-truth_alt0],
                        rotation_enu_flu=enu_flu_rotation(rotation_world_body(truth_msg.attitude_quaternion)),wall_x_m=wall_x)
                    attitude=samples.get('ATTITUDE_QUATERNION');distance=samples.get('DISTANCE_SENSOR')
                    accepted=False
                    observation=dict(accepted=False,reasons=['missing_or_stale_fc_attitude_or_range'])
                    if attitude and distance and clock-attitude[1]<=.06 and clock-distance[1]<=.15:
                        a,d=attitude[0],distance[0]
                        fc_rotation=enu_flu_rotation(rotation_world_body([a.q1,a.q2,a.q3,a.q4]))
                        if d.orientation==mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270 and d.min_distance<=d.current_distance<=d.max_distance:
                            observation=scan_odometry.observe(ranges=ranges,stamp_s=clock,
                                rotation_enu_flu=fc_rotation,height_above_floor_m=d.current_distance*.01*fc_rotation[2,2])
                            accepted=observation['accepted']
                            if accepted:
                                sample_clock=observation['sample_midpoint_s']
                                value=np.array(observation['velocity_ne_m_s'])
                                c=np.array(observation['covariance_ne_m2_s2'])
                                cov=covariance(c[0,0]);cov[1]=c[0,1];cov[6]=c[1,1]
                                lidar_reset=(observation['reset_counter']+lidar_reset_offset)%256
                    scan_records.write(json.dumps(dict(clock_s=clock,ranges_m=ranges.tolist(),
                        observation=observation,truth_velocity_ne_m_s=truth_velocity.tolist(),
                        truth_used_by='virtual_sensor_and_offline_evaluator_only'),allow_nan=False)+'\n')
                if active_fault:
                    if args.scenario=='lidar_jump':value+=2.
                    elif args.scenario=='lidar_stale':sample_clock-=.5
                    elif args.scenario=='lidar_duplicate':sample_clock=last_lidar_sample
                    elif args.scenario=='lidar_bad_frame':frame=mavutil.mavlink.MAV_FRAME_BODY_FRD
                    elif args.scenario=='lidar_bad_cov':cov[1]=1.
                    elif args.scenario=='lidar_reset' and not lidar_reset_done:
                        lidar_reset=(lidar_reset+1)%256;lidar_reset_offset+=1;lidar_reset_done=True
                if accepted and not (active_fault and args.scenario=='lidar_gap'):
                    connection.mav.odometry_send(round(sample_clock*1e6),mavutil.mavlink.MAV_FRAME_LOCAL_NED,frame,
                        math.nan,math.nan,math.nan,[math.nan]*4,*value,math.nan,math.nan,math.nan,math.nan,
                        covariance(1e6),cov,reset_counter=lidar_reset,quality=100,
                        estimator_type=mavutil.mavlink.MAV_ESTIMATOR_TYPE_LIDAR)
                    status['lidar_samples']+=1
                    if args.scenario!='lidar_duplicate' or not active_fault:last_lidar_sample=sample_clock
                last_lidar_clock=clock
                last_lidar_truth=truth_seq
                next_lidar_clock=max(next_lidar_clock+.1,clock+.001)
            if clock-last_tick<.05:continue
            last_tick=clock;ns=round(clock*1e9)
            def age(kind):return clock-samples[kind][1] if kind in samples else math.inf
            def data(kind):return samples[kind][0] if kind in samples else None
            hb,pose,est,land=data('HEARTBEAT'),data('LOCAL_POSITION_NED'),data('ESTIMATOR_STATUS'),data('EXTENDED_SYS_STATE')
            if not all(x is not None for x in (hb,pose,est,land)) or not truth_xy:continue
            mode=px4_mode(hb);armed=bool(hb.base_mode & 128)
            status['airborne_seen'] |= land.landed_state==2
            if elapsed>15. and not armed and mode!='POSCTL' and mission.phase in ('IDLE','READY') and clock-mode_request_s>2.:
                connection.mav.command_long_send(connection.target_system,1,176,0,1,3,0,0,0,0,0)
                mode_request_s=clock
            flags=est.flags
            s=Snapshot(connected=True,armed=armed,mode=mode,landed=land.landed_state,
                state_age_s=age('HEARTBEAT'),landed_age_s=age('EXTENDED_SYS_STATE'),
                estimator_age_s=age('ESTIMATOR_STATUS'),estimator_valid=bool(flags&1 and flags&2 and flags&8),
                vertical_estimator_valid=bool(flags&4 and flags&32),pose_age_s=age('LOCAL_POSITION_NED'),
                pose_stamp_ns=round(pose.time_boot_ms*1e6),xy=(pose.y+2.,pose.x+2.),velocity_xy=(pose.vy,pose.vx),
                altitude_enu_m=-pose.z,vertical_speed_m_s=-pose.vz,yaw_deg=90.,
                battery=.9,battery_age_s=0.,command_services_ready=True,land_service_ready=True,
                local_setpoint_ready=True,rc_required=False,rc_override=int(params.get('COM_RC_OVERRIDE',0)),
                rc_switch_channels=(),takeoff_action=0,mag_type=int(params.get('EKF2_MAG_TYPE',-1)),
                takeoff_alt_m=params.get('MIS_TAKEOFF_ALT'),takeoff_param_age_s=0. if params else math.inf,
                uwb_xy=uwb_xy or (),uwb_age_s=clock-last_ev_clock,uwb_stamp_ns=round(last_ev_clock*1e9),
                bridge_ready=status['ev_samples']>0,bridge_published=status['ev_samples'],bridge_age_s=0.,
                bridge_observation_age_s=clock-last_ev_clock,alignment_matches=True)
            # Battery, RC absence, mount, layout and bridge are declared
            # simulator fixtures. Pose/mode/ARM/land/estimator/ACK are telemetry.
            mission.last_web_s=clock
            if elapsed>22. and start_request is None:
                if not 0 < params.get('MPC_XY_P',float('inf')) <= native_gain_bound+1e-6:
                    raise RuntimeError('native_position_gain_exceeds_guard_contract')
                if any(abs(params.get(k,float('inf'))-v)>1e-6 for k,v in tuning.items()):
                    raise RuntimeError('simulator_parameter_profile_not_applied')
                if (args.fusion_horizon_ms is not None
                        and params.get('EKF2_DELAY_MAX') != args.fusion_horizon_ms):
                    raise RuntimeError('requested_fusion_horizon_not_applied')
                if args.zero_unused_airspeed_delay and params.get('EKF2_ASP_DELAY') != 0.:
                    raise RuntimeError('unused_airspeed_delay_not_applied')
                if (args.velocity_output_tau_s is not None and
                        abs(params.get('EKF2_TAU_VEL',float('inf'))-args.velocity_output_tau_s)>1e-6):
                    raise RuntimeError('velocity_output_tau_not_applied')
                wall=time.time()
                start_request=dict(ok=True,contract_version='1.0',drone_id='5',status='ACTIVE',
                    control_action='start',control_request_id='isolated-sitl',
                    control_requested_at=datetime.fromtimestamp(math.floor(wall),timezone.utc).isoformat(),
                    mission_db_id=1,mission_code='SITL',route_revision='1',anchor_layout_id=LAYOUT,
                    coordinate_frame='UWB_ANCHOR_LOCAL',origin='A1',x_axis='A1_TO_A2',y_axis='A1_TO_A3',
                    z_axis='UP_FROM_FLOOR',unit='meter',ceiling_height_m=3.,
                    route_tasks=[dict(id='P1',type='waypoint',x=2.9,y=2.,dwell_s=args.hover_duration_s)])
                if not mission.submit(start_request,clock,wall):
                    raise RuntimeError('start_request_rejected:'+json.dumps(mission.validation))
            moving=mission.phase in ('MOVING','DWELL','RETURNING')
            if moving and moving_start is None:moving_start=clock;status['moving_seen']=True
            in_fault=(args.scenario=='scan_gap' and moving_start is not None and 1.<=clock-moving_start<3.)
            if in_fault and fault_start is None:fault_start=clock
            if spinning and spin_guard_pending is not None and not in_fault:
                p=spin_guard_pending
                history=np.asarray(pose_history)
                stamp=p['reference_fc_s']
                if len(history)>1 and history[0,0]<=stamp<=history[-1,0]:
                    origin=tuple(float(np.interp(stamp,history[:,0],history[:,i])) for i in (1,2))
                    guard.observe(ordered_obstacles(relative_enu_xy=p['relative_enu_xy'],
                        valid=p['valid'],bearings_enu_rad=p['bearings_enu_rad'],sensor_origin_xy_m=origin,
                        stamp_ns=round(p['first_ray_fc_s']*1e9),sequence=sequence,clock_id='px4_sim',
                        source_id='synthetic_lidar',epoch='run',bins=guard_cfg.bins,
                        acquisition_motion_bound_m=p['acquisition_motion_bound_m']))
                    sequence+=1;last_scan=clock
                spin_guard_pending=None
            if not spinning and clock-last_scan>=.1 and not in_fault:
                scan=scan_box(truth_xy,wall_x)
                if args.scenario=='unknown_front' and moving_start is not None and clock-moving_start>1.:
                    scan['ranges'][360]=0.
                guard.observe(scan_obstacles(scan,stamp_ns=ns,sequence=sequence,clock_id='px4_sim',
                    source_id='synthetic_lidar',epoch='run',sensor_origin_xy_m=s.xy,sensor_yaw_rad=0.,
                    bins=guard_cfg.bins,mount_confirmed=True,horizontal_plane_confirmed=True,
                    oldest_stamp_confirmed=True,acquisition_motion_bound_m=.01))
                sequence+=1;last_scan=clock
            phase_before_tick=mission.phase
            target_before_tick=mission.goal
            height_before_tick=mission.height_target
            actions=mission.tick(s,clock,time.time(),ns)
            for action in actions:
                kind=action['kind']
                if kind=='offboard_mode':command,values=176,[1,6,0,0,0,0,0]
                elif kind=='position_mode':command,values=176,[1,3,0,0,0,0,0]
                elif kind=='arm':command,values=400,[1,0,0,0,0,0,0]
                elif kind=='land':command,values=21,[0,0,0,math.nan,math.nan,math.nan,math.nan]
                else:raise RuntimeError('unexpected mission command:'+kind)
                connection.mav.command_long_send(connection.target_system,1,command,0,*values)
                pending_command=(command,action['token'])
            goal=mission.local_setpoint()
            if goal:
                fields=local_command_fields(goal,cfg);p=fields['position_enu'];v=fields['velocity_enu']
                connection.mav.set_position_target_local_ned_send(round(clock*1000),connection.target_system,1,1,
                    fields['type_mask'],p[1],p[0],-p[2],v[1],v[0],-v[2],0.,0.,0.,math.pi/2-fields['yaw_enu_rad'],0.)
                if fault_start and zero_after_fault is None and (goal.get('velocity_xy_m_s')==(0.,0.) or goal.get('obstacle_braking_hold')):zero_after_fault=clock
            if moving_start is not None:
                min_clearance=min(min_clearance,wall_x-truth_xy[0]-guard_cfg.radius_m)
                max_speed=max(max_speed,math.hypot(pose.vx,pose.vy))
            records.write(json.dumps(dict(elapsed_s=elapsed,clock_s=clock,phase=mission.phase,reason=mission.reason,
                pose_xy=s.xy,velocity_xy=s.velocity_xy,truth_xy=truth_xy,truth_clock_age_s=clock-last_truth_clock,
                truth_velocity_xy=(float(truth_velocity[1]),float(truth_velocity[0])) if truth_velocity is not None else None,
                snapshot=dict(uwb_xy=s.uwb_xy,altitude_enu_m=s.altitude_enu_m,
                    vertical_speed_m_s=s.vertical_speed_m_s,pose_stamp_ns=s.pose_stamp_ns),
                mission_target_xy=mission.goal,stable_since_s=mission.stable_since,
                phase_before_tick=phase_before_tick,target_before_tick=target_before_tick,
                height_before_tick_m=height_before_tick,
                goal=goal,guard=getattr(mission,'guard_decision',None),mode=mode,armed=armed,landed=land.landed_state,
                estimator_flags=flags,validation=mission.validation,actions=actions),allow_nan=False)+'\n')
            if mission.phase in mission.TERMINAL:break
        status.update(final_phase=mission.phase,final_reason=mission.reason,elapsed_s=elapsed,
            required_parameters={k:params.get(k) for k in sorted(set(('COM_RC_OVERRIDE','EKF2_MAG_TYPE','MIS_TAKEOFF_ALT','MPC_XY_P','EKF2_DELAY_MAX','EKF2_ASP_DELAY','EKF2_TAU_VEL')) | set(tuning))},
            min_wall_surface_clearance_m=min_clearance if math.isfinite(min_clearance) else None,
            max_estimated_horizontal_speed_m_s=max_speed,
            scan_fault_to_zero_command_s=zero_after_fault-fault_start if zero_after_fault and fault_start else None)
        shell('logger stop');time.sleep(.5);shell('shutdown');proc.wait(timeout=8)
    except Exception as exc:
        status['errors'].append(repr(exc));raise
    finally:
        if proc and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
        stop.set()
        if 'thread' in locals():thread.join(timeout=1)
        if connection:connection.close()
        console.close();log.close();records.close();scan_records.close();os.close(master)
        status['ulog_files']=[str(p.relative_to(output)) for p in output.rglob('*.ulg')]
        status['binary_unchanged']=sha(binary)==expected_binary
        (output/'run_status.json').write_text(json.dumps(status,indent=2),encoding='utf-8')
    return status


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=['wall','scan_gap','unknown_front','clear','lidar_gap',
        'lidar_jump','lidar_stale','lidar_duplicate','lidar_bad_frame','lidar_bad_cov','lidar_reset',
        'uwb_gap','uwb_jump','person','partial_occlusion','imu_gap','clock_gap'],default='wall')
    parser.add_argument('--baseline',action='store_true');parser.add_argument('--seed',type=int,default=11)
    parser.add_argument('--synthetic-flow',action='store_true',help='Add truth-derived optical flow and downward range; never physical streams')
    parser.add_argument('--patched-runtime',action='store_true',help='Verify a pinned LiDAR2D runtime manifest')
    parser.add_argument('--lidar2d',action='store_true',help='Enable separate 2D velocity aiding with synthetic observations')
    parser.add_argument('--dwell-control',choices=['position','velocity'],default='position')
    parser.add_argument('--lidar-source',choices=['truth_velocity','scan_matcher','spinning'],default='truth_velocity')
    parser.add_argument('--no-deskew',action='store_true')
    parser.add_argument('--no-spatial-support',action='store_true')
    parser.add_argument('--sensor-capture-time',action='store_true',
        help='Use timestamped HIL samples for 40 Hz UWB and integrated flow; spinning runtime only')
    parser.add_argument('--lidar-velocity-intervals',type=int,choices=[1,2],default=1)
    parser.add_argument('--hover-duration-s',type=float,choices=[2.,10.],default=2.)
    parser.add_argument('--sitl-parameters',type=Path,
        help='Bounded JSON native PX4 tuning profile; only this isolated process receives the values')
    parser.add_argument('--fusion-horizon-ms',type=float,choices=[50.,100.,150.,200.],default=None,
        help='Explicit simulator-only EKF delay buffer ablation; hardware timing is uncalibrated')
    parser.add_argument('--zero-unused-airspeed-delay',action='store_true',
        help='Remove inactive SIH quad airspeed delay from EKF horizon calculation, simulation only')
    parser.add_argument('--velocity-output-tau-s',type=float,choices=[.03,.05,.1,.25],default=None,
        help='Simulation-only output predictor correction response ablation')
    parser.add_argument('--duration',type=float,default=85.);parser.add_argument('--instance',type=int,default=50)
    parser.add_argument('--speed-factor',type=float,default=2.)
    args=parser.parse_args()
    if os.name!='posix' or not 50<=args.instance<=59 or not 1<=args.speed_factor<=4 or not 40<=args.duration<=180:
        parser.error('isolated Linux simulation parameters required')
    if not os.environ.get('OBSTACLE_SITL_PARENT_NETNS'):
        env=dict(os.environ,OBSTACLE_SITL_PARENT_NETNS=os.readlink('/proc/self/ns/net'))
        raise SystemExit(subprocess.run(['unshare','-Unc','--keep-caps',sys.executable,
            str(Path(__file__).resolve()),*sys.argv[1:]],env=env).returncode)
    print(json.dumps(run(args),indent=2),flush=True)


if __name__=='__main__':main()
