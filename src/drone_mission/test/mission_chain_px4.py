"""Actual PX4/MAVROS mission with synthetic RAW UWB, ToF, flow and scan worker.

Endpoints are fixed to loopback. ROS domain 99 and localhost are mandatory.
No serial endpoint, physical FC parameter, ARM or motor output is accessed.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import hashlib
import math
import os
from pathlib import Path
import pty
import select
import signal
import socket
import subprocess
import threading
import time
import traceback
from urllib.request import Request, urlopen

import numpy as np
from virtual_uwb import VirtualUwb


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4-root',type=Path,required=True)
    p.add_argument('--px4-build',choices=['px4_sitl_missionchain','px4_sitl_hover'],default='px4_sitl_missionchain')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--imu-noise-scale',type=float,default=.1)
    p.add_argument('--takeoff-alt',type=float,default=1.7)
    p.add_argument('--mag-type',type=int,choices=[0,1,6],default=0)
    p.add_argument('--transport',choices=['web','ros'],default='web')
    p.add_argument('--trial-case',choices=['hover','x','y','xy'],default=None)
    p.add_argument('--scenario',choices=['nominal','spike','nlos','short_gap','long_gap','tof_short_gap','tof_long_gap','coherent_step','phase_delay','scan_missing','manual','rc_stick','scan_partial','scanner_missing_partial','scan_failed_partial'],default='nominal')
    args=p.parse_args()
    assert 0 <= args.imu_noise_scale <= 1
    assert .6 <= args.takeoff_alt <= 2.
    assert os.environ.get('ROS_DOMAIN_ID')=='99' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    os.environ['MAVLINK20']='1'
    # Reserve separate CPU sets for this isolated sensor/FC test only.
    # Do not alter the affinity of any existing service or physical FC process.
    cpus=sorted(os.sched_getaffinity(0))
    pins=dict(px4=cpus[:2],mavros=cpus[2:3],btf=cpus[3:4],
              bridge=cpus[4:5],mission=cpus[4:5],web=cpus[5:6],platform=cpus[5:6]) if len(cpus)>=6 else {}
    def pinned(name, command):
        return ['taskset','-c',','.join(map(str,pins[name])),*command] if name in pins else command
    from pymavlink import mavutil
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from drone_mission.contracts import Settings,LAYOUT
    from drone_mission.field_presets import field_route
    from drone_uwb.processing.gazebo_geometry import rotation_world_body

    root=Path(__file__).resolve().parents[2]
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    pxroot=args.px4_root.resolve();binary=pxroot/'build'/args.px4_build/'bin/px4'
    assert binary.is_file()
    master,slave=pty.openpty();processes=[];logs=[];stopped=threading.Event()
    node=None;connection=None;threads=[];errors=[]
    def spawn(name,command,**kw):
        log=(output/(name+'.log')).open('wb');logs.append(log)
        proc=subprocess.Popen(pinned(name,command),stdout=log,stderr=log,start_new_session=True,**kw)
        processes.append(proc);return proc
    def shell(command):
        os.write(master,(command+'\n').encode());time.sleep(.04)
    def pump():
        log=(output/'px4.log').open('wb');logs.append(log)
        while not stopped.is_set():
            if select.select([master],[],[],.1)[0]:
                try:log.write(os.read(master,65536));log.flush()
                except OSError:return
    status={};request={};shared={'truth':None,'scenario_start':None,'started':False}
    stats={'uwb_cycles':0,'flow_messages':0,'tof_messages':0,'max_sensor_gap_s':0.}
    def event(name):
        f=(output/(name+'.jsonl')).open('w',encoding='utf-8');logs.append(f);return f
    truthlog,eventlog=event('truth'),event('sensor_inputs')
    try:
        proc=subprocess.Popen(pinned('px4',[str(binary),str(pxroot/'build'/args.px4_build/'etc'),'-w',str(output/'rootfs')]),
            stdin=slave,stdout=slave,stderr=slave,
            env=dict(os.environ,PX4_SIM_MODEL='sihsim_quadx',PX4_SIMULATOR='sihsim',
                     PX4_PARAM_SIH_DISTSNSR_OVR='10001',
                     PX4_PARAM_EKF2_MAG_TYPE=str(args.mag_type),
                     PX4_MISSIONCHAIN_IMU_NOISE_SCALE=str(args.imu_noise_scale)),start_new_session=True)
        processes.append(proc);os.close(slave)
        threads.append(threading.Thread(target=pump));threads[-1].start();time.sleep(5)
        values=dict(SIH_DISTSNSR_OVR=10001,MIS_TAKEOFF_ALT=args.takeoff_alt,EKF2_MAG_TYPE=args.mag_type,COM_TAKEOFF_ACT=0,COM_RC_IN_MODE=1,
            COM_RC_OVERRIDE=3,EKF2_GPS_CTRL=7,EKF2_EV_CTRL=1,EKF2_EV_NOISE_MD=0,EKF2_EV_DELAY=0,
            EKF2_EVP_NOISE=.01,EKF2_OF_N_MIN=.01,
            EKF2_EV_POS_X=0,EKF2_EV_POS_Y=0,EKF2_EV_POS_Z=0,
            EKF2_OF_CTRL=1,EKF2_RNG_CTRL=1,EKF2_RNG_GND_CLEAR=.3,
            SENS_FLOW_ROT=0,EKF2_OF_DELAY=0,SENS_FLOW_MINHGT=.1,SENS_FLOW_MAXHGT=10.)
        for k,v in values.items():shell(f'param set {k} {v}')
        connection=mavutil.mavlink_connection('udpout:127.0.0.1:18570',source_system=255,source_component=190)
        connection.mav.heartbeat_send(6,8,0,0,4)
        assert connection.wait_heartbeat(timeout=10) is not None
        shell('mavlink stream -r 100 -s HIL_STATE_QUATERNION -u 18570')
        rclpy.init();node=rclpy.create_node('virtual_mission_sensors')
        rawpub=node.create_publisher(String,'/uwb/received',qos_profile_sensor_data)
        assignmentpub=node.create_publisher(String,'/mission/assignment',10)
        markerpub=node.create_publisher(String,'/mission/marker_observation',10)
        resultpub=node.create_publisher(String,'/mission/scan_result',10)
        def state_received(msg):
            status.clear();status.update(json.loads(msg.data))
        def scan_received(msg):
            value=json.loads(msg.data)
            if (value.get('window_id'),value.get('action')) != (request.get('window_id'),request.get('action')):
                shared['scan_started']=time.monotonic()
            request.clear();request.update(value)
        node.create_subscription(String,'/flight_state',state_received,10)
        node.create_subscription(String,'/mission/scan_request',scan_received,10)
        poselog=event('fc_pose')
        def pose_received(msg):
            stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            q=msg.pose.pose.orientation;v=msg.twist.twist.linear;x=msg.pose.pose.position
            poselog.write(json.dumps(dict(received_ns=time.time_ns(),stamp_ns=stamp,
                frame_id=msg.header.frame_id,child_frame_id=msg.child_frame_id,
                xyz_m=[x.x,x.y,x.z],velocity=[v.x,v.y,v.z],quaternion_wxyz=[q.w,q.x,q.y,q.z]))+'\n')
        node.create_subscription(Odometry,'/mavros/local_position/odom',pose_received,qos_profile_sensor_data)
        btf=json.loads((root/'drone_uwb/config/runtime/uwb_btf_tag_b_z2p2.json').read_text())
        btf['bias_m']=[.02,-.015,.01,.025]
        # This declared virtual model has 3 mm range noise and known biases.
        # Hardware retains its uncalibrated 0.30 m observation uncertainty.
        btf['xy_stddev_m']=.01
        btf['B']['window_s']=.2
        btf['provenance']['simulation']='all confirmation flags below describe virtual geometry only'
        btf['height'].update(mount_confirmed=True,flat_floor_confirmed=True,tof_to_tag_body_flu_m=[0,0,0])
        (output/'btf.json').write_text(json.dumps(btf),encoding='utf-8')
        settings=Settings(full_mission=True,heading_control_confirmed=True,drone_id='6',layout_id=LAYOUT,
            layout_confirmed=True,alignment_confirmed=True,fusion_confirmed=True,
            timing_confirmed=True,sensor_mount_confirmed=True,takeoff_settings_confirmed=True,
            expected_mis_takeoff_alt_m=args.takeoff_alt,expected_ekf2_mag_type=args.mag_type,enu_offset_x_m=-2.,enu_offset_y_m=-2.,
            leg_timeout_s=30.,scan_timeout_s=30.,pose_timeout_s=.3)
        (output/'mission.json').write_text(json.dumps(asdict(settings)),encoding='utf-8')
        volume=dict(polygon_xy_m=[[0,0],[6.3,0],[6.3,4.6],[0,4.6]],z_min_m=0,z_max_m=3.)
        native_map=dict(frame='UWB_ANCHOR_LOCAL',unit='meter',layout_id=settings.layout_id,
            survey_status='VIRTUAL',launch_body_floor_height_m=.3,native_body_floor_height_m=args.takeoff_alt,
            altitude_policy='px4_MIS_TAKEOFF_ALT',native_height_m=settings.expected_mis_takeoff_alt_m,
            clearance_xy_m=.5,clearance_z_m=.15,boundary=[volume],altitude_zones=[volume],
            yaw_zones=[volume],forbidden=[])
        raw_map=json.dumps(native_map).encode()
        (output/'native-map.json').write_bytes(raw_map)
        mavconfig=root/'drone_uwb/config/runtime/mavros_test_flight.yaml'
        spawn('mavros',['ros2','run','mavros','mavros_node','--ros-args','-r','__ns:=/mavros','--params-file',str(mavconfig),
            '-p','fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14580','-p','gcs_url:=""','-p','tgt_system:=1','-p','tgt_component:=1'])
        spawn('btf',['ros2','run','drone_uwb','uwb_btf_node','--ros-args','-p',f'config_file:={output}/btf.json',
            '-p','require_height_for_pose:=true','-p',f'record_directory:={output}/btf'])
        bridge=dict(enabled=True,test_mode=True,input_source='btf_xy',tag_id='6',ground_only=False,
            layout_confirmed=True,alignment_confirmed=True,timing_confirmed=True,sensor_mount_confirmed=True,
            enu_offset_x_m=-2.,enu_offset_y_m=-2.)
        command=['ros2','run','drone_uwb','uwb_px4_bridge','--ros-args']
        for k,v in bridge.items():
            encoded = str(v).lower() if type(v) is bool else json.dumps(v) if type(v) is str else str(v)
            command+=['-p',f'{k}:={encoded}']
        spawn('bridge',command)
        spawn('mission',['ros2','run','drone_mission','flight_mission','--ros-args','-p',f'config_file:={output}/mission.json',
            '-p','test_mode:=true','-p','execute:=true','-p',f'record_directory:={output}/mission',
            '-p',f'native_plan_binary:={root.parent}/build/sangwon_ai_replay/sangwon_native_plan',
            '-p',f'native_map_file:={output}/native-map.json',
            '-p',f'native_map_sha256:={hashlib.sha256(raw_map).hexdigest()}',
            '-p',f'ledger_file:={output}/requests.json'])
        web_origin = None
        if args.transport == 'web':
            def free_port():
                with socket.socket() as sock:
                    sock.bind(('127.0.0.1',0));return sock.getsockname()[1]
            http_port,ws_port=free_port(),free_port()
            web_origin=f'http://127.0.0.1:{http_port}'
            spawn('web',['ros2','run','drone_mission','local_flight_web','--config',str(output/'mission.json'),
                         '--site-config',str(output/'site.json'),
                         '--http-port',str(http_port),'--ws-port',str(ws_port)])
            spawn('platform',['python3','-m','drone_platform_link.runtime'],env=dict(os.environ,
                DRONESTOCK_SERVER_URL=web_origin,DRONESTOCK_WS_URL=f'ws://127.0.0.1:{ws_port}/ws/drones/6/',
                DRONESTOCK_DRONE_ID='6',DRONESTOCK_POSE_SOURCE='btf_xy',DRONESTOCK_UWB_TOPIC='/uwb/btf_pose',
                DRONESTOCK_MISSION_FORWARDING='true',DRONESTOCK_STATE_DIR=str(output/'platform')))
        if pins:os.sched_setaffinity(0,{cpus[5]})
        origin=None;uwb=VirtualUwb();base=time.monotonic_ns()-100000000
        def sensor_loop():
            nonlocal origin
            last_send=last_hb=0.;last_tick=time.monotonic();last_packet=None
            last_hil_received=-math.inf;last_hil_seq=None
            try:
                while not stopped.is_set():
                    now=time.monotonic();until=now+.006
                    while time.monotonic()<until:
                        packet=connection.recv_match(blocking=False)
                        if packet is None:break
                        # Pinned PX4 v1.17's HIL stream leaves time_usec at zero.
                        # This loopback-only truth generator therefore checks the
                        # MAVLink sequence and receive age, never a fabricated
                        # source timestamp. Production observations keep their
                        # original sensor timestamps and stricter freshness gates.
                        if packet.get_type()=='HIL_STATE_QUATERNION' and packet.get_seq()!=last_hil_seq:
                            last_packet=packet;last_hil_seq=packet.get_seq();last_hil_received=time.monotonic()
                    if now-last_hb>1:
                        connection.mav.heartbeat_send(6,8,0,0,4);last_hb=now
                    stick = (700 if shared.get('stick_at') is not None
                             and 0 <= now-shared['stick_at'] <= .5 else 0)
                    connection.mav.manual_control_send(1,stick,0,500,0,0)
                    if last_packet is None or now-last_hil_received>.1 or now-last_send<.025:
                        stopped.wait(.003);continue
                    dt=min(.06,now-last_send) if last_send else .025
                    stats['max_sensor_gap_s']=max(stats['max_sensor_gap_s'],now-last_tick);last_tick=now;last_send=now
                    packet=last_packet
                    if origin is None:origin=(packet.lat*1e-7,packet.lon*1e-7,packet.alt*.001)
                    north=math.radians(packet.lat*1e-7-origin[0])*6371000
                    east=math.radians(packet.lon*1e-7-origin[1])*6371000*math.cos(math.radians(origin[0]))
                    height=packet.alt*.001-origin[2]
                    ned=np.array([north,east,-height]);vn=np.array([packet.vx*.01,packet.vy*.01,packet.vz*.01])
                    rot=rotation_world_body(packet.attitude_quaternion)
                    body=rot.T@vn
                    projection=float(rot[2,2]);tof=max(.3,(height+.3)/max(.8,projection))
                    phase=status.get('state')
                    if phase=='MOVING' and shared['scenario_start'] is None:shared['scenario_start']=now
                    elapsed=now-shared['scenario_start'] if shared['scenario_start'] else -1
                    tof_drop=(args.scenario in ('tof_short_gap','tof_long_gap') and
                              .3<=elapsed<(.95 if args.scenario=='tof_short_gap' else 5.5))
                    if not tof_drop:
                        connection.mav.distance_sensor_send(int(now*1000)&0xffffffff,10,1000,round(tof*100),0,0,25,1,.04,.04,[1.,0.,0.,0.],100)
                        stats['tof_messages']+=1
                    connection.mav.optical_flow_rad_send(time.time_ns()//1000,0,round(dt*1e6),
                        (-body[1]/tof+packet.rollspeed)*dt,(body[0]/tof+packet.pitchspeed)*dt,
                        packet.rollspeed*dt,packet.pitchspeed*dt,packet.yawspeed*dt,25,255,0,tof)
                    stats['flow_messages']+=1
                    fault='none';drop=False
                    if args.scenario in ('nlos','coherent_step','phase_delay') and .3<=elapsed<1.1:fault=args.scenario
                    if args.scenario=='spike' and .3<height<.6 and not shared.get('spiked'):
                        fault='spike';shared['spiked']=True
                    if args.scenario in ('short_gap','long_gap') and .3<=elapsed<(.95 if args.scenario=='short_gap' else 5.5):drop=True
                    mono,ros=time.monotonic_ns(),time.time_ns()
                    end_us=(mono-base)//1000-2000
                    msgs=uwb.messages(end_us,[east+2,north+2,height+.3],[vn[1],vn[0],-vn[2]],fault=fault,elapsed=elapsed)
                    if not drop:
                        for msg in msgs:
                            rawpub.publish(String(data=json.dumps(dict(message=msg,host_received_monotonic_ns=mono,host_received_ros_ns=ros))))
                        stats['uwb_cycles']+=1
                    truth=dict(monotonic_s=now,enu_m=[east,north,height],yaw_deg=90-math.degrees(math.atan2(rot[1,0],rot[0,0])),phase=phase)
                    shared['truth']=truth;truthlog.write(json.dumps(truth)+'\n')
                    eventlog.write(json.dumps(dict(monotonic_s=now,fault=fault,dropped=drop,tof_dropped=tof_drop,raw=msgs[1],tof_m=tof,
                        hil_time_us=packet.time_usec,velocity_body_m_s=body.tolist(),dt_s=dt))+'\n')
                    stopped.wait(.003)
            except Exception:
                errors.append(traceback.format_exc())
        threads.append(threading.Thread(target=sensor_loop));threads[-1].start()
        samples=event('observations');previous=None;last_mode=0.;warmup=time.monotonic()+35
        assignment=None;scan_sent=set();gps_disabled=False;manual_sent=False
        deadline=time.monotonic()+180
        while time.monotonic()<deadline:
            assert all(p.poll() is None for p in processes),'component exited'
            assert not errors,errors
            rclpy.spin_once(node,timeout_sec=.003);now=time.monotonic()
            if status:
                samples.write(json.dumps(dict(monotonic_s=now,status=dict(status)))+'\n');samples.flush()
                if previous!=status['state']:
                    previous=status['state'];print(previous,status.get('reason'),flush=True)
                if not shared['started']:
                    if now> warmup-5 and not gps_disabled:
                        shell('param set EKF2_GPS_CTRL 0');gps_disabled=True
                        values['EKF2_GPS_CTRL']=0
                    if status.get('fc_mode')!='POSCTL' and now-last_mode>3:
                        shell('commander mode posctl');last_mode=now
                    if now>warmup:
                        if args.trial_case and assignment is None:
                            checks={r['code']:r['passed'] for r in status.get('preflight',{}).get('checks',[])}
                            if not all(checks.get(k) for k in ('layout_confirmed','alignment_confirmed','transform','pose','estimator')):
                                time.sleep(.02);continue
                        if assignment is None:
                            assignment=dict(ok=True,contract_version='1.0',drone_id='6',status='ACTIVE',control_action='start',
                                control_request_id='virtual-'+args.scenario,control_requested_at=datetime.now(timezone.utc).isoformat(),
                                mission_db_id=1,mission_code='VIRTUAL-CHAIN',route_revision='r1',anchor_layout_id=LAYOUT,
                                coordinate_frame='UWB_ANCHOR_LOCAL',origin='A1',x_axis='A1_TO_A2',y_axis='A1_TO_A3',z_axis='UP_FROM_FLOOR',unit='meter',
                                ceiling_height_m=3.0,
                                route_tasks=[dict(id='P1',type='waypoint',x=2.4,y=2.,yaw_deg=0.,dwell_s=.5),
                                    dict(id='S1',type='scan',x=3.2,y=2.2,staging_xy_m=[2.7,2.2],
                                         path_validation_ref='VIRTUAL-EMPTY-FLOOR-v1',yaw_deg=15.,marker_id=7,label_id='LABEL7',
                                         calibration_ref='virtual-camera',mounting_ref='virtual-mount')])
                            if args.scenario in ('scan_partial','scanner_missing_partial','scan_failed_partial'):
                                assignment['route_tasks'] += [
                                    dict(id='P2',type='waypoint',x=2.7,y=2.5,yaw_deg=15.,dwell_s=.3),
                                    dict(id='S2',type='scan',x=3.2,y=2.5,staging_xy_m=[2.7,2.5],
                                         path_validation_ref='VIRTUAL-EMPTY-FLOOR-v1',yaw_deg=15.,marker_id=8,label_id='LABEL8',
                                         calibration_ref='virtual-camera',mounting_ref='virtual-mount'),
                                    dict(id='P3',type='waypoint',x=2.4,y=2.5,yaw_deg=15.,dwell_s=.3)]
                            if args.trial_case:
                                assignment['route_tasks']=field_route(args.trial_case,status['px4_map_xy_m'],settings)
                                assignment['planned_launch_xy_m']=status['px4_map_xy_m']
                            if web_origin:
                                req=Request(web_origin+'/local/command',data=json.dumps(dict(action='set_ceiling',ceiling_height_m=3.)).encode(),
                                    headers={'Content-Type':'application/json'},method='POST')
                                with urlopen(req,timeout=2) as response:
                                    assert json.load(response)['flight_command_sent'] is False
                                body=dict(action='start',route_tasks=assignment['route_tasks'])
                                if args.trial_case:body['trial_case']=args.trial_case
                                req=Request(web_origin+'/local/command',data=json.dumps(body).encode(),
                                    headers={'Content-Type':'application/json'},method='POST')
                                with urlopen(req,timeout=2) as response:assignment=json.load(response)
                                (output/'web_command.json').write_text(json.dumps(assignment,indent=2)+'\n',encoding='utf-8')
                        if not web_origin:assignmentpub.publish(String(data=json.dumps(assignment)))
                if status['state'] in ('PREPARING','ARMING','TAKING_OFF'):shared['started']=True
                if assignment and not web_origin:assignmentpub.publish(String(data=json.dumps(assignment)))
                if args.scenario=='manual' and status['state']=='MOVING' and not manual_sent:
                    shell('commander mode posctl');manual_sent=True
                if args.scenario=='rc_stick' and status['state']=='MOVING' and not manual_sent:
                    shared['stick_at']=now;manual_sent=True
                missing_marker = args.scenario=='scan_missing' or (args.scenario=='scan_partial' and request.get('task_id')=='S1')
                if request and status['state'] in ('ALIGNING','SCANNING') and not missing_marker:
                    task=request['task'];ns=time.time_ns()
                    marker=dict(execution_id=request['execution_id'],task_id=request['task_id'],window_id=request['window_id'],
                        marker_id=task['marker_id'],calibration_ref=task['calibration_ref'],mounting_ref=task['mounting_ref'],
                        stamp_ns=ns,frame_id='uwb_map',valid=True,xy_m=[task['xy'][0]+.025,task['xy'][1]],yaw_deg=task['yaw_deg']+4.)
                    markerpub.publish(String(data=json.dumps(marker)))
                    missing_scanner = args.scenario=='scanner_missing_partial' and request['task_id']=='S1'
                    if request['action']=='SCAN' and not missing_scanner and now-shared['scan_started']>.3 and request['window_id'] not in scan_sent:
                        outcome='FAILED' if args.scenario=='scan_failed_partial' and request['task_id']=='S1' else 'SUCCEEDED'
                        resultpub.publish(String(data=json.dumps(dict(marker,stamp_ns=time.time_ns(),label_id=task['label_id'],
                            outcome=outcome,stored=True,result_id='SIMULATED-SCAN-'+request['window_id']))))
                        scan_sent.add(request['window_id'])
                if status['state']=='PILOT_OVERRIDE' and args.scenario=='rc_stick':
                    if 'override_at' not in shared:
                        assert status.get('fc_mode') in ('POSCTL','ALTCTL'),status
                        shared['override_at']=now
                    # Virtual-only mode return proves the mission never reclaims
                    # control after the operator releases the sticks.
                    if now-shared['override_at']>1 and not shared.get('hold_returned'):
                        shell('commander mode auto:loiter');shared['hold_returned']=True
                    if now-shared['override_at']<3:continue
                if status['state'] in ('END','FAILED','UNCONFIRMED','PILOT_OVERRIDE'):break
            time.sleep(.02)
        shell('listener estimator_status_flags 1');shell('listener estimator_aid_src_optical_flow 1')
        shell('listener estimator_aid_src_ev_pos 1');shell('listener estimator_aid_src_rng_hgt 1')
        web_status=None
        if web_origin:
            end=time.monotonic()+3
            while time.monotonic()<end:
                with urlopen(web_origin+'/local/status',timeout=2) as response:web_status=json.load(response)
                if web_status['telemetry'].get('flight_state')==status['state']:break
                time.sleep(.1)
            assert web_status['telemetry_fresh'] and web_status['telemetry'].get('flight_state')==status['state']
            (output/'web_status.json').write_text(json.dumps(web_status,indent=2)+'\n',encoding='utf-8')
        summary=dict(actual_px4=True,physical_flight=False,scan_source='EXPLICIT_VIRTUAL_WORKER',
            transport='HTTP+ROS2+MAVROS+WebSocket' if web_origin else 'ROS2+MAVROS',
            px4_build=args.px4_build,
            isolated_cpu_sets=pins,
            virtual_imu_noise_scale=args.imu_noise_scale,
            uwb_source='SEEDED_RAW_DS_TWR_MODEL',scenario=args.scenario,status=dict(status),stats=stats,
            trial_case=args.trial_case,
            simulation_parameters=values,truth=shared['truth'],px4_commit='d6f12ad1c4f70ad3230afd7d86e971421e02fef4')
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        expected='FAILED' if args.scenario in ('long_gap','tof_long_gap') else 'PILOT_OVERRIDE' if args.scenario in ('manual','rc_stick') else 'END'
        assert status.get('state')==expected,summary
        if expected=='END':
            assert status['home_verified'] and status['landing_verified'],summary
            assert status['mission_complete'] is (args.scenario not in ('scan_missing','scan_partial','scanner_missing_partial','scan_failed_partial')),summary
            if args.scenario in ('scan_partial','scanner_missing_partial','scan_failed_partial'):
                assert status['attempted_task_ids']==['P1','S1','P2','S2','P3'] and not status['remaining_task_ids'],summary
                assert status['route_complete'] and status['failed_scan_task_ids']==['S1'],summary
                assert [(r['task_id'],r['outcome']) for r in status['scan_results']]==[('S1','FAILED'),('S2','SUCCEEDED')],summary
                telemetry=web_status['telemetry'] if web_status else status
                assert telemetry['route_complete'] and telemetry['failed_scan_task_ids']==['S1'],summary
            assert np.linalg.norm(np.array(shared['truth']['enu_m'][:2]))<.3,summary
        print('PASS',args.scenario,expected,flush=True)
    finally:
        stopped.set()
        for thread in threads:thread.join(timeout=3)
        for proc in reversed(processes):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGTERM)
        for proc in processes:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        ulogs=list((output/'rootfs/log').glob('*/*.ulg'))
        if ulogs:
            import shutil
            shutil.copy2(max(ulogs,key=lambda f:f.stat().st_mtime),output/'px4.ulg')
        if node:node.destroy_node()
        if 'rclpy' in locals() and rclpy.ok():rclpy.shutdown()
        if connection:connection.close()
        os.close(master)
        for log in logs:log.close()


if __name__=='__main__':main()
