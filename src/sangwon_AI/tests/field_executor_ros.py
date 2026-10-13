"""HTTP START -> actual Python observer -> actual C++ AI/executor -> mock FC.

Only localhost ROS domain 99 is allowed. No physical MAVROS connection exists.
"""
from dataclasses import asdict
from http.server import ThreadingHTTPServer
import json
import hashlib
import asyncio
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from urllib.request import Request, urlopen

assert os.environ.get('ROS_DOMAIN_ID')=='99' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, RCIn, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandInt, CommandLong
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, PointCloud, ChannelFloat32
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue
from builtin_interfaces.msg import Time
from drone_mission.contracts import Settings
from drone_mission.local_web import LocalPlatform
from lidar_relative_contracts import CONFIG,sdk_state

BINARY=sys.argv[1]
DIRECTORY=Path(sys.argv[2]) if len(sys.argv)>2 else Path(tempfile.mkdtemp(prefix='cpp-executor-'))
DIRECTORY.mkdir(parents=True,exist_ok=True)

class FC:
    def __init__(self,node,platform,cfg,scenario):
        self.n=node;self.p=platform;self.cfg=cfg;self.scenario=scenario
        self.mode='POSCTL';self.armed=False;self.z=.1;self.target=None;self.calls=[];self.rows=[]
        self.xy=[2.,2.];self.moving=scenario in ('mission_x','mission_y','mission_xy')
        self.real_link=scenario=='platform_link' or self.moving
        self.setpoint_rows=[]
        self.status={};self.gap_start=None;self.last_uwb_ns=0;self.last=time.monotonic();self.stop=threading.Event()
        self.setpoint_times=[];self.source_times=[]
        self.pubs={key:node.create_publisher(cls,topic,qos_profile_sensor_data) for key,cls,topic in (
            ('state',State,'/mavros/state'),('landed',ExtendedState,'/mavros/extended_state'),
            ('odom',Odometry,'/mavros/local_position/odom'),('est',EstimatorStatus,'/mavros/estimator_status'),
            ('rc',RCIn,'/mavros/rc/in'),('battery',BatteryState,'/mavros/battery'),
            ('uwb',PoseWithCovarianceStamped,'/uwb/btf_pose'),('lidar',PointCloud,'/point_cloud'))}
        self.bridge=node.create_publisher(String,cfg.bridge_topic,10)
        self.assignment=node.create_publisher(String,'/mission/assignment',10)
        self.services=[node.create_service(cls,topic,callback) for cls,topic,callback in (
            (SetMode,'/mavros/set_mode',self.mode_call),(CommandBool,'/mavros/cmd/arming',self.arm_call),
            (CommandTOL,'/mavros/cmd/land',self.land_call),(CommandTOL,'/mavros/cmd/takeoff',self.nop),
            (CommandInt,'/mavros/cmd/command_int',self.nop),(CommandLong,'/mavros/cmd/command',self.nop),
            (GetParameters,'/mavros/param/get_parameters',self.params))]
        self.subs=[node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',self.setpoint,qos_profile_sensor_data),
                   node.create_subscription(String,'/flight_state',self.telemetry,qos_profile_sensor_data)]
        self.echo=node.create_publisher(PositionTarget,'/mavros/setpoint_raw/target_local',qos_profile_sensor_data)
        self.cloud=PointCloud();self.cloud.header.frame_id='laser_frame'
        sc=sdk_state(0.)['lidar_scan']
        self.cloud.points=[Point32(x=r*math.cos(a),y=r*math.sin(a),z=0.) for r,a in zip(sc['ranges'],sc['point_angles_rad'])]
        self.cloud.channels=[ChannelFloat32(name='stamps',values=sc['point_times_s'])]
        self.empty_cloud=PointCloud();self.empty_cloud.header.frame_id='laser_frame'
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
        self.lidar_thread=threading.Thread(target=self.lidar_loop,daemon=True);self.lidar_thread.start()
    def nop(self,req,res):res.success=True;res.result=0;return res
    def mode_call(self,req,res):self.calls.append(req.custom_mode);self.mode=req.custom_mode;res.mode_sent=True;return res
    def arm_call(self,req,res):self.calls.append('ARM');self.armed=req.value;res.success=True;res.result=0;return res
    def land_call(self,req,res):
        self.calls.append('LAND')
        if self.scenario=='land_mode_delay':self.land_requested_at=time.monotonic()
        else:self.mode='AUTO.LAND'
        res.success=True;res.result=0;return res
    def params(self,req,res):
        values=dict(MIS_TAKEOFF_ALT=1.7,COM_TAKEOFF_ACT=0,EKF2_MAG_TYPE=0,COM_RC_OVERRIDE=3,
                    COM_RC_IN_MODE=0,RC_MAP_FLTMODE=5,RC_MAP_ARM_SW=6,RC_MAP_KILL_SW=7,MPC_Z_VEL_MAX_UP=.5,
                    EKF2_EV_DELAY=0.,EKF2_EV_CTRL=1,EKF2_OF_CTRL=1,EKF2_RNG_CTRL=1,COM_OF_LOSS_T=.7)
        res.values=[ParameterValue(type=3,double_value=values[k]) if type(values[k]) is float else ParameterValue(type=2,integer_value=values[k]) for k in req.names];return res
    def setpoint(self,m):
        assert m.coordinate_frame==1 and m.type_mask==2552
        if not self.moving or self.z < 1.65:
            assert abs(m.position.x-2.)<1e-6 and abs(m.position.y-2.)<1e-6,'takeoff/home target moved'
        else:
            assert 1.99<=m.position.x<=3.01 and 1.99<=m.position.y<=3.01,'route escaped bounds'
        self.setpoint_rows.append([time.monotonic(),m.position.x,m.position.y,m.position.z])
        self.target=m;self.setpoint_times.append(time.monotonic())
        if self.scenario=='target_silence' and self.armed:return
        m.header.stamp=self.n.get_clock().now().to_msg();self.echo.publish(m)
    def telemetry(self,m):
        value=json.loads(m.data);self.status=value
        if not self.real_link:
            with self.p.lock:self.p.telemetry=dict(value,flight_state=value['state']);self.p.telemetry_received_s=time.monotonic()
        self.rows.append(dict(elapsed_s=time.monotonic()-self.started,state=value['state'],height_m=self.z,
            xy=list(self.xy),uwb_age_s=(self.n.get_clock().now().nanoseconds-self.last_uwb_ns)/1e9,
            bridge_observation_age_s=value.get('bridge_observation_age_s'),executor_backend=value['executor_backend'],cpp_decision=value['cpp_decision'],sender_gap_s=value['max_sender_gap_s'],
            observation_status=value.get('observation_status'),
            decision_age_s=time.monotonic()-value['cpp_decision']['sample_time_s'] if 'sample_time_s' in value['cpp_decision'] else None))
    def loop(self):
        self.started=time.monotonic()
        while not self.stop.is_set():
            t=time.monotonic();self.source_times.append(t);dt=min(.1,t-self.last);self.last=t;prev=self.z;previous_xy=list(self.xy)
            if self.scenario=='land_mode_delay' and hasattr(self,'land_requested_at') and t-self.land_requested_at>=1.:
                if self.mode=='OFFBOARD':
                    assert self.setpoint_times[-1]>t-.15,'hold stopped before PX4 landing mode'
                    self.land_mode_delay_verified=True;self.mode='AUTO.LAND'
            if self.mode=='AUTO.LAND':
                self.z=max(.1,self.z-.5*dt)
                if self.z<=.100001:self.armed=False
            elif self.armed and self.target:self.z+=max(-.5*dt,min(.5*dt,self.target.position.z-self.z))
            if self.moving and self.armed and self.target and self.mode=='OFFBOARD':
                dx,dy=self.target.position.x-self.xy[0],self.target.position.y-self.xy[1]
                length=math.hypot(dx,dy);scale=min(1.,.25*dt/length) if length else 0.
                self.xy=[self.xy[0]+dx*scale,self.xy[1]+dy*scale]
            stamp=self.n.get_clock().now().to_msg();ns=int(stamp.sec)*1_000_000_000+stamp.nanosec
            state=State(connected=True,armed=self.armed,mode=self.mode)
            landed=ExtendedState(landed_state=1 if self.z<=.100001 else 2)
            odom=Odometry();odom.header.frame_id='map';odom.child_frame_id='map'
            odom.pose.pose.position.x,odom.pose.pose.position.y=self.xy
            odom.pose.pose.position.z=self.z;odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.z=(self.z-prev)/dt
            odom.twist.twist.linear.x=(self.xy[0]-previous_xy[0])/dt;odom.twist.twist.linear.y=(self.xy[1]-previous_xy[1])/dt
            est=EstimatorStatus(attitude_status_flag=True,velocity_horiz_status_flag=True,velocity_vert_status_flag=True,
                pos_horiz_rel_status_flag=True,pos_vert_abs_status_flag=True,pred_pos_horiz_rel_status_flag=True)
            if self.scenario=='fc_estimator_loss' and self.gap_start is not None:est.pos_horiz_rel_status_flag=False
            rc=RCIn(channels=[1500]*8,rssi=255);battery=BatteryState(present=True,voltage=24.,percentage=.9)
            for key,m in [('state',state),('landed',landed),('odom',odom),('est',est),('rc',rc),('battery',battery)]:
                if key=='state' and self.scenario=='state_1hz':
                    if t-getattr(self,'last_state_publish',-1e9)<1.:continue
                    self.last_state_publish=t
                if key=='odom' and self.gap_start is not None and (self.scenario=='fc_pose_loss' or self.scenario=='pose_gap' and t-self.gap_start<2. or self.scenario=='takeoff_pose_gap' and t-self.gap_start<2.5):continue
                if key=='est' and self.scenario=='fc_estimator_silence' and self.gap_start is not None:continue
                m.header.stamp=stamp;self.pubs[key].publish(m)
            gap=self.gap_start is not None and (self.scenario=='lidar_loss' or t-self.gap_start<(2. if self.scenario=='uwb_gap' else .6))
            if not gap:
                uwb=PoseWithCovarianceStamped();uwb.header.stamp=Time(sec=(ns-130_000_000)//1_000_000_000,nanosec=(ns-130_000_000)%1_000_000_000) if self.scenario in ('bridge_delay','report_delay') else stamp;uwb.header.frame_id='uwb_map';uwb.pose.pose.position.x,uwb.pose.pose.position.y=self.xy;self.pubs['uwb'].publish(uwb);self.last_uwb_ns=int(uwb.header.stamp.sec)*1_000_000_000+uwb.header.stamp.nanosec
            c=dict(input_source='btf_xy',tag_id='6',layout_confirmed=True,source_frame='uwb_map',ground_only=False,
                map_y_axis_sign=1,enu_yaw_deg=0.,enu_offset_x_m=0.,enu_offset_y_m=0.,expected_ev_delay_ms=0.,
                antenna_body_frd_x_m=0.,antenna_body_frd_y_m=0.,antenna_body_frd_z_m=0.)
            report=dict(settings=c,gate='ready',published=100,
                last_observation_stamp_ns=ns-1_200_000_000 if self.scenario=='report_delay' and self.armed else ns-480_000_000 if self.scenario=='bridge_delay' else self.last_uwb_ns)
            if not (self.scenario=='report_silence' and self.armed):
                self.bridge.publish(String(data='{malformed' if self.scenario=='report_malformed' and self.armed else json.dumps(report)))
            if not self.real_link:self.assignment.publish(String(data=json.dumps(self.p.assignment)))
            self.stop.wait(.02)
    def lidar_loop(self):
        # The real LiDAR driver is independent of the FC telemetry producer.
        # Reuse static room geometry rather than allocate 720 ROS points in
        # the FC loop; a stalled synthetic FC would invalidate this test.
        while not self.stop.is_set():
            if self.scenario.startswith('lidar_'):
                first=self.n.get_clock().now().nanoseconds-90_000_000
                cloud=self.empty_cloud if self.scenario=='lidar_loss' and self.gap_start is not None else self.cloud
                cloud.header.stamp=Time(sec=first//1_000_000_000,nanosec=first%1_000_000_000)
                self.pubs['lidar'].publish(cloud)
            self.stop.wait(.1)
    def close(self):self.stop.set();self.thread.join(3);self.lidar_thread.join(3)

def run(scenario):
    root=DIRECTORY/scenario;root.mkdir()
    cfg=Settings(command_frame='local_xy',mission_ai='cpp_bt',execution_profile='anchor_trial',full_mission=True,
        execute=True,layout_confirmed=True,sensor_mount_confirmed=True,takeoff_settings_confirmed=True,
        speed_m_s=.25,max_leg_m=1.,expected_mis_takeoff_alt_m=1.7,drone_id='6',
        bridge_observation_timeout_s=.7 if scenario=='bridge_delay' else .35,
        lidar_gap=dict(CONFIG,input_source='sdk_pointcloud') if scenario.startswith('lidar_') else {})
    mission=root/'mission.json';mission.write_text(json.dumps(asdict(cfg)),encoding='utf-8')
    native=root/'native.json';native.write_text(json.dumps(dict(mission=asdict(cfg),output_enabled=True,test_mode=True,record_file=str(root/'native-events.jsonl'))),encoding='utf-8')
    p=LocalPlatform('6',cfg.layout_id,settings=cfg,mission_file=root/'missions.sqlite3');p.site['ceiling_height_m']=3.
    server=ThreadingHTTPServer(('127.0.0.1',0),p.http_handler());threading.Thread(target=server.serve_forever,daemon=True).start()
    node=rclpy.create_node('synthetic_fc');fc=FC(node,p,cfg,scenario)
    executor=SingleThreadedExecutor();executor.add_node(node);spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    log=(root/'process.log').open('wb')
    native_proc=subprocess.Popen([BINARY,str(native)],stdout=log,stderr=log)
    observer=subprocess.Popen([sys.executable,'-c','from drone_mission.cpp_observer import main; main()',
        '--ros-args','-p','config_file:='+str(mission),'-p','execute:=true','-p','test_mode:=true',
        '-p','record_directory:='+str(root/'observer'),'-p','ledger_file:='+str(root/'ledger.json')],stdout=log,stderr=log)
    paused=False;pause_end=None;injected=False;explicit_cancel=False;link=None;ws_context={};ws_thread=None
    try:
        if fc.real_link:
            from websockets.asyncio.server import serve
            ready=threading.Event()
            async def ws_run():
                ws_context['loop']=asyncio.get_running_loop();ws_context['stop']=asyncio.Event()
                async with serve(p.websocket,'127.0.0.1',0) as ws:
                    ws_context['port']=ws.sockets[0].getsockname()[1];ready.set()
                    await ws_context['stop'].wait()
            ws_thread=threading.Thread(target=lambda:asyncio.run(ws_run()),daemon=True);ws_thread.start();assert ready.wait(3)
            env=dict(os.environ,DRONESTOCK_SERVER_URL=f'http://127.0.0.1:{server.server_port}',
                DRONESTOCK_WS_URL=f'ws://127.0.0.1:{ws_context["port"]}/ws/drones/6/',
                DRONESTOCK_DRONE_ID='6',DRONESTOCK_MISSION_FORWARDING='true',DRONESTOCK_POSE_SOURCE='btf_xy',
                DRONESTOCK_DEVICE_AUTH_ID='',DRONESTOCK_DEVICE_AUTH_SECRET='',DRONESTOCK_STATE_DIR=str(root/'platform'))
            link=subprocess.Popen([sys.executable,'-m','drone_platform_link.runtime'],env=env,stdout=log,stderr=log)
        end=time.monotonic()+12
        while not p.can_start() and time.monotonic()<end:
            assert native_proc.poll() is None and observer.poll() is None,(scenario,(root/'process.log').read_text())
            time.sleep(.05)
        assert p.can_start(),(scenario,fc.status,(root/'process.log').read_text())
        url=f'http://127.0.0.1:{server.server_port}/local/command'
        trial_case=scenario.removeprefix('mission_') if fc.moving else 'hover'
        req=Request(url,data=json.dumps(dict(action='start',trial_case=trial_case,client_request_id='native-'+scenario)).encode(),headers={'Content-Type':'application/json'},method='POST')
        with urlopen(req,timeout=2) as response:assignment=json.load(response)
        start=time.monotonic();end=start+(100 if fc.moving else 35)
        while time.monotonic()<end:
            assert native_proc.poll() is None and observer.poll() is None,(scenario,(root/'process.log').read_text())
            phase=fc.status.get('state')
            if phase=='TAKING_OFF' and scenario=='takeoff_pose_gap' and not injected:fc.gap_start=time.monotonic();injected=True
            if phase=='DWELL' and not injected:
                injected=True
                if scenario=='python_pause':os.kill(observer.pid,signal.SIGSTOP);paused=True;pause_end=time.monotonic()+5.
                elif scenario.startswith('lidar_'):fc.gap_start=time.monotonic()
                elif scenario in ('uwb_gap','pose_gap','fc_estimator_loss','fc_estimator_silence','fc_pose_loss'):fc.gap_start=time.monotonic()
                elif scenario=='pilot':fc.mode='ALTCTL'
            if scenario in ('lidar_loss','fc_pose_loss') and fc.gap_start is not None and time.monotonic()-fc.gap_start>2. and not explicit_cancel:
                assert 'LAND' not in fc.calls and fc.status['completed_waypoints']==0
                cancel=Request(url,data=json.dumps(dict(action='land',client_request_id='explicit-lidar-cancel')).encode(),headers={'Content-Type':'application/json'},method='POST')
                with urlopen(cancel,timeout=2):pass
                explicit_cancel=True
            if paused and time.monotonic()>=pause_end:os.kill(observer.pid,signal.SIGCONT);paused=False
            if phase in ('END','FAILED','UNCONFIRMED','PILOT_OVERRIDE'):break
            time.sleep(.025)
        expected='PILOT_OVERRIDE' if scenario=='pilot' else 'FAILED' if scenario in ('lidar_loss','fc_estimator_loss','fc_pose_loss') else 'END'
        assert fc.status.get('state')==expected,(scenario,fc.status,fc.calls,(root/'process.log').read_text())
        assert fc.calls==(['OFFBOARD','ARM'] if scenario=='pilot' else ['OFFBOARD','ARM','LAND']),fc.calls
        if expected=='END':assert fc.status['mission_complete'] and fc.status['landing_verified']
        if fc.real_link:
            deadline=time.monotonic()+3
            while p.telemetry.get('flight_state')!='END' and time.monotonic()<deadline:time.sleep(.025)
            assert p.telemetry.get('flight_state')=='END' and p.telemetry.get('mission_complete') is True,p.telemetry
            assert p.telemetry['mission_db_id']==assignment['mission_db_id']
        if fc.moving:
            assert math.dist(fc.xy,[2.,2.])<.08,'END away from original launch XY'
            for axis in ('x','y'):
                index=0 if axis=='x' else 1
                extent=max(r['xy'][index] for r in fc.rows)-2.
                assert extent>.9 if axis in trial_case else abs(extent)<.01,(axis,extent)
            assert fc.status['landing_verified'] and not fc.armed
        if scenario=='land_mode_delay':assert getattr(fc,'land_mode_delay_verified',False)
        if scenario=='lidar_gap':assert any(r['cpp_decision'].get('gap_supported') for r in fc.rows),'LiDAR bypass not exercised'
        if scenario=='bridge_delay':
            assert any(r.get('bridge_observation_age_s') is not None and .48 <= r['bridge_observation_age_s'] <= .7 for r in fc.rows),'delivery tolerance not exercised'
        if scenario in ('report_delay','report_silence','report_malformed','fc_estimator_silence','target_silence'):
            assert any(r.get('observation_status',{}).get('warnings') for r in fc.rows if r['state']=='DWELL'),'report warning not exercised'
        if scenario in ('uwb_gap','pose_gap','takeoff_pose_gap'):
            assert any(r['cpp_decision'].get('observation_status',{}).get('progress_held') for r in fc.rows if r['state'] in ('DWELL','TAKING_OFF')),'real observation hold not exercised'
            assert all(r['cpp_decision'].get('completed_waypoints',0)==0 for r in fc.rows if r['cpp_decision'].get('observation_status',{}).get('progress_held'))
        recorded=[json.loads(x) for x in (root/'native-events.jsonl').read_text(encoding='utf-8').splitlines()]
        samples=[x for x in recorded if x.get('type')=='flight_sample']
        assert len(samples)>20,'continuous native evidence missing'
        assert all('input_snapshot' in x and 'sample_monotonic_ns' in x and 'sample_ros_ns' in x for x in samples)
        assert any(x['input_snapshot']['source_stamp_ns'][6]>0 for x in samples),'original UWB source stamp missing'
        assert any(x.get('type')=='command_ack' for x in recorded),'native ACK evidence missing'
        if expected=='END':assert any(x.get('type')=='transition' and x.get('state')=='END' for x in recorded)
        # This measures received actual C++ setpoints. It does not certify an OS.
        gaps=[b-a for a,b in zip(fc.setpoint_times,fc.setpoint_times[1:])]
        assert max(gaps)<.2,(scenario,max(gaps))
        out=dict(scenario=scenario,physical_flight=False,actual_cpp_executor=True,commands=fc.calls,
            binary_sha256=hashlib.sha256(Path(BINARY).read_bytes()).hexdigest(),
            state=fc.status,trace=fc.rows,setpoints=fc.setpoint_rows,trial_case=trial_case,
            final_xy_m=fc.xy,real_platform_link=fc.real_link,max_received_setpoint_gap_s=max(gaps),
            max_mock_fc_source_gap_s=max(b-a for a,b in zip(fc.source_times,fc.source_times[1:])),source='HTTP/ROS/C++/mock-FC')
        (root/'evidence.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
        print(scenario,expected,'max_setpoint_gap_ms=',round(max(gaps)*1000,3),flush=True)
    finally:
        if fc.rows:
            (root/'trace.json').write_text(json.dumps(dict(status=fc.status,commands=fc.calls,trace=fc.rows,
                max_mock_fc_source_gap_s=max(b-a for a,b in zip(fc.source_times,fc.source_times[1:]))),indent=2),encoding='utf-8')
        if paused:os.kill(observer.pid,signal.SIGCONT)
        children=[observer,native_proc]+([link] if link else [])
        for child in children:
            if child.poll() is None:child.send_signal(signal.SIGINT)
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        fc.close();executor.shutdown();spin.join(3);node.destroy_node();server.shutdown();server.server_close();p.store.close();log.close()
        if ws_thread:
            ws_context['loop'].call_soon_threadsafe(ws_context['stop'].set);ws_thread.join(3)

if __name__=='__main__':
    rclpy.init()
    try:
        cases=sys.argv[3].split(',') if len(sys.argv)>3 else ('nominal','python_pause','bridge_delay','report_delay','report_silence','report_malformed','uwb_gap','pose_gap','takeoff_pose_gap','target_silence','state_1hz','fc_estimator_silence','fc_estimator_loss','fc_pose_loss','pilot','platform_link','mission_x','mission_y','mission_xy','land_mode_delay')
        for scenario in cases:run(scenario)
    finally:rclpy.shutdown()
