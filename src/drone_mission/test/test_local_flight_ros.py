"""Local ROS/HTTP/WS contract trials. The FC is synthetic, with no real EKF."""

import asyncio
from dataclasses import asdict
from http.server import ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.request import Request, urlopen

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, GlobalPositionTarget, State, TimesyncStatus
from mavros_msgs.srv import CommandBool, CommandInt, CommandLong, CommandTOL
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String
from websockets.asyncio.server import serve

from drone_demo.node import require_demo_environment
from drone_mission.contracts import Settings
from drone_mission.local_web import LocalPlatform
from drone_uwb.integration.sitl.sitl_target_contract import PX4GlobalReference, global_to_ned


class FakeFC(Node):
    """Protocol fixture; its simple motion is not a flight simulator."""

    def __init__(self):
        require_demo_environment(os.environ)
        super().__init__('local_fc_fixture')
        if self.context.get_domain_id() != 99:
            raise ValueError('fixture_requires_domain_99')
        self.xy = np.array([2.,2.])
        self.z = .3
        self.target = self.xy.copy()
        self.target_global = None
        self.armed = False
        self.mode = 'POSCTL'
        self.sequence = 0
        self.drop_uwb = False
        self.calls, self.vision = [], []
        self.base_mono = time.monotonic_ns()-100_000_000
        self.previous = time.monotonic()
        self.origin = PX4GlobalReference(47.,8.,1)
        self.pubs = {name:self.create_publisher(cls,topic,qos_profile_sensor_data) for name,cls,topic in (
            ('state',State,'/mavros/state'),('landed',ExtendedState,'/mavros/extended_state'),
            ('estimator',EstimatorStatus,'/mavros/estimator_status'),
            ('pose',Odometry,'/mavros/local_position/odom'),('raw',String,'/uwb/received'),
            ('imu',Imu,'/mavros/imu/data'),('tof',Range,'/mavros/downward_0'),
            ('sync',TimesyncStatus,'/mavros/timesync_status'),
            ('target',GlobalPositionTarget,'/mavros/setpoint_raw/target_global'))}
        self.origin_pub=self.create_publisher(GeoPointStamped,'/mavros/global_position/gp_origin',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov',
                                 self.vision.append,10)
        self.create_service(GetParameters,'/mavros/param/get_parameters',self.parameters)
        self.create_service(CommandBool,'/mavros/cmd/arming',self.arm)
        self.create_service(CommandTOL,'/mavros/cmd/takeoff',self.takeoff)
        self.create_service(CommandTOL,'/mavros/cmd/land',self.land)
        self.create_service(CommandInt,'/mavros/cmd/command_int',self.reposition)
        self.create_service(CommandLong,'/mavros/cmd/command',self.stream_request)
        self.create_timer(.025,self.tick)

    def parameters(self,request,response):
        values={'EKF2_EV_CTRL':1,'EKF2_EV_DELAY':0.,'EKF2_EV_NOISE_MD':0,
                'EKF2_EV_POS_X':0.,'EKF2_EV_POS_Y':0.,'EKF2_EV_POS_Z':-.12,'MIS_TAKEOFF_ALT':.6}
        response.values=[ParameterValue(type=2,integer_value=values[k]) if type(values.get(k)) is int
                         else ParameterValue(type=3,double_value=values[k]) if k in values
                         else ParameterValue() for k in request.names]
        return response

    def arm(self,request,response):
        self.calls.append('arm' if request.value else 'disarm')
        self.armed=request.value
        response.success,response.result=True,0
        return response

    def takeoff(self,request,response):
        assert all(math.isnan(v) for v in (request.yaw,request.latitude,request.longitude,request.altitude))
        self.calls.append('takeoff')
        self.mode='AUTO.TAKEOFF'
        response.success,response.result=True,0
        return response

    def land(self,request,response):
        assert math.isnan(request.altitude)
        self.calls.append('land')
        self.mode='AUTO.LAND'
        self.target=self.xy.copy()
        response.success,response.result=True,0
        return response

    def reposition(self,request,response):
        assert request.command==192 and request.frame==0 and request.param2==0. and math.isnan(request.z)
        north,east=global_to_ned(request.x/1e7,request.y/1e7,self.origin)
        self.calls.append('reposition')
        self.target=np.array([east,north])
        self.target_global=(request.x/1e7,request.y/1e7)
        response.success=True
        return response

    def stream_request(self,request,response):
        response.success,response.result=True,0
        return response

    def tick(self):
        now=time.monotonic()
        dt=min(.1,now-self.previous)
        self.previous=now
        velocity=np.zeros(2)
        if self.mode=='AUTO.TAKEOFF':
            self.z=min(.9,self.z+.3*dt)
            if self.z>=.9:
                self.mode='AUTO.LOITER'
        elif self.mode=='AUTO.LOITER' and self.armed:
            delta=self.target-self.xy
            distance=float(np.linalg.norm(delta))
            if distance>1e-6:
                velocity=delta/distance*min(.3,distance/dt)
                self.xy+=velocity*dt
        elif self.mode=='AUTO.LAND':
            self.z=max(.3,self.z-.3*dt)
            if self.z<=.3:
                self.armed=False
        stamp=self.get_clock().now().to_msg()
        mono,ros=time.monotonic_ns(),self.get_clock().now().nanoseconds
        state=State(connected=True,armed=self.armed,mode=self.mode)
        state.header.stamp=stamp
        self.pubs['state'].publish(state)
        landed=ExtendedState(landed_state=2 if self.z>.31 else 1)
        landed.header.stamp=stamp
        self.pubs['landed'].publish(landed)
        est=EstimatorStatus(attitude_status_flag=True,velocity_horiz_status_flag=True,
                            pos_horiz_rel_status_flag=True)
        est.header.stamp=stamp
        self.pubs['estimator'].publish(est)
        pose=Odometry(child_frame_id='map')
        pose.header.stamp,pose.header.frame_id=stamp,'map'
        pose.pose.pose.position.x,pose.pose.pose.position.y=map(float,self.xy)
        pose.pose.pose.position.z=self.z
        pose.pose.pose.orientation.w=1.
        pose.twist.twist.linear.x,pose.twist.twist.linear.y=map(float,velocity)
        self.pubs['pose'].publish(pose)
        origin=GeoPointStamped()
        origin.header.stamp=stamp
        origin.position.latitude,origin.position.longitude=47.,8.
        self.origin_pub.publish(origin)
        sync=TimesyncStatus(remote_timestamp_ns=mono-self.base_mono,
            estimated_offset_ns=ros-(mono-self.base_mono),round_trip_time_ms=1.)
        self.pubs['sync'].publish(sync)
        sensor_stamp=rclpy.time.Time(nanoseconds=ros-20_000_000).to_msg()
        imu=Imu()
        imu.header.stamp,imu.header.frame_id=sensor_stamp,'base_link'
        imu.orientation.w=1.
        self.pubs['imu'].publish(imu)
        tof=Range(range=self.z,min_range=.02,max_range=10.)
        tof.header.stamp= sensor_stamp
        self.pubs['tof'].publish(tof)
        if self.target_global:
            target=GlobalPositionTarget(coordinate_frame=0,latitude=self.target_global[0],
                                        longitude=self.target_global[1])
            target.header.stamp=stamp
            self.pubs['target'].publish(target)
        end=(mono-self.base_mono)//1000-2000
        if self.sequence % 20==0 and not self.drop_uwb:
            status=dict(schema=1,type='uwb_raw_status',tag_id='5',
                clock_domain='esp32_monotonic_boot_us',anchor_order=['A1','A2','A3','A4'],
                anchor_count=4,uwb_ready=True,temporal_filter_applied=False)
            self.raw(status,mono,ros)
        self.sequence+=1
        if not self.drop_uwb:
            samples=[end-13000+i*4000 for i in range(4)]
            anchors=np.array([[0,0,2.2],[6.3,0,2.2],[0,4.6,2.2],[6.3,4.6,2.2]])
            ranges=[float(np.linalg.norm(np.array([*self.xy,self.z+.12])-a)) for a in anchors]
            self.raw(dict(schema=1,type='uwb_raw_cycle',tag_id='5',seq=self.sequence,
                cycle_start_us=end-16000,cycle_end_us=end,valid_mask=15,raw_slant_m=ranges,
                sample_time_us=samples,failure=['ok']*4),mono,ros)

    def raw(self,message,mono,ros):
        event=dict(message=message,host_received_monotonic_ns=mono,host_received_ros_ns=ros)
        self.pubs['raw'].publish(String(data=json.dumps(event)))


async def wait_for(predicate, timeout=15):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return
        await asyncio.sleep(.05)
    raise AssertionError('local flight condition timed out')


@pytest.mark.parametrize('scenario',['nominal','uwb_gap','web_gap','restart'])
def test_local_web_raw_btf_px4_mission_telemetry(tmp_path,scenario):
    require_demo_environment(os.environ)
    rclpy.init()
    fc=FakeFC()
    executor=SingleThreadedExecutor()
    executor.add_node(fc)
    spin=threading.Thread(target=executor.spin,daemon=True)
    spin.start()
    platform=LocalPlatform()
    http=ThreadingHTTPServer(('127.0.0.1',0),platform.http_handler())
    thread=threading.Thread(target=http.serve_forever,daemon=True)
    thread.start()
    processes=[]
    logs=[]

    def launch(name,command,env=None):
        out=(tmp_path/(name+'.log')).open('w',encoding='utf-8')
        logs.append(out)
        processes.append(subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,
                                         env=env,start_new_session=True))
        return processes[-1]

    def executable(package,name):
        return str(Path(get_package_prefix(package)) / 'lib' / package / name)

    async def scenario_run():
        async with serve(platform.websocket,'127.0.0.1',0) as ws:
            ws_port=ws.sockets[0].getsockname()[1]
            cfg=asdict(Settings(layout_confirmed=True,alignment_confirmed=True,fusion_confirmed=True,
                timing_confirmed=True,sensor_mount_confirmed=True,takeoff_settings_confirmed=True,
                expected_ev_pos_z_m=-.12))
            config=tmp_path/'fixture-settings.json'
            config.write_text(json.dumps(cfg),encoding='utf-8')
            launch('btf',[executable('drone_uwb','uwb_btf_node'),'--ros-args','-p','require_height_for_pose:=true'])
            bridge=[executable('drone_uwb','uwb_px4_bridge'),'--ros-args']
            for setting in ('enabled:=true','alignment_confirmed:=true','timing_confirmed:=true',
                            'sensor_mount_confirmed:=true','layout_confirmed:=true','ground_only:=false',
                            'antenna_body_frd_z_m:=-0.12','input_source:=btf_xy','test_mode:=true'):
                bridge+=['-p',setting]
            launch('bridge',bridge)
            mission_command=[executable('drone_mission','flight_mission'),'--ros-args',
                '-p','config_file:='+str(config),'-p','execute:=true','-p','test_mode:=true',
                '-p','ledger_file:='+str(tmp_path/'ledger.json'),
                '-p','record_directory:='+str(tmp_path/'mission')]
            mission_process=launch('mission',mission_command)
            env=dict(os.environ,DRONESTOCK_SERVER_URL=f'http://127.0.0.1:{http.server_port}',
                DRONESTOCK_WS_URL=f'ws://127.0.0.1:{ws_port}/ws/drones/5/',
                DRONESTOCK_MISSION_FORWARDING='true',DRONESTOCK_UWB_TOPIC='/uwb/btf_pose',
                DRONESTOCK_STATE_DIR=str(tmp_path/'platform'))
            launch('platform',['python3','-m','drone_platform_link.runtime'],env)
            await wait_for(lambda:len(fc.vision)>10 and platform.telemetry.get('current_z_m') is not None)
            assert fc.calls==[]
            assert all(m.pose.pose.position.z==0. and m.pose.covariance[14]==1e6 for m in fc.vision)
            request=Request(f'http://127.0.0.1:{http.server_port}/local/command',
                data=json.dumps(dict(action='start',route_tasks=[dict(id='P1',type='waypoint',x=2.3,y=2.)])).encode(),
                headers={'Content-Type':'application/json'},method='POST')
            await asyncio.to_thread(lambda:urlopen(request,timeout=2).read())
            await wait_for(lambda:'reposition' in fc.calls)
            if scenario=='uwb_gap':
                fc.drop_uwb=True
            elif scenario=='web_gap':
                await asyncio.to_thread(http.shutdown)
                http.server_close()
            elif scenario=='restart':
                os.killpg(mission_process.pid,signal.SIGTERM)
                await asyncio.to_thread(mission_process.wait,5)
                restart_command=mission_command[:-1]+['record_directory:='+str(tmp_path/'mission-restart')]
                launch('mission-restart',restart_command)
            await wait_for(lambda:platform.telemetry.get('flight_state') in ('LANDED','FAILED'),20)
            telemetry=dict(platform.telemetry)
            assert fc.calls==['arm','takeoff','reposition','land']
            assert not fc.armed
            assert platform.acks[0]['action']=='start'
            if scenario=='nominal':
                assert telemetry['mission_complete'] is True
                assert telemetry['landing_verified'] is True
                assert telemetry['target_applied'] is True
                assert math.dist(fc.xy,[2.3,2.])<.02
                assert telemetry['current_z_source']=='tof_imu_uwb_antenna'
            else:
                assert telemetry['mission_complete'] is False
                assert telemetry['flight_state']=='FAILED'
                assert telemetry['landing_verified'] is True
                if scenario=='web_gap':
                    assert telemetry['flight_reason']=='web_assignment_stale'
                if scenario=='restart':
                    assert telemetry['flight_reason']=='airborne_without_active_session'
                fc.drop_uwb=False
                await asyncio.sleep(.5)
                assert fc.calls.count('arm')==1
            (tmp_path/'summary.json').write_text(json.dumps(dict(
                scenario=scenario,transport='ROS2+HTTP+WebSocket',source='synthetic_protocol_fixture',
                real_ekf=False,physical_flight=False,fc_calls=fc.calls,
                vision_messages=len(fc.vision),telemetry=telemetry),indent=2),encoding='utf-8')

    try:
        asyncio.run(scenario_run())
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait(timeout=2)
        for out in logs:
            out.close()
        http.shutdown()
        http.server_close()
        executor.shutdown(timeout_sec=2.)
        spin.join(2)
        fc.destroy_node()
        rclpy.shutdown()
    assert all(process.returncode == 0 for process in processes), [
        process.returncode for process in processes]
