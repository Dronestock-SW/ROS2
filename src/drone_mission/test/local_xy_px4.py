"""Isolated PX4 SIH check with synthetic UWB XY, no geographic origin assignment.

Only connects to loopback SITL, never ROS domain 2 or a serial device.
Run in WSL with the pinned PX4 SITL build and pymavlink installed.
"""
import argparse
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
from datetime import datetime, timezone

root=Path(__file__).resolve().parents[2]
for package in ('drone_mission','drone_uwb','drone_demo'):
    sys.path.insert(0,str(root/package))
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
from drone_mission.contracts import Settings
from drone_mission.local_xy import LocalXYMission
from drone_mission.session import Snapshot
from drone_mission.field_presets import field_route


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--case',choices=['hover','x','y','xy'],default='xy')
    a=ap.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    px=Path('/home/asw/PX4-Autopilot/build/px4_sitl_default')
    master,slave=pty.openpty();stop=threading.Event()
    env=dict(os.environ,PX4_SIM_MODEL='sihsim_quadx',PX4_SIMULATOR='sihsim',
        PX4_PARAM_EKF2_GPS_CTRL='0',PX4_PARAM_EKF2_EV_CTRL='1',PX4_PARAM_EKF2_MAG_TYPE='6',
        PX4_PARAM_EKF2_RNG_CTRL='2',PX4_PARAM_EKF2_HGT_REF='2',PX4_PARAM_SIH_DISTSNSR_OVR='10001')
    proc=subprocess.Popen([str(px/'bin/px4'),str(px/'etc'),'-w',str(out/'rootfs')],
        stdin=slave,stdout=slave,stderr=slave,env=env,start_new_session=True);os.close(slave)
    def pump():
        with (out/'px4.log').open('wb') as f:
            while not stop.is_set():
                if select.select([master],[],[],.1)[0]:
                    try:f.write(os.read(master,65536));f.flush()
                    except OSError:return
    thread=threading.Thread(target=pump);thread.start()
    def shell(s):os.write(master,(s+'\n').encode());time.sleep(.03)
    c=None;events=[];result={};state={};origin=None;last_phase=None
    try:
        time.sleep(4)
        for k,v in dict(EKF2_GPS_CTRL=0,EKF2_EV_CTRL=1,EKF2_MAG_TYPE=6,COM_RC_IN_MODE=1,
            COM_RC_OVERRIDE=3,MIS_TAKEOFF_ALT=1.7,COM_TAKEOFF_ACT=0,COM_ARM_WO_GPS=1,
            EKF2_EV_DELAY=0,EKF2_EV_NOISE_MD=1,EKF2_EVP_NOISE=.05,EKF2_RNG_CTRL=2,EKF2_HGT_REF=2,
            SIH_DISTSNSR_OVR=10001,
            MPC_Z_VEL_MAX_UP=.25,MPC_Z_VEL_MAX_DN=.25,MPC_TKO_SPEED=.25,MPC_LAND_SPEED=.25).items():shell(f'param set {k} {v}')
        c=mavutil.mavlink_connection('udpout:127.0.0.1:18570',source_system=255,source_component=190)
        c.mav.heartbeat_send(6,8,0,0,4)
        assert c.wait_heartbeat(timeout=10)
        for name,rate in [('HIL_STATE_QUATERNION',40),('LOCAL_POSITION_NED',30),('ATTITUDE',30),
                          ('EXTENDED_SYS_STATE',10),('ESTIMATOR_STATUS',10),('POSITION_TARGET_LOCAL_NED',20)]:
            shell(f'mavlink stream -r {rate} -s {name} -u 18570')
        cfg=Settings(command_frame='local_xy',execution_profile='anchor_trial',full_mission=True,
            execute=True,layout_confirmed=True,sensor_mount_confirmed=True,takeoff_settings_confirmed=True,
            expected_ekf2_mag_type=6,expected_mis_takeoff_alt_m=1.7,speed_m_s=.25,
            enu_offset_x_m=-2,enu_offset_y_m=-2,takeoff_timeout_s=40,leg_timeout_s=30)
        mission=LocalXYMission(cfg);start=time.monotonic();last_hb=last_vision=last_tick=0.;request=None
        while time.monotonic()-start<150:
            now=time.monotonic();wall=time.time();ns=time.time_ns()
            for _ in range(100):
                m=c.recv_match(blocking=False)
                if m is None:break
                if m.get_srcSystem()!=1 or m.get_srcComponent()!=1:continue
                typ=m.get_type();state[typ]=(m,now)
                if typ=='STATUSTEXT':events.append(dict(type=typ,text=m.text))
                if typ=='HIL_STATE_QUATERNION' and origin is None:origin=(m.lat/1e7,m.lon/1e7,m.alt*.001)
                if typ=='COMMAND_ACK' and mission.pending:
                    kind=mission.pending['kind'];cmd={'offboard_mode':176,'position_mode':176,'arm':400,'land':21}.get(kind)
                    if m.command==cmd:mission.command_result(mission.pending['token'],m.result==0,now)
                if typ=='POSITION_TARGET_LOCAL_NED':
                    matched=mission.applied_local_target((m.y+2,m.x+2),-m.z,ns)
                    events.append(dict(type='target_feedback',elapsed_s=now-start,matched=matched,
                        xy=(m.y+2,m.x+2),z=-m.z,goal=mission.local_setpoint()))
            if now-last_hb>=.5:c.mav.heartbeat_send(6,8,0,0,4);last_hb=now
            if 'HIL_STATE_QUATERNION' in state and now-last_vision>=.025:
                h=state['HIL_STATE_QUATERNION'][0];north=math.radians(h.lat/1e7-origin[0])*6371000
                east=math.radians(h.lon/1e7-origin[1])*6371000*math.cos(math.radians(origin[0]))
                cov=[0.]*21;cov[0]=cov[6]=.0025;cov[11]=cov[15]=cov[18]=cov[20]=1e6
                c.mav.vision_position_estimate_send(int(wall*1e6),north,east,0.,0.,0.,0.,cov,0)
                # Synthetic downward range has a declared 0.30 m ground mount,
                # matching the existing mission SIH fixture, never hardware.
                q=h.attitude_quaternion;projection=1-2*(q[1]*q[1]+q[2]*q[2])
                tof=max(.3,(h.alt*.001-origin[2]+.3)/max(.8,projection))
                c.mav.distance_sensor_send(int(now*1000)&0xffffffff,10,1000,round(tof*100),
                    0,0,25,1,.04,.04,[1.,0.,0.,0.],100)
                last_vision=now
            if now-last_tick<.05:time.sleep(.002);continue
            last_tick=now
            if not all(x in state for x in ['HEARTBEAT','LOCAL_POSITION_NED','ATTITUDE','EXTENDED_SYS_STATE','ESTIMATOR_STATUS']):continue
            hb=state['HEARTBEAT'][0];p=state['LOCAL_POSITION_NED'][0];at=state['ATTITUDE'][0]
            est=state['ESTIMATOR_STATUS'][0];xy=(p.y+2,p.x+2);mode=mavutil.mode_string_v10(hb)
            mode={'LOITER':'AUTO.LOITER','LAND':'AUTO.LAND'}.get(mode,mode)
            if (hb.custom_mode >> 16) & 255 == 6:mode='OFFBOARD'
            def age(name):return now-state[name][1]
            sample=Snapshot(connected=True,armed=bool(hb.base_mode&128),mode=mode,
                landed=state['EXTENDED_SYS_STATE'][0].landed_state,state_age_s=age('HEARTBEAT'),landed_age_s=age('EXTENDED_SYS_STATE'),
                estimator_age_s=age('ESTIMATOR_STATUS'),estimator_valid=bool(est.flags&256 and est.flags&8 and est.flags&2),
                vertical_estimator_valid=bool(est.flags&4 and est.flags&32),pose_age_s=age('LOCAL_POSITION_NED'),pose_stamp_ns=ns,
                xy=xy,velocity_xy=(p.vy,p.vx),vertical_speed_m_s=-p.vz,altitude_enu_m=-p.z,yaw_deg=90-math.degrees(at.yaw),
                uwb_xy=(east+2,north+2),uwb_age_s=now-last_vision,uwb_stamp_ns=ns,
                bridge_age_s=0.,bridge_observation_age_s=0.,bridge_ready=True,bridge_published=1,alignment_matches=True,
                local_setpoint_ready=True,command_services_ready=True,land_service_ready=True,takeoff_alt_m=1.7,takeoff_param_age_s=0.,
                mag_type=6,takeoff_action=0,rc_required=False,rc_valid=True,rc_mapping_valid=True,rc_age_s=0.,rc_override=3,
                battery=.9,battery_age_s=0.)
            if request is None and now-start>8:
                stamp=datetime.fromtimestamp(wall,timezone.utc).isoformat()
                request=dict(ok=True,contract_version='1.0',drone_id='5',status='ACTIVE',control_action='start',
                    control_request_id='isolated-sih-local',control_requested_at=stamp,mission_db_id=1,mission_code='SIH',route_revision='local',
                    anchor_layout_id=cfg.layout_id,coordinate_frame='UWB_ANCHOR_LOCAL',origin='A1',x_axis='A1_TO_A2',y_axis='A1_TO_A3',
                    z_axis='UP_FROM_FLOOR',unit='meter',route_tasks=field_route(a.case,xy,cfg))
            if request:mission.submit(request,now,wall)
            actions=mission.tick(sample,now,wall,ns)
            for action in actions:
                k=action['kind'];events.append(dict(type='command',kind=k,elapsed_s=now-start))
                if k in ('offboard_mode','position_mode'):c.mav.set_mode_send(1,1,(6 if k=='offboard_mode' else 3)<<16)
                elif k=='arm':c.mav.command_long_send(1,1,400,0,1,0,0,0,0,0,0)
                elif k=='land':c.mav.command_long_send(1,1,21,0,0,0,0,float('nan'),float('nan'),float('nan'),float('nan'))
                else:raise AssertionError(k)
            goal=mission.local_setpoint()
            if goal:c.mav.set_position_target_local_ned_send(int((now-start)*1000),1,1,1,2552,
                goal['xy'][1]-2,goal['xy'][0]-2,-goal['altitude_enu_m'],0,0,0,0,0,0,math.radians(90-goal['yaw_deg']),0)
            if mission.phase!=last_phase:
                row=dict(type='transition',phase=mission.phase,reason=mission.reason,xy=xy,z=-p.z,mode=mode,
                    base_mode=hb.base_mode,custom_mode=hb.custom_mode,flags=est.flags,elapsed_s=now-start)
                events.append(row);print(json.dumps(row),flush=True);last_phase=mission.phase
            if mission.phase in mission.TERMINAL:
                result=mission.status();break
        shell('listener vehicle_local_position -n 1')
        time.sleep(.2)
        (out/'events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
        (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        assert result.get('mission_complete'),result or 'timeout'
        import re
        log=re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]','',(out/'px4.log').read_text(errors='replace'))
        assert 'ref_lat: nan' in log
        print('PASS: actual PX4 SIH completed local mission without geographic origin',flush=True)
    finally:
        if c:c.close()
        stop.set();os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        thread.join(timeout=2);os.close(master)


if __name__=='__main__':main()
