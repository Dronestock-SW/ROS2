"""Native TAKEOFF parameter regression in loopback-only PX4 SIH, no ROS/FC link."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import select
import shutil
import signal
import subprocess
import threading
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


def run(build, output, auto_up, max_down, no_global=False, local_position=False):
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(build/'etc', output/'etc', symlinks=False)
    # Replace every default MAVLink endpoint before boot: only this loopback pair.
    (output/'etc/init.d-posix/px4-rc.mavlink').write_text(
        'param set MAV_SYS_ID 81\n'
        'mavlink start -x -u 18655 -o 18656 -t 127.0.0.1 -r 4000000 -m custom\n'
        'mavlink stream -r 2 -s HEARTBEAT -u 18655\n'
        'mavlink stream -r 40 -s HIL_STATE_QUATERNION -u 18655\n'
        'mavlink stream -r 30 -s LOCAL_POSITION_NED -u 18655\n'
        'mavlink stream -r 10 -s EXTENDED_SYS_STATE -u 18655\n')
    env=dict(os.environ, PX4_SIM_MODEL='sihsim_quadx', PX4_SIMULATOR='sihsim',
        PX4_PARAM_MAV_SYS_ID='81', PX4_PARAM_COM_RC_IN_MODE='1',
        PX4_PARAM_COM_DISARM_PRFLT='10', PX4_PARAM_MPC_Z_VEL_MAX_UP='0.5',
        PX4_PARAM_MPC_Z_V_AUTO_UP=str(auto_up), PX4_PARAM_MPC_Z_VEL_MAX_DN=str(max_down),
        PX4_PARAM_MPC_Z_V_AUTO_DN='0.25', PX4_PARAM_MPC_TKO_SPEED='0.5',
        PX4_PARAM_MPC_LAND_SPEED='0.25', PX4_PARAM_MIS_TAKEOFF_ALT='1.7',
        PX4_PARAM_COM_TAKEOFF_ACT='0', PX4_PARAM_SDLOG_MODE='-1')
    if no_global:
        env.update(PX4_PARAM_EKF2_GPS_CTRL='0', PX4_PARAM_EKF2_EV_CTRL='1',
                   PX4_PARAM_COM_ARM_WO_GPS='1', PX4_PARAM_EKF2_EV_DELAY='0')
    master,slave=pty.openpty()
    proc=subprocess.Popen([str(build/'bin/px4'),str(output/'etc'),'-i','8','-w',str(output/'rootfs')],
        env=env,stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
    os.close(slave);stop=threading.Event()
    def pump():
        with (output/'px4.log').open('wb') as f:
            while not stop.is_set():
                if select.select([master],[],[],.1)[0]:
                    try:f.write(os.read(master,65536));f.flush()
                    except OSError:return
    thread=threading.Thread(target=pump);thread.start()
    c=mavutil.mavlink_connection('udpin:127.0.0.1:18656',source_system=254,source_component=190)
    start=time.monotonic();last_hb=0.;stage=0;state={};rows=[];ground=None;command_at=None;origin=None
    target=None;last_target=0.;stable_since=None;position_requested=False
    try:
        while time.monotonic()-start<55:
            now=time.monotonic()
            if now-last_hb>.5:
                c.mav.heartbeat_send(6,8,0,0,4);last_hb=now
            if local_position:
                # A centered simulated joystick is required for POSCTL. This
                # endpoint is loopback-only and belongs to simulated sysid 81.
                c.mav.manual_control_send(81,0,0,500,0,0)
                if target and not position_requested and now-last_target>.04:
                    h=state.get('HEARTBEAT');z=ground-1.7 if h and h.base_mode&128 else ground
                    c.mav.set_position_target_local_ned_send(int((now-start)*1000),81,1,1,3576,
                        target[0],target[1],z,0,0,0,0,0,0,0,0)
                    last_target=now
            m=c.recv_match(blocking=True,timeout=.05)
            if m is not None and m.get_srcSystem()==81:
                state[m.get_type()]=m
                if no_global and m.get_type()=='HIL_STATE_QUATERNION':
                    if origin is None:origin=(m.lat/1e7,m.lon/1e7)
                    north=math.radians(m.lat/1e7-origin[0])*6371000+3.
                    east=math.radians(m.lon/1e7-origin[1])*6371000*math.cos(math.radians(origin[0]))+3.
                    cov=[0.]*21;cov[0]=cov[6]=.0025;cov[11]=cov[15]=cov[18]=cov[20]=1e6
                    c.mav.vision_position_estimate_send(int(time.time()*1e6),north,east,0.,0.,0.,0.,cov,0)
                if m.get_type()=='LOCAL_POSITION_NED' and 'HEARTBEAT' in state:
                    h=state['HEARTBEAT'];land=state.get('EXTENDED_SYS_STATE')
                    rows.append(dict(t=now-start,z=m.z,xy=[m.x,m.y],armed=bool(h.base_mode&128),
                        mode=h.custom_mode,landed=land.landed_state if land else None))
            if now-start>20 and stage==0 and 'LOCAL_POSITION_NED' in state:
                ground=state['LOCAL_POSITION_NED'].z
                if local_position:
                    target=(state['LOCAL_POSITION_NED'].x,state['LOCAL_POSITION_NED'].y)
                    stage=-1
                else:os.write(master,b'commander mode auto:takeoff\n');stage=1
            if local_position and stage==-1 and now-start>22:
                os.write(master,b'commander mode offboard\n');stage=1
            if stage==1 and 'HEARTBEAT' in state and state['HEARTBEAT'].custom_mode==((6<<16) if local_position else ((4<<16)|(2<<24))):
                os.write(master,b'commander arm\n');stage=2;command_at=now
            if local_position and stage==2 and 'LOCAL_POSITION_NED' in state:
                p=state['LOCAL_POSITION_NED']
                stable=abs(p.z-ground+1.7)<.15 and abs(p.vz)<.1 and math.hypot(p.vx,p.vy)<.1
                if not stable:stable_since=None
                elif stable_since is None:stable_since=now
                elif now-stable_since>3 and not position_requested:
                    os.write(master,b'commander mode posctl\n');position_requested=True
            if command_at and now-command_at>25:break
        for query in ['listener vehicle_constraints','listener takeoff_status','listener position_setpoint_triplet','listener vehicle_local_position']:
            os.write(master,(query+'\n').encode());time.sleep(.2)
        result=dict(auto_up=auto_up,max_down=max_down,stage=stage,samples=len(rows),
            height=max([ground-r['z'] for r in rows if r['armed']] or [0]) if ground is not None else None,
            airborne=any(r['landed']==2 for r in rows),
            xy_travel=math.dist(rows[0]['xy'],rows[-1]['xy']) if rows else None,
            hold=any(r['armed'] and r['landed']==2 and r['mode']==((3<<16) if local_position else ((4<<16)|(3<<24))) for r in rows))
        (output/'result.json').write_text(json.dumps(result,indent=2))
        (output/'samples.json').write_text(json.dumps(rows))
        print(json.dumps(result),flush=True)
    finally:
        c.close();os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10)
        stop.set();thread.join();os.close(master)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--auto-up',type=float,required=True)
    p.add_argument('--max-down',type=float,required=True);p.add_argument('--no-global',action='store_true');a=p.parse_args()
    run(a.build.resolve(),a.output.resolve(),a.auto_up,a.max_down,a.no_global,os.environ.get('TEST_LOCAL_POSITION')=='1')
