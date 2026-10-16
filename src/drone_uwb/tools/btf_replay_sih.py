"""Replay approved B_TF XY into disarmed PX4 SIH in a private network.

Only loopback exists in the child namespace. Source timestamps are translated
once into simulated time; observed arrival gaps remain. The first XY is moved
to simulation origin. This tests transport/fusion, not physical flight truth.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import pty
import select
import signal
import shutil
import socket
import subprocess
import sys
import threading
import time


def run(args):
    parent = os.environ.get('BTF_SIH_PARENT_NETNS')
    if not parent or os.readlink('/proc/self/ns/net') == parent:
        raise RuntimeError('private network namespace required')
    if {name for _,name in socket.if_nameindex()} != {'lo'}:
        raise RuntimeError('only loopback may exist')
    subprocess.run(['ip','link','set','lo','up'],check=True)
    os.environ['MAVLINK20']='1'
    from pymavlink import mavutil
    from drone_uwb.integration.local_odometry import LocalObservationSettings, horizontal_odometry
    rows=[r for line in args.decisions.open(encoding='utf-8')
          if (r:=json.loads(line)).get('replay_publish')
          and args.start_ros_ns <= r['callback_ros_ns'] <= args.end_ros_ns]
    if len(rows)<100:
        raise ValueError('insufficient replay observations')
    rows.sort(key=lambda r:r['callback_ros_ns'])
    args.output.mkdir(parents=True,exist_ok=False)
    rootfs=args.output/'rootfs';rootfs.mkdir()
    binary=args.runtime/'bin/px4'
    manifest=dict(scope=__doc__,input_sha256=hashlib.sha256(args.decisions.read_bytes()).hexdigest(),
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        netns=os.readlink('/proc/self/ns/net'),physical_writes=0,armed=False,
        coordinate_offset=rows[0]['xy_m'],start_ros_ns=args.start_ros_ns,end_ros_ns=args.end_ros_ns)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    # Explicit logger topics distinguish packet receipt from EKF fusion.
    logger=rootfs/'fs/microsd/etc/logging';logger.mkdir(parents=True)
    (logger/'logger_topics.txt').write_text('vehicle_visual_odometry 0\nestimator_aid_src_ev_pos 0\n'
        'estimator_status_flags 0\nvehicle_local_position 20\nvehicle_status 100\n',encoding='utf-8')
    # POSIX builds use ./etc/logging, hardware builds use /fs/microsd/etc/logging.
    shutil.copytree(args.runtime/'etc',rootfs/'etc')
    local_logger=rootfs/'etc/logging';local_logger.mkdir(parents=True,exist_ok=True)
    (local_logger/'logger_topics.txt').write_text((logger/'logger_topics.txt').read_text(),encoding='utf-8')
    master,slave=pty.openpty();stop=threading.Event()
    console=(args.output/'px4.log').open('wb')
    sent=(args.output/'sent.jsonl').open('w',encoding='utf-8')
    def pump():
        while not stop.is_set():
            if select.select([master],[],[],.05)[0]:
                try:console.write(os.read(master,65536));console.flush()
                except OSError:return
    def shell(command):os.write(master,(command+'\n').encode())
    env=dict(os.environ,PX4_SIM_MODEL='sihsim_quadx',PX4_SIMULATOR='sihsim',PX4_SIM_SPEED_FACTOR='2',
        PX4_PARAM_EKF2_GPS_CTRL='0',PX4_PARAM_EKF2_EV_CTRL='1',PX4_PARAM_EKF2_EV_DELAY='0',
        PX4_PARAM_EKF2_EV_QMIN='0',PX4_PARAM_EKF2_EV_NOISE_MD='0',
        PX4_PARAM_EKF2_EV_POS_X='0',PX4_PARAM_EKF2_EV_POS_Y='0',PX4_PARAM_EKF2_EV_POS_Z='0',
        PX4_PARAM_SDLOG_MODE='1',PX4_PARAM_SDLOG_PROFILE='1')
    proc=subprocess.Popen([str(binary),str(rootfs/'etc'),'-i','9','-w',str(rootfs)],
        cwd=args.output,env=env,stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
    os.close(slave);thread=threading.Thread(target=pump,daemon=True);thread.start()
    port=18579
    conn=mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}',source_system=255,source_component=196)
    settings=LocalObservationSettings(map_y_axis_sign=1)
    started=time.monotonic();hb=0;requested=False;clock=first=None;index=0;last_stamp=None
    stats=dict(sent=0,expired=0,timesync_responses=0,errors=[])
    duration=20+(args.end_ros_ns-args.start_ros_ns)/1e9+4
    try:
        while time.monotonic()-started<180:
            if proc.poll() is not None:raise RuntimeError(f'PX4 exited {proc.returncode}')
            if time.monotonic()-hb>1:
                conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,8,0,0,4)
                hb=time.monotonic()
            m=conn.recv_match(blocking=False)
            if m:
                if m.get_type()=='HEARTBEAT':
                    if m.base_mode & 128:raise RuntimeError('unexpected ARM')
                    if not requested:
                        shell(f'mavlink stream -r 100 -s LOCAL_POSITION_NED -u {port}')
                        shell(f'mavlink stream -r 50 -s TIMESYNC -u {port}')
                        shell('ver all');shell('param show EKF2_EV*');requested=True
                if m.get_type()=='TIMESYNC' and m.tc1==0:
                    conn.mav.timesync_send(m.ts1,m.ts1);stats['timesync_responses']+=1
                if m.get_type()=='LOCAL_POSITION_NED':
                    clock=m.time_boot_ms*1000
                    if first is None:first=clock
            if first is None or clock is None:
                time.sleep(.0005);continue
            elapsed=(clock-first)/1e6
            while index<len(rows):
                row=rows[index]
                # Ten milliseconds models processing/transport, with no
                # rewriting of capture time to the moment of sending.
                arrival=first+20_010_000+(row['callback_ros_ns']-args.start_ros_ns)//1000
                if clock<arrival:break
                index+=1
                stamp=(first+20_000_000)*1000+row['stamp_ns']-args.start_ros_ns
                try:
                    f=horizontal_odometry(settings,frame='uwb_map',stamp_ns=stamp,now_ns=clock*1000,
                        last_stamp_ns=last_stamp,x=row['xy_m'][0]-rows[0]['xy_m'][0],
                        y=row['xy_m'][1]-rows[0]['xy_m'][1],covariance=[[.09,0],[0,.09]])
                except ValueError:
                    stats['expired']+=1;continue
                e=mavutil.mavlink
                conn.mav.odometry_send(f['time_usec'],e.MAV_FRAME_LOCAL_NED,e.MAV_FRAME_BODY_FRD,
                    f['x'],f['y'],f['z'],f['q'],f['vx'],f['vy'],f['vz'],f['rollspeed'],
                    f['pitchspeed'],f['yawspeed'],f['pose_covariance'],f['velocity_covariance'],
                    reset_counter=0,estimator_type=e.MAV_ESTIMATOR_TYPE_VISION,quality=0)
                sent.write(json.dumps(dict(seq=row.get('seq'),capture_us=f['time_usec'],send_us=clock,
                    x=f['x'],y=f['y']))+'\n')
                last_stamp=stamp;stats['sent']+=1
            if elapsed>duration:break
            if m is None:time.sleep(.0005)
        else:raise TimeoutError('simulation clock did not reach replay end')
        shell('listener estimator_status_flags -n 1')
        shell('listener estimator_aid_src_ev_pos -n 1')
        shell('logger stop');time.sleep(1);shell('shutdown');proc.wait(timeout=8)
    except Exception as exc:
        stats['errors'].append(repr(exc));raise
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        stop.set();thread.join(1);conn.close();sent.close();console.close();os.close(master)
        stats['ulog_files']=[str(p.relative_to(args.output)) for p in args.output.rglob('*.ulg')]
        (args.output/'result.json').write_text(json.dumps(stats,indent=2),encoding='utf-8')
    print(json.dumps(stats),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime',type=lambda s:Path(s).resolve(),required=True)
    p.add_argument('--decisions',type=Path,required=True)
    p.add_argument('--output',type=lambda s:Path(s).resolve(),required=True)
    p.add_argument('--start-ros-ns',type=int,required=True)
    p.add_argument('--end-ros-ns',type=int,required=True)
    args=p.parse_args()
    if os.name!='posix':p.error('Linux SIH host required')
    if not os.environ.get('BTF_SIH_PARENT_NETNS'):
        env=dict(os.environ,BTF_SIH_PARENT_NETNS=os.readlink('/proc/self/ns/net'))
        raise SystemExit(subprocess.run(['unshare','-Unc','--keep-caps',sys.executable,
            str(Path(__file__).resolve()),*sys.argv[1:]],env=env).returncode)
    run(args)
