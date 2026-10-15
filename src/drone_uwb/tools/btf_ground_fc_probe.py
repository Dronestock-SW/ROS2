"""Explicit, bounded synthetic ground replay through the existing MAVROS router.

No ARM, mode, setpoint, serial-port access or parameter writes. This changes
the real FC estimator temporarily; motor isolation and exclusive ground use
must be confirmed by the operator. Never use these coordinates for flight.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

QUERIES=('param show EKF2_EV*','param show EKF2_GPS_CTRL',
         'listener vehicle_visual_odometry -n 1',
         'listener estimator_aid_src_ev_pos -n 1',
         'listener estimator_status_flags -n 1')


def run(args):
    if not args.confirm_ground_isolated or not 0 < args.duration_s <= 60:
        raise ValueError('confirmed isolated ground test, at most 60 seconds, required')
    sys.path.insert(0,str(args.ops.resolve()))
    from px4_mavros_readback import MavrosReadback
    from px4_sensor_readback import completed
    from drone_uwb.integration.local_odometry import LocalObservationSettings, horizontal_odometry
    from drone_uwb.processing.gazebo_geometry import rotation_world_body
    from mavros_msgs.msg import ExtendedState, TimesyncStatus
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from pymavlink import mavutil
    rows=[r for l in args.decisions.open(encoding='utf-8') if (r:=json.loads(l)).get('replay_publish')
          and args.start_ros_ns <= r['callback_ros_ns'] < args.start_ros_ns+args.duration_s*1e9]
    if len(rows)<100:raise ValueError('insufficient recorded observations')
    rows.sort(key=lambda r:r['callback_ros_ns'])
    args.output.mkdir(parents=True,exist_ok=False)
    c=MavrosReadback(2,QUERIES)
    cache={}; sent=[]; answers=[]; error=None; graph_at=-float('inf')
    def store(kind,msg):cache[kind]=(msg,time.monotonic())
    for kind,typ,topic in (('landed',ExtendedState,'/mavros/extended_state'),
        ('sync',TimesyncStatus,'/mavros/timesync_status'),('odom',Odometry,'/mavros/local_position/odom')):
        c.node.create_subscription(typ,topic,lambda m,k=kind:store(k,m),qos_profile_sensor_data)
    def guard():
        nonlocal graph_at
        now=time.monotonic()
        if (not c.state or not c.heartbeat or not c.state.connected or c.state.armed
            or c.heartbeat.base_mode & 128 or now-c.state_at>2.5 or now-c.heartbeat_at>2.5
            or 'landed' not in cache or now-cache['landed'][1]>.5 or cache['landed'][0].landed_state!=1):
            raise RuntimeError('ground/disarm evidence lost; synthetic transmission stopped')
        if c.publisher.get_subscription_count()!=1:raise RuntimeError('router owner changed')
        if now-graph_at>=1:
            if any(n in {'uwb_local_odometry_bridge','uwb_px4_bridge','uwb_lidar_velocity_bridge'}
                   for n in c.node.get_node_names()):raise RuntimeError('another EV sender exists')
            graph_at=now
    def issue(q):
        b=(q+'\n').encode();c.serial_control_send(10,6,0,0,len(b),list(b)+[0]*(70-len(b)))
    try:
        if c.wait_heartbeat(8) is None:raise RuntimeError('no FC heartbeat')
        end=time.monotonic()+3
        while time.monotonic()<end:c.recv_match(blocking=True,timeout=.01)
        guard()
        for q in QUERIES[:2]:
            issue(q);response='';deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                m=c.recv_match(blocking=True,timeout=.01)
                if m and m.get_type()=='SERIAL_CONTROL':response+=bytes(m.data[:m.count]).decode(errors='replace')
                if completed(response,q):break
            if not completed(response,q):raise RuntimeError('parameter readback incomplete')
            answers.append(dict(phase='before',command=q,response=response))
        import re
        parameters={}
        for answer in answers:
            parameters.update({k:float(v) for k,v in re.findall(r'(EKF2_\w+)\s*(?:\[[\d,]+\])?\s*:\s*(-?[\d.]+)',answer['response'])})
        for k,v in {'EKF2_EV_CTRL':1,'EKF2_EV_NOISE_MD':0,'EKF2_EV_QMIN':0,'EKF2_GPS_CTRL':0}.items():
            if parameters.get(k)!=v:raise RuntimeError('unexpected parameter '+k)
        if 'odom' not in cache or time.monotonic()-cache['odom'][1]>.2:raise RuntimeError('no fresh initial odometry')
        odom=cache['odom'][0].pose.pose;q=odom.orientation
        lever=[parameters['EKF2_EV_POS_X'],-parameters['EKF2_EV_POS_Y'],-parameters['EKF2_EV_POS_Z']]
        offset=rotation_world_body([q.w,q.x,q.y,q.z])@lever
        origin=[odom.position.x+offset[0],odom.position.y+offset[1]]
        if not all(math.isfinite(v) for v in origin):raise RuntimeError('invalid initial origin')
        manifest=dict(scope=__doc__,synthetic=True,flight_valid=False,parameters=parameters,
            input_sha256=hashlib.sha256(args.decisions.read_bytes()).hexdigest(),
            one_time_origin_enu_m=origin,recorded_origin_xy=rows[0]['xy_m'],
            duration_s=args.duration_s,source_start_ros_ns=args.start_ros_ns)
        (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
        settings=LocalObservationSettings(map_y_axis_sign=1)
        encoder=mavutil.mavlink.MAVLink(None,srcSystem=255,srcComponent=196)
        start_mono=time.monotonic();start_ros=c.node.get_clock().now().nanoseconds
        index=0;last_stamp=None;pending=None;response='';last_query=-10.;query_index=0
        # Twelve seconds after final input also records fusion expiry; never
        # replay a stale coordinate to hide the deliberate or recorded gap.
        while time.monotonic()-start_mono < args.duration_s+12:
            m=c.recv_match(blocking=True,timeout=.001)
            elapsed=time.monotonic()-start_mono
            if m and m.get_type()=='SERIAL_CONTROL' and pending:
                response+=bytes(m.data[:m.count]).decode(errors='replace')
                if completed(response,pending):
                    answers.append(dict(elapsed_s=elapsed,phase='inject' if elapsed<args.duration_s else 'after',command=pending,response=response))
                    pending=None;response=''
            guard()
            if pending and elapsed-last_query>4:raise RuntimeError('diagnostic query timed out')
            if not pending and elapsed-last_query>=2:
                pending=QUERIES[2+query_index%3];query_index+=1;issue(pending);last_query=elapsed
            while index<len(rows) and elapsed >= (rows[index]['callback_ros_ns']-args.start_ros_ns)/1e9:
                row=rows[index];index+=1
                stamp=start_ros+row['stamp_ns']-args.start_ros_ns
                now=c.node.get_clock().now().nanoseconds
                if now-stamp>150_000_000:
                    sent.append(dict(seq=row.get('seq'),expired=True));continue
                sync=cache.get('sync')
                if not sync or time.monotonic()-sync[1]>.5 or sync[0].round_trip_time_ms>20:
                    raise RuntimeError('fresh low-RTT timesync missing')
                f=horizontal_odometry(settings,frame='uwb_map',stamp_ns=stamp,now_ns=now,last_stamp_ns=last_stamp,
                    x=origin[0]+row['xy_m'][0]-rows[0]['xy_m'][0],y=origin[1]+row['xy_m'][1]-rows[0]['xy_m'][1],
                    covariance=[[.09,0],[0,.09]])
                e=mavutil.mavlink
                packet=encoder.odometry_encode(frame_id=e.MAV_FRAME_LOCAL_NED,child_frame_id=e.MAV_FRAME_BODY_FRD,
                    estimator_type=e.MAV_ESTIMATOR_TYPE_VISION,**f)
                packet.pack(encoder);encoder.seq=(encoder.seq+1)%256;c.publisher.publish(c.to_ros(packet))
                sent.append(dict(synthetic=True,seq=row.get('seq'),capture_us=f['time_usec'],send_ros_ns=now,
                    x=f['x'],y=f['y'],fc_estimated_offset_ns=sync[0].estimated_offset_ns))
                last_stamp=stamp
    except BaseException as exc:
        error=repr(exc)
        raise
    finally:
        try:c.serial_control_send(10,0,0,0,0,[0]*70)
        finally:
            c.close()
            (args.output/'sent.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in sent),encoding='utf-8')
            (args.output/'readback.json').write_text(json.dumps(answers,indent=2)+'\n',encoding='utf-8')
            (args.output/'result.json').write_text(json.dumps(dict(error=error,sent=sum(not r.get('expired') for r in sent),
                expired=sum(bool(r.get('expired')) for r in sent),sender_stopped=True,flight_commands=0,parameter_writes=0),indent=2)+'\n',encoding='utf-8')
    print(f'Synthetic ground replay finished: {len(sent)} rows; sender removed.',flush=True)


if __name__=='__main__':
    os.environ['MAVLINK20']='1'
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('decisions','ops','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--start-ros-ns',type=int,required=True)
    p.add_argument('--duration-s',type=float,default=30.)
    p.add_argument('--confirm-ground-isolated',action='store_true')
    run(p.parse_args())
