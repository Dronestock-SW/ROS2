"""Receive-only front/left target calibration for the measured PointCloud frame.

Put a narrow vertical target approximately 1 m from the LiDAR, first on the
vehicle's forward ray and then on its left ray. Keep the aircraft stationary.
The analyzer requires both placements and checks handedness; it never enables
PX4 fusion or writes FC parameters. Raw captures are retained in new paths.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def profile(path):
    scans, yaws, stamps = [], [], []
    invalid_state = 0
    states = 0
    for line in (path/'topics.jsonl').open(encoding='utf-8'):
        row = json.loads(line)
        msg = row['message']
        if row['topic'] == '/mavros/state':
            states += 1
            invalid_state += not msg['connected'] or msg['armed']
        if row['topic'] == '/mavros/imu/data':
            q = msg['orientation']
            yaws.append(math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2)))
        if row['topic'] != '/point_cloud':
            continue
        xy = np.array([[p['x'],p['y']] for p in msg['points']])
        r = np.linalg.norm(xy,axis=1)
        a = np.arctan2(xy[:,1],xy[:,0])
        bins = np.rint(np.degrees(a)).astype(int)%360
        values = np.full(360,np.nan)
        for index in np.unique(bins):
            candidate = r[(bins==index)&(r>.2)&(r<12.)]
            if len(candidate):
                values[index] = np.median(candidate)
        scans.append(values)
        stamps.append(row['received_ros_ns'])
    if len(scans)<40 or len(yaws)<100 or states<3 or invalid_state:
        raise ValueError('forty_scans_and_fresh_disarmed_IMU_required')
    mean = math.atan2(np.mean(np.sin(yaws)),np.mean(np.cos(yaws)))
    span = np.ptp([wrap(y-mean) for y in yaws])
    if span > math.radians(.5):
        raise ValueError('aircraft_moved_during_target_capture')
    with np.errstate(all='ignore'):
        data = np.asarray(scans)
        values = np.array([np.median(col[np.isfinite(col)]) if np.isfinite(col).sum()>=len(col)*.7 else np.nan
            for col in data.T])
    return values, mean, dict(scans=len(scans),imu_samples=len(yaws),yaw_span_deg=math.degrees(span),
        first_received_ns=min(stamps),last_received_ns=max(stamps),
        raw_sha256=hashlib.sha256((path/'topics.jsonl').read_bytes()).hexdigest())


def target_angle(baseline, target, *, distance_m=1., tolerance_m=.3):
    changed = (np.isfinite(baseline)&np.isfinite(target)&(baseline-target>.25)
        &(np.abs(target-distance_m)<tolerance_m))
    indices = np.flatnonzero(changed)
    if len(indices)<5:
        raise ValueError('target_not_distinct_from_background')
    groups = []
    visited = set()
    for index in indices:
        if int(index) in visited:
            continue
        todo, group = [int(index)], []
        while todo:
            current = todo.pop()
            if current in visited or not changed[current]:
                continue
            visited.add(current)
            group.append(current)
            todo.extend([(current-1)%360,(current+1)%360])
        if len(group)>=5:
            groups.append(group)
    if len(groups)!=1 or len(groups[0])>35:
        raise ValueError('ambiguous_or_oversized_target')
    angles = np.radians(groups[0])
    angle = math.atan2(np.mean(np.sin(angles)),np.mean(np.cos(angles)))
    return angle, dict(target_bins=len(groups[0]),angle_deg=math.degrees(angle),
        median_range_m=float(np.median(target[groups[0]])))


def analyze(args):
    reference, yaw0, evidence0 = profile(args.baseline)
    front, yaw1, evidence1 = profile(args.front)
    left, yaw2, evidence2 = profile(args.left)
    if max(abs(wrap(yaw1-yaw0)),abs(wrap(yaw2-yaw0))) > math.radians(.5):
        raise ValueError('aircraft_heading_changed_between_targets')
    a, front_info = target_angle(reference,front,distance_m=args.distance)
    b, left_info = target_angle(reference,left,distance_m=args.distance)
    if abs(wrap((b-a)-math.pi/2)) > math.radians(5.):
        raise ValueError('front_left_order_or_handedness_mismatch')
    yaw = math.atan2(math.sin(-a)+math.sin(math.pi/2-b),math.cos(-a)+math.cos(math.pi/2-b))
    original = json.loads(args.config.read_text(encoding='utf-8'))
    original['settings'].update(enabled=False,sensor_to_body_yaw_rad=yaw,
        mount_yaw_ref=str(args.output.resolve()))
    original['mount_calibration'] = dict(method='two_operator_placed_targets',
        yaw_deg=math.degrees(yaw),front=front_info,left=left_info,
        raw=[dict(path=str(p.resolve()),**e) for p,e in
            ((args.baseline,evidence0),(args.front,evidence1),(args.left,evidence2))],
        vertical_plane_verified=False,dynamic_timing_verified=False,
        physical_accuracy_verified=False,fc_parameter_writes=False)
    with args.output.open('x',encoding='utf-8') as output:
        json.dump(original,output,ensure_ascii=False,indent=2)
        output.write('\n')
    print(json.dumps(original['mount_calibration'],ensure_ascii=False,indent=2))


def inspect_target(args):
    baseline, yaw0, _ = profile(args.baseline)
    target, yaw1, evidence = profile(args.target)
    if abs(wrap(yaw1-yaw0)) > math.radians(.5):
        raise ValueError('aircraft_heading_changed_between_targets')
    angle, info = target_angle(baseline,target,distance_m=args.distance)
    print(json.dumps(dict(info,raw=evidence,final_calibration=False),indent=2))


def record(args):
    from drone_uwb.integration.ros.local_transport import configure_local_dds
    configure_local_dds()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data as qos
    from sensor_msgs.msg import PointCloud, Imu
    from mavros_msgs.msg import State
    from rosidl_runtime_py.convert import message_to_ordereddict
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('lidar_mount_readonly_capture')
    stream = (args.output/'topics.jsonl').open('x',encoding='utf-8')
    count = {}
    def receive(name,msg):
        count[name] = count.get(name,0)+1
        stream.write(json.dumps(dict(topic=name,received_ros_ns=node.get_clock().now().nanoseconds,
            message=message_to_ordereddict(msg)))+'\n')
    subscriptions = [node.create_subscription(cls,topic,lambda msg,n=topic:receive(n,msg),qos)
        for topic,cls in (('/point_cloud',PointCloud),('/mavros/imu/data',Imu),('/mavros/state',State))]
    start = time.monotonic()
    while time.monotonic()-start<10.:
        rclpy.spin_once(node,timeout_sec=.05)
    stream.close()
    (args.output/'stage.json').write_text(json.dumps(dict(stage=args.stage,counts=count,
        commands_sent=False),indent=2)+'\n',encoding='utf-8')
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(count))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action',required=True)
    capture = sub.add_parser('record')
    capture.add_argument('--stage',choices=['baseline','front','left'],required=True)
    capture.add_argument('--output',type=Path,required=True)
    analysis = sub.add_parser('analyze')
    for key in ('baseline','front','left','config','output'):
        analysis.add_argument('--'+key,type=Path,required=True)
    analysis.add_argument('--distance',type=float,default=1.)
    inspect = sub.add_parser('inspect-target')
    inspect.add_argument('--baseline',type=Path,required=True)
    inspect.add_argument('--target',type=Path,required=True)
    inspect.add_argument('--distance',type=float,default=1.)
    args = parser.parse_args()
    {'record':record,'analyze':analyze,'inspect-target':inspect_target}[args.action](args)
