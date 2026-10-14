#!/usr/bin/env python3
"""Read-only measured scan replay. No ROS, FC output, or calibration writes.

Replays recorded callback arrivals with a 20 ms processing timer and the
bridge's 50 ms IMU-bracket wait. Geometry assumptions are explicitly diagnostic.
"""
import argparse
from collections import Counter, deque
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from drone_uwb.processing.lidar_motion import LidarMotionSettings, MeasuredLidarMotion
from drone_uwb.integration.lidar_velocity import VelocityReadinessWindow


def replay(events, settings, *, drop_tof=False):
    model=MeasuredLidarMotion(settings)
    readiness=VelocityReadinessWindow()
    queue=deque(maxlen=2)
    floor=None
    last_imu_ns=0
    counts=Counter()
    accepted=[]
    i=0
    tick=events[0]['received_ros_ns']
    finish=events[-1]['received_ros_ns']+60_000_000
    while tick<=finish:
        while i<len(events) and events[i]['received_ros_ns']<=tick:
            row=events[i];i+=1
            data=row['data'];topic=row['topic'];ns=row['header_ns']
            age=(row['received_ros_ns']-ns)/1e9 if ns else math.inf
            if topic=='/mavros/imu/data':
                if (data['header']['frame_id']!=settings.imu_frame or not 0<=age<=.2
                    or data['orientation_covariance'][0]<0 or data['angular_velocity_covariance'][0]<0):
                    counts['imu_input_invalid']+=1;continue
                q=np.array([data['orientation'][k] for k in ('x','y','z','w')])
                if abs(np.linalg.norm(q)-1)>.01:
                    counts['imu_input_invalid']+=1;continue
                try:
                    model.imu.add_gyro(ns/1e9,[data['angular_velocity'][k] for k in ('x','y','z')])
                    model.imu.add_attitude(ns/1e9,Rotation.from_quat(q).as_matrix())
                    last_imu_ns=ns
                except ValueError:counts['imu_input_invalid']+=1
            elif topic=='/mavros/downward_0':
                valid=0<=age<=.15 and data['min_range']<=data['range']<=data['max_range']
                floor=(data['range'],ns) if valid and not drop_tof else None
                counts['tof_valid' if valid else 'tof_invalid']+=1
            elif topic=='/point_cloud':
                if len(queue)==queue.maxlen:
                    queue.popleft();model.break_continuity();counts['queue_dropped']+=1
                queue.append(row)
        if queue:
            row=queue[0];data=row['data']
            channels=[c['values'] for c in data['channels'] if c['name']=='stamps']
            receipt_age=(tick-row['received_ros_ns'])/1e9
            last_ns=row['header_ns']+round((channels[0][-1] if channels and channels[0] else 0)*1e9)
            waiting=settings.geometry_gate()=='ready' and receipt_age<.05 and last_imu_ns<last_ns
            if not waiting:
                queue.popleft()
                if len(channels)!=1:
                    counts['sdk_stamps_channel_missing']+=1
                else:
                    distance,stamp=floor or (math.nan,0)
                    result=model.observe(points=[[p[k] for k in ('x','y','z')] for p in data['points']],
                        offsets=channels[0],first_stamp_ns=row['header_ns'],now_ns=tick,
                        receipt_age_s=receipt_age,frame_id=data['header']['frame_id'],
                        floor_range_m=distance,floor_stamp_ns=stamp)
                    counts['scans_processed']+=1
                    counts['accepted' if result['accepted'] else result['reasons'][0]]+=1
                    if result.get('floor_range_valid') is False:counts['scans_floor_unavailable']+=1
                    if result['accepted']:
                        readiness.observe(result)
                        window=readiness.status()
                        if window['ready']:counts['readiness_window_available']+=1
                        if window['ready'] and window['speed_m_s']<=.1:counts['readiness_window_stationary']+=1
                        accepted.append(dict(stamp_ns=result['representative_stamp_ns'],
                            velocity_ne_m_s=result['velocity_ne_m_s'],floor_valid=result['floor_range_valid'],
                            stationary_window=window))
        tick+=20_000_000
    speeds=[np.linalg.norm(r['velocity_ne_m_s']) for r in accepted]
    return dict(counts=dict(counts),accepted_without_floor=sum(not r['floor_valid'] for r in accepted),
        accepted_speed_p95_m_s=float(np.percentile(speeds,95)) if speeds else None,
        accepted=accepted,fc_transmission=False,fc_fusion_verified=False,
        independent_accuracy_verified=False,callback_timer_simulated=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--events',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--assume-geometry-for-diagnostics',action='store_true')
    parser.add_argument('--diagnostic-max-range-m',type=float)
    args=parser.parse_args()
    if args.diagnostic_max_range_m is not None and not args.assume_geometry_for_diagnostics:
        parser.error('Diagnostic range changes require --assume-geometry-for-diagnostics')
    raw=args.events.read_bytes()
    rows=[json.loads(line) for line in raw.splitlines()]
    events=sorted([r for r in rows if r.get('topic') in ('/point_cloud','/mavros/imu/data','/mavros/downward_0')],key=lambda r:r['received_ros_ns'])
    if not events:parser.error('No recorded scan/IMU/ToF events')
    settings=LidarMotionSettings(**json.loads(args.config.read_text(encoding='utf-8'))['settings'])
    result=dict(source_file_sha256=hashlib.sha256(raw).hexdigest(),
        source_receipts=[r for r in rows if r.get('type')=='source_receipt'],
        configured_geometry_gate=settings.geometry_gate(),configured=replay(events,settings),
        geometry_assumptions=False)
    if args.assume_geometry_for_diagnostics:
        assumed=replace(settings,scan_plane_parallel_confirmed=True,
            timing_ref='OFFLINE_UNQUALIFIED_ZERO_OFFSET_ASSUMPTION',
            vertical_structure_ref='OFFLINE_UNQUALIFIED_VERTICAL_WALL_ASSUMPTION')
        if args.diagnostic_max_range_m is not None:
            assumed=replace(assumed,range_max_m=args.diagnostic_max_range_m)
        result.update(geometry_assumptions=True,diagnostic_sensor_time_offset_s=assumed.sensor_time_offset_s,
            diagnostic_range_max_m=assumed.range_max_m,
            required_floor=replay(events,replace(assumed,floor_policy='required')),
            metric_scan=replay(events,replace(assumed,floor_policy='metric_scan')),
            metric_scan_tof_outage_injected=replay(events,replace(assumed,floor_policy='metric_scan'),drop_tof=True))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:{x:y for x,y in v.items() if x!='accepted'} if isinstance(v,dict) else v
        for k,v in result.items()},ensure_ascii=False,allow_nan=False))


if __name__=='__main__':main()
