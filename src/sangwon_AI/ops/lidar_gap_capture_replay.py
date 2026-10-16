"""Replay original ROS point/time records through the C++ monitor, shadow only.

Accepts manual_capture or bench_probe. No FC connection or clock repair.
Legacy angular-bin timing is only a labelled diagnostic.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from drone_mission.lidar_input import beam_attitudes, cloud_scan


def header_ns(data):
    stamp=data.get('header',{}).get('stamp',{})
    return stamp.get('sec',0)*1_000_000_000+stamp.get('nanosec',0)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--max-scans',type=int,default=500)
    p.add_argument('--input-source',choices=('sdk_pointcloud','uniform_laserscan'),default='sdk_pointcloud')
    args=p.parse_args()
    if not 1<=args.max_scans<=5000:raise ValueError('one_to_5000_scans_required')
    args.output.mkdir(parents=True,exist_ok=False)
    events=args.capture/'events.jsonl'
    if not events.exists():events=args.capture/'topics.jsonl'
    history=[];pending=[];counts=Counter();samples=0;durations=[];digest=hashlib.sha256()
    timing=[];scan_receipts=[];cloud_receipts=[];duplicates=[];input_errors=Counter()
    process=subprocess.Popen([str(args.binary.resolve())],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    process.stdin.write(json.dumps(dict(input_source=args.input_source))+'\n');process.stdin.flush()
    try:
        with events.open(encoding='utf-8') as source,(args.output/'relative.jsonl').open('w',encoding='utf-8') as out:
            for line in source:
                digest.update(line.encode('utf-8'));row=json.loads(line);topic=row.get('topic')
                data=row.get('data',row.get('message',{}));stamp=row.get('header_ns',header_ns(data))
                mono=row.get('received_monotonic_ns',row.get('monotonic_ns',0));ros=row.get('received_ros_ns')
                if topic=='/mavros/local_position/odom':
                    q=data['pose']['pose']['orientation'];quat=tuple(q[k] for k in ('w','x','y','z'))
                    if abs(sum(v*v for v in quat)-1)<=.002:
                        if not history or stamp>history[-1][0]:history.append((stamp,quat));history=history[-256:]
                if topic=='/scan':
                    scan_receipts.append(mono)
                    timing.append(dict(count=len(data['ranges']),scan_time=data['scan_time'],
                        angular_bin_duration=(len(data['ranges'])-1)*data['time_increment']))
                if topic=='/point_cloud':cloud_receipts.append(mono)
                selected='/point_cloud' if args.input_source=='sdk_pointcloud' else '/scan'
                if topic==selected:pending.append((data,stamp,mono))
                if not pending or ros is None:continue
                raw,first,receipt=pending[0];scan=None
                try:
                    if args.input_source=='sdk_pointcloud':
                        scan=cloud_scan(raw['points'],raw['channels'],history,first,
                            raw['header']['frame_id'],ros,(mono-receipt)/1e9,lambda y:y)
                        duplicates.append(scan['duplicate_points_dropped'])
                    else:
                        reference,beams=beam_attitudes(history,first,raw['time_increment'],len(raw['ranges']))
                        end=first+round((len(raw['ranges'])-1)*raw['time_increment']*1e9);end_age=(ros-end)/1e9
                        if not 0<=end_age<=.15:raise ValueError('legacy_scan_stale_or_future')
                        w,x,y,z=reference
                        scan=dict(raw,input_source='uniform_laserscan',stamp_ns=end,first_beam_stamp_ns=first,
                            frame_id=raw['header']['frame_id'],first_beam_age_s=(ros-first)/1e9,age_s=end_age,
                            roll_rad=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                            pitch_rad=math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
                            map_yaw_rad=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)),
                            reference_quaternion=reference,beam_quaternions=beams,
                            attitude_age_s=end_age,attitude_time_error_s=0.)
                except (ValueError,KeyError) as error:
                    if (ros-first)/1e9<.3:continue
                    input_errors[str(error)]+=1
                pending.pop(0)
                state=dict(lidar_scan=scan,uwb_xy=[],xy=[],velocity_xy=[],uwb_age_s=1e9,uwb_stamp_ns=0)
                request=dict(sample_time_s=mono/1e9,state=state)
                start=time.monotonic();process.stdin.write(json.dumps(request,allow_nan=False)+'\n');process.stdin.flush()
                response=process.stdout.readline()
                if not response:raise RuntimeError('cpp_lidar_replay_exited')
                result=json.loads(response);durations.append(time.monotonic()-start)
                assert not result['approved'] and not result['fc_aiding']
                counts[result['reason']]+=1;samples+=1
                out.write(json.dumps(dict(sample=samples,received_monotonic_ns=mono,**result))+'\n')
                if samples>=args.max_scans:break
        gaps=lambda rows:[(b-a)/1e6 for a,b in zip(rows,rows[1:])]
        summary=dict(scope='PRESERVED_CAPTURE_RELATIVE_LIDAR_SHADOW',physical_flight=False,
            approved_observations=0,fc_commands_sent=0,px4_aiding=False,
            absolute_position_reconstructed=False,heading_reference='raw_FC_ENU_unsurveyed_to_UWB_map',
            input_source=args.input_source,legacy_angular_timing_verified=False,
            capture_name=args.capture.name,processed_events_prefix_sha256=digest.hexdigest(),
            scans=samples,reasons=dict(counts),input_errors=dict(input_errors),pending=len(pending),
            roundtrip_max_ms=max(durations,default=0)*1000,
            duplicate_points_dropped_max=max(duplicates,default=0),
            scan_receipt_gap_max_ms=max(gaps(scan_receipts),default=0),
            cloud_receipt_gap_max_ms=max(gaps(cloud_receipts),default=0),
            angular_bin_timing_exceeds_scan_period=sum(x['angular_bin_duration']>x['scan_time'] for x in timing),
            timing_examples=timing[:3])
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(summary))
    finally:
        process.stdin.close();process.wait(timeout=3);process.stdout.close()


if __name__=='__main__':main()
