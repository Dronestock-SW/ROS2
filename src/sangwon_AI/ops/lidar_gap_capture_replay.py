"""Read preserved ROS capture into the actual C++ LiDAR monitor, shadow only.

No ROS, serial, FC connection, resampling, clock repair or input file writes.
Unsurveyed map alignment is not fabricated. Output is relative diagnostics.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from drone_mission.lidar_input import beam_attitudes


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--max-scans',type=int,default=500)
    args=p.parse_args()
    if not 1<=args.max_scans<=5000:raise ValueError('one_to_5000_scans_required')
    args.output.mkdir(parents=True,exist_ok=False)
    manifest=args.capture/'manifest.json';events=args.capture/'events.jsonl'
    history=[];pending=[];counts=Counter();samples=0;durations=[];digest=hashlib.sha256()
    process=subprocess.Popen([str(args.binary.resolve())],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    process.stdin.write('{}\n');process.stdin.flush()  # Calibration gate stays closed.
    try:
        with events.open(encoding='utf-8') as source,(args.output/'relative.jsonl').open('w',encoding='utf-8') as out:
            for line in source:
                digest.update(line.encode('utf-8'));row=json.loads(line);topic=row.get('topic');data=row.get('data',{})
                if topic=='/mavros/local_position/odom':
                    q=data['pose']['pose']['orientation'];quat=tuple(q[k] for k in ('w','x','y','z'))
                    if abs(sum(v*v for v in quat)-1)<=.002:
                        if not history or row['header_ns']>history[-1][0]:history.append((row['header_ns'],quat));history=history[-256:]
                if topic=='/scan':pending.append(row)
                if not pending:continue
                original=pending[0];raw=original['data'];first=original.get('header_ns',0)
                end=first+round((len(raw.get('ranges',[]))-1)*raw.get('time_increment',0)*1e9)
                scan=dict(raw,stamp_ns=end,first_beam_stamp_ns=first,frame_id=raw.get('header',{}).get('frame_id',''),
                    first_beam_age_s=(row['received_ros_ns']-first)/1e9,
                    age_s=(row['received_ros_ns']-end)/1e9)
                try:
                    reference,beams=beam_attitudes(history,first,raw.get('time_increment',0),len(raw.get('ranges',[])))
                    if scan['age_s']<0:continue
                    w,x,y,z=reference
                    scan.update(roll_rad=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                        pitch_rad=math.asin(max(-1.,min(1.,2*(w*y-z*x)))),map_yaw_rad=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)),
                        reference_quaternion=reference,beam_quaternions=beams,
                        attitude_age_s=scan['age_s'],attitude_time_error_s=0.)
                except ValueError:
                    if scan['first_beam_age_s']<.3:continue
                pending.pop(0)
                state=dict(lidar_scan=scan,uwb_xy=[],xy=[],velocity_xy=[],uwb_age_s=1e9,uwb_stamp_ns=0)
                request=dict(sample_time_s=row['received_monotonic_ns']/1e9,state=state)
                start=time.monotonic();process.stdin.write(json.dumps(request,allow_nan=False)+'\n');process.stdin.flush()
                response=process.stdout.readline()
                if not response:raise RuntimeError('cpp_lidar_replay_exited')
                result=json.loads(response);durations.append(time.monotonic()-start)
                assert not result['approved'] and not result['fc_aiding']
                counts[result['reason']]+=1;samples+=1
                out.write(json.dumps(dict(sample=samples,received_monotonic_ns=row['received_monotonic_ns'],**result))+'\n')
                if samples>=args.max_scans:break
        summary=dict(scope='PRESERVED_CAPTURE_RELATIVE_LIDAR_SHADOW',physical_flight=False,
            approved_observations=0,fc_commands_sent=0,px4_aiding=False,
            absolute_position_reconstructed=False,heading_reference='raw_FC_ENU_unsurveyed_to_UWB_map',
            capture_name=args.capture.name,manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
            processed_events_prefix_sha256=digest.hexdigest(),scans=samples,reasons=dict(counts),
            roundtrip_max_ms=max(durations,default=0)*1000)
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(summary))
    finally:
        process.stdin.close();process.wait(timeout=3);process.stdout.close()


if __name__=='__main__':main()
