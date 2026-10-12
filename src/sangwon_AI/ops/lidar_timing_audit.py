"""Audit preserved SDK point offsets and ROS clock domains, without FC access."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from statistics import median, quantiles
from drone_mission.lidar_input import point_cloud_samples


def stats(values):
    if not values:return None
    return dict(count=len(values),min=min(values),median=median(values),
        p95=quantiles(values,n=100,method='inclusive')[94] if len(values)>1 else values[0],max=max(values))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    source=args.capture/'topics.jsonl'
    if not source.exists():source=args.capture/'events.jsonl'
    scan={};cloud={};times={};errors=Counter();count=Counter();pose_stamp=[];pose_receipt=[]
    for line in source.open(encoding='utf-8'):
        row=json.loads(line);topic=row['topic'];d=row.get('data',row.get('message',{}));count[topic]+=1
        h=d.get('header',{}).get('stamp',{});stamp=h.get('sec',0)*1_000_000_000+h.get('nanosec',0)
        receipt=row.get('received_monotonic_ns',row.get('monotonic_ns',0));ros=row.get('received_ros_ns')
        if topic=='/mavros/local_position/odom':pose_stamp.append(stamp);pose_receipt.append(receipt)
        if topic=='/scan':scan[stamp]=d
        if topic=='/point_cloud':cloud[stamp]=d;times[stamp]=(receipt,ros)
    raw_duration=[];actual_duration=[];duplicates=[];first_age=[];last_age=[];cloud_receipt=[];counts=[];paired=0;exceeds=0
    for stamp,d in cloud.items():
        cloud_receipt.append(times[stamp][0])
        try:
            ranges,angles,offsets,dropped=point_cloud_samples(d['points'],d['channels'])
        except ValueError as error:errors[str(error)]+=1;continue
        actual_duration.append(offsets[-1]*1000);duplicates.append(dropped);counts.append(len(ranges))
        sdk=[x['values'] for x in d['channels'] if x['name']=='stamps'][0]
        raw_duration.append(sdk[-1]*1000)
        receipt,ros=times[stamp]
        if ros is not None:
            first_age.append((ros-stamp)/1e6);last_age.append((ros-stamp-round(offsets[-1]*1e9))/1e6)
        if stamp in scan:
            paired+=1;s=scan[stamp]
            exceeds+=(len(s['ranges'])-1)*s['time_increment']>s['scan_time']
    gap=lambda rows:[(b-a)/1e6 for a,b in zip(rows,rows[1:])]
    result=dict(scope='STATIC_TIMING_CONSISTENCY_ONLY',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        topic_counts=dict(count),paired_scan_cloud=paired,angular_bin_extent_exceeds_period=exceeds,
        sdk_point_extent_ms=stats(raw_duration),distinct_point_extent_ms=stats(actual_duration),
        duplicate_points_dropped=stats(duplicates),distinct_points=stats(counts),
        cloud_first_stamp_age_ms=stats(first_age),cloud_last_distinct_point_age_ms=stats(last_age),
        cloud_receipt_gap_ms=stats(gap(cloud_receipt)),fc_pose_receipt_gap_ms=stats(gap(pose_receipt)),
        fc_pose_source_gap_ms=stats(gap(pose_stamp)),input_errors=dict(errors),
        timing_calibration_complete=False,mount_calibration_complete=False,dynamic_sign_verified=False,
        fc_commands_sent=0,fc_parameters_written=0)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8');print(json.dumps(result))


if __name__=='__main__':main()
