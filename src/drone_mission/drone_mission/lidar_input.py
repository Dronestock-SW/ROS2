"""Measured-time attitude interpolation for scan rotation compensation.

Only quaternion orientation is interpolated. FC position/velocity is never
used to create LiDAR displacement. No missing beam times are invented.
"""
from bisect import bisect_left
import math


def quaternion_at(history, stamp_ns):
    stamps=[x[0] for x in history];i=bisect_left(stamps,stamp_ns)
    if i<len(history) and stamps[i]==stamp_ns:return history[i][1]
    if i==0 or i==len(history):raise ValueError('attitude_not_bracketed')
    a,qa=history[i-1];b,qb=history[i]
    if b-a>50_000_000:raise ValueError('attitude_history_gap')
    if sum(x*y for x,y in zip(qa,qb))<0:qb=tuple(-x for x in qb)
    f=(stamp_ns-a)/(b-a);q=tuple((1-f)*x+f*y for x,y in zip(qa,qb));n=math.sqrt(sum(v*v for v in q))
    if n<.99:raise ValueError('attitude_rotation_too_large')
    return tuple(v/n for v in q)


def beam_attitudes(history, stamp_ns, increment_s, count):
    if not math.isfinite(increment_s) or increment_s<=0 or not 40<=count<=2048:
        raise ValueError('measured_beam_timing_required')
    if increment_s*(count-1)>.15:raise ValueError('scan_duration_exceeds_limit')
    beams=[quaternion_at(history,stamp_ns+round(i*increment_s*1e9)) for i in range(count)]
    return beams[-1],beams


def nearest_uwb(history, stamp_ns):
    if not history:raise ValueError('uwb_anchor_missing')
    stamp,xy=min(history,key=lambda row:abs(row[0]-stamp_ns))
    if abs(stamp-stamp_ns)>30_000_000:raise ValueError('uwb_scan_alignment_missing')
    return stamp,xy


def point_cloud_samples(points, channels):
    """Keep SDK acquisition-order timestamps through angular rebinning.

    The vendor may pad a fixed-resolution scan by repeating its last point.
    Identical adjacent XY samples add no geometry; retain their first time.
    """
    if not 40<=len(points)<=2048:raise ValueError('lidar_point_count_invalid')
    matches=[c['values'] for c in channels if c['name']=='stamps']
    if len(matches)!=1 or len(matches[0])!=len(points):raise ValueError('sdk_point_times_missing')
    ranges=[];angles=[];offsets=[];previous=None;last=-1.;dropped=0
    for p,t in zip(points,matches[0]):
        x,y,z=p['x'],p['y'],p['z'];t=float(t)
        if not all(math.isfinite(v) for v in (x,y,z,t)) or abs(z)>.001 or not 0<=t<=.15 or t<=last:
            raise ValueError('sdk_point_or_time_invalid')
        last=t
        if previous==(x,y):dropped+=1;continue
        previous=(x,y);ranges.append(math.hypot(x,y));angles.append(math.atan2(y,x));offsets.append(t)
    if len(ranges)<40:raise ValueError('insufficient_distinct_sdk_points')
    return ranges,angles,offsets,dropped


def point_attitudes(history, first_stamp_ns, offsets):
    if not 40<=len(offsets)<=2048 or any(not math.isfinite(t) or not 0<=t<=.15 for t in offsets):
        raise ValueError('sdk_point_times_invalid')
    if any(b<=a for a,b in zip(offsets,offsets[1:])):raise ValueError('sdk_point_times_not_increasing')
    beams=[quaternion_at(history,first_stamp_ns+round(t*1e9)) for t in offsets]
    return beams[-1],beams


def cloud_scan(points, channels, history, first_stamp_ns, frame_id,
               ros_now_ns, receipt_age_s, yaw_to_map):
    """Prepare the exact payload used by the executor and receive-only audit."""
    ranges,angles,offsets,dropped=point_cloud_samples(points,channels)
    reference,beams=point_attitudes(history,first_stamp_ns,offsets)
    end_stamp=first_stamp_ns+round(offsets[-1]*1e9)
    end_age=(ros_now_ns-end_stamp)/1e9
    first_age=(ros_now_ns-first_stamp_ns)/1e9
    if first_stamp_ns<=0 or not 0<=end_age<=.15 or not 0<=receipt_age_s<=.15:
        raise ValueError('lidar_cloud_stale_or_future')
    w,x,y,z=reference
    return dict(input_source='sdk_pointcloud',stamp_ns=end_stamp,
        first_beam_stamp_ns=first_stamp_ns,first_beam_age_s=first_age,
        frame_id=frame_id,age_s=max(end_age,receipt_age_s),
        range_min=.03,range_max=12.,ranges=ranges,point_angles_rad=angles,
        point_times_s=offsets,duplicate_points_dropped=dropped,
        roll_rad=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        pitch_rad=math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
        map_yaw_rad=yaw_to_map(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))),
        reference_quaternion=reference,beam_quaternions=beams,
        attitude_age_s=end_age,attitude_time_error_s=0.)
