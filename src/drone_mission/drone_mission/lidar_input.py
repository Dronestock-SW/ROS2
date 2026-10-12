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
