"""Capture one top-level Gazebo model pose; no ROS or flight commands.

Harmonic uses gz.transport13 and gz.msgs10. Run with the system Python that
owns those apt packages. Only simulator timestamps determine sample spacing.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import sys
import time


def pose_record(message, model):
    matches = [p for p in message.pose if p.name == model]
    if len(matches) != 1:
        return None
    pose = matches[0]
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nsec < 1_000_000_000:
        raise ValueError('invalid_gazebo_timestamp')
    xyz = [pose.position.x, pose.position.y, pose.position.z]
    quaternion = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    if not all(math.isfinite(v) for v in xyz+quaternion):
        raise ValueError('invalid_gazebo_pose')
    return dict(source='gazebo', clock_domain='gazebo_sim_us', model=model,
                time_us=stamp.sec*1_000_000+stamp.nsec//1000,
                position_xyz_m=xyz, quaternion_wxyz=quaternion)


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/world/default/dynamic_pose/info')
    parser.add_argument('--model', required=True, help='Exact top-level model name from gz topic output')
    parser.add_argument('--duration-s', type=float, default=60)
    parser.add_argument('--rate-hz', type=float, default=40, help='Upper rate limit; actual timestamps are preserved, without interpolation')
    parser.add_argument('--idle-timeout-s', type=float, default=120)
    parser.add_argument('--output', type=Path, required=True)
    opts = parser.parse_args(args)
    if (not all(math.isfinite(v) for v in (opts.duration_s, opts.rate_hz, opts.idle_timeout_s))
            or min(opts.duration_s, opts.rate_hz, opts.idle_timeout_s) <= 0 or opts.rate_hz > 1000):
        parser.error('duration/rate/timeout must be positive; rate <= 1000')
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    except ImportError as exc:
        parser.error('Harmonic bindings missing: use /usr/bin/python3 with python3-gz-transport13 and python3-gz-msgs10; '+str(exc))
    queue, counters = Queue(maxsize=10000), Counter()

    def callback(message):
        try:
            row = pose_record(message, opts.model)
            if row is None:
                counters['no_exact_model_match'] += 1
                return
            queue.put_nowait(row)
        except Full:
            counters['queue_overflow'] += 1
        except (ValueError, AttributeError):
            counters['invalid_message'] += 1

    node = Node()
    if not node.subscribe(Pose_V, opts.topic, callback):
        raise RuntimeError('could_not_subscribe:'+opts.topic)
    opts.output.mkdir(parents=True, exist_ok=False)
    first = last = previous_seen = None
    max_gap_us = 0
    last_wall = time.monotonic()
    stop_reason = 'duration_complete'
    print('Gazebo pose recording: '+opts.model+' on '+opts.topic, flush=True)
    try:
        with (opts.output/'poses.jsonl').open('x', encoding='utf-8') as stream:
            while True:
                if time.monotonic()-last_wall > opts.idle_timeout_s:
                    stop_reason = 'no_advancing_pose_timeout'
                    break
                try:
                    row = queue.get(timeout=.25)
                except Empty:
                    if time.monotonic()-last_wall > opts.idle_timeout_s:
                        stop_reason = 'no_matching_pose_timeout'
                        break
                    continue
                stamp = row['time_us']
                if previous_seen is not None and stamp < previous_seen:
                    stop_reason = 'simulation_time_reversed'
                    break
                if previous_seen is None or stamp > previous_seen:
                    last_wall = time.monotonic()
                previous_seen = stamp
                if last is not None and stamp-last < round(1e6/opts.rate_hz):
                    counters['rate_filtered'] += 1
                    continue
                if first is None:
                    first = stamp
                if stamp-first > round(opts.duration_s*1e6):
                    break
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
                counters['recorded'] += 1
                if last is not None:
                    max_gap_us = max(max_gap_us, stamp-last)
                last = stamp
                if counters['recorded'] % 200 == 0:
                    stream.flush()
                    print(f'Recorded {counters["recorded"]} samples / {(stamp-first)/1e6:.1f} simulation seconds', flush=True)
    except KeyboardInterrupt:
        stop_reason = 'user_stopped'
    finally:
        info = dict(source='gazebo', topic=opts.topic, model=opts.model, counters=dict(counters),
                    start_sim_us=first, end_sim_us=last, stop_reason=stop_reason,
                    requested_rate_hz=opts.rate_hz, max_recorded_gap_us=max_gap_us,
                    effective_rate_hz=((counters['recorded']-1)*1e6/(last-first)
                                       if first is not None and last > first else None),
                    external_output_allowed=False)
        (opts.output/'capture.json').write_text(json.dumps(info, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(info, ensure_ascii=False), flush=True)
    if not counters['recorded'] or stop_reason in ('simulation_time_reversed', 'no_matching_pose_timeout', 'no_advancing_pose_timeout'):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
