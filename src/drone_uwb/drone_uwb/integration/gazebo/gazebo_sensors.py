"""Record Gazebo rangefinder and IMU samples in the Gazebo simulation clock.

The IMU orientation reference frame has to be checked against the model before
using it for height projection. This recorder has no PX4 output or flight API.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import sys
import time


def _stamp_us(header):
    stamp = header.stamp
    if (type(stamp.sec) is not int or type(stamp.nsec) is not int or stamp.sec < 0
            or not 0 <= stamp.nsec < 1_000_000_000):
        raise ValueError('invalid_gazebo_timestamp')
    return stamp.sec * 1_000_000 + stamp.nsec // 1000


def tof_record(message, topic):
    """Keep invalid/no-return scans as explicit missing observations."""
    stamp = _stamp_us(message.header)
    values = list(message.ranges)
    lo, hi = float(message.range_min), float(message.range_max)
    if len(values) != 1 or not all(math.isfinite(v) for v in (lo, hi)) or not 0 < lo < hi:
        raise ValueError('unexpected_tof_scan_geometry')
    value = float(values[0])
    valid = math.isfinite(value) and lo <= value <= hi
    return dict(schema=1, source='gazebo_sensor', type='tof_sample',
                clock_domain='gazebo_sim_us', time_us=stamp, topic=topic,
                sensor_frame=message.frame, distance_m=value if valid else None,
                range_min_m=lo, range_max_m=hi, valid=valid,
                reason='ok' if valid else 'no_valid_return')


def attitude_record(message, topic):
    """Record the IMU orientation without declaring its world alignment."""
    stamp = _stamp_us(message.header)
    q = message.orientation
    values = [float(q.w), float(q.x), float(q.y), float(q.z)]
    norm = math.sqrt(sum(v*v for v in values)) if all(math.isfinite(v) for v in values) else 0.
    valid = abs(norm-1.) <= 1e-3
    omega = message.angular_velocity
    rates = [float(omega.x), float(omega.y), float(omega.z)]
    if not all(math.isfinite(v) for v in rates):
        rates = None
    return dict(schema=1, source='gazebo_sensor', type='imu_attitude_sample',
                clock_domain='gazebo_sim_us', time_us=stamp, topic=topic,
                sensor_frame=message.entity_name,
                orientation_reference='initial_imu_frame_unverified',
                quaternion_wxyz=[v/norm for v in values] if valid else None,
                angular_velocity_rad_s=rates, valid=valid,
                reason='ok' if valid else 'invalid_orientation')


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='dronestock_x500_0')
    parser.add_argument('--world', default='dronestock_uwb')
    parser.add_argument('--tof-topic')
    parser.add_argument('--imu-topic')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--idle-timeout-s', type=float, default=120.)
    opts = parser.parse_args(args)
    if not math.isfinite(opts.idle_timeout_s) or opts.idle_timeout_s <= 0:
        parser.error('positive finite idle timeout required')
    prefix = f'/world/{opts.world}/model/{opts.model}/link'
    tof_topic = opts.tof_topic or prefix+'/lidar_sensor_link/sensor/lidar/scan'
    imu_topic = opts.imu_topic or prefix+'/base_link/sensor/imu_sensor/imu'
    try:
        from gz.transport13 import Node
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.imu_pb2 import IMU
    except ImportError as exc:
        parser.error('use /usr/bin/python3 with Harmonic Gazebo bindings: '+str(exc))
    queue, counters = Queue(maxsize=10000), Counter()

    def callback(kind, parser_fn, topic):
        def accept(message):
            try:
                queue.put_nowait((kind, parser_fn(message, topic)))
            except Full:
                counters[kind+'_queue_overflow'] += 1
            except (ValueError, TypeError, AttributeError):
                counters[kind+'_invalid_message'] += 1
        return accept

    node = Node()
    if not node.subscribe(LaserScan, tof_topic, callback('tof', tof_record, tof_topic)):
        raise RuntimeError('tof_subscription_failed:'+tof_topic)
    if not node.subscribe(IMU, imu_topic, callback('imu', attitude_record, imu_topic)):
        raise RuntimeError('imu_subscription_failed:'+imu_topic)
    opts.output.mkdir(parents=True, exist_ok=False)
    last = {'tof': None, 'imu': None}
    first = {'tof': None, 'imu': None}
    last_progress = time.monotonic()
    reason = 'user_stopped'
    print('Gazebo ToF and IMU recording; Ctrl+C to save and stop.', flush=True)
    try:
        with (opts.output/'tof.jsonl').open('x', encoding='utf-8') as tof_stream, \
             (opts.output/'attitude.jsonl').open('x', encoding='utf-8') as imu_stream:
            streams = {'tof': tof_stream, 'imu': imu_stream}
            while True:
                if time.monotonic()-last_progress > opts.idle_timeout_s:
                    reason = 'no_advancing_sensor_timeout'
                    break
                try:
                    kind, row = queue.get(timeout=.25)
                except Empty:
                    continue
                stamp = row['time_us']
                if last[kind] is not None and stamp < last[kind]:
                    reason = kind+'_time_reversed'
                    break
                if last[kind] is not None and stamp == last[kind]:
                    counters[kind+'_duplicate_stamp'] += 1
                    continue
                first[kind] = stamp if first[kind] is None else first[kind]
                last[kind] = stamp
                last_progress = time.monotonic()
                streams[kind].write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
                counters[kind+'_recorded'] += 1
                if not row['valid']:
                    counters[kind+'_invalid_sample'] += 1
    except KeyboardInterrupt:
        pass
    except Exception:
        reason = 'recording_error'
        raise
    finally:
        summary = dict(source='gazebo_sensor', clock_domain='gazebo_sim_us',
                       topics={'tof': tof_topic, 'imu': imu_topic}, counters=dict(counters),
                       first_sim_us=first, last_sim_us=last, stop_reason=reason,
                       orientation_reference='initial_imu_frame_unverified',
                       external_output_allowed=False, flight_valid=False)
        (opts.output/'capture.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    if not counters['tof_recorded'] or not counters['imu_recorded'] or reason != 'user_stopped':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
