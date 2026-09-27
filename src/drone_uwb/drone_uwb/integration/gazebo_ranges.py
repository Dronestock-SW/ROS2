"""Publish simulated UWB RAW on Gazebo Transport and record the same samples.

This supplies virtual sensor observations only, with an explicit simulation
clock and message type. It does not speak MAVLink or send flight commands.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
from queue import Queue, Full, Empty
import sys
import time

from drone_uwb.integration.gazebo_capture import pose_record
from drone_uwb.processing.gazebo_geometry import VirtualRanges
from drone_uwb.processing.experiments.gazebo_trial import validate_config


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/world/dronestock_uwb/dynamic_pose/info')
    parser.add_argument('--model', default='dronestock_x500_0')
    parser.add_argument('--output-topic', default='/dronestock/sim/uwb/ranges')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate-hz', type=float, default=40)
    parser.add_argument('--idle-timeout-s', type=float, default=120)
    opts = parser.parse_args(args)
    if (not all(math.isfinite(v) and v > 0 for v in (opts.rate_hz, opts.idle_timeout_s))
            or opts.rate_hz > 1000):
        parser.error('positive finite rate <= 1000 and timeout required')
    config = json.loads(opts.config.read_text(encoding='utf-8'))
    validate_config(config)
    if config['faults']:
        parser.error('live RAW supports Gaussian noise and bias only; use replay for scheduled faults')
    source = VirtualRanges(config)
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
    except ImportError as exc:
        parser.error('use /usr/bin/python3 with Harmonic Python bindings: '+str(exc))
    queue, counters = Queue(maxsize=10000), Counter()

    def callback(message):
        try:
            row = pose_record(message, opts.model)
            if row is None:
                counters['no_exact_model_match'] += 1
            else:
                queue.put_nowait(row)
        except Full:
            counters['queue_overflow'] += 1
        except (ValueError, AttributeError):
            counters['invalid_message'] += 1

    node = Node()
    publisher = node.advertise(opts.output_topic, StringMsg)
    if not publisher or not node.subscribe(Pose_V, opts.topic, callback):
        raise RuntimeError('gazebo_transport_setup_failed')
    opts.output.mkdir(parents=True, exist_ok=False)
    (opts.output/'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    first = last = previous_seen = None
    last_progress = time.monotonic()
    reason = 'user_stopped'
    print('Simulated UWB RAW: '+opts.output_topic+'; Ctrl+C to save and stop.', flush=True)
    try:
        with (opts.output/'poses.jsonl').open('x', encoding='utf-8') as poses, \
             (opts.output/'raw_ranges.jsonl').open('x', encoding='utf-8') as raw, \
             (opts.output/'truth.jsonl').open('x', encoding='utf-8') as truth:
            while True:
                if time.monotonic()-last_progress > opts.idle_timeout_s:
                    reason = 'no_advancing_pose_timeout'
                    break
                try:
                    pose = queue.get(timeout=.25)
                except Empty:
                    continue
                stamp = pose['time_us']
                if previous_seen is not None and stamp < previous_seen:
                    reason = 'simulation_time_reversed'
                    break
                if previous_seen is None or stamp > previous_seen:
                    last_progress = time.monotonic()
                previous_seen = stamp
                if last is not None and stamp-last < round(1e6/opts.rate_hz):
                    counters['rate_filtered'] += 1
                    continue
                message, reference = source.sample(pose)
                packet = StringMsg()
                packet.header.stamp.sec = stamp//1_000_000
                packet.header.stamp.nsec = stamp%1_000_000*1000
                packet.data = json.dumps(message, ensure_ascii=False, allow_nan=False)
                if not publisher.publish(packet):
                    counters['publish_failed'] += 1
                for stream, row in ((poses, pose), (raw, message), (truth, reference)):
                    stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
                counters['recorded'] += 1
                first = stamp if first is None else first
                last = stamp
    except KeyboardInterrupt:
        pass
    except Exception:
        reason = 'recording_error'
        raise
    finally:
        summary = dict(source='simulation', model=opts.model, input_topic=opts.topic,
                       output_topic=opts.output_topic, counters=dict(counters), stop_reason=reason,
                       start_sim_us=first, end_sim_us=last,
                       effective_rate_hz=((counters['recorded']-1)*1e6/(last-first)
                                          if first is not None and last > first else None),
                       external_output_allowed=False, flight_valid=False)
        (opts.output/'capture.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(summary, ensure_ascii=False))
    if not counters['recorded'] or reason != 'user_stopped':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
