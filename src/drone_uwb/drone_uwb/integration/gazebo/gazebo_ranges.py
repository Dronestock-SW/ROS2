"""Publish simulated UWB RAW on Gazebo Transport and record the same samples.

This supplies virtual sensor observations only, with an explicit simulation
clock and message type. It does not speak MAVLink or send flight commands.
"""
import argparse
from collections import Counter
import hashlib
import inspect
import json
import math
from pathlib import Path
from queue import Queue, Full, Empty
import sys
import time
import platform
import subprocess

from drone_uwb.integration.gazebo.gazebo_capture import pose_record
from drone_uwb.integration.gazebo.gazebo_faults import RangeFaultPlan
from drone_uwb.processing.geometry.gazebo_geometry import VirtualRanges
from drone_uwb.processing.experiments.gazebo_trial import validate_config


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/world/dronestock_uwb/dynamic_pose/info')
    parser.add_argument('--model', default='dronestock_x500_0')
    parser.add_argument('--output-topic', default='/dronestock/sim/uwb/ranges')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--fault-plan', type=Path, help='Optional declared bias or drop intervals')
    parser.add_argument('--duration-s', type=float, help='Optional limit in simulation time')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate-hz', type=float, default=40)
    parser.add_argument('--idle-timeout-s', type=float, default=120)
    opts = parser.parse_args(args)
    if (not all(math.isfinite(v) and v > 0 for v in (opts.rate_hz, opts.idle_timeout_s))
            or opts.rate_hz > 1000 or opts.duration_s is not None
            and (not math.isfinite(opts.duration_s) or opts.duration_s <= 0)):
        parser.error('positive finite rate <= 1000 and timeout required')
    config = json.loads(opts.config.read_text(encoding='utf-8'))
    validate_config(config)
    if config['faults']:
        parser.error('keep config faults empty; use --fault-plan for live injection')
    fault_input = json.loads(opts.fault_plan.read_text(encoding='utf-8')) if opts.fault_plan else None
    if opts.fault_plan and not isinstance(fault_input, dict):
        parser.error('fault plan must be a JSON object')
    faults = RangeFaultPlan(fault_input)
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
    (opts.output/'fault_plan.json').write_text(json.dumps(faults.plan, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    sources = [Path(__file__), Path(inspect.getfile(RangeFaultPlan)), Path(inspect.getfile(VirtualRanges)),
               Path(inspect.getfile(pose_record)), Path(inspect.getfile(validate_config))]
    source_root = Path(__file__).resolve().parents[5]
    try:
        revision = subprocess.run(['git', '-C', str(source_root), 'rev-parse', 'HEAD'],
                                  capture_output=True, text=True, timeout=5)
        commit = revision.stdout.strip() if revision.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        commit = None
    manifest = dict(source='simulation', scope='live_virtual_range_fault_trial', model=opts.model,
        input_topic=opts.topic, output_topic=opts.output_topic, argv=list(sys.argv[1:] if args is None else args),
        requested_rate_hz=opts.rate_hz, duration_sim_s=opts.duration_s, seed=config['seed'],
        fault_clock_origin='first_recorded_pose', fault_plan=faults.plan,
        code_commit=commit, source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        python=sys.version, platform=platform.platform(), flight_valid=False, external_output_allowed=False)
    (opts.output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    first = last = previous_seen = None
    first_published = last_published = None
    last_progress = time.monotonic()
    reason = 'user_stopped'
    print('Simulated UWB RAW: '+opts.output_topic+'; Ctrl+C to save and stop.', flush=True)
    try:
        with (opts.output/'poses.jsonl').open('x', encoding='utf-8') as poses, \
             (opts.output/'raw_ranges.jsonl').open('x', encoding='utf-8') as raw, \
             (opts.output/'raw_before_fault.jsonl').open('x', encoding='utf-8') as before_fault, \
             (opts.output/'truth.jsonl').open('x', encoding='utf-8') as truth, \
             (opts.output/'emission.jsonl').open('x', encoding='utf-8') as emission:
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
                if first is not None and opts.duration_s is not None and stamp-first > round(opts.duration_s*1e6):
                    reason = 'duration_complete'
                    break
                original, reference = source.sample(pose)
                message, fault = faults.apply(original)
                packet = StringMsg()
                packet.header.stamp.sec = stamp//1_000_000
                packet.header.stamp.nsec = stamp%1_000_000*1000
                packet.data = json.dumps(message, ensure_ascii=False, allow_nan=False)
                publish_error = None
                try:
                    published = False if fault['drop_requested'] else bool(publisher.publish(packet))
                except Exception as exc:
                    published, publish_error = None, exc
                if fault['drop_requested']:
                    counters['scheduled_drop'] += 1
                elif not published:
                    counters['publish_failed'] += 1
                else:
                    counters['published'] += 1
                    first_published = stamp if first_published is None else first_published
                    last_published = stamp
                if fault['active_fault_id'] is not None:
                    counters['fault:'+fault['active_fault_id']] += 1
                fault.update(publish_attempted=not fault['drop_requested'], published=published,
                             reason='publish_exception' if publish_error else 'scheduled_drop'
                             if fault['drop_requested'] else 'published' if published else 'publish_failed')
                if publish_error:
                    fault['error'] = type(publish_error).__name__+':'+str(publish_error)
                for stream, row in ((poses, pose), (raw, message), (before_fault, original),
                                    (truth, reference), (emission, fault)):
                    stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
                counters['recorded'] += 1
                first = stamp if first is None else first
                last = stamp
                if publish_error:
                    raise publish_error
                if opts.duration_s is not None and stamp-first >= round(opts.duration_s*1e6):
                    reason = 'duration_complete'
                    break
    except KeyboardInterrupt:
        pass
    except Exception:
        reason = 'recording_error'
        raise
    finally:
        cleanup_error = None
        try:
            node.unsubscribe(opts.topic)
        except Exception as exc:
            cleanup_error = type(exc).__name__+':'+str(exc)
        summary = dict(source='simulation', model=opts.model, input_topic=opts.topic,
                       output_topic=opts.output_topic, counters=dict(counters), stop_reason=reason,
                       start_sim_us=first, end_sim_us=last,
                       effective_rate_hz=((counters['recorded']-1)*1e6/(last-first)
                                          if first is not None and last > first else None),
                       publisher_success_rate_hz=((counters['published']-1)*1e6/(last_published-first_published)
                           if first_published is not None and last_published > first_published else None),
                       fault_plan_provided=opts.fault_plan is not None,
                       cleanup_error=cleanup_error,
                       external_output_allowed=False, flight_valid=False)
        (opts.output/'capture.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        index = {p.name:dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                 for p in sorted(opts.output.iterdir()) if p.is_file()}
        (opts.output/'input_index.json').write_text(json.dumps(index, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(summary, ensure_ascii=False))
    if (cleanup_error or counters['publish_failed'] or not counters['recorded']
            or reason not in ('user_stopped', 'duration_complete')):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
