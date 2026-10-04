"""Record live Gazebo UWB, ToF and IMU; default to no PX4 output.

Start the existing virtual-range publisher separately. This consumer records
every sensor stream and each A/B/C/D/WLS decision for later independent review.
Explicit SITL settings enable clock training or gated observation transmission.
"""
import argparse
from collections import Counter, deque
import json
import hashlib
import math
import os
from pathlib import Path
import platform
from queue import Empty, Full, Queue
import sys
from threading import Event
import time

from drone_uwb.contracts.protocol import decode_line
from drone_uwb.integration.gazebo_sensors import attitude_record, tof_record, _stamp_us
from drone_uwb.integration.gazebo_clock import GazeboSimulationClock, simulation_clock_ns
from drone_uwb.integration.sitl_observer import SITLObserver, SITLObserverSettings, log_value
from drone_uwb.integration.sitl_odometry_contract import SITLOdometrySettings
from drone_uwb.processing.experiments.gazebo_live_shadow import GazeboLiveShadow
from drone_uwb.processing.runner import require_finite_json


def raw_record(message, topic):
    stamp = _stamp_us(message.header)
    row = decode_line(message.data)
    require_finite_json(row)
    if row.get('time_us') != stamp:
        raise ValueError('raw_transport_stamp_mismatch')
    return row


def with_callback_clock(row, started_ns, completed_ns):
    """Preserve the Gazebo sample time and bracket callback parsing on the host."""
    if (type(started_ns) is not int or type(completed_ns) is not int
            or started_ns <= 0 or completed_ns < started_ns):
        raise ValueError('invalid_host_callback_clock')
    return dict(row, host_clock_domain='wsl_monotonic_ns',
                host_callback_start_monotonic_ns=started_ns,
                host_callback_end_monotonic_ns=completed_ns)


def main(args=None, *, session_factory=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='dronestock_uwb')
    parser.add_argument('--model', default='dronestock_x500_0')
    parser.add_argument('--raw-topic', default='/dronestock/sim/uwb/ranges')
    parser.add_argument('--tof-topic')
    parser.add_argument('--imu-topic')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--height-profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=60.)
    parser.add_argument('--holdback-ms', type=float, default=50.)
    parser.add_argument('--idle-timeout-s', type=float, default=120.)
    parser.add_argument('--sitl-odometry', type=Path)
    parser.add_argument('--sitl-observer', type=Path)
    parser.add_argument('--sitl-mode', choices=('shadow', 'timesync', 'send'), default='shadow')
    opts = parser.parse_args(args)
    if (not all(math.isfinite(v) for v in (opts.duration_s, opts.holdback_ms, opts.idle_timeout_s))
            or opts.duration_s <= 0 or opts.holdback_ms < 0 or opts.holdback_ms > 500
            or opts.idle_timeout_s <= 0):
        parser.error('positive duration/idle timeout and holdback 0..500 ms required')
    config = json.loads(opts.config.read_text(encoding='utf-8'))
    profile = json.loads(opts.height_profile.read_text(encoding='utf-8'))
    processor = GazeboLiveShadow(config, profile)
    if bool(opts.sitl_odometry) != bool(opts.sitl_observer):
        parser.error('provide both --sitl-odometry and --sitl-observer')
    if opts.sitl_mode != 'shadow' and not opts.sitl_odometry:
        parser.error('SITL modes require both configuration files')
    observer = connection = session = None
    odometry_settings = observer_settings = None
    if opts.sitl_odometry:
        odometry_settings = SITLOdometrySettings(**json.loads(opts.sitl_odometry.read_text(encoding='utf-8')))
        observer_settings = SITLObserverSettings(**json.loads(opts.sitl_observer.read_text(encoding='utf-8')))
        if list(odometry_settings.tag_offset_body_flu_m) != list(config['tag_offset_body_flu_m']):
            parser.error('height_and_odometry_tag_mount_mismatch')
        if odometry_settings.sender_clock_domain != 'gazebo_sim_us':
            parser.error('live_observer_requires_gazebo_clock')
    try:
        from gz.transport13 import Node
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.clock_pb2 import Clock
    except ImportError as exc:
        parser.error('use /usr/bin/python3 with Gazebo Harmonic bindings: '+str(exc))
    prefix = f'/world/{opts.world}/model/{opts.model}/link'
    topics = dict(raw=opts.raw_topic,
                  tof=opts.tof_topic or prefix+'/lidar_sensor_link/sensor/lidar/scan',
                  imu=opts.imu_topic or prefix+'/base_link/sensor/imu_sensor/imu')
    queue, counters = Queue(maxsize=10000), Counter()
    clock, callback_fault = GazeboSimulationClock(), Event()

    def clock_record(message, topic):
        stamp = simulation_clock_ns(message)
        clock.update(stamp)
        return dict(type='gazebo_clock', topic=topic, time_ns=stamp,
                    clock_domain='gazebo_sim_ns')

    def accept(kind, convert):
        def callback(message):
            started_ns = time.monotonic_ns()
            try:
                row = convert(message, topics[kind])
                completed_ns = time.monotonic_ns()
                queue.put_nowait((kind, row, started_ns, completed_ns))
            except Full:
                counters[kind+'_queue_overflow'] += 1
                callback_fault.set()
            except (ValueError, TypeError, AttributeError):
                counters[kind+'_transport_invalid'] += 1
                callback_fault.set()
        return callback

    node = Node()
    for kind, message_type, convert in (
            ('raw', StringMsg, raw_record), ('tof', LaserScan, tof_record),
            ('imu', IMU, attitude_record)):
        if not node.subscribe(message_type, topics[kind], accept(kind, convert)):
            raise RuntimeError('gazebo_subscription_failed:'+topics[kind])
    if odometry_settings is not None:
        topics['clock'] = f'/world/{opts.world}/clock'
        if not node.subscribe(Clock, topics['clock'], accept('clock', clock_record)):
            raise RuntimeError('gazebo_subscription_failed:'+topics['clock'])

    opts.output.mkdir(parents=True, exist_ok=False)
    (opts.output/'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    (opts.output/'height_profile.json').write_text(json.dumps(profile, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if odometry_settings is not None:
        for source, name in ((opts.sitl_odometry, 'sitl_odometry.json'),
                             (opts.sitl_observer, 'sitl_observer.json')):
            (opts.output/name).write_text(source.read_text(encoding='utf-8'), encoding='utf-8')
    source_root = Path(__file__).resolve().parents[1]
    manifest = dict(schema=1, scope='gazebo_sitl_observer' if odometry_settings else 'gazebo_shadow',
        sitl_mode=opts.sitl_mode, topics=topics, duration_sim_s=opts.duration_s,
        holdback_wall_ms=opts.holdback_ms, idle_timeout_wall_s=opts.idle_timeout_s,
        argv=list(sys.argv[1:] if args is None else args),
        python=sys.version, platform=platform.platform(),
        source_sha256={str(path.relative_to(source_root)): hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in sorted(source_root.rglob('*.py'))},
        input_config_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in (opts.config, opts.height_profile, opts.sitl_odometry,
                                          opts.sitl_observer) if path is not None},
        wire_unknown_values='NaN', json_unknown_values='null',
        flight_valid=False, fusion_verified=False)
    (opts.output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    first_sim_us = last_sim_us = None
    last_raw_wall_ns = time.monotonic_ns()
    pending = deque()
    stop_reason = 'duration_complete'
    print('Live UWB recording; SITL mode: '+opts.sitl_mode+'. Ctrl+C saves this run.', flush=True)
    try:
        with (opts.output/'raw_ranges.jsonl').open('x', encoding='utf-8') as raw_stream, \
             (opts.output/'tof.jsonl').open('x', encoding='utf-8') as tof_stream, \
             (opts.output/'attitude.jsonl').open('x', encoding='utf-8') as imu_stream, \
             (opts.output/'results.jsonl').open('x', encoding='utf-8') as results, \
             (opts.output/'failures.jsonl').open('x', encoding='utf-8') as failures, \
             (opts.output/'sitl_events.jsonl').open('x', encoding='utf-8') as sitl_events, \
             (opts.output/'clock.jsonl').open('x', encoding='utf-8') as clock_stream:
            streams = dict(raw=raw_stream, tof=tof_stream, imu=imu_stream)
            streams['clock'] = clock_stream

            if odometry_settings is not None:
                dialect = None
                if opts.sitl_mode != 'shadow':
                    print('Importing pymavlink; opening loopback SITL UDP 14540.', flush=True)
                    os.environ['MAVLINK20'] = '1'
                    from pymavlink import mavutil
                    dialect = mavutil.mavlink
                    connection = mavutil.mavlink_connection('udpin:127.0.0.1:14540', dialect='common',
                        source_system=245, source_component=191, autoreconnect=False)
                    # mavlink_connection may replace the default dialect.
                    dialect = mavutil.mavlink
                observer = SITLObserver(odometry_settings, observer_settings, clock,
                    mode=opts.sitl_mode, connection=connection, dialect=dialect)
            if session_factory is not None:
                if observer is None or connection is None:
                    raise ValueError('navigation_requires_a_live_sitl_connection')
                session = session_factory(observer, opts.output)
                observer.message_listener = session.on_message

            def poll_sitl():
                drained = True
                if observer is not None:
                    for event in observer.poll_link():
                        sitl_events.write(json.dumps(log_value(event), ensure_ascii=False, allow_nan=False)+'\n')
                        if event['type'] == 'receive_backlog':
                            drained = False
                return drained

            def flush_ready(force=False):
                nonlocal last_sim_us
                while pending and (force or time.monotonic_ns()-pending[0][2]
                                   >= round(opts.holdback_ms*1e6)):
                    row, started_ns, completed_ns = pending.popleft()
                    try:
                        result = processor.process_cycle(row)
                    except (ValueError, TypeError, KeyError) as exc:
                        counters['raw_rejected'] += 1
                        failure = dict(seq=row.get('seq'), time_us=row.get('time_us'),
                                       reason=str(exc), external_output_allowed=False)
                        failures.write(json.dumps(failure, ensure_ascii=False, allow_nan=False)+'\n')
                        if str(exc) == 'non_increasing_cycle_time_or_sequence':
                            return 'simulation_time_or_sequence_reversed'
                    else:
                        result['host_clock_domain'] = 'wsl_monotonic_ns'
                        result['host_callback_start_monotonic_ns'] = started_ns
                        result['host_callback_end_monotonic_ns'] = completed_ns
                        result['queue_delay_wall_ms'] = (time.monotonic_ns()-completed_ns)/1e6
                        results.write(json.dumps(result, ensure_ascii=False, allow_nan=False)+'\n')
                        counters['processed'] += 1
                        if observer is not None:
                            drained = poll_sitl()
                            if callback_fault.is_set():
                                observer.suspend('gazebo_callback_error_or_overflow')
                            event = observer.observe(result, raw_received_host_ns=started_ns,
                                                     mavlink_drained=drained)
                            sitl_events.write(json.dumps(log_value(event), ensure_ascii=False, allow_nan=False)+'\n')
                            counters['odometry_sent' if event['transmitted'] else 'odometry_not_sent'] += 1
                            counters['odometry_reason:'+event['reason']] += 1
                            if session is not None and not force:
                                session.on_observation(event)
                        for name, model in result['models'].items():
                            counters[name+'_ok' if model['ok'] else name+'_failed'] += 1
                        last_sim_us = row['time_us']
                return None

            try:
                while True:
                    poll_sitl()
                    if observer is not None and (callback_fault.is_set() or observer.fault):
                        stop_reason = observer.fault or 'gazebo_callback_error_or_overflow'
                        observer.suspend(stop_reason)
                        break
                    if session is not None:
                        session.tick()
                    if (time.monotonic_ns()-last_raw_wall_ns)/1e9 > opts.idle_timeout_s:
                        stop_reason = 'no_advancing_raw_timeout'
                        break
                    try:
                        first_item = queue.get(timeout=.05)
                    except Empty:
                        first_item = None
                    batch = [] if first_item is None else [first_item]
                    for _ in range(4095 if batch else 0):
                        try:
                            batch.append(queue.get_nowait())
                        except Empty:
                            break
                    # Drain callbacks already queued before deciding that a RAW
                    # has no matching sensor sample at its measurement time.
                    for kind, row, started_ns, completed_ns in batch:
                        recorded_row = with_callback_clock(row, started_ns, completed_ns)
                        streams[kind].write(json.dumps(recorded_row, ensure_ascii=False, allow_nan=False)+'\n')
                        counters[kind+'_recorded'] += 1
                        if kind == 'raw':
                            if first_sim_us is None:
                                first_sim_us = row['time_us']
                            last_raw_wall_ns = completed_ns
                            pending.append((row, started_ns, completed_ns))
                        elif kind != 'clock':
                            try:
                                processor.add_sensor(row)
                            except (ValueError, TypeError, KeyError) as exc:
                                counters[kind+'_rejected'] += 1
                                failures.write(json.dumps(dict(type=kind, time_us=row.get('time_us'),
                                        reason=str(exc)), ensure_ascii=False, allow_nan=False)+'\n')
                                if observer is not None:
                                    observer.suspend('sensor_stream_invalid:'+str(exc))
                    reversal = flush_ready()
                    if reversal:
                        stop_reason = reversal
                        break
                    if (first_sim_us is not None and last_sim_us is not None
                            and last_sim_us-first_sim_us >= round(opts.duration_s*1e6)):
                        break
            except KeyboardInterrupt:
                stop_reason = 'user_stopped'
            if observer is not None:
                # Stopping a capture must never send a queued old observation.
                observer.suspend('capture_stopped')
            if stop_reason != 'simulation_time_or_sequence_reversed':
                reversal = flush_ready(force=True)
                if reversal:
                    stop_reason = reversal
    except Exception as exc:
        stop_reason = 'recording_error'
        (opts.output/'terminal_error.json').write_text(json.dumps(dict(
            type=type(exc).__name__, reason=str(exc), observer_fault=observer.fault if observer else None),
            ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        raise
    finally:
        cleanup_errors = []
        if session is not None:
            for action in (lambda: session.stop(stop_reason), session.close):
                try:
                    action()
                except Exception as exc:
                    cleanup_errors.append(type(exc).__name__+':'+str(exc))
        if connection is not None:
            try:
                connection.close()
            except Exception as exc:
                cleanup_errors.append(type(exc).__name__+':'+str(exc))
        for topic in topics.values():
            try:
                node.unsubscribe(topic)
            except Exception as exc:
                cleanup_errors.append(type(exc).__name__+':'+str(exc))
        summary = dict(source='gazebo_sitl_observer_live' if observer else 'gazebo_shadow_live', topics=topics,
                       config_path=str(opts.config), height_profile_path=str(opts.height_profile),
                       first_sim_us=first_sim_us, last_sim_us=last_sim_us,
                       counters=dict(counters), stop_reason=stop_reason,
                       observation_send_requested=opts.sitl_mode == 'send', flight_valid=False,
                       observations_sent=counters['odometry_sent'],
                       external_output_allowed=bool(observer and opts.sitl_mode == 'send'
                                                    and observer.settings.enabled),
                       sitl_mode=opts.sitl_mode, fusion_verified=False,
                       truth_used=False, cleanup_errors=cleanup_errors)
        (opts.output/'capture.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        index = {path.name: dict(bytes=path.stat().st_size,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                 for path in sorted(opts.output.iterdir()) if path.is_file()}
        (opts.output/'input_index.json').write_text(json.dumps(index, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    if cleanup_errors or not counters['processed'] or stop_reason not in ('duration_complete', 'user_stopped'):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
