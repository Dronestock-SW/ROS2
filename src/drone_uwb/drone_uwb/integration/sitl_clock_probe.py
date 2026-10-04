"""Measure PX4 SITL boot-clock replies without changing flight state.

The probe sends only MAVLink TIMESYNC requests. Its output is a diagnostic
clock comparison, not a verified Gazebo-to-PX4 timestamp mapping.
"""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time


SOURCE_SYSTEM = 245
SOURCE_COMPONENT = 191
PX4_SYSTEM = 1
PX4_COMPONENT = 1


def timesync_sample(message, *, sent_ns, received_ns):
    """Validate one targeted PX4 reply and preserve its timing bounds."""
    if (message.get_type() != 'TIMESYNC'
            or message.get_srcSystem() != PX4_SYSTEM
            or message.get_srcComponent() != PX4_COMPONENT
            or type(message.tc1) is not int or message.tc1 <= 0
            or type(message.ts1) is not int or message.ts1 != sent_ns
            or type(received_ns) is not int or received_ns < sent_ns):
        raise ValueError('unmatched_timesync_reply')
    target_system = getattr(message, 'target_system', None)
    target_component = getattr(message, 'target_component', None)
    if ((target_system is None) != (target_component is None)
            or (target_system is not None
                and (target_system != SOURCE_SYSTEM
                     or target_component != SOURCE_COMPONENT))):
        raise ValueError('unmatched_timesync_target')
    rtt_ns = received_ns-sent_ns
    midpoint_ns = sent_ns+rtt_ns//2
    return dict(sent_host_monotonic_ns=sent_ns,
                received_host_monotonic_ns=received_ns,
                px4_boot_ns=message.tc1, round_trip_ns=rtt_ns,
                host_midpoint_monotonic_ns=midpoint_ns,
                px4_minus_host_midpoint_ns=message.tc1-midpoint_ns,
                half_round_trip_ns=rtt_ns/2,
                reply_target_verified=target_system is not None)


def probe(connection, *, duration_s=20., samples=8, interval_s=.4,
          now=time.monotonic, now_ns=time.monotonic_ns, pause=time.sleep):
    """Collect independent targeted replies; never declare mapping ready."""
    if (not math.isfinite(duration_s) or duration_s <= 0
            or type(samples) is not int or not 1 <= samples <= 30
            or not math.isfinite(interval_s) or not 0 <= interval_s <= 5):
        raise ValueError('invalid_probe_limits')
    started = now()
    heartbeat = connection.wait_heartbeat(timeout=min(duration_s, 10.))
    if heartbeat is None:
        return dict(status='no_heartbeat', samples=[], requested_samples=samples,
                    heartbeat=None, message_counts={}, clock_mapping_verified=False,
                    external_output_allowed=False)
    identity = dict(system=heartbeat.get_srcSystem(),
                    component=heartbeat.get_srcComponent())
    if identity != dict(system=PX4_SYSTEM, component=PX4_COMPONENT):
        return dict(status='unexpected_autopilot_identity', samples=[],
                    requested_samples=samples, heartbeat=identity,
                    message_counts={}, clock_mapping_verified=False,
                    external_output_allowed=False)
    identity.update(autopilot=heartbeat.autopilot, vehicle_type=heartbeat.type)
    records = []
    counts = {'HEARTBEAT': 1, 'TIMESYNC_sent': 0,
              'TIMESYNC_unmatched': 0, 'TIMESYNC_timeout': 0}
    deadline = started+duration_s
    for index in range(samples):
        if now() >= deadline:
            break
        sent_ns = now_ns()
        # Older pymavlink common dialects expose only tc1/ts1. The unique
        # echoed timestamp and PX4 source still identify this diagnostic reply.
        connection.mav.timesync_send(0, sent_ns)
        counts['TIMESYNC_sent'] += 1
        reply_deadline = min(deadline, now()+1.5)
        matched = False
        while now() < reply_deadline:
            message = connection.recv_match(
                blocking=True, timeout=min(.25, max(0., reply_deadline-now())))
            received_ns = now_ns()
            if message is None:
                continue
            kind = message.get_type()
            counts[kind] = counts.get(kind, 0)+1
            if kind != 'TIMESYNC':
                continue
            try:
                record = timesync_sample(message, sent_ns=sent_ns,
                                         received_ns=received_ns)
            except (ValueError, AttributeError, TypeError):
                counts['TIMESYNC_unmatched'] += 1
                continue
            records.append(record)
            matched = True
            break
        if not matched:
            counts['TIMESYNC_timeout'] += 1
        if index+1 < samples and interval_s and now() < deadline:
            pause(min(interval_s, max(0., deadline-now())))
    offsets = [record['px4_minus_host_midpoint_ns'] for record in records]
    ratios = []
    for earlier, later in zip(records, records[1:]):
        host_span = (later['host_midpoint_monotonic_ns']
                     - earlier['host_midpoint_monotonic_ns'])
        boot_span = later['px4_boot_ns']-earlier['px4_boot_ns']
        if host_span > 0:
            ratios.append(boot_span/host_span)
    return dict(status='complete' if len(records) == samples else 'partial',
                samples=records, requested_samples=samples,
                heartbeat=identity, message_counts=counts,
                median_offset_ns=statistics.median(offsets) if offsets else None,
                offset_span_ns=max(offsets)-min(offsets) if offsets else None,
                adjacent_clock_rate_ratios=ratios,
                all_reply_targets_verified=bool(records) and all(
                    record['reply_target_verified'] for record in records),
                clock_mapping_verified=False, external_output_allowed=False)


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--listen-port', type=int, default=14540)
    parser.add_argument('--duration-s', type=float, default=20.)
    parser.add_argument('--samples', type=int, default=8)
    opts = parser.parse_args(args)
    if (not 1024 <= opts.listen_port <= 65535
            or not math.isfinite(opts.duration_s) or not 2. <= opts.duration_s <= 60.
            or not 1 <= opts.samples <= 30):
        parser.error('port 1024..65535, duration 2..60s, samples 1..30 required')
    if opts.output.exists():
        parser.error('output_already_exists')
    print('Importing pymavlink from this Python environment...', flush=True)
    started = time.monotonic()
    os.environ['MAVLINK20'] = '1'
    result = None
    try:
        from pymavlink import mavutil
        import_seconds = time.monotonic()-started
        print(f'pymavlink import completed in {import_seconds:.2f}s; '
              f'listening on 127.0.0.1:{opts.listen_port}', flush=True)
        connection = mavutil.mavlink_connection(
            f'udpin:127.0.0.1:{opts.listen_port}', dialect='common',
            source_system=SOURCE_SYSTEM, source_component=SOURCE_COMPONENT,
            autoreconnect=False)
        try:
            result = probe(connection, duration_s=opts.duration_s,
                           samples=opts.samples)
        finally:
            connection.close()
    except (ImportError, OSError, ValueError, TypeError, AttributeError) as exc:
        import_seconds = time.monotonic()-started
        result = dict(status='probe_error', reason=type(exc).__name__+':'+str(exc),
                      samples=[], requested_samples=opts.samples,
                      heartbeat=None, message_counts={},
                      clock_mapping_verified=False, external_output_allowed=False)
    except KeyboardInterrupt:
        import_seconds = time.monotonic()-started
        result = dict(status='user_stopped', reason='keyboard_interrupt',
                      samples=[], requested_samples=opts.samples,
                      heartbeat=None, message_counts={},
                      clock_mapping_verified=False, external_output_allowed=False)
    result.update(scope='px4_sitl_clock_diagnostic', local_udp_port=opts.listen_port,
                  pymavlink_import_s=import_seconds, observations_sent=False,
                  parameters_changed=False, flight_commands_sent=False)
    opts.output.write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                      allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    if result['status'] != 'complete':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
