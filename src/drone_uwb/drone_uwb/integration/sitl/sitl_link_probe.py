"""Read PX4 SITL MAVLink link and parameter state without changing settings.

This tool does not arm, send odometry, set parameters or command flight.
PX4 encodes integer PARAM_VALUEs bytewise in the float field, so decoding
must preserve the original four bytes instead of numerically converting them.
"""
import argparse
import json
import math
import os
from pathlib import Path
import struct
import sys
import time


PARAMETERS = (
    'EKF2_EV_CTRL', 'EKF2_EV_NOISE_MD', 'EKF2_EV_DELAY',
    'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
    'EKF2_EVP_NOISE', 'EKF2_GPS_CTRL', 'EKF2_OF_CTRL',
    'EKF2_RNG_CTRL', 'EKF2_HGT_REF',
)


def decode_px4_param(name, raw_float, param_type, *, int32_type, real32_type,
                     allowed_names=PARAMETERS):
    """Decode PX4's bytewise MAVLink PARAM_VALUE, retaining raw evidence."""
    if name not in allowed_names or type(param_type) is not int:
        raise ValueError('unexpected_parameter')
    if type(raw_float) not in (int, float):
        raise ValueError('invalid_parameter_payload')
    if param_type == int32_type:
        # A valid integer such as -1 appears as NaN when its bits are viewed
        # as float; finite() must not run before the bytewise conversion.
        value = struct.unpack('<i', struct.pack('<f', raw_float))[0]
    elif param_type == real32_type:
        if not math.isfinite(raw_float):
            raise ValueError('invalid_parameter_payload')
        value = float(raw_float)
    else:
        raise ValueError('unsupported_parameter_type')
    return dict(name=name, value=value, mav_param_type=param_type,
                raw_float=raw_float if math.isfinite(raw_float) else None,
                raw_bits_hex=struct.pack('<f', raw_float).hex(),
                encoding='px4_bytewise')


def _name(message):
    value = message.param_id
    if isinstance(value, bytes):
        return value.decode('ascii', errors='strict').rstrip('\x00')
    if isinstance(value, str):
        return value.rstrip('\x00')
    raise ValueError('invalid_parameter_name')


def probe(connection, mavlink, *, duration_s, time_now=time.monotonic):
    """Collect a bounded read-only parameter snapshot from a live PX4 link."""
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('positive_duration_required')
    started = time_now()
    heartbeat = connection.wait_heartbeat(timeout=min(duration_s, 10.))
    if heartbeat is None:
        return dict(status='no_heartbeat', parameters={}, missing=list(PARAMETERS),
                    heartbeat=None, message_counts={})
    if heartbeat.get_srcSystem() != 1 or heartbeat.get_srcComponent() != 1:
        return dict(status='unexpected_autopilot_identity', parameters={},
                    missing=list(PARAMETERS), heartbeat=dict(system=heartbeat.get_srcSystem(),
                    component=heartbeat.get_srcComponent()), message_counts={})
    identity = dict(system=heartbeat.get_srcSystem(), component=heartbeat.get_srcComponent(),
                    autopilot=heartbeat.autopilot, vehicle_type=heartbeat.type)
    pending = set(PARAMETERS)
    values = {}
    counts = {'HEARTBEAT': 1, 'PARAM_VALUE': 0}
    last_request = -math.inf
    deadline = started+duration_s
    while pending and time_now() < deadline:
        now = time_now()
        if now-last_request >= 2.:
            for name in sorted(pending):
                connection.mav.param_request_read_send(1, 1, name.encode('ascii'), -1)
            last_request = now
        message = connection.recv_match(blocking=True, timeout=min(.25, max(0., deadline-time_now())))
        if message is None:
            continue
        kind = message.get_type()
        counts[kind] = counts.get(kind, 0)+1
        if kind != 'PARAM_VALUE' or message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
            continue
        try:
            name = _name(message)
            if name not in pending:
                continue
            decoded = decode_px4_param(name, message.param_value, message.param_type,
                                       int32_type=mavlink.MAV_PARAM_TYPE_INT32,
                                       real32_type=mavlink.MAV_PARAM_TYPE_REAL32)
        except (ValueError, UnicodeError, struct.error):
            counts['PARAM_VALUE_invalid'] = counts.get('PARAM_VALUE_invalid', 0)+1
            continue
        values[name] = decoded
        pending.remove(name)
    return dict(status='complete' if not pending else 'partial',
                parameters=values, missing=sorted(pending),
                heartbeat=identity, message_counts=counts,
                external_output_allowed=False)


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--listen-port', type=int, default=14540)
    parser.add_argument('--duration-s', type=float, default=15.)
    opts = parser.parse_args(args)
    if not 1024 <= opts.listen_port <= 65535 or not math.isfinite(opts.duration_s) or not 2. <= opts.duration_s <= 60.:
        parser.error('port 1024..65535 and duration 2..60 seconds required')
    if opts.output.exists():
        parser.error('output_already_exists')
    print('Importing pymavlink from this Python environment...', flush=True)
    started = time.monotonic()
    os.environ['MAVLINK20'] = '1'
    result = None
    try:
        from pymavlink import mavutil
        import_seconds = time.monotonic()-started
        print(f'pymavlink import completed in {import_seconds:.2f}s; listening on 127.0.0.1:{opts.listen_port}', flush=True)
        connection = mavutil.mavlink_connection(
            f'udpin:127.0.0.1:{opts.listen_port}', dialect='common',
            source_system=245, source_component=191, autoreconnect=False)
        try:
            result = probe(connection, mavutil.mavlink, duration_s=opts.duration_s)
        finally:
            connection.close()
    except (ImportError, OSError, ValueError) as exc:
        import_seconds = time.monotonic()-started
        result = dict(status='probe_error', reason=type(exc).__name__+':'+str(exc),
                      parameters={}, missing=list(PARAMETERS),
                      heartbeat=None, message_counts={})
    except KeyboardInterrupt:
        import_seconds = time.monotonic()-started
        result = dict(status='user_stopped', reason='keyboard_interrupt',
                      parameters={}, missing=list(PARAMETERS),
                      heartbeat=None, message_counts={})
    result.update(scope='px4_sitl_read_only', local_udp_port=opts.listen_port,
                  pymavlink_import_s=import_seconds,
                  observations_sent=False, parameters_changed=False,
                  flight_commands_sent=False)
    opts.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    if result['status'] != 'complete':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
