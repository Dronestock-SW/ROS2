"""Validate private C++ MAVROS transport reports; no physical readiness authority."""
import json
import math
import re

SCHEMA = 'sangwon-px4-health/1'
REPORT_TTL_S = 5
CHANNELS = ('state', 'extended_state', 'battery', 'rc')


def _number(value, low, high, nullable=False):
    return (nullable and value is None) or (type(value) in (int, float) and math.isfinite(value) and low <= value <= high)


def _integer(value, low, high):
    return type(value) is int and low <= value <= high


def _fields(channel, f):
    if not isinstance(f, dict):
        return False
    if channel == 'local_position':
        return (set(f) == {'frame_id', 'x_m', 'y_m', 'z_m', 'qw', 'qx', 'qy', 'qz'}
                and f['frame_id'] == 'map'
                and all(_number(f[k], -1e6, 1e6) for k in ('x_m', 'y_m', 'z_m'))
                and all(_number(f[k], -1, 1) for k in ('qw', 'qx', 'qy', 'qz'))
                and abs(sum(f[k]**2 for k in ('qw', 'qx', 'qy', 'qz'))-1) < .02)
    if channel == 'state':
        return (set(f) == {'connected', 'armed', 'guided', 'manual_input', 'mode', 'system_status'}
                and all(type(f[x]) is bool for x in ('connected', 'armed', 'guided', 'manual_input'))
                and isinstance(f['mode'], str) and len(f['mode']) <= 32
                and re.fullmatch(r'[A-Z0-9_.]{1,32}', f['mode']) is not None and _integer(f['system_status'], 0, 8))
    if channel == 'extended_state':
        return set(f) == {'landed_state', 'vtol_state'} and _integer(f['landed_state'], 0, 4) and _integer(f['vtol_state'], 0, 4)
    if channel == 'battery':
        return (set(f) == {'present', 'voltage_v', 'current_a', 'percentage', 'health'} and type(f['present']) is bool
                and _number(f['voltage_v'], 0, 1000) and _number(f['current_a'], -10000, 10000, True)
                and _number(f['percentage'], 0, 1, True) and _integer(f['health'], 0, 8))
    return (set(f) == {'channel_count', 'rssi', 'rssi_known'} and _integer(f['channel_count'], 0, 32)
            and _integer(f['rssi'], 0, 255) and type(f['rssi_known']) is bool and f['rssi_known'] == (f['rssi'] != 255))


def read_report(path):
    try:
        with path.open('rb') as handle:
            body = handle.read(65537)
        return json.loads(body) if len(body) <= 65536 else {}
    except (OSError, ValueError):
        return {}


def host_checks(report, boot, monotonic_now):
    """Optional OBS_* diagnostics only. Never map these observations to BP-C approval."""
    unavailable = [{'id': 'OBS_PX4_MONITOR', 'status': 'UNKNOWN', 'required_for_flight': False,
                    'detail': 'No current read-only PX4 transport report',
                    'operator_action': 'Check C++ observer service, ROS domain and MAVROS topics'}]
    try:
        if (report['schema_version'] != SCHEMA or report['scope'] != 'MAVROS_TRANSPORT_ONLY'
                or report['boot_id'] != boot or report['profile'] != 'HOST_OBSERVE'
                or any(report[x] is not False for x in ('can_start', 'flight_authority', 'physical_output_enabled',
                                                      'source_sample_time_verified', 'aircraft_identity_verified'))
                or report['observation_source'] != 'MAVROS_TOPICS_UNVERIFIED_AIRCRAFT'
                or report['display_ttl_s'] != REPORT_TTL_S or type(report['display_ttl_s']) is not int
                or not _number(report['generated_monotonic_s'], 0, 1e12)
                or not _integer(report['monitor_seq'], 1, 2**63-1)
                or not isinstance(report['monitor_session_id'], str)
                or re.fullmatch(r'[0-9a-f]{32}', report['monitor_session_id']) is None
                or not _integer(report['ros_domain_id'], 0, 232) or 170 <= report['ros_domain_id'] <= 199
                or report['runtime_code'] != 'OK'):
            return unavailable
        age = monotonic_now - report['generated_monotonic_s']
        if not math.isfinite(age) or not 0 <= age < REPORT_TTL_S:
            return unavailable
        streams = report['streams']
        if not isinstance(streams, dict) or set(streams) not in (set(CHANNELS), set(CHANNELS) | {'local_position'}):
            return unavailable
        checks = [{'id': 'OBS_PX4_MONITOR', 'status': 'PASS', 'required_for_flight': False,
                   'detail': f"Read-only C++ MAVROS subscriber; domain={report['ros_domain_id']}; FC identity/time unverified",
                   'operator_action': 'Bind actual aircraft ID/firmware/boot and validate PX4 source time before physical preflight'}]
        topics = set()
        for name in streams:
            s = streams[name]
            status, code, topic = s['status'], s['code'], s['topic']
            if (status not in ('LIVE', 'WARN', 'STALE', 'UNKNOWN') or not isinstance(code, str)
                    or not re.fullmatch(r'[A-Z0-9_]{1,96}', code) or not isinstance(topic, str)
                    or len(topic) > 192 or not re.fullmatch(r'(/[A-Za-z_][A-Za-z0-9_]*)+', topic)
                    or topic in topics or not _integer(s['publisher_count'], 0, 2**31-1)
                    or any(not _integer(s[x], 0, 2**63-1) for x in ('received_count', 'accepted_count', 'rejected_count'))
                    or s['accepted_count'] + s['rejected_count'] != s['received_count']
                    or not _integer(s['max_header_age_ms'], 10, 10000)
                    or type(s['transport_observation_valid']) is not bool):
                return unavailable
            topics.add(topic)
            detail = f'{topic}; received={s["received_count"]}; {code}'
            if status == 'LIVE':
                if (code != 'OK' or s['transport_observation_valid'] is not True or s['publisher_count'] != 1
                        or s['accepted_count'] < 1 or not _number(s['header_age_ms'], 0, 10000)
                        or not _fields(name, s['fields'])):
                    return unavailable
                if s['header_age_ms'] + age * 1000 >= s['max_header_age_ms']:
                    status, detail = 'STALE', f'{topic}; BRIDGE_OBSERVATION_EXPIRED'
                else:
                    f = s['fields']
                    if name == 'state':
                        detail += f'; MAVROS connected={f["connected"]}; mode={f["mode"]}; armed={f["armed"]}'
                        if not f['connected']:
                            status = 'WARN'
                    elif name == 'extended_state':
                        detail += f'; reported landed_state={f["landed_state"]}'
                        if f['landed_state'] == 0:
                            status = 'WARN'
                    elif name == 'battery':
                        pct = 'unknown' if f['percentage'] is None else str(round(100 * f['percentage'], 2))
                        detail += f'; reported present={f["present"]}; percentage={pct}'
                        if not f['present'] or f['percentage'] is None:
                            status = 'WARN'
                    elif name == 'local_position':
                        detail += f'; PX4 local ENU XYZ=({f["x_m"]}, {f["y_m"]}, {f["z_m"]}); warehouse alignment/fusion unverified'
                    else:
                        detail += f'; channels={f["channel_count"]}; takeover/RC link semantics unverified'
                        if not f['channel_count'] or not f['rssi_known']:
                            status = 'WARN'
            elif s['transport_observation_valid'] is not False or s['fields'] != {}:
                return unavailable
            checks.append({'id': 'OBS_PX4_' + name.upper(), 'status': 'PASS' if status == 'LIVE' else ('UNKNOWN' if status == 'UNKNOWN' else 'WARN'),
                           'required_for_flight': False, 'detail': detail,
                           'operator_action': 'Bridge transport observation only; validate aircraft binding, health/prearm and existing RC settings'})
        return checks
    except (KeyError, TypeError, ValueError, OverflowError):
        return unavailable
