"""Ground-only installation of the two LiDAR2D parameters; no flight commands."""
import argparse
import json
import math
import re
from pathlib import Path
import time


TARGET = {'EKF2_LID2_AGE': 180.0, 'EKF2_LID2_EN': 1}
COUNTERS = {'COM_FLIGHT_UUID', 'LND_FLIGHT_T_LO', 'LND_FLIGHT_T_HI'}
MAG_OFFSETS = {'CAL_MAG0_XOFF', 'CAL_MAG0_YOFF', 'CAL_MAG0_ZOFF'}
IGNORED = COUNTERS | {'_HASH_CHECK'} | set(TARGET)


def value(raw):
    return raw['integer_value'] if raw['type'] == 2 else raw['double_value'] if raw['type'] == 3 else None


def differences(before, after):
    return {k: {'before': value(v), 'after': value(after.get(k, {'type': 0}))}
            for k, v in before.items() if value(v) is not None and k not in IGNORED
            and (value(after.get(k, {'type': 0})) is None
                 or not math.isclose(value(v), value(after[k]), rel_tol=1e-6, abs_tol=1e-6))}


def parse_version(response, query):
    from px4_sensor_readback import completed
    clean = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', response)
    values = [line.strip() for line in clean.splitlines() if re.fullmatch(r'[+-]?\d+', line.strip())]
    if not completed(response, query) or len(values) != 1 or values[0] not in ('1', '2'):
        raise RuntimeError('dedicated_lidar2d_firmware_missing')
    return int(values[0])


def read_version():
    # Read FC directly before trusting a reconnecting MAVROS parameter cache.
    from px4_mavros_readback import MavrosReadback
    from px4_sensor_readback import completed
    query = 'param show -q EKF2_LID2_VER'
    connection = MavrosReadback(2, (query,))
    try:
        heartbeat = connection.wait_heartbeat(timeout=8.)
        if heartbeat is None or heartbeat.base_mode & 128:
            raise RuntimeError('disarmed_FC_required')
        while connection.recv_match(blocking=False):
            pass
        payload = (query+'\n').encode('ascii')
        connection.serial_control_send(10, 6, 0, 0, len(payload), list(payload)+[0]*(70-len(payload)))
        end, response = time.monotonic()+4., ''
        while time.monotonic() < end:
            packet = connection.recv_match(blocking=True, timeout=.1)
            if packet is not None and packet.get_type() == 'SERIAL_CONTROL':
                response += bytes(packet.data[:packet.count]).decode('utf-8', errors='replace')
                if completed(response, query):
                    break
        version = parse_version(response, query)
        return {'command': query, 'response': response, 'value': version}
    finally:
        connection.serial_control_send(10, 0, 0, 0, 0, [0]*70)
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backup', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--preserve-current-mag-offsets', action='store_true',
        help='Record only MAG0 offset differences without restoring old cached values.')
    args = parser.parse_args()
    backup = json.loads(args.backup.read_text(encoding='utf-8'))
    args.output.mkdir(parents=True, exist_ok=False)
    from drone_uwb.integration.ros.local_transport import configure_local_dds
    configure_local_dds()
    version_readback = read_version()
    import rclpy
    from mavros_msgs.msg import State, ExtendedState
    from mavros_msgs.srv import ParamPull
    from rcl_interfaces.msg import Parameter, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init()
    node = rclpy.create_node('lidar2d_ground_parameter_setup')
    latest = {}
    def receive(key, msg):
        latest[key] = (msg, time.monotonic())
    subscriptions = [node.create_subscription(cls, topic, lambda m, k=key: receive(k, m), qos_profile_sensor_data)
        for key, cls, topic in [('state', State, '/mavros/state'), ('landed', ExtendedState, '/mavros/extended_state')]]
    get = node.create_client(GetParameters, '/mavros/param/get_parameters')
    listing = node.create_client(ListParameters, '/mavros/param/list_parameters')
    set_params = node.create_client(SetParameters, '/mavros/param/set_parameters')
    pull = node.create_client(ParamPull, '/mavros/param/pull')
    def ground():
        now = time.monotonic()
        return (all(k in latest and now-latest[k][1] <= 2.5 for k in ('state', 'landed'))
            and latest['state'][0].connected and not latest['state'][0].armed
            and latest['landed'][0].landed_state == 1)
    def call(client, request, timeout=10.):
        if not client.wait_for_service(timeout_sec=4.):
            raise RuntimeError('service_unavailable')
        if not ground():
            raise RuntimeError('fresh_disarmed_ground_required')
        future = client.call_async(request)
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not ground():
                raise RuntimeError('fresh_disarmed_ground_lost')
        if not future.done():
            raise RuntimeError('service_timeout_effect_unknown')
        return future.result()
    def read_all():
        names = sorted(call(listing, ListParameters.Request(depth=0)).result.names)
        result = {}
        for start in range(0, len(names), 80):
            batch = names[start:start+80]
            vals = call(get, GetParameters.Request(names=batch)).values
            result.update({k: message_to_ordereddict(v) for k, v in zip(batch, vals)})
        return result
    report = {'flight_actions': False, 'requested_changes': TARGET if args.apply else {},
        'writes': [], 'version_readback': version_readback}
    try:
        end = time.monotonic()+10.
        while not ground() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if not ground():
            raise RuntimeError('fresh_disarmed_ground_required')
        pulled = call(pull, ParamPull.Request(force_pull=True), timeout=60.)
        report['fresh_pull'] = message_to_ordereddict(pulled)
        if not pulled.success:
            raise RuntimeError('fresh_parameter_pull_failed')
        current = read_all()
        (args.output/'before.json').write_text(json.dumps(current, indent=2)+'\n', encoding='utf-8')
        report['preserved_parameter_differences'] = differences(backup, current)
        allowed = MAG_OFFSETS if args.preserve_current_mag_offsets else set()
        if allowed and (value(current.get('CAL_MAG0_ID', {'type': 0})) != value(backup.get('CAL_MAG0_ID', {'type': 0}))
                or any(value(current.get(k, {'type': 0})) is None
                       or not math.isfinite(value(current[k])) for k in MAG_OFFSETS)):
            raise RuntimeError('mag_sensor_identity_or_offset_invalid')
        report['retained_current_mag_offsets'] = {k: v for k, v in report['preserved_parameter_differences'].items() if k in allowed}
        if set(report['preserved_parameter_differences']) - allowed:
            raise RuntimeError('existing_parameters_changed_review_required')
        if args.apply:
            for name, target in TARGET.items():
                if not ground():
                    raise RuntimeError('fresh_disarmed_ground_required')
                if value(current.get(name, {'type': 0})) == target:
                    continue
                pv = ParameterValue(type=2, integer_value=target) if type(target) is int else ParameterValue(type=3, double_value=target)
                reply = call(set_params, SetParameters.Request(parameters=[Parameter(name=name, value=pv)]))
                report['writes'].append({'name': name, 'value': target, 'results': [message_to_ordereddict(r) for r in reply.results]})
                if len(reply.results) != 1 or not reply.results[0].successful:
                    raise RuntimeError('parameter_write_rejected')
        after = read_all()
        (args.output/'after.json').write_text(json.dumps(after, indent=2)+'\n', encoding='utf-8')
        rows = ['# Verified FC readback; MAVLink system 1 component 1']
        rows += [f'1\t1\t{k}\t{value(v)}\t{6 if v["type"] == 2 else 9}' for k, v in sorted(after.items()) if value(v) is not None and not k.startswith('_')]
        (args.output/'verified.params').write_text('\n'.join(rows)+'\n', encoding='utf-8')
        report['preserved_parameter_differences'] = differences(backup, after)
        report['unplanned_changes_during_setup'] = differences(current, after)
        report['lidar_parameters'] = {k: value(after.get(k, {'type': 0})) for k in (*TARGET, 'EKF2_AVEL_DELAY')}
        report['lidar_parameters']['EKF2_LID2_VER'] = version_readback['value']
        if (set(report['preserved_parameter_differences']) - allowed
                or report['unplanned_changes_during_setup']
                or any(report['lidar_parameters'][k] != v for k, v in TARGET.items())):
            raise RuntimeError('parameter_readback_mismatch')
        report['passed'] = True
    except Exception as exc:
        report.update(passed=False, error=str(exc))
        raise
    finally:
        (args.output/'receipt.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps({k: v for k, v in report.items() if k != 'preserved_parameter_differences'}), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
