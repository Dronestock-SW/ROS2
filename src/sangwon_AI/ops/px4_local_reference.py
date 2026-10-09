"""Set one documented regional EKF origin on a stationary, disarmed Tag B.

This assigns a coordinate datum. It sends no GPS observation, flight command,
home-position command or parameter write. Default is a read-only preview.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import time

QUERIES = ('listener vehicle_local_position -n 1', 'listener actuator_armed -n 1',
           'listener failsafe_flags -n 1')


def validate_reference(value):
    for name, low, high in (('latitude', -90, 90), ('longitude', -180, 180),
                            ('altitude_amsl_m', -500, 9000)):
        number = value.get(name)
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or not low <= number <= high:
            raise ValueError('invalid_reference_' + name)
    if value.get('quality') not in ('regional_approximation', 'surveyed'):
        raise ValueError('reference_quality_required')
    if not isinstance(value.get('sources'), list) or not value['sources'] or not all(
            isinstance(s, str) and s.strip() for s in value['sources']):
        raise ValueError('reference_sources_required')
    if not isinstance(value.get('site'), str) or not value['site'].strip():
        raise ValueError('reference_site_required')
    return value


def field(text, name):
    matches = re.findall(r'^\s*' + re.escape(name) + r':\s+(\S+)', text, re.MULTILINE)
    if len(matches) != 1:
        raise ValueError('missing_or_ambiguous_' + name)
    return matches[0]


def validate_ground(local, armed, flags, require_unset=True):
    for name in ('armed', 'prearmed', 'termination', 'in_esc_calibration_mode'):
        if field(armed, name) != 'False':
            raise ValueError('not_disarmed_' + name)
    for name in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control'):
        if field(local, name) != 'True':
            raise ValueError('local_position_invalid_' + name)
    for name in ('manual_control_signal_lost', 'local_position_invalid', 'local_altitude_invalid'):
        if field(flags, name) != 'False':
            raise ValueError('failsafe_' + name)
    for name in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'heading'):
        if not math.isfinite(float(field(local, name))):
            raise ValueError('nonfinite_' + name)
    if math.sqrt(sum(float(field(local, n))**2 for n in ('vx', 'vy', 'vz'))) > .1:
        raise ValueError('not_stationary')
    if require_unset and (int(field(local, 'ref_timestamp')) != 0 or any(
            math.isfinite(float(field(local, n))) for n in ('ref_lat', 'ref_lon', 'ref_alt'))):
        raise ValueError('existing_origin_preserved')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    reference = validate_reference(json.loads(args.reference.read_text(encoding='utf-8')))
    if os.environ.get('ROS_DOMAIN_ID') != '2':
        raise RuntimeError('Tag B domain 2 required')
    os.environ['MAVLINK20'] = '1'
    from px4_mavros_readback import MavrosReadback
    from px4_sensor_readback import completed
    from drone_mission.writer_lock import WriterLock
    from mavros_msgs.msg import ExtendedState
    from rclpy.qos import qos_profile_sensor_data

    class Connection(MavrosReadback):
        def __init__(self):
            super().__init__(2, QUERIES)
            self.landed_at = float('-inf')
            self.landed = False
            self.node.create_subscription(ExtendedState, '/mavros/extended_state', self.extended, qos_profile_sensor_data)

        def extended(self, msg):
            self.landed = msg.landed_state == 1
            age = self.node.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
            self.landed_at = time.monotonic() if -.05 <= age <= 1.5 else float('-inf')

        def receive(self, msg):
            if msg.sysid == 1 and msg.compid == 1 and msg.msgid == 49:
                try:
                    self.queue.append(self.parser.decode(self.to_bytes(msg)))
                except Exception as exc:
                    self.decode_error = str(exc)
            else:
                super().receive(msg)

        def ground(self):
            now = time.monotonic()
            if (self.state is None or self.heartbeat is None or now-self.state_at > 2.5 or
                    now-self.heartbeat_at > 2.5 or not self.state.connected or self.state.armed or
                    self.heartbeat.base_mode & 128 or self.state.mode != 'POSCTL' or
                    not self.landed or now-self.landed_at > 1.5 or self.publisher.get_subscription_count() != 1):
                raise RuntimeError('fresh_disarmed_landed_POSCTL_required')

        def query(self, command):
            self.ground()
            data = (command+'\n').encode('ascii')
            self.serial_control_send(10, 6, 0, 0, len(data), list(data)+[0]*(70-len(data)))
            response, end = '', time.monotonic()+4
            while time.monotonic() < end:
                packet = self.recv_match(blocking=True, timeout=.1)
                self.ground()
                if packet and packet.get_type() == 'SERIAL_CONTROL':
                    response += bytes(packet.data[:packet.count]).decode('utf-8', errors='replace')
                    if completed(response, command):
                        return response
            raise RuntimeError('incomplete_query_' + command)

    result = dict(at_utc=datetime.now(timezone.utc).isoformat(), reference=reference,
                  applied=False, physical_flight=False, passed=False, before={}, after={})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve evidence before changing the FC; never overwrite an earlier run.
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    lock, connection = None, None
    try:
        lock = WriterLock(2)
        connection = Connection()
        if connection.wait_heartbeat(10) is None:
            raise RuntimeError('FC_heartbeat_missing')
        for _ in range(20):
            connection.recv_match(blocking=True, timeout=.1)
        for command in QUERIES:
            result['before'][command] = connection.query(command)
        before = result['before'][QUERIES[0]]
        validate_ground(before, result['before'][QUERIES[1]], result['before'][QUERIES[2]])
        if args.apply:
            connection.ground()
            packet = connection.encoder.set_gps_global_origin_encode(1, round(reference['latitude']*1e7),
                round(reference['longitude']*1e7), round(reference['altitude_amsl_m']*1000))
            packet.pack(connection.encoder)
            connection.encoder.seq = (connection.encoder.seq+1) % 256
            connection.publisher.publish(connection.to_ros(packet))
            result['applied'] = True
            end = time.monotonic()+5
            while time.monotonic() < end:
                packet = connection.recv_match(blocking=True, timeout=.1)
                connection.ground()
                if packet and packet.get_type() == 'GPS_GLOBAL_ORIGIN':
                    result['origin_reply'] = packet.to_dict()
            for command in QUERIES:
                result['after'][command] = connection.query(command)
            after = result['after'][QUERIES[0]]
            validate_ground(after, result['after'][QUERIES[1]], result['after'][QUERIES[2]], require_unset=False)
            if (field(after, 'xy_global') != 'True' or field(after, 'z_global') != 'True' or
                    abs(float(field(after, 'ref_lat'))-reference['latitude']) > 1e-6 or
                    abs(float(field(after, 'ref_lon'))-reference['longitude']) > 1e-6 or
                    abs(float(field(after, 'ref_alt'))-reference['altitude_amsl_m']) > .1 or
                    field(result['after'][QUERIES[2]], 'global_position_invalid') != 'False'):
                raise RuntimeError('global_origin_not_confirmed')
            result['local_delta_m'] = [float(field(after, n))-float(field(before, n)) for n in ('x','y','z')]
            if max(abs(n) for n in result['local_delta_m']) > .2:
                raise RuntimeError('local_position_changed_review_required')
        result['passed'] = True
    except Exception as exc:
        result['error'] = str(exc)
        raise
    finally:
        args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        if connection is not None:
            try:
                connection.serial_control_send(10, 0, 0, 0, 0, [0]*70)
            finally:
                connection.close()
        if lock is not None:
            lock.close()
    print(json.dumps(dict(applied=result['applied'], passed=result['passed'], output=str(args.output))))


if __name__ == '__main__':
    main()
