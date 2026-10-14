"""Authorized, bounded GPS2 UART range probe. No parameter or flight writes.

Starts only a previously absent tfmini driver on the verified FMUv6C GPS2 port.
Stops the driver it started after recording. Existing I2C/flow/RC stay untouched.
"""
import argparse
import json
from pathlib import Path
import sys
import time


READS = ('tfmini status', 'gps status', 'listener distance_sensor -n 1')
START = 'tfmini start -d /dev/ttyS6 -R 25'
STOP = 'tfmini stop'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ops-directory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError('refusing_to_overwrite_evidence')
    from drone_uwb.integration.ros.local_transport import configure_local_dds
    configure_local_dds()
    sys.path.insert(0, str(args.ops_directory))
    from px4_mavros_readback import MavrosReadback
    from px4_sensor_readback import completed
    from mavros_msgs.msg import ExtendedState, Mavlink
    from rclpy.qos import qos_profile_sensor_data

    # Reuse the transport's per-command fresh DISARM and single-router checks.
    connection = MavrosReadback(2, READS + (START, STOP))
    ground = [None, float('-inf')]
    record = dict(parameter_writes=0, flight_commands=0, serial_port='/dev/ttyS6',
        queries=[], range_packets=[], driver_start_attempted=False, driver_stopped=False)

    def landed(msg):
        ground[:] = [msg.landed_state, time.monotonic()]

    def distance(msg):
        if msg.sysid != 1 or msg.compid != 1 or msg.msgid != 132:
            return
        packet = connection.parser.decode(connection.to_bytes(msg))
        record['range_packets'].append(dict(received_monotonic_s=time.monotonic(), **packet.to_dict()))

    connection.node.create_subscription(ExtendedState, '/mavros/extended_state', landed, qos_profile_sensor_data)
    connection.node.create_subscription(Mavlink, '/uas1/mavlink_source', distance, qos_profile_sensor_data)

    def query(command):
        while connection.recv_match(blocking=False):
            pass
        if command in (START, STOP) and (ground[0] != 1 or time.monotonic() - ground[1] > .5):
            raise RuntimeError('fresh_landed_state_required')
        payload = (command + '\n').encode('ascii')
        connection.serial_control_send(10, 6, 0, 0, len(payload), list(payload) + [0] * (70 - len(payload)))
        end, response = time.monotonic() + 5., ''
        while time.monotonic() < end:
            packet = connection.recv_match(blocking=True, timeout=.05)
            if packet is not None and packet.get_type() == 'SERIAL_CONTROL':
                response += bytes(packet.data[:packet.count]).decode('utf-8', errors='replace')
                if completed(response, command):
                    break
        record['queries'].append(dict(command=command, response=response, complete=completed(response, command)))
        if not completed(response, command):
            raise RuntimeError('incomplete_response')
        return response

    owned = False
    try:
        if connection.wait_heartbeat(timeout=8.) is None:
            raise RuntimeError('missing_heartbeat')
        if 'driver not running' not in query('tfmini status'):
            raise RuntimeError('tfmini_already_running_no_changes')
        gps = query('gps status')
        if '/dev/ttyS6' in gps:
            raise RuntimeError('gps2_is_owned_by_gps_no_changes')
        record['driver_start_attempted'] = True
        owned = True
        query(START)
        end = time.monotonic() + 15.
        while time.monotonic() < end:
            connection.recv_match(blocking=True, timeout=.05)
            if connection.state is not None and connection.state.armed:
                raise RuntimeError('armed_during_ground_probe_no_further_commands')
        query('tfmini status')
        query('listener distance_sensor -n 1')
    except Exception as error:
        record['error'] = str(error)
    finally:
        if owned:
            try:
                query(STOP)
                record['driver_stopped'] = 'driver not running' in query('tfmini status')
            except Exception as error:
                record['cleanup_error'] = str(error)
        connection.serial_control_send(10, 0, 0, 0, 0, [0] * 70)
        connection.close()
        args.output.write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in record.items() if k != 'range_packets'}, indent=2))
    print(json.dumps(dict(range_packets=len(record['range_packets']), last=record['range_packets'][-1:])))


if __name__ == '__main__':
    main()
