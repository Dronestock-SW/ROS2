"""Read fixed PX4 sensor/EKF topics on a disarmed FC. No parameter or flight writes."""
import argparse
from datetime import datetime, timezone
import json
import os
import re
from pathlib import Path
import subprocess
import time


QUERIES = (
    'param show MIS_TAKEOFF_ALT', 'param show COM_TAKEOFF_ACT', 'param show EKF2_MAG_TYPE',
    'param show COM_RC_IN_MODE', 'param show COM_RC_OVERRIDE', 'param show COM_RC_STICK_OV',
    'param show EKF2_EV_DELAY', 'param show EKF2_EV_NOISE_MD',
    'param show EKF2_EV_POS_X', 'param show EKF2_EV_POS_Y', 'param show EKF2_EV_POS_Z',
    'param show EKF2_HGT_REF', 'param show COM_DISARM_LAND', 'param show NAV_RCL_ACT',
    'param show NAV_DLL_ACT', 'param show COM_DL_LOSS_T',
    'param show EKF2_OF_CTRL', 'param show EKF2_RNG_CTRL', 'param show EKF2_EV_CTRL',
    'param show SENS_FLOW_ROT', 'param show SENS_FLOW_SCALE',
    'listener sensor_optical_flow -n 1', 'listener vehicle_optical_flow -n 1',
    'listener distance_sensor -i 0 -n 1', 'listener distance_sensor -i 1 -n 1',
    'listener estimator_status_flags -n 1', 'listener estimator_aid_src_optical_flow -n 1',
    'listener estimator_aid_src_rng_hgt -n 1', 'listener vehicle_visual_odometry -n 1',
    'listener vehicle_local_position -n 1', 'listener vehicle_attitude -n 1',
    'listener sensor_combined -n 1',
    'listener vehicle_status -n 1', 'listener battery_status -n 1', 'listener input_rc -n 1',
)


def completed(text, query):
    terminal = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', text)
    return query in terminal and terminal.rstrip().endswith('nsh>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil
    owner = subprocess.run(['fuser', '/dev/pixhawk'], capture_output=True)
    if owner.returncode==0:
        raise RuntimeError('FC port already owned; do not open a second connection')
    result = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(), device='/dev/pixhawk',
                  outbound=['SERIAL_CONTROL fixed param show/listener queries only'],
                  physical_flight=False, fc_parameter_writes=False, queries=[])
    connection = mavutil.mavlink_connection('/dev/pixhawk', baud=921600,
                                          source_system=255, source_component=197)
    heartbeat = connection.wait_heartbeat(timeout=8)
    if heartbeat is None or heartbeat.get_srcSystem()!=1 or heartbeat.get_srcComponent()!=1:
        raise RuntimeError('Expected FC heartbeat missing')
    if heartbeat.base_mode & 128:
        raise RuntimeError('FC armed; readback requires a disarmed bench')
    result['initial_armed'] = False
    flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
    try:
        for query in QUERIES:
            while connection.recv_match(blocking=False):
                pass
            encoded = (query+'\n').encode('ascii')
            assert len(encoded)<=70
            connection.mav.serial_control_send(10, flags, 0, 0, len(encoded), list(encoded)+[0]*(70-len(encoded)))
            text, end = '', time.monotonic()+3
            while time.monotonic()<end:
                packet = connection.recv_match(blocking=True, timeout=.1)
                if packet is None:
                    continue
                if packet.get_type()=='HEARTBEAT' and packet.base_mode & 128:
                    raise RuntimeError('FC armed during readback; stopping')
                if packet.get_type()=='SERIAL_CONTROL':
                    text += bytes(packet.data[:packet.count]).decode('utf-8', errors='replace')
                    # A prompt before the echoed query is a startup/previous packet.
                    # Wait for the prompt *after* this query, or attribution shifts by one.
                    if completed(text, query):
                        break
            complete = completed(text, query)
            result['queries'].append(dict(command=query, response=text, prompt_seen=complete))
            if not complete:
                raise RuntimeError('Incomplete shell query; stop rather than misattribute the next response')
    finally:
        connection.mav.serial_control_send(10, 0, 0, 0, 0, [0]*70)
        connection.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(f'{len(result["queries"])} fixed readbacks saved: {args.output}', flush=True)


if __name__=='__main__':
    main()
