"""Fixed, DISARM-only readback of version and dedicated LiDAR admission/fusion."""
import argparse
import json
from pathlib import Path
import sys
import time


QUERIES = ('ver all', 'param show -q EKF2_LID2_VER', 'param show EKF2_LID2_EN',
    'param show EKF2_LID2_AGE', 'param show EKF2_AVEL_DELAY',
    'param show EKF2_EV_POS_X', 'param show EKF2_EV_POS_Y', 'param show EKF2_EV_POS_Z',
    'listener lidar_velocity2d_status -n 1', 'listener estimator_aid_src_aux_vel -n 1',
    'listener estimator_status_flags -n 1', 'listener vehicle_local_position -n 1')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ops-directory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--same-user-dds', action='store_true',
        help='Use the existing DDS transport when running as the MAVROS service user.')
    args = p.parse_args()
    from drone_uwb.integration.ros.local_transport import configure_local_dds
    if not args.same_user_dds:
        configure_local_dds()
    sys.path.insert(0,str(args.ops_directory.resolve()))
    from px4_mavros_readback import MavrosReadback
    from px4_sensor_readback import completed
    connection = MavrosReadback(2,QUERIES)
    result = dict(flight_actions=False,parameter_writes=False,queries=[])
    try:
        heartbeat = connection.wait_heartbeat(timeout=8.)
        if heartbeat is None or heartbeat.base_mode&128:
            raise RuntimeError('disarmed_FC_required')
        for query in QUERIES:
            while connection.recv_match(blocking=False):
                pass
            payload = (query+'\n').encode('ascii')
            connection.serial_control_send(10,6,0,0,len(payload),list(payload)+[0]*(70-len(payload)))
            end, text = time.monotonic()+4., ''
            while time.monotonic()<end:
                packet = connection.recv_match(blocking=True,timeout=.1)
                if packet is None:
                    continue
                if packet.get_type() == 'HEARTBEAT' and packet.base_mode&128:
                    raise RuntimeError('FC_armed_during_readback')
                if packet.get_type() == 'SERIAL_CONTROL':
                    text += bytes(packet.data[:packet.count]).decode('utf-8',errors='replace')
                    if completed(text,query):
                        break
            result['queries'].append(dict(command=query,response=text,complete=completed(text,query)))
            if not completed(text,query):
                raise RuntimeError('incomplete_fixed_readback')
    finally:
        connection.serial_control_send(10,0,0,0,0,[0]*70)
        connection.close()
        with args.output.open('x',encoding='utf-8') as output:
            json.dump(result,output,ensure_ascii=False,indent=2)
            output.write('\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
