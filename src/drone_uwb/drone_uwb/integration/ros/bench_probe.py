"""Read-only ROS topic capture; no publishers, parameter writes or commands."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu, LaserScan, Range
from mavros_msgs.msg import State, ExtendedState, TimesyncStatus, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--body-x-along-a1-a3', action='store_true')
    args = parser.parse_args()
    target = Path(args.output)
    target.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('uwb_readonly_bench_probe')
    records = (target / 'topics.jsonl').open('x', encoding='utf-8')
    times, last, yaw = {}, {}, []
    start = time.monotonic_ns()
    def receive(name, msg):
        now = time.monotonic_ns()
        value = message_to_ordereddict(msg)
        times.setdefault(name, []).append(now)
        last[name] = value
        records.write(json.dumps({'topic': name, 'monotonic_ns': now, 'message': value},
                                 ensure_ascii=False) + '\n')
        if name == '/mavros/imu/data' and msg.orientation_covariance[0] >= 0:
            q = msg.orientation
            angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            yaw.append(angle)
    topics = {
        '/mavros/state': State, '/mavros/imu/data_raw': Imu,
        '/mavros/imu/data': Imu, '/mavros/local_position/odom': Odometry,
        '/mavros/local_position/pose': PoseStamped,
        '/mavros/extended_state': ExtendedState, '/mavros/timesync_status': TimesyncStatus,
        '/mavros/rc/in': RCIn,
        '/mavros/downward_0': Range, '/mavros/downward_1': Range,
        '/scan': LaserScan, '/uwb_pose': PoseWithCovarianceStamped,
        '/uwb/btf_pose': PoseWithCovarianceStamped, '/uwb/btf_status': String,
        '/uwb/status': String, '/uwb/bridge_status': String,
        '/mavros/vision_pose/pose_cov': PoseWithCovarianceStamped,
    }
    subscriptions = [node.create_subscription(cls, name, lambda msg, n=name: receive(n, msg),
                                              qos_profile_sensor_data)
                     for name, cls in topics.items()]
    names = ['EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD',
             'EKF2_EVP_NOISE', 'EKF2_MAG_TYPE', 'SENS_BOARD_ROT',
             'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
             'EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'COM_RC_OVERRIDE', 'COM_RC_STICK_OV',
             'COM_RC_IN_MODE', 'COM_RCL_EXCEPT', 'COM_RC_LOSS_T', 'COM_OF_LOSS_T',
             'COM_OBL_RC_ACT', 'COM_FAIL_ACT_T', 'COM_TAKEOFF_ACT', 'MIS_TAKEOFF_ALT',
             'RC_MAP_FLTMODE', 'COM_FLTMODE1', 'COM_FLTMODE2', 'COM_FLTMODE3',
             'COM_FLTMODE4', 'COM_FLTMODE5', 'COM_FLTMODE6', 'RC_MAP_OFFB_SW',
             'RC_MAP_RETURN_SW', 'RC_MAP_KILL_SW', 'RC_MAP_ARM_SW',
             'RC_MAP_ROLL', 'RC_MAP_PITCH', 'RC_MAP_YAW', 'RC_MAP_THROTTLE']
    client = node.create_client(GetParameters, '/mavros/param/get_parameters')
    future = None
    params = {}
    requested_at = float('-inf')
    try:
        while rclpy.ok() and (time.monotonic_ns() - start) / 1e9 < args.seconds:
            now = time.monotonic()
            if future is not None and future.done():
                if future.exception() is None:
                    params = {name: val.integer_value if val.type == 2 else
                              val.double_value if val.type == 3 else None
                              for name, val in zip(names, future.result().values)}
                future = None
            if future is not None and now-requested_at > 2:
                client.remove_pending_request(future)
                future = None
            if future is None and now-requested_at >= 1 and client.service_is_ready():
                future = client.call_async(GetParameters.Request(names=names))
                requested_at = now
            rclpy.spin_once(node, timeout_sec=0.05)
        seconds = (time.monotonic_ns() - start) / 1e9
        summary = {'utc': datetime.now(timezone.utc).isoformat(), 'duration_s': seconds,
                   'body_x_along_a1_a3_user_confirmed': args.body_x_along_a1_a3,
                   'parameters': params, 'topics': {}, 'last_messages': last,
                   'nodes': node.get_node_names_and_namespaces()}
        # Report XYZ only from the same fresh PX4 message. Never splice UWB XY
        # and ToF distance into a fictitious vehicle pose or a flight target.
        pose = last.get('/mavros/local_position/pose')
        summary['px4_xyz'] = {'valid': False, 'xyz_m': None, 'frame': 'px4_local_enu',
                              'fusion_verified': False, 'warehouse_aligned': False}
        if pose:
            stamp = pose['header']['stamp']
            ns = stamp['sec']*1_000_000_000+stamp['nanosec']
            age = (node.get_clock().now().nanoseconds-ns)/1e9
            p = pose['pose']['position']
            xyz = [p[k] for k in ('x', 'y', 'z')]
            valid = (ns > 0 and pose['header']['frame_id'] == 'map' and 0 <= age <= .2
                     and all(math.isfinite(v) for v in xyz)
                     and node.count_publishers('/mavros/local_position/pose') == 1)
            summary['px4_xyz'].update(valid=valid, xyz_m=xyz if valid else None,
                                     source_stamp_ns=ns, age_s=age)
        for name in topics:
            t = np.array(times.get(name, []), dtype=np.int64)
            gaps = np.diff(t) / 1e9
            summary['topics'][name] = {
                'count': len(t), 'rate_over_window_hz': len(t)/seconds,
                'rate_between_first_last_hz': (len(t)-1)/((t[-1]-t[0])/1e9) if len(t)>1 else None,
                'median_gap_ms': float(np.median(gaps)*1000) if len(gaps) else None,
                'max_gap_ms': float(max(gaps)*1000) if len(gaps) else None,
                'gaps_over_twice_median': int(sum(gaps > 2*np.median(gaps))) if len(gaps) else 0,
            }
        if yaw:
            mean = math.atan2(np.mean(np.sin(yaw)), np.mean(np.cos(yaw)))
            deviations = np.angle(np.exp(1j*(np.array(yaw)-mean)))
            summary['heading'] = {
                'count': len(yaw), 'mean_yaw_enu_deg': math.degrees(mean),
                'stddev_deg': float(np.std(np.degrees(deviations))),
                'span_deg': float(np.ptp(np.degrees(deviations))),
                'provisional_uwb_to_enu_rotation_deg':
                    (math.degrees(mean)-90+180)%360-180 if args.body_x_along_a1_a3 else None,
                'independently_verified': False,
            }
        (target / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)
                                              + '\n', encoding='utf-8')
        print(json.dumps({k: v for k, v in summary.items() if k != 'last_messages'},
                         ensure_ascii=False, indent=2))
    finally:
        records.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
