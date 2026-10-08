#!/usr/bin/env python3
"""Observe existing Jetson ROS topics and the MAVROS parameter mirror.

No serial port is opened. No command, parameter write, stream-rate change,
launch, or mission assignment is sent. Source the existing ROS environment first.
"""

import argparse
import datetime
import json
import math
import os
import sys
import time


def safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(item) for item in value]
    return value


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--domain', type=int, choices=(1, 2, 99), required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error('--seconds must be from 1 to 30.')
    current_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if current_domain != args.domain:
        parser.error('Source the existing ROS environment and set the matching ROS_DOMAIN_ID.')

    try:
        from geographic_msgs.msg import GeoPointStamped
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from mavros_msgs.msg import EstimatorStatus, ExtendedState, RCIn, State, TimesyncStatus
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import GetParameters
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import BatteryState, Imu, Range
        from std_msgs.msg import String
    except ImportError as exc:
        print(json.dumps({'result': 'missing_ros_dependency', 'module': exc.name}))
        return 2

    rclpy.init(args=[])
    node = rclpy.create_node('dronestock_readonly_ground_observer')
    started = time.monotonic()
    checked_at = datetime.datetime.now(datetime.timezone.utc).isoformat()
    samples = {}
    subscriptions = []

    def receive(topic, message):
        row = samples.setdefault(topic, {'count': 0})
        row['count'] += 1
        row['received_monotonic'] = time.monotonic()
        row['latest'] = message_to_ordereddict(message)
        if isinstance(message, String):
            if len(message.data) <= 65536:
                try:
                    row['latest'] = json.loads(message.data)
                except (ValueError, TypeError):
                    pass
            else:
                row['latest'] = {'omitted': 'string_exceeds_65536_characters'}

    for topic, message_type in (
        ('/mavros/state', State),
        ('/mavros/rc/in', RCIn),
        ('/mavros/battery', BatteryState),
        ('/mavros/extended_state', ExtendedState),
        ('/mavros/estimator_status', EstimatorStatus),
        ('/mavros/local_position/odom', Odometry),
        ('/mavros/downward_0', Range),
        ('/mavros/imu/data', Imu),
        ('/mavros/timesync_status', TimesyncStatus),
        ('/uwb/received', String),
        ('/uwb_pose', PoseWithCovarianceStamped),
        ('/uwb/btf_status', String),
        ('/uwb/btf_pose', PoseWithCovarianceStamped),
        ('/uwb/btf_xyz', PoseStamped),
        ('/uwb/bridge_status', String),
        ('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped),
        ('/flight_state', String),
    ):
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda msg, key=topic: receive(key, msg),
            qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        GeoPointStamped, '/mavros/global_position/gp_origin',
        lambda msg: receive('/mavros/global_position/gp_origin', msg),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    parameter_names = [
        'EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD',
        'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
        'EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'MIS_TAKEOFF_ALT',
        'COM_DISARM_LAND', 'COM_DL_LOSS_T', 'NAV_DLL_ACT',
        'COM_RC_OVERRIDE', 'COM_RC_IN_MODE', 'COM_RC_STICK_OV',
        'RC_MAP_FLTMODE', 'RC_MAP_ARM_SW', 'RC_MAP_KILL_SW',
        'COM_RC_LOSS_T', 'NAV_RCL_ACT', 'COM_LOW_BAT_ACT',
    ]
    client = node.create_client(GetParameters, '/mavros/param/get_parameters')
    future = None
    parameters = None
    report = None
    try:
        while rclpy.ok() and time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if future is None and client.service_is_ready():
                future = client.call_async(GetParameters.Request(names=parameter_names))
            if future is not None and future.done() and parameters is None:
                if future.exception() is not None:
                    parameters = {'error': type(future.exception()).__name__}
                else:
                    parameters = {}
                    for name, value in zip(parameter_names, future.result().values):
                        field = {1: 'bool_value', 2: 'integer_value',
                                 3: 'double_value', 4: 'string_value'}.get(value.type)
                        parameters[name] = getattr(value, field) if field else None
        finished = time.monotonic()
        for row in samples.values():
            row['receipt_age_s'] = round(finished-row.pop('received_monotonic'), 3)
            header = row['latest'].get('header') if isinstance(row['latest'], dict) else None
            if header and 'stamp' in header:
                stamp = header['stamp']
                source_ns = stamp['sec']*1_000_000_000+stamp['nanosec']
                row['source_age_s'] = round((node.get_clock().now().nanoseconds-source_ns)/1e9, 3)
        report = {
            'checked_at': checked_at,
            'domain': args.domain,
            'duration_s': round(finished-started, 3),
            'result': 'observation_only',
            'interpretation': 'Topic receipt is not proof of PX4 fusion or flight readiness.',
            'nodes': sorted(namespace.rstrip('/')+'/'+name for name, namespace in node.get_node_names_and_namespaces()
                            if name != node.get_name()),
            'topics': dict(sorted(node.get_topic_names_and_types())),
            'topic_publishers': {
                subscription.topic_name: [info.node_namespace.rstrip('/')+'/'+info.node_name
                                          for info in node.get_publishers_info_by_topic(subscription.topic_name)]
                for subscription in subscriptions
            },
            'received': samples,
            'mavros_cached_parameters': parameters,
            'parameter_service_available': client.service_is_ready(),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(safe(report), ensure_ascii=False, allow_nan=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
