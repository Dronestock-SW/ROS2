"""Read ROS sensors, parameter cache and FC identity without flight actions."""
import argparse
import json
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=25.)
    parser.add_argument('--all-parameters', action='store_true')
    parser.add_argument('--same-user-dds', action='store_true',
        help='Use the existing DDS transport when running as the MAVROS service user.')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from drone_uwb.integration.ros.local_transport import configure_local_dds
    if not args.same_user_dds:
        configure_local_dds()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from rcl_interfaces.srv import GetParameters, ListParameters
    from mavros_msgs.msg import Mavlink, State, ExtendedState, TimesyncStatus
    from mavros_msgs.srv import CommandLong
    from sensor_msgs.msg import Imu, PointCloud, Range
    from rosidl_runtime_py.convert import message_to_ordereddict
    from std_msgs.msg import String
    rclpy.init()
    node = Node('lidar_hardware_probe')
    stream = (args.output/'topics.jsonl').open('x', encoding='utf-8')
    counts, last = {}, {}
    def receive(topic, msg):
        row = message_to_ordereddict(msg)
        counts[topic] = counts.get(topic, 0)+1
        last[topic] = row
        stream.write(json.dumps(dict(topic=topic, received_ros_ns=node.get_clock().now().nanoseconds,
            monotonic_ns=time.monotonic_ns(), message=row))+'\n')
    topics = {'/mavros/state': State, '/mavros/extended_state': ExtendedState,
        '/mavros/timesync_status': TimesyncStatus, '/mavros/imu/data': Imu,
        '/mavros/imu/data_raw': Imu, '/point_cloud': PointCloud,
        '/mavros/downward_0': Range, '/uas1/mavlink_source': Mavlink,
        '/uwb/local_odometry_status': String, '/uwb/btf_status': String}
    subscriptions = [node.create_subscription(cls, name, lambda msg, n=name: receive(n,msg), qos)
        for name, cls in topics.items()]
    names = ['EKF2_LID2_VER', 'EKF2_LID2_EN', 'EKF2_LID2_AGE', 'EKF2_AVEL_DELAY', 'EKF2_EV_CTRL',
        'EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'EKF2_OF_CTRL', 'SYS_AUTOSTART',
        'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z']
    client = node.create_client(GetParameters, '/mavros/param/get_parameters')
    command = node.create_client(CommandLong, '/mavros/cmd/command')
    future = identity = None
    start = time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.025)
        if future is None and client.service_is_ready():
            future = client.call_async(GetParameters.Request(names=names))
        state = last.get('/mavros/state', {})
        if identity is None and state.get('connected') and state.get('armed') is False and command.service_is_ready():
            # REQUEST_MESSAGE(AUTOPILOT_VERSION) only; never writes parameters.
            identity = command.call_async(CommandLong.Request(command=512, param1=148.))
    params = {}
    if future is not None and future.done() and future.exception() is None:
        params = {name: message_to_ordereddict(value) for name,value in zip(names,future.result().values)}
    summary = dict(counts=counts, parameters=params, last_messages=last,
        nodes=node.get_node_names_and_namespaces(),
        publishers={name:[dict(name=i.node_name, namespace=i.node_namespace) for i in node.get_publishers_info_by_topic(name)] for name in topics},
        raw_writers=[i.node_name for i in node.get_publishers_info_by_topic('/uas1/mavlink_sink')],
        flight_actions=False, parameter_writes=False)
    if args.all_parameters:
        listing = node.create_client(ListParameters, '/mavros/param/list_parameters')
        backup = {}
        if not listing.wait_for_service(timeout_sec=3.):
            raise RuntimeError('parameter_listing_unavailable')
        request = listing.call_async(ListParameters.Request(depth=0))
        rclpy.spin_until_future_complete(node,request,timeout_sec=8.)
        if not request.done() or request.exception() is not None:
            raise RuntimeError('parameter_listing_incomplete')
        all_names = sorted(request.result().result.names)
        for start in range(0,len(all_names),80):
            batch = all_names[start:start+80]
            query = client.call_async(GetParameters.Request(names=batch))
            rclpy.spin_until_future_complete(node,query,timeout_sec=5.)
            if not query.done() or query.exception() is not None:
                raise RuntimeError('parameter_backup_incomplete')
            backup.update({n:message_to_ordereddict(v) for n,v in zip(batch,query.result().values)})
        summary['parameter_backup_count'] = len(backup)
        (args.output/'parameter_backup.json').write_text(json.dumps(backup,indent=2)+'\n',encoding='utf-8')
    stream.close()
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(counts=counts, parameters=params, state=last.get('/mavros/state'), raw_writers=summary['raw_writers'])))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
