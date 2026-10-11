#!/usr/bin/env python3
"""Read-only live UWB diagnosis; no sensor publishers or FC writes."""
import json
import os
import tempfile
import time
import uuid
from collections import Counter, deque
from datetime import datetime, timezone

XML = '''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>uwb_diag_udp</transport_id>
<type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
</transport_descriptor></transport_descriptors>
<participant profile_name="uwb_diag_participant" is_default_profile="true"><rtps>
<prefix>01.0f.a1.b2.c3.d4.e5.f6.11.22.66.88</prefix>
<userTransports><transport_id>uwb_diag_udp</transport_id></userTransports>
<useBuiltinTransports>false</useBuiltinTransports>
<builtin><initialPeersList><locator><udpv4><address>127.0.0.1</address></udpv4>
</locator></initialPeersList></builtin></rtps></participant>
<subscriber profile_name="uwb_diag_subscriber" is_default_profile="true">
<qos><data_sharing><kind>OFF</kind></data_sharing></qos>
<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy></subscriber>
<publisher profile_name="uwb_diag_publisher" is_default_profile="true">
<qos><data_sharing><kind>OFF</kind></data_sharing></qos>
<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy></publisher>
</profiles>'''

def main():
    # Give each diagnostic its own participant identity, including repeat runs.
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                     suffix='.xml', delete=False) as profile:
        profile.write(XML.replace('01.0f.a1.b2.c3.d4.e5.f6.11.22.66.88',
            '.'.join(f'{value:02x}' for value in uuid.uuid4().bytes[:12])))
    os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=profile.name,
                      RMW_FASTRTPS_USE_QOS_FROM_XML='1')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from mavros_msgs.msg import State, ExtendedState, RCIn, TimesyncStatus
    from sensor_msgs.msg import Imu, Range
    from nav_msgs.msg import Odometry
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init()
    node = Node('uwb_live_readonly_diagnostic')
    counts, reasons, last, times = Counter(), Counter(), {}, {}
    decisions, raw = deque(maxlen=120), deque(maxlen=120)
    def receive(topic, msg):
        now = time.monotonic()
        counts[topic] += 1
        times.setdefault(topic, []).append(now)
        if isinstance(msg, String):
            try: row = json.loads(msg.data)
            except ValueError: row = {'invalid_json': True}
        else:
            row = message_to_ordereddict(msg)
        last[topic] = row
        if topic == '/uwb/btf_decision':
            reasons[row.get('reason')] += 1
            decisions.append(row)
        if topic == '/uwb/received': raw.append(row)
    topics = {name: String for name in ('/uwb/status', '/uwb/btf_status',
        '/uwb/btf_decision', '/uwb/received', '/uwb/bridge_status',
        '/uwb/local_odometry_status', '/flight_state')}
    topics.update({'/uwb/btf_pose': PoseWithCovarianceStamped,
        '/mavros/vision_pose/pose_cov': PoseWithCovarianceStamped,
        '/mavros/state': State, '/mavros/extended_state': ExtendedState,
        '/mavros/rc/in': RCIn, '/mavros/timesync_status': TimesyncStatus,
        '/mavros/imu/data': Imu, '/mavros/downward_0': Range,
        '/mavros/local_position/odom': Odometry})
    subscriptions = [node.create_subscription(cls, topic,
        lambda msg, topic=topic: receive(topic, msg), qos) for topic, cls in topics.items()]
    started = time.monotonic()
    try:
        while time.monotonic()-started < 25:
            rclpy.spin_once(node, timeout_sec=.05)
        result = {'utc': datetime.now(timezone.utc).isoformat(),
            'duration_s': time.monotonic()-started, 'counts': dict(counts),
            'decision_reasons': dict(reasons), 'last': last,
            'decisions': list(decisions), 'raw': list(raw),
            'topics': node.get_topic_names_and_types(),
            'nodes': node.get_node_names_and_namespaces(),
            'raw_sink_publishers': [{'name': info.node_name, 'namespace': info.node_namespace}
                for info in node.get_publishers_info_by_topic('/uas1/mavlink_sink')],
            'timing': {topic: {'last_age_s': time.monotonic()-stamps[-1],
                'max_gap_s': max((b-a for a,b in zip(stamps,stamps[1:])),default=0)}
                for topic, stamps in times.items()}}
        print(json.dumps(result, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.unlink(profile.name)

if __name__ == '__main__':
    main()
