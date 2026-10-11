#!/usr/bin/env python3
"""Bounded receive-only trace of MAVROS router, decoded topics and scheduling.

No publisher, FC service client, serial handle or flight command is created.
Rows stay in bounded memory during capture to avoid diagnostic disk IO stalls.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=90.)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mavros-pid', type=int, required=True)
    parser.add_argument('--tof-topic', default='/mavros/downward_0')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 180 or args.output.exists():
        parser.error('Use a new output directory and 10..180 seconds')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from mavros_msgs.msg import Mavlink, State, TimesyncStatus
    from sensor_msgs.msg import Imu, Range
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String
    rclpy.init()
    rows, counts, dropped = [], Counter(), 0
    node = Node('fc_transport_receive_only_probe_'+str(os.getpid()))
    started = time.monotonic_ns()

    def append(topic, msg=None, **extra):
        nonlocal dropped
        mono = time.monotonic_ns()
        row = dict(topic=topic, monotonic_ns=mono, received_ros_ns=node.get_clock().now().nanoseconds)
        if msg is not None and hasattr(msg, 'header'):
            row['header_ns'] = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        row.update(extra)
        counts[topic] += 1
        if len(rows) < 250_000:
            rows.append(row)
        else:
            dropped += 1

    def mavlink(msg):
        # Capture wire bytes only; boot-time units depend on message definition.
        payload = b''.join(struct.pack('<Q', n) for n in msg.payload64)
        append('router_rx', msg, msgid=msg.msgid, seq=msg.seq, sysid=msg.sysid,
               compid=msg.compid, first8_hex=payload[:8].hex())

    node.create_subscription(Mavlink, '/uas1/mavlink_source', mavlink, qos)
    for topic, typ in [('/mavros/imu/data', Imu), (args.tof_topic, Range),
                       ('/mavros/state', State), ('/uwb/btf_pose', PoseWithCovarianceStamped),
                       ('/uwb/received', String)]:
        node.create_subscription(typ, topic, lambda msg, t=topic:append(t,msg), qos)
    node.create_subscription(TimesyncStatus, '/mavros/timesync_status',
        lambda msg:append('timesync', msg, remote_ns=msg.remote_timestamp_ns,
            offset_ns=msg.estimated_offset_ns, rtt_ms=msg.round_trip_time_ms), qos)
    proc = Path('/proc')/str(args.mavros_pid)/'task'
    tids = []
    for task in proc.iterdir():
        try:
            if (task/'comm').read_text().strip().startswith('mserial'):
                tids.append(task)
        except OSError:
            pass

    def tick():
        threads = {}
        for task in tids:
            try:
                stat = (task/'stat').read_text().split()
                threads[task.name] = dict(state=stat[2], wchan=(task/'wchan').read_text(),
                                         cpu_ticks=int(stat[13])+int(stat[14]))
            except OSError:
                threads[task.name] = dict(gone=True)
        append('sampler', threads=threads)

    node.create_timer(.05, tick)
    try:
        while rclpy.ok() and (time.monotonic_ns()-started)/1e9 < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output/'trace.jsonl').open('x', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
    last, max_gaps, gaps = {}, {}, []
    for row in rows:
        topic, now = row['topic'], row['monotonic_ns']
        if topic in last:
            gap = (now-last[topic])/1e9
            max_gaps[topic] = max(max_gaps.get(topic, 0), gap)
            if gap > (1.5 if topic == '/mavros/state' else .3):
                gaps.append(dict(topic=topic, at_monotonic_ns=now, gap_s=gap))
        last[topic] = now
    report = dict(scope='read_only_transport_trace', seconds=args.seconds, counts=dict(counts),
                  dropped=dropped, max_gaps_s=max_gaps, gaps=gaps,
                  missing_topics=[name for name in ('router_rx', 'timesync', args.tof_topic,
                      '/mavros/imu/data', '/mavros/state', '/uwb/btf_pose') if not counts[name]],
                  commands_sent=0, publishers_created=0)
    (args.output/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
