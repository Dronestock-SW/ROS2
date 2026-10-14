"""Actual DDS/executor test; only an isolated localhost ROS domain is allowed."""
import json
import os
from pathlib import Path
import threading
import time

import pytest

pytest.importorskip('rclpy')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String
from drone_uwb.integration.ros.btf_node import BtfNode
from drone_uwb.integration.ros.qos import btf_sensor_qos, received_stream_qos


def wait_for(predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return
        time.sleep(.005)
    assert predicate()


def test_sensor_callbacks_run_during_fit_and_next_snapshot_selects_2ms(tmp_path):
    if os.environ.get('ROS_DOMAIN_ID') != '99' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('requires explicit isolated ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1')
    config = Path(__file__).parents[2]/'config/runtime/uwb_btf_real.json'
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config}',
                    '-p', f'trace_directory:={tmp_path}/trace'])
    node = BtfNode()
    feeder = Node('btf_isolated_test_feeder', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ingress_executor = SingleThreadedExecutor()
    ingress_executor.add_node(node.sensor_node)
    runner = threading.Thread(target=executor.spin, daemon=True)
    receiver = threading.Thread(target=ingress_executor.spin, daemon=True)
    imu_pub = feeder.create_publisher(Imu, node.config['imu_topic'], btf_sensor_qos())
    tof_pub = feeder.create_publisher(Range, node.config['tof_topic'], btf_sensor_qos())
    raw_pub = feeder.create_publisher(String, '/uwb/received', received_stream_qos())
    entered, release = threading.Event(), threading.Event()
    runner.start()
    receiver.start()
    try:
        wait_for(lambda: imu_pub.get_subscription_count() and tof_pub.get_subscription_count()
                 and raw_pub.get_subscription_count())
        with node.sensor_lock:
            node.clock_readiness.qualified = True
            node.clock_readiness.last_valid_s = time.monotonic()
        stamp = feeder.get_clock().now().nanoseconds-50_000_000
        def send_imu(offset):
            msg = Imu()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp+offset*1_000_000, 10**9)
            msg.header.frame_id = 'base_link'
            msg.orientation.w = 1.
            imu_pub.publish(msg)
        tof = Range()
        tof.header.stamp.sec, tof.header.stamp.nanosec = divmod(stamp, 10**9)
        tof.range, tof.min_range, tof.max_range = .5, .1, 12.
        tof_pub.publish(tof)
        send_imu(-22)
        wait_for(lambda: node.sensor_counts['tof'] == 1 and node.sensor_counts['imu'] == 1)
        def fit(event):
            entered.set()
            assert release.wait(2.)
            return dict(ok=False, reason='status')
        node.processor.process = fit
        raw_pub.publish(String(data=json.dumps(dict(host_received_ros_ns=feeder.get_clock().now().nanoseconds))))
        assert entered.wait(1.)
        for offset in (-12, -2, 8, 18, 28, 38, 48):
            send_imu(offset)
            time.sleep(.003)
        wait_for(lambda: node.sensor_counts['imu'] == 8, timeout=.3)
        # The old snapshot stays coherent even while ingress receives the burst.
        assert node.processor.height.at(stamp)[1]['reason'] == 'attitude_stale'
        assert not release.is_set()
        release.set()
        executor.shutdown(timeout_sec=3.)
        executor.wake()
        runner.join(3.)
        node.freeze_sensors()
        meta = node.processor.height.at(stamp)[1]
        assert meta['reason'] == 'ok' and meta['attitude_skew_s'] == .002
        selected_id = meta['imu_receive_id']
        node.trace.close()
        records = [json.loads(line) for line in (tmp_path/'trace/trace.jsonl').read_text().splitlines()]
        selected = next(r['row'] for r in records if r.get('stream') == 'inputs'
                        and r['row'].get('receive_id') == selected_id)
        assert selected['buffer_accepted'] and selected['stamp_ns'] == stamp-2_000_000
        assert node.trace.error is None
    finally:
        release.set()
        executor.shutdown(timeout_sec=3.)
        executor.wake()
        ingress_executor.shutdown(timeout_sec=3.)
        ingress_executor.wake()
        runner.join(3.)
        receiver.join(3.)
        node.destroy_node()
        feeder.destroy_node()
        rclpy.shutdown()
