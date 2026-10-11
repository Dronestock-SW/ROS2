"""Actual ROS transport burst test, isolated from physical ROS domains."""
import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from drone_uwb.integration.ros.qos import received_stream_qos


def test_status_and_raw_tdma_pairs_survive_serial_read_burst(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=99)
    executor = SingleThreadedExecutor(context=context)
    token = uuid.uuid4().hex
    sender = rclpy.create_node('burst_sender_'+token, context=context)
    receiver = rclpy.create_node('burst_receiver_'+token, context=context)
    topic = '/test_uwb_burst_'+token
    received, legacy = [], []
    publisher = sender.create_publisher(String, topic, received_stream_qos())
    receiver.create_subscription(String, topic, lambda msg: received.append(msg.data), received_stream_qos())
    receiver.create_subscription(String, topic, lambda msg: legacy.append(msg.data), qos_profile_sensor_data)
    messages = ['status'] + [f'{kind}:{seq}' for seq in range(4) for kind in ('raw', 'tdma')] + ['heartbeat']
    try:
        deadline = time.monotonic()+5
        while publisher.get_subscription_count() != 2 and time.monotonic() < deadline:
            rclpy.spin_once(sender, executor=executor, timeout_sec=.02)
        assert publisher.get_subscription_count() == 2
        # Do not consume callbacks while the serial poll's complete burst arrives.
        for text in messages:
            publisher.publish(String(data=text))
        deadline = time.monotonic()+.2
        while time.monotonic() < deadline:
            rclpy.spin_once(sender, executor=executor, timeout_sec=.01)
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and (len(received)<len(messages) or len(legacy)<5):
            rclpy.spin_once(receiver, executor=executor, timeout_sec=.02)
        assert received == messages
        assert legacy == messages[-5:]  # Reproduces lost status/pair context.
    finally:
        executor.shutdown()
        sender.destroy_node()
        receiver.destroy_node()
        context.shutdown()
