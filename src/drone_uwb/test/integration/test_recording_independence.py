"""Storage fault injection at live ROS adapters; no FC output is created."""
from collections import Counter
import json
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
from drone_uwb.integration.ros import node, btf_node
from drone_uwb.processing.solvers.observations import Observation, Decision


def publisher():
    messages = []
    return NS(publish=messages.append, messages=messages)


def failed_recorder():
    return NS(error='recording_disk_reserve_low', write=lambda *args: False)


@pytest.mark.parametrize('valid', [True, False])
def test_uwb_keeps_fresh_input_and_processor_decision_on_storage_failure(monkeypatch, valid):
    observation = Observation(1, 9_990_000_000, 2., 3., .09, 15, .01, .02, 'raw_ranges')
    decision = Decision('accepted' if valid else 'range_inconsistent', observation if valid else None)
    wire = {'type': 'uwb_raw_cycle'}
    monkeypatch.setattr(node, 'decode_line', lambda line: wire)
    obj = NS(get_parameter=lambda name: NS(value=0), started=0, serial=NS(read=lambda: b'cycle\n'),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)),
        record_files={'raw': True, 'received': True, 'decisions': True}, recorder=failed_recorder(),
        counts=Counter(), framer=NS(feed=lambda chunk: [chunk]), parse_streak=0,
        processor=NS(process=lambda *args: decision), raw_pub=publisher(),
        received_pub=publisher(), pose_pub=publisher(), layout={'coordinate_frame': 'uwb_map'})
    obj.record = lambda name, value: node.UwbNode.record(obj, name, value)
    node.UwbNode.poll(obj)
    assert len(obj.raw_pub.messages) == len(obj.received_pub.messages) == 1
    assert obj.last_decision == decision.reason
    assert obj.counts['recording_failed'] == 1
    assert len(obj.pose_pub.messages) == int(valid)
    assert obj.recorder.error == 'recording_disk_reserve_low'
    if valid:
        pose = obj.pose_pub.messages[0]
        assert pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec == observation.stamp_ns
        assert pose.pose.pose.position.z == 0. and pose.pose.covariance[14] == 1e6


@pytest.mark.parametrize('case', ['fresh', 'queued', 'expired', 'invalid', 'height_missing'])
def test_btf_storage_failure_does_not_override_observation_validity(case):
    now_ns = 10_000_000_000
    result = dict(ok=case != 'invalid', reason='accepted' if case != 'invalid' else 'range_inconsistent',
        seq=1, stamp_ns=now_ns - (300_000_000 if case == 'expired' else 10_000_000),
        xy_m=[2., 3.], xyz_m=None)
    rejected = []
    obj = NS(recorder=failed_recorder(), files={'inputs': True, 'decisions': True},
        counts=Counter(), height_counts=Counter(),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=now_ns)),
        sync_ready=lambda: True, ground_height=None, fc_state=None, landed=None,
        fc_state_at=float('-inf'), landed_at=float('-inf'),
        processor=NS(height=NS(ground_reference=None), process=lambda event: result,
                     reject_queued_input=lambda: rejected.append(True)),
        require_height=case == 'height_missing',
        config={'max_output_age_s': .2, 'xy_stddev_m': .3},
        pose_pub=publisher(), xyz_pub=publisher(), decision_pub=publisher())
    obj.record = lambda name, row: btf_node.BtfNode.record(obj, name, row)
    event = {'host_received_ros_ns': now_ns - (200_000_000 if case == 'queued' else 0)}
    btf_node.BtfNode.raw(obj, NS(data=json.dumps(event)))
    assert obj.counts['recording_failed'] == 1
    assert obj.recorder.error == 'recording_disk_reserve_low'
    assert len(obj.pose_pub.messages) == int(case == 'fresh')
    assert not obj.xyz_pub.messages  # No invented height to recover logging.
    assert bool(rejected) == (case == 'queued')
    if case == 'fresh':
        pose = obj.pose_pub.messages[0]
        assert pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec == result['stamp_ns']
        assert pose.pose.covariance[0] == .09 and pose.pose.covariance[14] == 1e6
    elif case == 'expired':
        assert obj.counts['output_expired'] == 1
    elif case == 'height_missing':
        assert obj.counts['measured_height_required'] == 1
