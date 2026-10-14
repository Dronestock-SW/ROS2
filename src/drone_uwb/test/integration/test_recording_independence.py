"""Storage fault injection at live ROS adapters; no FC output is created."""
from collections import Counter
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
from drone_uwb.integration.ros import node, btf_node
from drone_uwb.processing.solvers.observations import Observation, Decision
from drone_uwb.processing.measured_btf import MeasuredBtf


def publisher():
    messages = []
    return NS(publish=messages.append, messages=messages)


def failed_recorder():
    return NS(error='recording_disk_reserve_low', write=lambda *args: False)


@pytest.mark.parametrize('armed,accepted', [(False,True),(True,False),(None,False)])
def test_reset_service_is_explicit_ground_only_and_records_outcome(monkeypatch,armed,accepted):
    monkeypatch.setattr(btf_node.time,'monotonic',lambda:20.)
    config=json.loads((Path(__file__).parents[2]/'config/runtime/uwb_btf_real.json').read_text())
    processor=MeasuredBtf(config)
    processor.raw_guard.check((1.,1.),1_000_000_000)
    records=[]
    obj=NS(processor=processor,fc_state=dict(connected=True,armed=armed),landed=1,
        fc_state_at=19.9,landed_at=19.9,ground_velocity_ok=True,ground_velocity_at=19.95,
        last={'old':'pose'},last_source_ns=123,decision_pub=publisher(),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=20_000_000_000)),
        record=lambda name,row:records.append(row))
    reply=btf_node.BtfNode.reset_ground_history(obj,None,NS())
    assert reply.success is accepted and records[-1]['accepted'] is accepted
    assert (obj.last is None) is accepted
    assert (processor.raw_guard.prior_xy is None) is accepted
    assert not hasattr(obj,'create_client')


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


@pytest.mark.parametrize('case', ['fresh', 'queued', 'expired', 'invalid', 'height_missing',
    'b4_no_height', 'b4_expired', 'b4_disabled', 'b3_no_height'])
def test_btf_storage_failure_does_not_override_observation_validity(case):
    now_ns = 10_000_000_000
    result = dict(ok=case != 'invalid', reason='accepted' if case != 'invalid' else 'range_inconsistent',
        seq=1, stamp_ns=now_ns - (300_000_000 if case in ('expired','b4_expired') else 10_000_000),
        xy_m=[2., 3.], xyz_m=None)
    if case.startswith(('b4_', 'b3_')):
        result.update(time_us=100,models={
            'B':dict(ok=True,fresh_observation_count=4,newest_sample_us=100,fit=dict(mask=15)),
            'B_TF':dict(source='ToF_validated_B3' if case.startswith('b3_') else 'unchanged_B4')})
    rejected = []
    obj = NS(recorder=failed_recorder(), files={'inputs': True, 'decisions': True},
        counts=Counter(), height_counts=Counter(), last_published_stamp_ns=None,
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=now_ns)),
        sync_ready=lambda: True, ground_height=None, fc_state=None, landed=None,
        freeze_sensors=lambda:dict(clock_sync_ready=True, receive_id=0),
        fc_state_at=float('-inf'), landed_at=float('-inf'),
        processor=NS(height=NS(ground_reference=None), process=lambda event: result,
                     needs_sensor_snapshot=lambda event: True,
                     reject_queued_input=lambda: rejected.append(True), observation_session=0),
        require_height=case == 'height_missing' or case.startswith(('b4_','b3_')),
        config={'max_output_age_s': .2, 'xy_stddev_m': .3, 'allow_b4_xy_without_tof':case!='b4_disabled'},
        pose_pub=publisher(), xyz_pub=publisher(), decision_pub=publisher())
    obj.record = lambda name, row, **kwargs: btf_node.BtfNode.record(obj, name, row, **kwargs)
    event = {'host_received_ros_ns': now_ns - (200_000_000 if case == 'queued' else 0)}
    btf_node.BtfNode.raw(obj, NS(data=json.dumps(event)))
    assert obj.counts['recording_failed'] == 1
    assert obj.recorder.error == 'recording_disk_reserve_low'
    assert len(obj.pose_pub.messages) == int(case in ('fresh','b4_no_height'))
    assert not obj.xyz_pub.messages  # No invented height to recover logging.
    assert bool(rejected) == (case == 'queued')
    if case == 'queued':
        d=json.loads(obj.decision_pub.messages[-1].data)
        assert d['reason']=='receiver_queue_expired' and not d['published']
    if case in ('fresh','b4_no_height'):
        pose = obj.pose_pub.messages[0]
        assert pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec == result['stamp_ns']
        assert pose.pose.covariance[0] == .09 and pose.pose.covariance[14] == 1e6
        assert pose.pose.pose.position.z == 0.
    elif case in ('expired','b4_expired'):
        assert obj.counts['output_expired'] == 1
        assert result['pose_blocked_reason']=='output_expired'
    elif case in ('height_missing','b4_disabled','b3_no_height'):
        assert obj.counts['measured_height_required'] == 1
