from collections import Counter
from types import SimpleNamespace as NS
import threading
import pytest

pytest.importorskip('rclpy')
from drone_uwb.integration.clock_readiness import ClockReadiness
from drone_uwb.integration.ros import btf_node
from drone_uwb.processing.measured_btf import MeasuredHeight


def test_late_reply_does_not_reset_height_but_real_clock_change_does(monkeypatch):
    monkeypatch.setattr(btf_node.time,'monotonic',lambda:10.1)
    records, resets = [], []
    height=MeasuredHeight({})
    height.add('tof',dict(stamp_ns=100,valid=True))
    height.add('imu',dict(stamp_ns=100,valid=True))
    samples=height.samples
    obj = NS(clock_readiness=ClockReadiness(30,10.,1_000_000_000,100,True),
        processor=NS(height=height.snapshot(),reset_models=lambda:resets.append(True)),
        sensor_height=height, sensor_epoch=0, sensor_receive_id=2, clock_reset_pending=False,
        counts=Counter(), get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),
        record=lambda name,row:records.append(row), sensor_lock=threading.RLock())
    obj.clear_sensors=lambda reason:btf_node.BtfNode.clear_sensors(obj,reason)
    obj.sync_ready=lambda:obj.clock_readiness.ready(btf_node.time.monotonic())
    late = NS(remote_timestamp_ns=200,estimated_offset_ns=1_000_000_000,round_trip_time_ms=1800.)
    btf_node.BtfNode.sync(obj,late)
    assert not records[-1]['accepted'] and all(len(s)==1 for s in samples.values())
    assert obj.clock_readiness.last_valid_s == 10. and not resets
    monkeypatch.setattr(btf_node.time,'monotonic',lambda:10.6)
    snapshot=btf_node.BtfNode.freeze_sensors(obj)
    assert not snapshot['clock_sync_ready'] and snapshot['tof']['count']==1
    assert snapshot['imu']['count']==1 and not resets  # expiry never deletes history
    changed = NS(remote_timestamp_ns=300,estimated_offset_ns=1_010_000_000,round_trip_time_ms=2.)
    btf_node.BtfNode.sync(obj,changed)
    assert records[-1]['clock_changed'] and all(not s for s in samples.values())
    assert not resets and obj.clock_reset_pending
    btf_node.BtfNode.freeze_sensors(obj)
    assert resets == [True] and not obj.clock_readiness.ready(10.6)
