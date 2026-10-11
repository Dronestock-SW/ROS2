from collections import Counter
from types import SimpleNamespace as NS
import pytest

pytest.importorskip('rclpy')
from drone_uwb.integration.clock_readiness import ClockReadiness
from drone_uwb.integration.ros import btf_node


def test_late_reply_does_not_reset_height_but_real_clock_change_does(monkeypatch):
    monkeypatch.setattr(btf_node.time,'monotonic',lambda:10.1)
    records, resets = [], []
    samples = {'tof':[1], 'imu':[2]}
    obj = NS(clock_readiness=ClockReadiness(30,10.,1_000_000_000,100,True),
        processor=NS(height=NS(samples=samples),reset_models=lambda:resets.append(True)),
        counts=Counter(), get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),
        record=lambda name,row:records.append(row))
    late = NS(remote_timestamp_ns=200,estimated_offset_ns=1_000_000_000,round_trip_time_ms=1800.)
    btf_node.BtfNode.sync(obj,late)
    assert not records[-1]['accepted'] and samples == {'tof':[1], 'imu':[2]}
    assert obj.clock_readiness.last_valid_s == 10. and not resets
    changed = NS(remote_timestamp_ns=300,estimated_offset_ns=1_010_000_000,round_trip_time_ms=2.)
    btf_node.BtfNode.sync(obj,changed)
    assert records[-1]['clock_changed'] and samples == {'tof':[], 'imu':[]}
    assert resets == [True] and not obj.clock_readiness.ready(10.1)
