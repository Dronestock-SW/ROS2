"""Call real adapter methods with message/publisher doubles, without ROS/DDS.

This verifies adapter decisions, not ROS middleware or executor behavior.
Import doubles are scoped to each test and never instantiate a ROS node.
"""
from collections import Counter
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace as NS, MethodType

import pytest

from drone_uwb.processing.measured_btf import MeasuredHeight
from drone_uwb.acquisition.tdma import TdmaGate
from test_tdma_stream import warmed_measured_processor, pair, event, status


class Pose:
    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
        value = NS(position=NS(x=0., y=0., z=0.), orientation=NS(w=0.))
        self.pose = NS(pose=value, covariance=[0.]*36)


@pytest.fixture
def adapter(monkeypatch):
    imports = {
        'ament_index_python.packages': dict(get_package_share_directory=lambda _: ''),
        'geometry_msgs.msg': dict(PoseStamped=Pose, PoseWithCovarianceStamped=Pose),
        'mavros_msgs.msg': dict(State=NS, TimesyncStatus=NS, ExtendedState=NS),
        'nav_msgs.msg': dict(Odometry=NS),
        'rclpy': {}, 'rclpy.node': dict(Node=object),
        'rclpy.executors': dict(ExternalShutdownException=RuntimeError),
        'rclpy.callback_groups': dict(MutuallyExclusiveCallbackGroup=object),
        'rclpy.qos': dict(qos_profile_sensor_data=None),
        'rcl_interfaces.msg': dict(ParameterDescriptor=NS),
        'sensor_msgs.msg': dict(Imu=NS, Range=NS), 'std_msgs.msg': dict(String=NS),
        'std_srvs.srv': dict(Trigger=NS),
        'drone_uwb.integration.ros.qos': dict(received_stream_qos=lambda: None, btf_sensor_qos=lambda: None),
    }
    for name, attrs in imports.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[2]/'drone_uwb/integration/ros/btf_node.py'
    spec = importlib.util.spec_from_file_location('_btf_adapter_contract', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def receiver(adapter, processor):
    clock, records, poses, decisions = [0], [], [], []
    obj = NS(processor=processor, recorder=None, counts=Counter(), height_counts=Counter(),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=clock[0])),
        sensor_lock=threading.RLock(), sensor_height=MeasuredHeight(processor.config['height']),
        sensor_receive_id=0, sensor_epoch=0, sync_ready=lambda: True,
        record=lambda name, row, **kw: records.append((name, row)),
        ground_height=None, fc_state=None, landed=None, fc_state_at=float('-inf'),
        landed_at=float('-inf'), require_height=False, config=processor.config,
        last_published_stamp_ns=None, pose_pub=NS(publish=poses.append),
        xyz_pub=NS(publish=lambda _: pytest.fail('unexpected XYZ')),
        decision_pub=NS(publish=decisions.append))
    obj.freeze_sensors = MethodType(adapter.BtfNode.freeze_sensors, obj)
    obj.clear_sensors = MethodType(adapter.BtfNode.clear_sensors, obj)
    return obj, clock, records, poses


def feed(adapter, obj, clock, e, *, now=None):
    clock[0] = e['host_received_ros_ns'] if now is None else now
    adapter.BtfNode.raw(obj, NS(data=json.dumps(e)))


def test_publication_history_blocks_output_without_resetting_calculation(adapter):
    p, mono = warmed_measured_processor('5')
    obj, clock, _, poses = receiver(adapter, p)
    prior = mono+1_700_000_000_000_000_000+100_000_000
    obj.last_published_stamp_ns = prior
    for seq in range(81, 87):
        raw, diag = pair(seq)
        received = 10_000_000_000+raw['cycle_end_us']*1000
        feed(adapter, obj, clock, event(raw, received))
        feed(adapter, obj, clock, event(diag, received+5_000_000))
        assert obj.last['ok']  # B keeps its history while publication is blocked.
        if seq <= 84:
            assert not poses and obj.last_published_stamp_ns == prior
            assert obj.last['pose_blocked_reason'] == 'nonincreasing_published_stamp'
        else:
            assert obj.last['published'] and obj.last_published_stamp_ns > prior
    published = obj.last_published_stamp_ns
    p.reset_models()
    p.reject_queued_input()
    p.reset()
    assert obj.last_published_stamp_ns == published


def test_expired_boot_status_cannot_make_old_map_validate_new_boot(adapter):
    p, mono = warmed_measured_processor('5')
    obj, clock, _, _ = receiver(adapter, p)
    boot = event(status(event='boot'), mono+6_000_000)
    feed(adapter, obj, clock, boot, now=boot['host_received_ros_ns']+160_000_000)
    assert p.clock_recovery.held is not None  # Boot was never consumed.
    raw, diag = pair(88, boot=11)
    received = 10_000_000_000+raw['cycle_end_us']*1000
    feed(adapter, obj, clock, event(raw, received))
    feed(adapter, obj, clock, event(diag, received+5_000_000))
    assert obj.last['reason'] == 'status_unavailable'
    assert p.clock_recovery.held is None and not p.clock.ready


def test_failed_publish_does_not_advance_publication_history(adapter):
    p, mono = warmed_measured_processor('5')
    obj, clock, _, _ = receiver(adapter, p)
    obj.pose_pub.publish = lambda _: (_ for _ in ()).throw(RuntimeError('injected publish failure'))
    raw, diag = pair(81)
    feed(adapter, obj, clock, event(raw, mono+25_000_000))
    with pytest.raises(RuntimeError, match='injected publish failure'):
        feed(adapter, obj, clock, event(diag, mono+30_000_000))
    assert obj.last_published_stamp_ns is None


def test_strict_raw_skips_copy_but_tdma_selects_new_sensor_membership(adapter, monkeypatch):
    p, mono = warmed_measured_processor('5')
    obj, clock, records, _ = receiver(adapter, p)
    copies = []
    original = obj.sensor_height.snapshot
    monkeypatch.setattr(obj.sensor_height, 'snapshot', lambda: (copies.append(True), original())[1])
    raw, diag = pair(81)
    feed(adapter, obj, clock, event(raw, mono+25_000_000))
    raw_record = records[-1][1]
    assert not copies and p.tdma.pending is not None
    assert raw_record['event']['message'] == raw
    meta = raw_record['sensor_snapshot']
    assert meta['used'] is False and 'frozen_monotonic_ns' not in meta
    assert meta['receive_id'] == 0 and meta['epoch'] == 0
    stamp = 1_700_000_010_000_000_000+max(raw['sample_time_us'])*1000
    obj.sensor_height.add('tof', dict(stamp_ns=stamp, range_m=.5, valid=True))
    obj.sensor_height.add('imu', dict(stamp_ns=stamp-2_000_000, valid=True, quaternion_wxyz=[1, 0, 0, 0]))
    p.height.config['mount_confirmed'] = False  # Test membership only, no XYZ.
    feed(adapter, obj, clock, event(diag, mono+30_000_000))
    assert len(copies) == 1
    assert obj.last['sensor_snapshot']['used'] is True
    assert 'frozen_monotonic_ns' in obj.last['sensor_snapshot']
    assert obj.last['height_selection'][-1]['attitude_skew_s'] == .002
    assert obj.last['height_selection'][-1]['imu_stamp_ns'] == stamp-2_000_000


def test_copy_free_raw_still_resets_unready_live_buffers_and_records_epoch(adapter):
    p, mono = warmed_measured_processor('5')
    obj, clock, records, _ = receiver(adapter, p)
    old_snapshot = p.height
    obj.sync_ready = lambda: False
    obj.sensor_height.add('tof', dict(stamp_ns=1, range_m=.5, valid=True))
    obj.sensor_height.add('imu', dict(stamp_ns=1, valid=True, quaternion_wxyz=[1, 0, 0, 0]))
    raw, _ = pair(81)
    feed(adapter, obj, clock, event(raw, mono+25_000_000))
    assert p.height is old_snapshot and p.tdma.pending is not None
    assert all(not v for v in obj.sensor_height.samples.values())
    assert obj.sensor_epoch == 1
    assert records[0][1]['type'] == 'sensor_reset'
    assert records[-1][1]['sensor_snapshot']['epoch'] == 1
    assert records[-1][1]['sensor_snapshot']['used'] is False


@pytest.mark.parametrize('mode,activate,needed', [
    ('required', False, False), ('auto', False, True),
    ('auto', True, False), ('disabled', True, True),
])
def test_snapshot_selection_uses_effective_tdma_gate(mode, activate, needed):
    p, mono = warmed_measured_processor('5')
    p.tdma = TdmaGate('5', mode)
    if activate:
        p.process(event(status(), mono+6_000_000))
    raw, diag = pair(81)
    assert p.needs_sensor_snapshot(event(raw, mono+25_000_000)) is needed
    assert p.needs_sensor_snapshot(event(diag, mono+30_000_000))
    assert p.needs_sensor_snapshot(event(status(), mono+31_000_000))
    assert p.needs_sensor_snapshot({'message': None})


def test_missing_snapshot_fails_closed_before_any_old_height_query(monkeypatch):
    p, mono = warmed_measured_processor('5')
    p.tdma = TdmaGate('5', 'disabled')  # Unexpected RAW calculation path.
    monkeypatch.setattr(p.height, 'at', lambda _: pytest.fail('old snapshot queried'))
    raw, _ = pair(81)
    result = p.process(event(raw, mono+25_000_000), height_available=False)
    assert not result['ok'] and result['reason'] == 'sensor_snapshot_not_available'


def test_copy_free_raw_still_detects_host_clock_jump(adapter):
    p, mono = warmed_measured_processor('5')
    obj, clock, _, poses = receiver(adapter, p)
    raw, _ = pair(81)
    feed(adapter, obj, clock, event(raw, mono-1))
    assert obj.last['reason'] == 'host_clock_jump' and not poses
    assert obj.sensor_epoch == 1 and p.tdma.pending is None
    assert p.clock_recovery.held is None and not p.clock.ready
