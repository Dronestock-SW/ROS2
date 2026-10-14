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
