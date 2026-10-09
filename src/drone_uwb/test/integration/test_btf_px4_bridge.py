"""Antenna/FC boundary tests; all inputs here are synthetic, no flight output."""
from dataclasses import replace
import math
import os
from types import SimpleNamespace as NS
import numpy as np
import pytest

from drone_uwb.integration.ros.frames import BridgeSettings, gate, observation_xy

PARAMS = dict(EKF2_EV_CTRL=1, EKF2_EV_DELAY=0., EKF2_EV_NOISE_MD=0,
              EKF2_EV_POS_X=.14, EKF2_EV_POS_Y=-.02, EKF2_EV_POS_Z=-.12)


def ready():
    return BridgeSettings(enabled=True, input_source='btf_xy', tag_id='6', ground_only=True,
        layout_confirmed=True, alignment_confirmed=True, timing_confirmed=True,
        sensor_mount_confirmed=True, antenna_body_frd_x_m=.14,
        antenna_body_frd_y_m=-.02, antenna_body_frd_z_m=-.12)


def test_source_selection_and_antenna_reference_not_fc_feedback():
    assert BridgeSettings().input_topic == '/uwb_pose'
    assert ready().input_topic == '/uwb/btf_pose' and ready().ros_domain_id == 2
    assert gate(ready(), True, .1, PARAMS, .1) == 'ready'
    for key in ('EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z'):
        for value in (None, True, math.nan, .3):
            assert gate(ready(), True, .1, {**PARAMS, key: value}, .1) != 'ready'
    assert gate(ready(), True, .1, PARAMS, .1, armed=True) == 'ground_only_requires_disarmed'


@pytest.mark.parametrize('field', ['enabled', 'layout_confirmed', 'alignment_confirmed',
                                  'timing_confirmed', 'sensor_mount_confirmed'])
def test_every_unmeasured_gate_stays_closed(field):
    assert gate(replace(ready(), **{field: False}), True, 0, PARAMS, 0) != 'ready'


@pytest.mark.parametrize('change', [dict(stamp_ns=0), dict(stamp_ns=2_000_000_001),
    dict(stamp_ns=1_700_000_000), dict(last_stamp_ns=1_900_000_000),
    dict(frame='map'), dict(x=math.nan), dict(covariance=[[.1,.2],[0,.1]]),
    dict(covariance=[[.1,0],[0,-.1]])])
def test_original_stamp_frame_finite_and_covariance_are_required(change):
    data=dict(frame='uwb_map', stamp_ns=1_900_000_000, now_ns=2_000_000_000,
              last_stamp_ns=None, x=2., y=1., covariance=[[.04,.01],[.01,.09]])
    with pytest.raises(ValueError):
        observation_xy(ready(), **{**data, **change})


def test_ros_adapter_keeps_stamp_rotates_covariance_and_does_not_inject_height():
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from drone_uwb.integration.ros.bridge import UwbPx4Bridge
    outputs=[]
    from drone_uwb.processing.observation_guard import ObservationGuard
    guard=ObservationGuard()
    guard.check((2.,1.),1_650_000_000)  # Continuous synthetic observation span acquires the gate.
    settings=replace(ready(), enu_yaw_deg=90., enu_offset_x_m=3., enu_offset_y_m=-1.)
    obj=NS(settings=settings, current_gate=lambda:'ready', last_stamp=None,
           get_clock=lambda:NS(now=lambda:NS(nanoseconds=2_000_000_000)),
           publisher=NS(publish=outputs.append), published=0, rejected=0, observation_guard=guard)
    msg=PoseWithCovarianceStamped()
    msg.header.frame_id='uwb_map'; msg.header.stamp.sec=1; msg.header.stamp.nanosec=900_000_000
    msg.pose.pose.position.x=2.; msg.pose.pose.position.y=1.; msg.pose.pose.position.z=123.
    for i,v in zip((0,1,6,7),(.04,.01,.01,.09)): msg.pose.covariance[i]=v
    UwbPx4Bridge.receive_pose(obj,msg)
    out=outputs[0]
    assert out.header.stamp == msg.header.stamp
    np.testing.assert_allclose([out.pose.pose.position.x,out.pose.pose.position.y],[2,1])
    np.testing.assert_allclose([out.pose.covariance[i] for i in (0,1,6,7)],[.09,-.01,-.01,.04])
    assert out.pose.pose.position.z == 0. and all(out.pose.covariance[i] == 1e6 for i in (14,21,28,35))
    UwbPx4Bridge.receive_pose(obj,msg)
    assert len(outputs)==1 and obj.rejected==1


def test_bridge_domain_timesync_and_publisher_gates(monkeypatch):
    pytest.importorskip('rclpy')
    from drone_uwb.integration.ros import bridge
    monkeypatch.setattr(bridge.time, 'monotonic', lambda:10.)
    monkeypatch.setenv('ROS_DOMAIN_ID','2')
    obj=NS(settings=ready(), connected=True, armed=False, state_time=10., params=PARAMS,
           param_time=10., sync_count=30, sync_time=10., input_topic='/uwb/btf_pose',
           count_publishers=lambda topic:1, test_mode=False,
           context=NS(get_domain_id=lambda:int(os.environ['ROS_DOMAIN_ID'])))
    assert bridge.UwbPx4Bridge.current_gate(obj)=='ready'
    monkeypatch.setenv('ROS_DOMAIN_ID','1')
    assert bridge.UwbPx4Bridge.current_gate(obj)=='tag_domain_mismatch'
    monkeypatch.setenv('ROS_DOMAIN_ID','2')
    obj.sync_time=9.
    assert bridge.UwbPx4Bridge.current_gate(obj)=='timesync_unavailable_or_unstable'
    obj.sync_time=10.; obj.count_publishers=lambda topic:2
    assert bridge.UwbPx4Bridge.current_gate(obj)=='ambiguous_or_missing_publisher'


def test_delayed_clock_probe_does_not_refresh_or_destroy_valid_clock(monkeypatch):
    pytest.importorskip('rclpy')
    from drone_uwb.integration.ros import bridge
    monkeypatch.setattr(bridge.time,'monotonic',lambda:10.1)
    obj=NS(sync_count=30,sync_time=10.,sync_offset=1000000000,sync_remote_ns=100)
    late=NS(remote_timestamp_ns=200,round_trip_time_ms=25.,estimated_offset_ns=1000000000)
    bridge.UwbPx4Bridge.receive_sync(obj,late)
    assert obj.sync_count==30 and obj.sync_time==10. and obj.sync_remote_ns==100
    jump=NS(remote_timestamp_ns=200,round_trip_time_ms=1.,estimated_offset_ns=1010000000)
    bridge.UwbPx4Bridge.receive_sync(obj,jump)
    assert obj.sync_count==1  # A clock change requires acquisition again.


def test_continuous_bridge_keeps_publishing_across_arm_and_disarm(monkeypatch):
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from mavros_msgs.msg import State
    from drone_uwb.integration.ros import bridge
    from drone_uwb.processing.observation_guard import ObservationGuard
    monkeypatch.setattr(bridge.time, 'monotonic', lambda:10.)
    clock = NS(nanoseconds=2_000_000_000)
    outputs = []
    obj = NS(settings=replace(ready(), ground_only=False), connected=True, armed=False,
        state_time=10., params=PARAMS.copy(), param_time=10., sync_count=30, sync_time=10.,
        input_topic='/uwb/btf_pose', count_publishers=lambda topic:1, test_mode=True,
        get_clock=lambda:NS(now=lambda:clock), last_stamp=None, published=0, rejected=0,
        publisher=NS(publish=outputs.append), observation_guard=ObservationGuard())
    obj.current_gate = lambda:bridge.UwbPx4Bridge.current_gate(obj)
    obj.observation_guard.check((2.,1.),1_650_000_000)
    for index, armed in enumerate((False, True, True, False)):
        clock.nanoseconds = 2_000_000_000+index*20_000_000
        state = State(connected=True, armed=armed)
        state.header.stamp.sec = 2
        state.header.stamp.nanosec = index*20_000_000
        bridge.UwbPx4Bridge.receive_state(obj, state)
        msg = PoseWithCovarianceStamped()
        stamp = clock.nanoseconds-100_000_000
        msg.header.frame_id = 'uwb_map'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp,1_000_000_000)
        msg.pose.pose.position.x, msg.pose.pose.position.y = 2.,1.
        msg.pose.covariance[0] = msg.pose.covariance[7] = .09
        bridge.UwbPx4Bridge.receive_pose(obj,msg)
        assert obj.current_gate() == 'ready'
        assert len(outputs) == index+1
    # A stale/duplicate pose still cannot be kept alive by the constant stream.
    bridge.UwbPx4Bridge.receive_pose(obj,msg)
    assert obj.rejected == 1 and len(outputs) == 4
