"""Explicitly synthetic chart/ENU tests, including the clockwise anchor layout."""
from dataclasses import asdict
import math
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest

from drone_mission.contracts import Settings
from drone_mission.session import FlightSession
from drone_mission.mission_chain import MissionChain
from drone_uwb.integration.planar import (
    map_xy_to_enu, enu_xy_to_map, enu_vector_to_map, map_to_enu_matrix,
    map_yaw_to_enu, enu_yaw_to_map)
from drone_uwb.integration.ros.frames import BridgeSettings, rotate_xy_covariance
from drone_uwb.integration.sitl.sitl_odometry_contract import map_reference_xy_to_ned, ned_xy_to_map_reference
from drone_uwb.integration.sitl.sitl_target_contract import global_to_ned
from test_session import settings, snapshot


@pytest.mark.parametrize('sign', [1, -1])
@pytest.mark.parametrize('angle', [0, 30, 90, 180, -140])
def test_observation_targets_inverse_velocity_and_heading_agree(sign, angle):
    values=dict(map_y_axis_sign=sign, enu_yaw_deg=angle, enu_offset_x_m=2., enu_offset_y_m=-3.)
    s, bridge = Settings(**values), BridgeSettings(**values)
    xy, covariance = [4., 1.], np.array([[.04, .01], [.01, .09]])
    enu, cov = rotate_xy_covariance(*xy, covariance, bridge)
    np.testing.assert_allclose(map_xy_to_enu(xy, s), enu, atol=1e-12)
    np.testing.assert_allclose(enu_xy_to_map(enu, s), xy, atol=1e-12)
    ned = map_reference_xy_to_ned(xy, s)
    np.testing.assert_allclose(ned, enu[::-1], atol=1e-12)
    np.testing.assert_allclose(ned_xy_to_map_reference(ned, s), xy, atol=1e-12)
    np.testing.assert_allclose(np.linalg.eigvalsh(cov), np.linalg.eigvalsh(covariance))
    for heading in [0, 45, 90, 180, -90]:
        unit = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
        direction = map_to_enu_matrix(s) @ unit
        enu_yaw = math.degrees(math.atan2(direction[1], direction[0]))
        assert math.remainder(map_yaw_to_enu(heading, s)-enu_yaw, 360) == pytest.approx(0, abs=1e-12)
        assert math.remainder(enu_yaw_to_map(enu_yaw, s)-heading, 360) == pytest.approx(0, abs=1e-12)
        np.testing.assert_allclose(enu_vector_to_map(direction, s), unit, atol=1e-12)


def test_heading_does_not_rotate_fixed_warehouse_goals():
    s=Settings(map_y_axis_sign=-1, enu_yaw_deg=180.)
    np.testing.assert_allclose(map_xy_to_enu([.5,0],s),[-.5,0],atol=1e-12)
    np.testing.assert_allclose(map_xy_to_enu([0,.5],s),[0,.5],atol=1e-12)
    assert map_yaw_to_enu(180,s)==0  # Body faces -map X, ENU +X.
    assert map_yaw_to_enu(90,s)==90  # Turning the body does not change either goal.


def test_reposition_and_scan_yaw_use_same_reflected_chart():
    cfg=settings(map_y_axis_sign=-1,enu_yaw_deg=180.)
    snap=snapshot(10,xy=(2.,2.))
    session=FlightSession(cfg)
    session.move((2.,2.5),snap,10,1_000_000_000)
    north,east=global_to_ned(*session.target_global,snap.origin)
    np.testing.assert_allclose([east,north],[-2.,2.5],atol=.02)
    chain=MissionChain(cfg)
    chain.move((2.,2.5),snap,10,1_000_000_000,yaw_deg=90)
    assert chain.actions[-1]['yaw_rad']==pytest.approx(0.)


@pytest.mark.parametrize('bad', [True, False, 0, 2, -2, 1., '-1', None])
def test_invalid_axis_sign_cannot_become_a_configuration(bad):
    for cls in (Settings, BridgeSettings):
        with pytest.raises(ValueError, match='map_y_axis_sign'):
            cls(map_y_axis_sign=bad)


@pytest.mark.parametrize('child_frame', ['map', 'base_link'])
def test_ros_snapshot_separates_body_heading_and_chart_axes(child_frame):
    pytest.importorskip('rclpy')
    from nav_msgs.msg import Odometry
    from drone_mission.node import FlightNode
    cfg=Settings(map_y_axis_sign=-1,enu_yaw_deg=180.)
    pose=Odometry()
    pose.header.frame_id='map';pose.header.stamp.sec=1
    pose.child_frame_id=child_frame
    pose.pose.pose.position.x=-2.;pose.pose.pose.position.y=3.
    pose.pose.pose.orientation.w=math.sqrt(.5)
    pose.pose.pose.orientation.z=math.sqrt(.5)
    if child_frame=='base_link':pose.twist.twist.linear.x=1.
    else:pose.twist.twist.linear.y=1.
    bridge=asdict(BridgeSettings(input_source='btf_xy',layout_confirmed=True,
                                map_y_axis_sign=-1,enu_yaw_deg=180.))
    obj=NS(settings=cfg,origin=None,param_value=.6,param_received=time.monotonic(),
           command_clients={'land':NS(service_is_ready=lambda:True)},
           age=lambda key:.1,samples={'pose':(pose,0),'bridge':({'settings':bridge},time.monotonic())})
    s=FlightNode.snapshot(obj)
    np.testing.assert_allclose(s.xy,[2,3],atol=1e-12)
    np.testing.assert_allclose(s.velocity_xy,[0,1],atol=1e-12)
    assert s.yaw_deg==pytest.approx(90.)
    assert s.alignment_matches
    bridge['map_y_axis_sign']=1
    assert not FlightNode.snapshot(obj).alignment_matches
    del bridge['map_y_axis_sign']
    assert not FlightNode.snapshot(obj).alignment_matches
