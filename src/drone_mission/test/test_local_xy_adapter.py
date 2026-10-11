"""Check anchor/ENU command and feedback boundary without starting ROS."""
import math
from types import SimpleNamespace
import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from drone_mission.node import FlightNode, local_target_message
from drone_mission.contracts import Settings


def test_mavros_receives_enu_once_with_fixed_fc_height_and_yaw():
    cfg=Settings(command_frame='local_xy',execution_profile='anchor_trial',full_mission=True,
                 enu_yaw_deg=90.,map_y_axis_sign=-1,enu_offset_x_m=2.,enu_offset_y_m=-3.)
    msg=local_target_message(dict(xy=(1.,2.),altitude_enu_m=1.73,yaw_deg=40.),cfg,Time(sec=10))
    assert (msg.position.x,msg.position.y,msg.position.z)==pytest.approx((4.,-2.,1.73))
    assert msg.yaw==pytest.approx(math.radians(50.))
    assert msg.coordinate_frame==1 and msg.type_mask==2552
    received=[]
    obj=SimpleNamespace(settings=cfg,get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_100_000_000)),
                        session=SimpleNamespace(applied_local_target=lambda *v:received.append(v)))
    FlightNode.local_target_received(obj,msg)
    assert received[0][0]==pytest.approx((1.,2.)) and received[0][1]==1.73
    msg.header.stamp=Time(sec=9)
    FlightNode.local_target_received(obj,msg)
    assert len(received)==1
    msg.header.stamp=Time(sec=10);msg.type_mask|=4
    FlightNode.local_target_received(obj,msg)
    assert len(received)==1
