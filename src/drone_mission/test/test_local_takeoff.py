import time
import pytest
from drone_mission.local_xy import LocalXYMission
from drone_mission.local_takeoff import sticks_centered
from test_local_xy import sample
from test_anchor_trial import config
from test_session import WALL, payload


def rig():
    m=LocalXYMission(config(command_frame='local_xy',start_mode='takeoff_hold',takeoff_backend='local_position'))
    for now in (6.,7.,8.,9.,10.):m.tick(sample(now),now,WALL,time.time_ns())
    m.begin_native(dict(action='takeoff',request_id='local-test',requested_at='synthetic'),sample(10.),10.)
    return m


def reach_position(m):
    mode='POSCTL';armed=False;alt=.1;landed=1;commands=[]
    for i in range(280):
        now=10.+i*.05;ns=time.time_ns();m.last_web_s=now
        goal=m.local_setpoint()
        if goal:
            assert tuple(goal['xy'])==m.native_home
            m.applied_local_target(goal['xy'],goal['altitude_enu_m'],ns)
            if armed:alt=min(goal['altitude_enu_m'],alt+.025);landed=2 if alt>.2 else 1
        actions=m.tick(sample(now,mode=mode,armed=armed,landed=landed,altitude_enu_m=alt),now,WALL,ns)
        for a in actions:
            commands.append(a['kind']);m.command_result(a['token'],True,now)
            if a['kind']=='offboard_mode':mode='OFFBOARD'
            elif a['kind']=='arm':armed=True
            elif a['kind']=='position_mode':mode='POSCTL'
        if m.native_ready:break
    assert m.phase=='HOLD_READY' and m.native_ready
    assert commands==['offboard_mode','arm','position_mode']
    assert m.local_setpoint() is None and m.assignment is None
    assert m.status()['home_xy_m'] == list(m.native_home)
    return now,alt


def test_local_takeoff_separate_start_uses_current_height_and_never_rearms():
    m=rig();now,alt=reach_position(m)
    assert m.submit(payload(),now,WALL)
    m.tick(sample(now,mode='POSCTL',armed=True,landed=2,altitude_enu_m=alt),now,WALL,time.time_ns())
    assert m.phase=='PREPARING' and m.height_target==pytest.approx(alt)


def test_ground_warmup_does_not_require_prearm_controller_echo():
    m=rig();m.last_web_s=11.
    assert m.tick(sample(11.),11.,WALL,time.time_ns())==[]
    m.last_web_s=11.3
    actions=m.tick(sample(11.3),11.3,WALL,time.time_ns())
    assert [a['kind'] for a in actions]==['offboard_mode']
    assert m.phase=='LOCAL_TAKEOFF_MODE'
    assert m.feedback_ns is None


def test_position_rc_takeover_never_reclaims_or_starts():
    m=rig();now,alt=reach_position(m);m.last_web_s=now+.1
    assert m.tick(sample(now+.1,mode='POSCTL',armed=True,landed=2,altitude_enu_m=alt,
        rc_sticks_centered=False),now+.1,WALL,time.time_ns())==[]
    assert m.phase=='PILOT_OVERRIDE' and m.local_setpoint() is None
    assert not m.submit(payload(),now+.2,WALL)


def test_calibrated_sticks_center_including_throttle_midpoint():
    params={f'RC_MAP_{axis}':i for i,axis in enumerate(('ROLL','PITCH','YAW','THROTTLE'),1)}
    params.update({f'RC{i}_{k}':v for i in range(1,5) for k,v in [('MIN',1000),('MAX',2000),('TRIM',1000 if i==4 else 1500)]})
    assert sticks_centered((1500,1500,1500,1500),params)
    assert not sticks_centered((1500,1500,1500,1000),params)
    assert not sticks_centered((1500,)*4,{})
