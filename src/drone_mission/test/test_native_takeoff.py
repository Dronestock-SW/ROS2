"""Synthetic FC checks: native ascent never streams OFFBOARD or auto-starts a route."""
from dataclasses import replace
import time
import pytest
from drone_mission.local_xy import LocalXYMission
from drone_mission.local_web import LocalPlatform
from drone_mission.field_presets import field_route
from test_local_xy import sample
from test_anchor_trial import config
from test_session import payload, WALL


def rig():
    cfg = config(command_frame='local_xy', start_mode='takeoff_hold')
    m = LocalXYMission(cfg)
    for now in (6.,7.,8.,9.,10.): m.tick(sample(now),now,WALL,time.time_ns())
    request = dict(action='takeoff', request_id='native-takeoff-1', requested_at='synthetic')
    m.begin_native(request,sample(10),10.)
    return m


def advance(m, now, **changes):
    m.last_web_s=now
    return m.tick(sample(now,**changes),now,WALL,time.time_ns())


def hold(m):
    action=advance(m,10.)[0]; assert action['kind']=='takeoff_mode'
    m.command_result(action['token'],True,10.)
    action=advance(m,10.1,mode='AUTO.TAKEOFF')[0]; assert action['kind']=='arm'
    m.command_result(action['token'],True,10.1)
    advance(m,10.2,mode='AUTO.TAKEOFF',armed=True)
    assert m.phase=='NATIVE_WAIT_ASCENT'
    advance(m,10.3,mode='AUTO.TAKEOFF',armed=True,landed=2)
    assert m.phase=='NATIVE_CLIMB'
    for now in (11.,12.,13.,14.1):
        assert advance(m,now,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4)==[]
        assert m.local_setpoint() is None
    assert m.phase=='HOLD_READY' and m.native_ready


def test_takeoff_hold_waits_for_separate_start_without_any_local_target():
    m=rig();hold(m)
    for now in (15.,20.,25.):
        assert advance(m,now,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4)==[]
        assert m.phase=='HOLD_READY' and m.assignment is None and m.local_setpoint() is None
    req=payload(route_tasks=field_route('hover',(2.,2.),m.settings))
    assert m.submit(req,25.,WALL)
    advance(m,25.05,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4)
    assert m.phase=='PREPARING'
    assert m.local_setpoint()['altitude_enu_m']==pytest.approx(1.4)
    actions=[]
    for i in range(1,30):
        now=25.05+i*.05
        commands=advance(m,now,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4)
        actions+=commands
        for a in commands:m.command_result(a['token'],True,now)
    assert [a['kind'] for a in actions]==['offboard_mode']
    advance(m,26.6,mode='OFFBOARD',armed=True,landed=2,altitude_enu_m=1.4)
    assert m.phase=='HANDOFF' and m.height_target==pytest.approx(1.4)


@pytest.mark.parametrize('phase',['NATIVE_MODE','NATIVE_CLIMB','HOLD_READY'])
def test_rc_takeover_is_latched_and_never_reclaims(phase):
    m=rig()
    if phase!='NATIVE_MODE':hold(m);m.phase=phase
    assert advance(m,15.,mode='POSCTL',armed=True,landed=2)==[] if phase!='NATIVE_MODE' else True
    if phase=='NATIVE_MODE':advance(m,10.,mode='ALTCTL')
    assert m.phase=='PILOT_OVERRIDE'
    assert not m.submit(payload(),16.,WALL)
    assert m.local_setpoint() is None


def test_no_start_before_hold_no_queued_launch_after_takeoff():
    m=rig()
    assert not m.submit(payload(),10.,WALL)
    assert m.intent is None
    hold(m)
    assert m.intent is None and m.assignment is None


def test_stale_web_or_drifting_hold_requests_land_not_offboard():
    m=rig();hold(m)
    m.last_web_s=10.
    a=m.tick(sample(15.,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4),15.,WALL,time.time_ns())
    assert [x['kind'] for x in a]==['land']
    m=rig();hold(m)
    a=advance(m,15.,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4,xy=(2.7,2.))
    assert [x['kind'] for x in a]==['land']


def test_unknown_mode_response_never_arms():
    m=rig();a=advance(m,10.)
    assert a[0]['kind']=='takeoff_mode'
    assert advance(m,14.)==[] and m.phase=='UNCONFIRMED'


def test_one_hz_fc_state_uses_configured_timeout_but_stale_state_stops():
    m=rig();hold(m)
    assert advance(m,15.,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4,
                   state_age_s=1.1,landed_age_s=1.1)==[]
    assert m.phase=='HOLD_READY'
    advance(m,16.,mode='AUTO.LOITER',armed=True,landed=2,altitude_enu_m=1.4,
            state_age_s=m.settings.state_timeout_s+.01)
    assert m.phase=='UNCONFIRMED' and not m.native_ready


@pytest.mark.parametrize('speeds',[(),(.5,.5,.25,.6),(.5,.5,.5,.25)])
def test_incompatible_native_speed_configuration_never_arms(speeds):
    m=rig();m.phase='IDLE'
    assert m.takeoff_blocker(sample(10.,native_takeoff_speeds=speeds),10.)=='native_takeoff_speed_incompatible'


def test_armed_on_ground_is_not_reported_as_climb():
    m=rig();a=advance(m,10.)[0];m.command_result(a['token'],True,10.)
    a=advance(m,10.1,mode='AUTO.TAKEOFF')[0];m.command_result(a['token'],True,10.1)
    advance(m,10.2,mode='AUTO.TAKEOFF',armed=True)
    assert m.phase=='NATIVE_WAIT_ASCENT' and not m.native_ready
    advance(m,11.,mode='AUTO.TAKEOFF',armed=False)
    assert m.reason=='disarmed_before_liftoff'


def test_web_separate_takeoff_idempotent_and_start_only_in_hold(tmp_path):
    cfg=config(command_frame='local_xy',start_mode='takeoff_hold')
    p=LocalPlatform(cfg.drone_id,cfg.layout_id,settings=cfg,mission_file=tmp_path/'m.sqlite3')
    try:
        p.site['ceiling_height_m']=3.
        p.telemetry_received_s=time.monotonic()
        p.telemetry=dict(fc_connected=True,fc_armed=False,fc_landed=1,fc_mode='POSCTL',
            flight_state='IDLE',operator_session_id='session',native_takeoff=dict(can_takeoff=True),
            preflight=dict(checked_inputs_passed=True))
        assert p.can_takeoff() and not p.can_start() and not p.can_arm()
        req=dict(action='takeoff',client_request_id='takeoff-request',operator_session_id='session')
        first=p.takeoff(req)
        assert p.takeoff(req)==first and p.store.active() is None
        p.assignment['control_action']=None
        p.telemetry.update(fc_armed=True,fc_landed=2,fc_mode='AUTO.TAKEOFF',flight_state='NATIVE_CLIMB')
        assert not p.can_start() and not p.can_takeoff()
        p.telemetry.update(fc_mode='AUTO.LOITER',flight_state='HOLD_READY',native_takeoff=dict(can_start=True))
        assert p.can_start()
        p.telemetry_received_s-=2
        assert not p.can_start()
    finally:p.store.close()
