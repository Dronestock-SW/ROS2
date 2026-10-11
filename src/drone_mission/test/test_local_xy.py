import math
from dataclasses import replace
import pytest
from drone_mission.local_xy import LocalXYMission
from drone_mission.field_presets import field_route
from drone_mission.preflight import field_preflight
from test_anchor_trial import config, inputs
from test_session import payload, WALL


def rig(case='hover'):
    cfg = config(command_frame='local_xy')
    m = LocalXYMission(cfg)
    m.test_request = payload(route_tasks=field_route(case,(2.,2.),cfg))
    for now in (7.,8.,9.):
        m.tick(sample(now),now,WALL+now-10,round((WALL+now)*1e9))
    assert m.submit(m.test_request,10.,WALL)
    return m


def sample(now=10., **kw):
    return replace(inputs(now, origin=None, altitude_enu_m=.1, local_setpoint_ready=True), **kw)


def test_local_preflight_has_current_xy_and_never_demands_geographic_origin():
    report=field_preflight(config(command_frame='local_xy'),sample(),map_loaded=False,recording_ok=True)
    assert report['checked_inputs_passed']
    assert 'origin' not in {x['code'] for x in report['checks']}
    assert 'local_start' in {x['code'] for x in report['checks']}
    report=field_preflight(config(command_frame='local_xy'),sample(local_setpoint_ready=False),map_loaded=False,recording_ok=True)
    assert 'local_transport' in report['blockers']


@pytest.mark.parametrize('case',['hover','x','y','xy'])
def test_local_full_cycle_no_origin_all_four_routes(case):
    m=rig(case);now=9.95;armed=False;mode='POSCTL';landed=1;alt=.1;xy=(2.,2.)
    prior=None;commands=[];max_step=0.;path_length=0.
    for _ in range(2600):
        now+=.05;ns=round((WALL+now)*1e9)
        goal=m.local_setpoint()
        if goal:
            xy=tuple(goal['xy'])
            alt+=max(-.025,min(.025,goal['altitude_enu_m']-alt))
            m.applied_local_target(xy,goal['altitude_enu_m'],ns)
            if prior:
                step=math.dist(xy,prior);max_step=max(max_step,step);path_length+=step
            prior=xy
        if armed and alt>.15:landed=2
        s=replace(sample(now),armed=armed,mode=mode,landed=landed,xy=xy,uwb_xy=xy,altitude_enu_m=alt)
        actions=m.tick(s,now,WALL+now-10,ns)
        m.submit(m.test_request,now,WALL+now-10)
        for a in actions:
            commands.append(a['kind']);m.command_result(a['token'],True,now)
            if a['kind']=='offboard_mode':mode='OFFBOARD'
            elif a['kind']=='arm':armed=True
            elif a['kind']=='land':mode='AUTO.LAND';armed=False;landed=1
        if m.phase in m.TERMINAL:break
    assert m.phase=='END',(m.phase,m.reason)
    assert m.home==(2.,2.) and m.home_verified and m.landing_verified
    assert m.status()['mission_complete'] and m.local_setpoint() is None
    assert commands==['offboard_mode','arm','land']
    assert max_step<=.25*.05+1e-9
    assert m.origin is None and m.target_global is None
    assert path_length==pytest.approx({'hover':0.,'x':2.,'y':2.,'xy':4.}[case])


def prepared():
    m=rig()
    m.tick(sample(),10.,WALL,round(WALL*1e9))
    assert m.phase=='PREPARING'
    return m


def test_no_stream_before_start_and_no_arm_without_actual_mode():
    m=rig();assert m.local_setpoint() is None
    m=prepared()
    for i in range(1,121):
        now=10+i*.05;m.last_web_s=now
        a=m.tick(sample(now,mode='POSCTL'),now,WALL+now-10,round((WALL+now)*1e9))
        assert all(x['kind']!='arm' for x in a)
        for x in a:m.command_result(x['token'],True,now)


@pytest.mark.parametrize('changes',[
    dict(uwb_age_s=.26),dict(bridge_observation_age_s=.36),dict(local_setpoint_ready=False),
    dict(vertical_estimator_valid=False),dict(rc_override=1),dict(pose_age_s=.3)])
def test_stale_inputs_abort_before_arm(changes):
    m=prepared();m.last_web_s=10.05
    a=m.tick(replace(sample(10.05),**changes),10.05,WALL+.05,round((WALL+.05)*1e9))
    assert not m.local_setpoint()
    assert [x['kind'] for x in a]==['position_mode']


def test_rc_takeover_and_unknown_command_timeout_never_resume_stream():
    m=prepared();m.last_web_s=10.05
    m.tick(sample(10.05,mode='ALTCTL'),10.05,WALL+.05,round((WALL+.05)*1e9))
    assert m.phase=='PILOT_OVERRIDE' and m.local_setpoint() is None
    assert not m.submit(payload(request_id='new'),10.1,WALL+.1)
    m=prepared();m.issue('arm',10.)
    m.tick(sample(14.),14.,WALL+4,round((WALL+4)*1e9))
    assert m.phase=='UNCONFIRMED' and m.local_setpoint() is None
