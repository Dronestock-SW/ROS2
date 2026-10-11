"""Anchor trial contracts and simulated state transitions, not flight proof."""
import math
from dataclasses import replace
import pytest
from drone_mission.anchor_trial import validate_anchor_trial
from drone_mission.contracts import parse_request
from drone_mission.field_presets import field_route
from drone_mission.mission_chain import MissionChain
from drone_mission.preflight import field_preflight
from test_session import settings, payload, WALL
from test_mission_chain import sample


def config(**kwargs):
    return settings(full_mission=True, execution_profile='anchor_trial', speed_m_s=.25,
                    expected_ekf2_mag_type=6, expected_mis_takeoff_alt_m=1.3,
                    alignment_confirmed=False, fusion_confirmed=False, timing_confirmed=False,
                    heading_control_confirmed=False, **kwargs)


def inputs(now=10., **kwargs):
    return replace(sample(now, rc_mode=3, height_age_s=math.inf,
                          vertical_estimator_valid=True), **kwargs)


def test_manual_evidence_is_optional_but_live_inputs_remain_required():
    cfg = config()
    report = field_preflight(cfg, inputs(), map_loaded=False, recording_ok=True)
    assert report['checked_inputs_passed']
    assert not cfg.heading_control_confirmed and not cfg.fusion_confirmed
    optional = {x['code'] for x in report['checks'] if not x['required']}
    assert {'height', 'map', 'alignment_confirmed', 'fusion_confirmed', 'timing_confirmed',
            'heading_control_confirmed'} <= optional
    assert 'rc_mode' not in {x['code'] for x in report['checks']}
    for changes, blocker in [({'vertical_estimator_valid':False}, 'vertical_estimator'),
                             ({'estimator_age_s':1}, 'estimator'),
                             ({'rc_valid':False}, 'rc'), ({'origin':None}, 'origin'),
                             ({'alignment_matches':False}, 'transform')]:
        report = field_preflight(cfg, inputs(**changes), map_loaded=False, recording_ok=True)
        assert blocker in report['blockers'] and not report['checked_inputs_passed']


@pytest.mark.parametrize('case', ['hover', 'x', 'y', 'xy'])
def test_four_routes_start_and_return_in_anchor_frame_with_no_yaw_commands(case):
    cfg = config()
    route = field_route(case, (2.,2.), cfg)
    assert all('yaw_deg' not in p for p in route)
    assert route[-1]['x'] == 2. and route[-1]['y'] == 2.
    assert max(p['x'] for p in route) == (3. if case in ('x','xy') else 2.)
    assert max(p['y'] for p in route) == (3. if case in ('y','xy') else 2.)
    req = payload(route_tasks=route, ceiling_height_m=3.)
    proof = validate_anchor_trial(req, (2.,2.), cfg, WALL)
    assert proof['speed_m_s'] == .25 and not proof['obstacle_map_checked']
    if case == 'hover':
        assert route[0]['dwell_s'] == 10.
    chain = MissionChain(cfg)
    chain._fixture_payload = req
    for now in (7.,8.,9.):
        assert chain.tick(inputs(now),now,WALL-10+now,round((WALL+now)*1e9)) == []
    assert chain.submit(req,10.,WALL)
    action = chain.tick(inputs(),10.,WALL,round(WALL*1e9))[0]
    assert action['kind'] == 'takeoff_mode'
    chain.command_result(action['token'],True,10.01)
    action = chain.tick(inputs(10.1,mode='AUTO.TAKEOFF'),10.1,WALL+.1,round(WALL*1e9))[0]
    assert action['kind']=='arm'


def test_trial_rejects_scan_yaw_long_leg_excess_speed_and_low_ceiling():
    cfg = config()
    base = payload(ceiling_height_m=3.)
    for point in [dict(id='S',type='scan',x=2.,y=2.),dict(id='P',x=2.,y=2.,yaw_deg=0)]:
        with pytest.raises(ValueError,match='xy_and_hover'):
            parse_request(dict(base,route_tasks=[point]),cfg,WALL)
    with pytest.raises(ValueError,match='leg'):
        validate_anchor_trial(base,(4.,2.),cfg,WALL)
    with pytest.raises(ValueError,match='ceiling'):
        validate_anchor_trial(dict(base,ceiling_height_m=1.4),(2.,2.),cfg,WALL)
    with pytest.raises(ValueError,match='speed'):
        validate_anchor_trial(base,(2.,2.),replace(cfg,speed_m_s=.3),WALL)


def test_vertical_estimator_loss_and_rc_takeover_stop_the_active_trial():
    chain = MissionChain(config())
    chain.last_web_s = 10.
    assert chain.ready(inputs(),10.,ground=True)=='ready'
    assert chain.ready(inputs(vertical_estimator_valid=False),10.)=='px4_vertical_estimator_unavailable'
    chain.phase='MOVING';chain.home=(2.,2.)
    assert chain.tick(inputs(mode='POSCTL',armed=True,landed=2),10.,WALL,1)==[]
    assert chain.phase=='PILOT_OVERRIDE'


@pytest.mark.parametrize('case', ['hover', 'x', 'y', 'xy'])
def test_complete_native_cycle_uses_slow_xy_and_returns_before_landing(case):
    cfg=config();chain=MissionChain(cfg)
    req=payload(route_tasks=field_route(case,(2.,2.),cfg),ceiling_height_m=3.)
    for now in (7.,8.,9.):
        chain.tick(inputs(now),now,WALL-10+now,round((WALL+now)*1e9))
    assert chain.submit(req,10.,WALL)
    kinds=[];dwelling_since=None
    for index in range(2000):
        now=10.+index*.05;ns=round((WALL+now)*1e9)
        phase=chain.phase
        state=dict(mode='POSCTL',armed=False,landed=1)
        if phase in ('PREPARING','ARMING'):
            state.update(mode='AUTO.TAKEOFF',armed=phase=='ARMING')
        elif phase=='LANDING':
            state.update(mode='AUTO.LAND')
        elif phase in chain.ACTIVE:
            state.update(mode='AUTO.LOITER',armed=True,landed=2)
        if chain.target_global:
            chain.applied_target(*chain.target_global,ns)
        chain.submit(req,now,WALL+now-10)
        actions=chain.tick(inputs(now,xy=chain.monitor.target or (2.,2.),**state),
                           now,WALL+now-10,ns)
        if chain.phase=='DWELL' and dwelling_since is None:dwelling_since=now
        for action in actions:
            kinds.append(action['kind'])
            if action['kind']=='reposition':
                assert action['speed']==.25 and 'yaw_rad' not in action and 'z' not in action
            if action['kind']=='land':
                assert chain.home_verified and chain.completed_waypoints==len(req['route_tasks'])
                if case=='hover':assert now-dwelling_since>=10.
            chain.command_result(action['token'],True,now+.001)
        if chain.phase=='END':
            assert chain.status()['mission_complete']
            assert kinds.count('arm')==1 and kinds.count('land')==1
            return
    raise AssertionError(chain.status())


@pytest.mark.parametrize("age, allowed", [(.202366484, True), (.35, True), (.350001, False), (float("inf"), False)])
def test_bridge_snapshot_age_budget_matches_web_and_runner(age, allowed):
    cfg = config()
    chain = MissionChain(cfg)
    chain.last_web_s = 10.
    s = inputs(bridge_observation_age_s=age)
    report = field_preflight(cfg, s, map_loaded=False, recording_ok=True)
    assert ("bridge" not in report["blockers"]) is allowed
    assert (chain.ready(s, 10., ground=True) == "ready") is allowed
    assert chain.ready(replace(s, uwb_age_s=.251), 10., ground=True) == "uwb_unavailable"
