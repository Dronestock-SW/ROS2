from dataclasses import replace
from datetime import datetime, timezone
import math

import pytest

from drone_mission.contracts import LAYOUT, Settings
from drone_mission.session import FlightSession, Snapshot
from drone_uwb.integration.sitl.sitl_target_contract import PX4GlobalReference


WALL = 1_791_417_600.


def settings(**changes):
    return replace(Settings(execute=True, layout_confirmed=True, alignment_confirmed=True,
        fusion_confirmed=True, timing_confirmed=True, sensor_mount_confirmed=True,
        takeoff_settings_confirmed=True), **changes)


def payload(action='start', request_id='start-1', **changes):
    result = dict(ok=True, contract_version='1.0', drone_id='5', status='ACTIVE',
        control_action=action, control_request_id=request_id,
        control_requested_at=datetime.fromtimestamp(WALL, timezone.utc).isoformat(),
        mission_db_id=1, mission_code='LOCAL-1', route_revision='r1',
        anchor_layout_id=LAYOUT, coordinate_frame='UWB_ANCHOR_LOCAL',
        origin='A1', x_axis='A1_TO_A2', y_axis='A1_TO_A3', z_axis='UP_FROM_FLOOR', unit='meter',
        route_tasks=[dict(id='P1', type='waypoint', x=2.3, y=2.)])
    result.update(changes)
    return result


def snapshot(now=10., **changes):
    return replace(Snapshot(command_services_ready=True, land_service_ready=True,
        connected=True, mode='POSCTL',
        landed=1, state_age_s=0., landed_age_s=0., estimator_age_s=0., estimator_valid=True,
        pose_stamp_ns=round((WALL+now)*1e9), pose_age_s=0., xy=(2., 2.), velocity_xy=(0., 0.),
        uwb_stamp_ns=round((WALL+now)*1e9), uwb_age_s=0., uwb_xy=(2., 2.), height_age_s=0.,
        bridge_age_s=0., bridge_observation_age_s=0., bridge_ready=True,
        bridge_published=100, alignment_matches=True,
        origin=PX4GlobalReference(47., 8., 1), takeoff_param_age_s=0., takeoff_alt_m=.6), **changes)


def tick(session, now, *, target_feedback=True, **changes):
    if target_feedback and session.target_applied:
        session.applied_target(*session.target_global, round((WALL+now)*1e9))
    return session.tick(snapshot(now, **changes), now, WALL+now-10, round((WALL+now)*1e9))


def flying(session=None):
    session = session or FlightSession(settings())
    assert session.submit(payload(), 10., WALL)
    arm = tick(session, 10.)[0]
    assert arm['kind'] == 'arm'
    session.command_result(arm['token'], True, 10.01)
    takeoff = tick(session, 10.05, armed=True)[0]
    assert takeoff['kind'] == 'takeoff'
    session.command_result(takeoff['token'], True, 10.06)
    move = tick(session, 10.1, armed=True, landed=2, mode='AUTO.LOITER')[0]
    assert move['kind'] == 'reposition'
    session.command_result(move['token'], True, 10.11)
    return session, move


def test_web_start_to_takeoff_target_arrival_land_and_disarm():
    remembered = []
    session, target = flying(FlightSession(settings(), remember=remembered.append))
    assert remembered == ['start-1']
    assert not session.status()['mission_complete']
    assert not session.status()['target_applied']  # MAVROS success is a handoff.
    assert session.applied_target(target['x']/1e7, target['y']/1e7, round((WALL+10.12)*1e9))
    land = None
    for i in range(1, 61):
        now = 10.1+i*.05
        session.submit(payload(), now, WALL+now-10)
        actions = tick(session, now, armed=True, landed=2, mode='AUTO.LOITER', xy=(2.3, 2.))
        if actions:
            land = actions[0]
            break
    assert land and land['kind'] == 'land'
    assert session.status()['completed_waypoints'] == 1
    session.command_result(land['token'], True, now+.01)
    tick(session, now+.05, armed=True, landed=1, mode='AUTO.LAND')
    assert session.phase == 'LANDING'  # Ground alone is not completed/disarmed.
    tick(session, now+.1, armed=False, landed=1, mode='AUTO.LAND')
    assert session.status()['mission_complete']
    assert session.status()['landing_verified']


@pytest.mark.parametrize('changes', [
    {'unit':'cm'}, {'x_axis':'A1_TO_A3'}, {'anchor_layout_id':'other-layout'},
    {'drone_id':'6'}, {'route_revision':''},
    {'route_tasks':[dict(id='P1',type='waypoint',x=math.nan,y=2)]},
    {'route_tasks':[dict(id='P1',type='waypoint',x=6,y=2)]},
    {'route_tasks':[dict(id='P1',type='scan',x=2,y=2)]},
    {'route_tasks':[dict(id='P1',x=2,y=2),dict(id='P1',x=2.1,y=2)]},
])
def test_invalid_web_coordinates_never_arm(changes):
    s=FlightSession(settings())
    assert not s.submit(payload(**changes),10.,WALL)
    assert tick(s,10.) == []


@pytest.mark.parametrize('field,value', [
    ('command_services_ready',False), ('connected',False), ('state_age_s',3.),
    ('estimator_valid',False), ('pose_age_s',.5), ('uwb_age_s',.5),
    ('height_age_s',.5), ('bridge_ready',False), ('bridge_published',0),
    ('bridge_observation_age_s',.3),
    ('alignment_matches',False), ('origin',None), ('takeoff_alt_m',2.5),
])
def test_ground_readiness_blocks_start(field,value):
    s=FlightSession(settings())
    s.submit(payload(),10.,WALL)
    assert tick(s,10.,**{field:value}) == []
    assert s.phase == 'READY'


@pytest.mark.parametrize('changes', [
    {'uwb_age_s':.6}, {'pose_age_s':.6}, {'height_age_s':.6},
    {'estimator_valid':False}, {'alignment_matches':False}, {'bridge_observation_age_s':.3},
    {'origin':PX4GlobalReference(48.,8.,2)},
])
def test_inflight_input_loss_requests_land_once_and_never_auto_resumes(changes):
    s,_=flying()
    actions=tick(s,10.2,armed=True,landed=2,mode='AUTO.LOITER',**changes)
    assert [a['kind'] for a in actions] == ['land']
    s.command_result(actions[0]['token'],True,10.21)
    assert tick(s,10.3,armed=True,landed=2,mode='AUTO.LAND') == []
    assert s.phase == 'LANDING'
    tick(s,10.4,mode='AUTO.LAND')
    assert s.phase == 'FAILED'
    assert s.status()['landing_verified']
    s.submit(payload(),10.5,WALL+.5)
    assert tick(s,10.5) == []


def test_manual_override_never_sends_land_or_restarts_on_late_reply():
    s,command=flying()
    assert tick(s,10.2,armed=True,landed=2,mode='POSCTL') == []
    assert s.phase == 'PILOT_OVERRIDE'
    s.command_result(command['token'],True,10.3)
    assert tick(s,10.4,armed=True,landed=2,mode='AUTO.LOITER') == []


def test_manual_override_during_arming_prevents_takeoff():
    s = FlightSession(settings())
    s.submit(payload(), 10., WALL)
    arm = tick(s, 10.)[0]
    assert tick(s, 10.1, armed=True, mode='ALTCTL') == []
    assert s.phase == 'PILOT_OVERRIDE'
    s.command_result(arm['token'], True, 10.2)
    assert tick(s, 10.3, armed=True) == []


def test_handoff_without_fresh_matching_fc_target_cannot_complete():
    s,target=flying()
    assert not s.applied_target(target['x']/1e7,target['y']/1e7,1)
    assert not s.applied_target(1.,2.,round((WALL+10.2)*1e9))
    s.submit(payload(),13.2,WALL+3.2)
    actions=tick(s,13.2,armed=True,landed=2,mode='AUTO.LOITER',xy=(2.3,2.))
    assert actions[0]['kind'] == 'land'
    assert s.reason == 'px4_target_application_unverified'
    assert not s.status()['mission_complete']


def test_request_replay_and_restart_do_not_launch_again():
    s=FlightSession(settings(),consumed=['start-1'])
    assert not s.submit(payload(),10.,WALL)
    assert tick(s,10.) == []
    s=FlightSession(settings())
    assert not s.submit(payload(),50.,WALL+40)
    assert tick(s,50.) == []


def test_active_route_revision_or_same_revision_content_change_rejected():
    s,_=flying()
    assert not s.submit(payload(request_id='start-2',route_revision='r2'),10.2,WALL+.2)
    assert not s.submit(payload(route_tasks=[dict(id='P1',x=2.6,y=2.)]),10.2,WALL+.2)
    assert s.assignment['waypoints'][0]['xy'] == (2.3,2.)


def test_waypoint_leg_limit_checked_before_arming():
    s=FlightSession(settings())
    assert s.submit(payload(route_tasks=[dict(id='P1',x=3.5,y=2.)]),10.,WALL)
    assert tick(s,10.) == []
    assert s.validation['reason'] == 'route_leg_exceeds_trial_limit'


def test_http_outage_requests_land_and_monitor_mode_never_arms():
    s,_=flying()
    assert tick(s,12.2,armed=True,landed=2,mode='AUTO.LOITER')[0]['kind'] == 'land'
    assert s.reason == 'web_assignment_stale'
    s=FlightSession(settings(execute=False))
    s.submit(payload(),10.,WALL)
    assert tick(s,10.) == []
    assert s.reason == 'execution_disabled'


def test_acknowledged_return_command_keeps_route_poll_heartbeat():
    s, _ = flying()
    request = payload(action='return_to_home', request_id='return-1')
    assert s.submit(request, 10.2, WALL+.2)
    target = tick(s, 10.2, armed=True, landed=2, mode='AUTO.LOITER')[0]
    assert target['kind'] == 'reposition'
    s.command_result(target['token'], True, 10.21)
    assert s.applied_target(target['x']/1e7, target['y']/1e7, round((WALL+10.22)*1e9))
    # The server retains the last request ID after clearing its action.
    polled = dict(request, control_action=None)
    assert s.submit(polled, 12.3, WALL+2.3)
    assert tick(s, 12.3, armed=True, landed=2, mode='AUTO.LOITER') == []
    assert s.phase == 'RETURNING'


def test_request_ledger_failure_happens_before_arming():
    def failed_write(request_id):
        raise OSError('fixture_disk_failure')

    s = FlightSession(settings(), remember=failed_write)
    s.submit(payload(), 10., WALL)
    with pytest.raises(OSError):
        tick(s, 10.)
    assert s.actions == []
    assert 'start-1' not in s.consumed


@pytest.mark.parametrize('manual_mode', ['POSCTL', 'ALTCTL'])
def test_restart_never_takes_over_a_manual_flight(manual_mode):
    s = FlightSession(settings())
    assert tick(s, 10., armed=True, landed=2, mode=manual_mode) == []


def test_restart_in_autonomous_airborne_state_lands_once_without_rearming():
    s = FlightSession(settings(), consumed=['start-1'])
    assert not s.submit(payload(), 10., WALL)
    assert tick(s, 10., armed=True, landed=2, mode='AUTO.LOITER', land_service_ready=False) == []
    assert s.reason == 'recovery_waiting_for_land_service'
    land = tick(s, 10.1, armed=True, landed=2, mode='AUTO.LOITER')[0]
    assert land['kind'] == 'land'
    s.command_result(land['token'], True, 10.11)
    assert tick(s, 10.2, armed=True, landed=2, mode='AUTO.LAND') == []
    tick(s, 10.3, mode='AUTO.LAND')
    assert s.phase == 'FAILED'
    assert s.failure == 'airborne_without_active_session'
    assert s.landing_verified and not s.status()['mission_complete']


def test_web_land_during_arming_cancels_takeoff_despite_late_arm_reply():
    s = FlightSession(settings())
    s.submit(payload(), 10., WALL)
    arm = tick(s, 10.)[0]
    s.submit(payload(action='land', request_id='cancel-1'), 10.1, WALL+.1)
    disarm = tick(s, 10.1, armed=True)[0]
    assert disarm['kind'] == 'disarm'
    s.command_result(arm['token'], True, 10.11)
    assert tick(s, 10.2, armed=True) == []
    s.command_result(disarm['token'], True, 10.21)
    tick(s, 10.3)
    assert s.phase == 'FAILED' and s.failure == 'web_start_cancelled'


@pytest.mark.parametrize('feedback', ['changed', 'expired'])
def test_lost_or_changed_fc_target_after_initial_match_cannot_complete(feedback):
    s, target = flying()
    assert s.applied_target(target['x']/1e7, target['y']/1e7, round((WALL+10.12)*1e9))
    if feedback == 'changed':
        assert not s.applied_target(target['x']/1e7+1e-5, target['y']/1e7,
                                    round((WALL+10.2)*1e9))
    s.submit(payload(), 10.8, WALL+.8)
    actions = tick(s, 10.8, target_feedback=False, armed=True, landed=2, mode='AUTO.LOITER')
    assert actions[0]['kind'] == 'land'
    assert s.failure == ('px4_target_changed' if feedback == 'changed'
                         else 'px4_target_feedback_stale')
    assert not s.status()['mission_complete']


def test_repeated_waypoint_starts_a_new_dwell_period():
    s = FlightSession(settings())
    route = [dict(id='P1', x=2.3, y=2.), dict(id='P2', x=2.3, y=2.)]
    start = payload(route_tasks=route)
    assert s.submit(start, 10., WALL)
    arm = tick(s, 10.)[0]
    s.command_result(arm['token'], True, 10.01)
    takeoff = tick(s, 10.05, armed=True)[0]
    s.command_result(takeoff['token'], True, 10.06)
    target = tick(s, 10.1, armed=True, landed=2, mode='AUTO.LOITER')[0]
    s.command_result(target['token'], True, 10.11)
    s.applied_target(target['x']/1e7, target['y']/1e7, round((WALL+10.12)*1e9))
    for i in range(1, 61):
        now = 10.1+i*.05
        s.submit(start, now, WALL+now-10.)
        actions = tick(s, now, armed=True, landed=2, mode='AUTO.LOITER', xy=(2.3,2.))
        if actions:
            break
    assert actions[0]['kind'] == 'reposition'
    assert s.completed_waypoints == 1 and s.arrival is None
    target = actions[0]
    s.command_result(target['token'], True, now+.01)
    s.applied_target(target['x']/1e7, target['y']/1e7, round((WALL+now+.02)*1e9))
    assert tick(s, now+.05, armed=True, landed=2, mode='AUTO.LOITER', xy=(2.3,2.)) == []
    assert s.completed_waypoints == 1


@pytest.mark.parametrize('command', ['arm', 'takeoff', 'reposition'])
def test_command_rejection_cannot_advance_the_mission(command):
    s = FlightSession(settings())
    s.submit(payload(), 10., WALL)
    action = tick(s, 10.)[0]
    if command != 'arm':
        s.command_result(action['token'], True, 10.01)
        action = tick(s, 10.05, armed=True)[0]
    if command == 'reposition':
        s.command_result(action['token'], True, 10.06)
        action = tick(s, 10.1, armed=True, landed=2, mode='AUTO.LOITER')[0]
    s.command_result(action['token'], False, 10.11)
    airborne = command != 'arm'
    abort = tick(s, 10.2, armed=True, landed=2 if airborne else 1,
                 mode='AUTO.LOITER' if airborne else 'POSCTL')[0]
    assert abort['kind'] == ('land' if airborne else 'disarm')
    assert s.failure == command+'_rejected_or_transport_failed'


def test_arming_timeout_and_unverified_landing_never_report_success():
    s = FlightSession(settings())
    s.submit(payload(), 10., WALL)
    tick(s, 10.)
    s.submit(payload(), 13.1, WALL+3.1)
    disarm = tick(s, 13.1, armed=True)[0]
    assert disarm['kind'] == 'disarm'
    s.command_result(disarm['token'], False, 13.11)
    tick(s, 43.2, armed=True)
    assert s.phase == 'FAILED'
    assert not s.landing_verified and not s.status()['mission_complete']


@pytest.mark.parametrize('response', ['pending', 'rejected'])
def test_ground_cancel_needs_disarm_acceptance_and_new_ground_state(response):
    s = FlightSession(settings())
    s.submit(payload(), 10., WALL)
    tick(s, 10.)
    s.submit(payload(action='land', request_id='cancel-1'), 10.1, WALL+.1)
    disarm = tick(s, 10.1)[0]
    if response == 'rejected':
        s.command_result(disarm['token'], False, 10.11)
    tick(s, 10.2)
    assert s.phase == 'ABORTING' and not s.landing_verified
