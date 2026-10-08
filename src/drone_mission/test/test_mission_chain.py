"""Policy tests using explicit protocol fixtures, not aircraft performance claims."""
from dataclasses import replace
import math
import pytest
from drone_mission.mission_chain import MissionChain
from test_session import settings, snapshot, payload, WALL


def rig():
    s = MissionChain(settings(full_mission=True, heading_control_confirmed=True, expected_ekf2_mag_type=6, expected_mis_takeoff_alt_m=1.3))
    route = [dict(id='P1',type='waypoint',x=2.3,y=2.,yaw_deg=0.,dwell_s=0.),
             dict(id='S1',type='scan',x=2.6,y=2.,yaw_deg=10.,marker_id=7,
                  staging_xy_m=[2.6,2.],path_validation_ref='virtual-map-v1',
                  label_id='LABEL7',calibration_ref='virtual-camera',mounting_ref='virtual-mount')]
    s._fixture_payload = payload(route_tasks=route)
    for now in (7.,8.,9.):
        assert s.tick(sample(now),now,WALL-10+now,round((WALL+now)*1e9))==[]
    assert s.submit(s._fixture_payload,10.,WALL)
    return s


def sample(now, **changes):
    return snapshot(now, **dict(dict(battery=.9, battery_age_s=0.,yaw_deg=0.,takeoff_action=0,mag_type=6,
                    rc_valid=True,rc_age_s=0.,rc_override=3,rc_mode=0,
                    vertical_speed_m_s=0.,
                    takeoff_alt_m=1.3), **changes))


def step(s, now, **changes):
    snap = sample(now, **changes)
    ns = round((WALL+now)*1e9)
    if s.target_global:
        s.applied_target(*s.target_global,ns)
    actions=s.tick(snap,now,WALL+now-10,ns)
    s.submit(s._fixture_payload,now,WALL+now-10)
    return actions


def airborne(s):
    a=step(s,10.)[0];assert a['kind']=='takeoff_mode'
    s.command_result(a['token'],True,10.01)
    assert step(s,10.05)==[]  # An ACK without actual mode cannot arm.
    a=step(s,10.1,mode='AUTO.TAKEOFF')[0];assert a['kind']=='arm'
    s.command_result(a['token'],True,10.11)
    step(s,10.15,mode='AUTO.TAKEOFF',armed=True)
    step(s,10.2,mode='AUTO.LOITER',armed=True,landed=2)
    assert s.phase=='STABILIZING'
    for i in range(42):
        actions=step(s,10.25+i*.05,mode='AUTO.LOITER',armed=True,landed=2)
        if actions:
            assert actions[0]['kind']=='reposition'
            s.command_result(actions[0]['token'],True,12.31)
            return
    raise AssertionError('No stabilized movement')


@pytest.mark.parametrize('scan_success',[True,False])
def test_full_cycle_retraces_home_then_fresh_land_and_disarm(scan_success):
    s=rig();airborne(s)
    now=12.4
    for _ in range(1000):
        now+=.05
        target=s.monitor.target or (2.,2.)
        yaw=s.target_yaw_deg or 0.
        if s.phase in ('ALIGNING','SCANNING'):
            ns=round((WALL+now)*1e9)
            marker=dict(execution_id='start-1',task_id='S1',window_id=s.scan_window,
                        marker_id=7,calibration_ref='virtual-camera',mounting_ref='virtual-mount',
                        stamp_ns=ns,frame_id='uwb_map',valid=True,xy_m=list(target),yaw_deg=yaw)
            assert s.observe_marker(marker,ns)
            if s.phase=='SCANNING' and scan_success:
                assert not s.observe_scan_result(dict(marker,label_id='WRONG',outcome='SUCCEEDED',stored=True,result_id='r'),ns)
                assert not s.observe_scan_result(dict(marker,stamp_ns=ns+1,label_id='LABEL7',calibration_ref='WRONG',outcome='SUCCEEDED',stored=True,result_id='r'),ns+1)
                assert s.observe_scan_result(dict(marker,stamp_ns=ns+1,label_id='LABEL7',outcome='SUCCEEDED',stored=True,result_id='r'),ns+1)
        actions=step(s,now,mode='AUTO.LOITER',armed=True,landed=2,xy=target,yaw_deg=yaw)
        if actions:
            if actions[0]['kind']=='land':
                assert s.home_verified and target==s.home
                s.command_result(actions[0]['token'],True,now+.01)
                step(s,now+.02,mode='AUTO.LAND',landed=1,armed=False,state_age_s=.5)
                assert s.phase=='LANDING'  # Old disarmed sample is insufficient.
                step(s,now+.04,mode='AUTO.LAND',landed=1,armed=True)
                assert s.phase=='LANDING'
                step(s,now+.06,mode='AUTO.LAND',landed=1,armed=False)
                assert s.phase=='END'
                assert s.status()['mission_complete'] is scan_success
                assert s.status()['flight_outcome']=='SUCCEEDED'
                assert not s.submit(payload(request_id='new-start'),now+.1,WALL+now-10)
                return
            s.command_result(actions[0]['token'],True,now+.001)
    raise AssertionError(s.status())


def test_cancel_during_pending_arm_waits_and_disarms_after_late_response():
    s=rig();a=step(s,10.)[0];s.command_result(a['token'],True,10.01)
    arm=step(s,10.1,mode='AUTO.TAKEOFF')[0]
    assert s.submit(payload('land','cancel'),10.2,WALL+.2)
    assert step(s,10.2,mode='AUTO.TAKEOFF')==[]
    assert step(s,10.3,mode='AUTO.TAKEOFF')==[]
    s.command_result(arm['token'],True,10.4)
    a=step(s,10.5,mode='AUTO.TAKEOFF',armed=True)[0]
    assert a['kind']=='disarm'


@pytest.mark.parametrize('field', ['uwb_age_s','height_age_s','bridge_observation_age_s'])
def test_gap_holds_without_advancing_tasks_then_times_out(field):
    s=rig();airborne(s)
    a=step(s,12.5,mode='AUTO.LOITER',armed=True,landed=2,**{field:.6})[0]
    assert s.phase=='RECOVERING' and a['kind']=='reposition'
    assert s.completed_waypoints==0
    s.command_result(a['token'],True,12.51)
    a=step(s,15.6,mode='AUTO.LOITER',armed=True,landed=2,**{field:3.6})[0]
    assert a['kind']=='land' and s.phase=='LANDING'


def test_manual_mode_precedes_input_loss_battery_and_pending_reply():
    s=rig();airborne(s)
    assert step(s,12.5,mode='POSCTL',armed=True,landed=2,uwb_age_s=9.,battery=.1)==[]
    assert s.phase=='PILOT_OVERRIDE'
    assert step(s,13.,mode='AUTO.LOITER',armed=True,landed=2)==[]


def test_command_timeout_is_unconfirmed_and_never_retries_arm():
    s=rig();a=step(s,10.)[0];s.command_result(a['token'],True,10.01)
    arm=step(s,10.1,mode='AUTO.TAKEOFF')[0]
    assert step(s,13.2,mode='AUTO.TAKEOFF')==[]
    assert s.phase=='UNCONFIRMED'
    s.command_result(arm['token'],True,13.3)
    assert step(s,13.4,mode='AUTO.TAKEOFF',armed=True)==[]


def test_large_marker_adjustment_and_wrong_context_cannot_move():
    s=rig();airborne(s)
    s.index=1;s.enter('ALIGNING','fixture',13.);s.scan_window='window'
    good=dict(execution_id='start-1',task_id='S1',window_id='window',marker_id=7,
              calibration_ref='virtual-camera',mounting_ref='virtual-mount',frame_id='uwb_map',
              valid=True,stamp_ns=1000000000,xy_m=[2.6,2.],yaw_deg=10.)
    for changes in ({'xy_m':[3.,2.]},{'window_id':'old'},{'stamp_ns':2000000000},{'marker_id':8}):
        assert not s.observe_marker(dict(good,**changes),1000000000)
    assert s.observe_marker(good,1000000000)


def test_short_gap_recovers_original_target_without_consuming_a_new_start():
    s=rig();airborne(s);before=set(s.consumed)
    target=s.monitor.target
    hold=step(s,12.5,mode='AUTO.LOITER',armed=True,landed=2,uwb_age_s=.7)[0]
    assert s.monitor.target==(2.,2.) and s.completed_waypoints==0
    s.command_result(hold['token'],True,12.51)
    step(s,12.6,mode='AUTO.LOITER',armed=True,landed=2)
    action=step(s,13.15,mode='AUTO.LOITER',armed=True,landed=2)[0]
    assert action['kind']=='reposition' and s.monitor.target==target
    assert s.consumed==before and s.phase=='MOVING'


def test_restart_does_not_adopt_an_airborne_execution():
    s=MissionChain(settings(full_mission=True))
    a=s.tick(sample(12.,mode='AUTO.LOITER',armed=True,landed=2),12.,WALL,round(WALL*1e9))[0]
    assert a['kind']=='land' and s.failure=='airborne_without_active_session'


def test_changed_marker_target_feedback_stays_required_after_move_phase():
    s=rig();airborne(s)
    for phase in ('ALIGNING','DWELL','SCANNING','EGRESS','RECOVERING'):
        s.enter(phase,'fixture',13.)
        s.target_applied=False;s.target_last_stamp_ns=None
        assert s.applied_target(*s.target_global,round((WALL+14)*1e9))
        assert s.target_applied


def test_scan_label_is_never_used_as_aircraft_goal():
    s=rig()
    route=s._fixture_payload['route_tasks'];route[1]['x']=4.
    assert s.submit(dict(s._fixture_payload,route_tasks=route),10.1,WALL+.1)
    assert s.intent['waypoints'][1]['label_xy']==(4.,2.)
    assert s.intent['waypoints'][1]['xy']==(2.6,2.)
    del route[1]['staging_xy_m']
    assert not s.submit(dict(s._fixture_payload,route_tasks=route),10.1,WALL+.1)


def test_global_integer_feedback_tolerates_one_unit_not_a_changed_goal():
    s=rig();airborne(s);lat,lon=s.target_global
    assert s.applied_target(lat-1e-7,lon,round((WALL+13)*1e9))
    assert not s.applied_target(lat+1e-5,lon,round((WALL+14)*1e9))
    assert s.failure=='px4_target_changed'


def test_loiter_mode_during_native_climb_does_not_allow_reposition():
    s=rig();a=step(s,10.)[0];s.command_result(a['token'],True,10.01)
    a=step(s,10.1,mode='AUTO.TAKEOFF')[0];s.command_result(a['token'],True,10.11)
    step(s,10.2,mode='AUTO.TAKEOFF',armed=True)
    step(s,10.3,mode='AUTO.LOITER',armed=True,landed=2)
    for i in range(70):
        assert step(s,10.35+i*.05,mode='AUTO.LOITER',armed=True,landed=2,vertical_speed_m_s=.3)==[]
    assert s.phase=='STABILIZING'


def test_unconfirmed_native_heading_blocks_start_before_mode_and_arm():
    s=rig()
    s.settings=replace(s.settings,heading_control_confirmed=False)
    assert s.tick(sample(10.),10.,WALL,round(WALL*1e9))==[]
    assert s.phase=='READY' and s.reason=='native_heading_control_not_confirmed'


@pytest.mark.parametrize('changes,reason', [
    (dict(rc_valid=False),'live_rc_input_required'),
    (dict(rc_age_s=1.1),'live_rc_input_required'),
    (dict(rc_override=0),'px4_auto_rc_override_required'),
    (dict(rc_mode=1),'physical_rc_receiver_mode_required')])
def test_field_rc_is_live_before_arm(changes,reason):
    s=rig()
    assert s.tick(sample(10.,**changes),10.,WALL,round(WALL*1e9))==[]
    assert s.phase=='READY' and s.reason==reason


@pytest.mark.parametrize('actual,expected,height,reason', [
    (0,6,1.3,'px4_heading_policy_mismatch_or_unavailable'),
    (0,0,1.3,'native_mag_final_alignment_requires_verified_height')])
def test_heading_readback_and_low_native_altitude_block_before_arm(actual,expected,height,reason):
    s=rig();s.settings=replace(s.settings,expected_ekf2_mag_type=expected,expected_mis_takeoff_alt_m=height)
    assert s.tick(sample(10.,mag_type=actual,takeoff_alt_m=height),10.,WALL,round(WALL*1e9))==[]
    assert s.phase=='READY' and s.reason==reason


def test_fresh_worker_result_can_arrive_after_a_newer_camera_frame():
    s=rig();s.assignment,s.intent=s.intent,None;s.index=1
    s.scan_window='window';s.enter('SCANNING','fixture',12.)
    ns=round((WALL+12)*1e9)
    marker=dict(execution_id=s.assignment['request_id'],task_id='S1',window_id='window',
        marker_id=7,calibration_ref='virtual-camera',mounting_ref='virtual-mount',
        frame_id='uwb_map',valid=True,xy_m=[2.6,2.],yaw_deg=10.,stamp_ns=ns)
    assert s.observe_marker(marker,ns)
    result=dict(marker,stamp_ns=ns-20_000_000,label_id='LABEL7',outcome='SUCCEEDED',stored=True,result_id='r')
    assert s.observe_scan_result(result,ns)
    assert not s.observe_scan_result(result,ns+1)


def test_marker_refinement_uses_scan_cruise_speed():
    s=rig();airborne(s);s.index=1
    s.scan_window='window';s.enter('ALIGNING','fixture',13.)
    ns=round((WALL+13)*1e9)
    marker=dict(execution_id=s.assignment['request_id'],task_id='S1',window_id='window',
        marker_id=7,calibration_ref='virtual-camera',mounting_ref='virtual-mount',
        frame_id='uwb_map',valid=True,xy_m=[2.625,2.],yaw_deg=14.,stamp_ns=ns)
    assert s.observe_marker(marker,ns)
    action=step(s,13.,xy=(2.4,2.),mode='AUTO.LOITER',armed=True,landed=2)[0]
    assert action['kind']=='reposition' and action['speed']==.05
