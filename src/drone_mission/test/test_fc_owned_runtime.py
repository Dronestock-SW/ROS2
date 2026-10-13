"""Delayed reports must not override valid FC hold; real faults still stop."""
from dataclasses import replace
import math
import pytest
from drone_mission.local_xy import LocalXYMission
from drone_mission.observation_policy import observation_status
from drone_mission.preflight import field_preflight
from drone_mission.bridge_status import normalize_bridge
from test_anchor_trial import config
from test_local_xy import sample, rig, prepared
from test_session import WALL


@pytest.mark.parametrize('changes', [
    dict(bridge_observation_age_s=10.), dict(bridge_age_s=10.),
    dict(bridge_ready=False,alignment_matches=False,bridge_age_s=10.),
    dict(estimator_age_s=10.),
])
def test_runtime_report_delay_is_warning_but_start_remains_strict(changes):
    cfg=config(command_frame='local_xy')
    s=sample(uwb_age_s=.13,**changes)
    m=LocalXYMission(cfg);m.last_web_s=10.
    assert m.ready(s,10.)=='ready'
    assert m.ready(s,10.,ground=True) in ('uwb_px4_bridge_unavailable','px4_estimator_unavailable')
    assert not field_preflight(cfg,s,map_loaded=False,recording_ok=True)['checked_inputs_passed']
    assessment=observation_status(cfg,s)
    assert not assessment['progress_held'] and not assessment['land_requested_by_report']
    assert assessment['warnings']


@pytest.mark.parametrize('changes,reason', [
    (dict(estimator_valid=False),'px4_estimator_unavailable'),
    (dict(connected=False),'fc_disconnected_or_stale'),
    (dict(bridge_contract_error='bridge_host_boot_changed'),'bridge_host_boot_changed'),
    (dict(alignment_matches=False),'observation_and_command_alignment_mismatch'),
])
def test_real_fc_or_coordinate_faults_remain_failures(changes,reason):
    m=LocalXYMission(config(command_frame='local_xy'))
    assert m.ready(sample(**changes),10.)==reason


def test_offboard_start_uses_local_height_goal_and_offboard_rc_override():
    cfg=config(command_frame='local_xy')
    s=sample(takeoff_alt_m=99.,takeoff_action=1,mag_type=6,rc_override=2)
    report=field_preflight(cfg,s,map_loaded=False,recording_ok=True)
    assert report['checked_inputs_passed']
    assert not {'local_start','height_parameter','takeoff_action','heading_policy','rc_auto_override'} & {x['code'] for x in report['checks']}
    m=LocalXYMission(cfg);m.last_web_s=10.
    assert m.ready(s,10.,ground=True)=='ready'


def test_late_bridge_normalization_retains_calibration_and_original_stamp():
    cfg=config(command_frame='local_xy',bridge_topic='/uwb/local_odometry_status',bridge_host_boot_id='boot')
    row=dict(gate='ready',settings=dict(host_boot_id='boot',enu_yaw_deg=12.),counts=dict(published=10),
             last_sample_stamp_ns=123,last_send_age_s=2.,timesync_ready=True)
    value=normalize_bridge(row,cfg)
    assert value['gate']!='ready' and value['settings']['enu_yaw_deg']==12.
    assert value['last_observation_stamp_ns']==123 and value['published']==10
    bad=normalize_bridge(dict(row,settings=dict(host_boot_id='other')),cfg)
    assert bad['bridge_contract_error']=='bridge_host_boot_changed'


@pytest.mark.parametrize('gap', [False,True])
def test_complete_hover_after_delayed_reports_and_real_observation_recovery(gap):
    m=rig();now=9.95;mode='POSCTL';armed=False;alt=.1;landed=1
    commands=[];gap_end=None;held_target=None;resumed_dwell=None
    for _ in range(1800):
        now+=.05;ns=round((WALL+now)*1e9)
        goal=m.local_setpoint()
        if goal:
            alt+=max(-.025,min(.025,goal['altitude_enu_m']-alt))
            m.applied_local_target(goal['xy'],goal['altitude_enu_m'],ns)
        if armed and alt>.15:landed=2
        if gap and m.phase=='DWELL' and gap_end is None:
            gap_end=now+4.;held_target=m.local_setpoint()
        in_gap=gap_end is not None and now<gap_end
        changes=dict(mode=mode,armed=armed,landed=landed,altitude_enu_m=alt)
        if armed:
            changes.update(bridge_age_s=9.,bridge_observation_age_s=8.,bridge_ready=False,alignment_matches=False)
            # The accepted onboard plan survives web reporting loss.
            m.last_web_s=0.
        if in_gap:changes['uwb_age_s']=2.
        actions=m.tick(sample(now,**changes),now,WALL+now-10,ns)
        if in_gap:
            assert m.completed_waypoints==0 and m.local_setpoint()==held_target
            assert not actions and m.home==(2.,2.)
        elif gap_end and resumed_dwell is None:resumed_dwell=now
        for a in actions:
            commands.append(a['kind']);m.command_result(a['token'],True,now)
            if a['kind']=='offboard_mode':mode='OFFBOARD'
            elif a['kind']=='arm':armed=True
            elif a['kind']=='land':armed=False;landed=1;mode='AUTO.LAND'
        if m.phase in m.TERMINAL:break
    assert m.phase=='END',(m.phase,m.reason)
    assert commands==['offboard_mode','arm','land']
    assert m.home==(2.,2.) and m.landing_verified
    if gap:assert now-resumed_dwell>=10.


def test_delayed_planner_cycle_keeps_existing_target_without_hover_credit():
    m=prepared();m.last_web_s=10.4
    goal=m.local_setpoint()
    assert m.tick(sample(10.4),10.4,WALL+.4,round((WALL+.4)*1e9))==[]
    assert m.local_setpoint()==goal and m.stable_since is None


def test_position_feedback_gap_is_hold_and_critical_battery_is_not_hidden():
    cfg=config(command_frame='local_xy');m=LocalXYMission(cfg)
    s=sample(armed=True,mode='OFFBOARD',landed=2,pose_age_s=2.,xy=(),velocity_xy=(),altitude_enu_m=None)
    assert m.ready(s,10.)=='ready'
    assessment=observation_status(cfg,s)
    assert assessment['progress_held'] and not assessment['land_requested_by_report']
    assert m.ready(s,10.,ground=True)!='ready'
    assert m.ready(replace(s,estimator_valid=False),10.)=='px4_estimator_unavailable'


def test_delayed_target_echo_is_diagnostic_not_progress_blocker():
    s=sample(armed=True,mode='OFFBOARD',landed=2)
    status=observation_status(config(command_frame='local_xy'),s,feedback_age_s=20.)
    assert 'target_feedback_delayed' in status['warnings']
    assert not status['progress_held']
