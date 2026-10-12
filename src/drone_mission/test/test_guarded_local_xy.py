from dataclasses import replace
import math

import pytest

from drone_mission.guarded_local_xy import GuardedLocalXYMission
from drone_mission.horizontal_obstacles import HorizontalGuard
from drone_mission.local_command import local_command_fields
from test_anchor_trial import config
from test_horizontal_obstacles import CFG, box_scan, frame
from test_local_xy import sample
from test_session import WALL, payload


def prepared():
    cfg = config(command_frame='local_xy')
    g = HorizontalGuard(CFG, source_id='lidar', clock_id='clock', epoch='boot')
    m = GuardedLocalXYMission(cfg, guard=g, proportional_gain_s_inv=1.,
                             resume_clear_s=.3, max_hold_s=2.,hover_position_hold=True,
                             native_position_gain_bound_s_inv=1.)
    for now in (7.,8.,9.):
        m.tick(sample(now),now,WALL+now-10,round((WALL+now)*1e9))
    assert m.submit(payload(),10.,WALL)
    m.tick(sample(),10.,WALL,round(WALL*1e9))
    m.phase='MOVING';m.goal=(3.,2.);m.height_target=1.8;m.entered_s=10.
    m.velocity_xy=(0.,0.)
    return m


def step(m, now, *, scan=True, front=3., **kw):
    ns=round((WALL+now)*1e9)
    if scan:
        m.guard.observe(frame(box_scan(front=front), stamp_ns=ns, sequence=round(now*100),
                              sensor_origin_xy_m=(2.,2.)))
    m.last_web_s=now
    if m.brake_hold_xy is None and not m._hover_position_active():
        m.applied_local_velocity(m.velocity_xy or (0.,0.),1.8,ns)
    else:
        m.applied_local_target(m.brake_hold_xy or m.goal,1.8,ns)
    return m.tick(sample(now,armed=True,mode='OFFBOARD',landed=2,altitude_enu_m=1.8,**kw),
                  now,WALL+now-10,ns)


def test_real_mission_state_machine_exports_limited_velocity_and_fixed_height():
    m=prepared();step(m,10.05)
    goal=m.local_setpoint()
    assert goal['horizontal_control']=='velocity'
    assert goal['velocity_xy_m_s']==pytest.approx((.25,0.))
    assert m.home==(2.,2.) and m.goal==(3.,2.)
    fields=local_command_fields(goal,m.settings)
    assert fields['type_mask'] & 3 == 3  # XY positions ignored
    assert fields['type_mask'] & (4|8|16) == 0  # z position, vx, vy enabled
    assert fields['type_mask'] & 32  # vz always ignored
    assert fields['position_enu'][2]==1.8


def test_stale_scan_brakes_and_new_clear_scans_resume_without_rebasing():
    m=prepared();step(m,10.05)
    for t in (10.15,10.25,10.35):step(m,t,scan=False)
    assert m.local_setpoint()['horizontal_control']=='position'
    held=m.local_setpoint()['xy']
    assert m.hold_since is not None
    step(m,10.4)
    for t in (10.5,10.6):step(m,t,scan=False)
    assert m.local_setpoint()['xy']==held
    for t in (10.7,10.8):step(m,t)
    assert m.local_setpoint()['velocity_xy_m_s'][0]>0
    assert m.home==(2.,2.) and m.goal==(3.,2.)


def test_obstacle_hold_latches_one_position_even_when_the_estimate_drifts():
    m=prepared();step(m,10.05,scan=False)
    held=m.local_setpoint()['xy']
    step(m,10.1,scan=False,xy=(2.03,2.))
    assert m.local_setpoint()['xy']==held
    assert m.local_setpoint()['obstacle_braking_hold']
    assert not m.applied_local_velocity((0.,0.),1.8,round((WALL+10.11)*1e9))


def test_obstacle_stop_timeout_uses_existing_land_and_cancels_stream():
    m=prepared()
    actions=[]
    for i in range(1,44):
        actions=step(m,10+i*.05,scan=False)
        if actions:break
    assert m.phase=='LANDING'
    assert m.reason=='horizontal_obstacle_hold_timeout'
    assert [a['kind'] for a in actions]==['land']
    assert m.local_setpoint() is None


def test_wrong_feedback_contract_is_rejected_and_manual_override_has_priority():
    m=prepared();step(m,10.05)
    assert not m.applied_local_target((2.,2.),1.8,round((WALL+10.06)*1e9))
    assert not m.applied_local_velocity((1.,0.),1.8,round((WALL+10.06)*1e9))
    now=10.1;m.last_web_s=now
    m.tick(sample(now,armed=True,mode='ALTCTL',landed=2,altitude_enu_m=1.8),
           now,WALL+.1,round((WALL+now)*1e9))
    assert m.phase=='PILOT_OVERRIDE' and m.local_setpoint() is None


def test_guard_does_not_hide_uwb_loss():
    m=prepared();step(m,10.05)
    step(m,10.1,uwb_age_s=.26)
    assert m.phase=='LANDING' and m.local_setpoint() is None


def test_velocity_path_keeps_existing_fifteen_cm_arrival_tolerance():
    m=prepared()
    m.goal=(2.08,2.)
    for i in range(1,25):
        step(m,10+i*.05)
        if m.phase=='DWELL':break
    # The original 1 cm condition applies to its ramped COMMAND, while the
    # measured position tolerance is 15 cm. Direct velocity control must not
    # silently impose a 1 cm measured-position requirement.
    assert m.phase=='DWELL'


def test_dwell_holds_immutable_goal_and_obstacle_braking_has_priority():
    m=prepared();m.goal=(2.08,2.)
    for i in range(1,25):
        step(m,10+i*.05)
        if m.phase=='DWELL':break
    goal=m.local_setpoint()
    assert goal['hover_position_hold'] and goal['xy']==(2.08,2.)
    assert local_command_fields(goal,m.settings)['type_mask'] & 3 == 0
    assert not m.applied_local_velocity((.08,0.),1.8,round((WALL+11.3)*1e9))
    step(m,11.1,xy=(2.03,2.))
    assert m.local_setpoint()['xy']==(2.08,2.)
    for t in (11.2,11.3,11.4):step(m,t,scan=False,xy=(2.04,2.))
    assert m.local_setpoint()['obstacle_braking_hold']
    assert m.local_setpoint()['xy']==(2.04,2.)
    assert m.stable_since is None


def test_hover_recovery_cannot_bypass_velocity_limit_or_native_gain_contract():
    m=prepared();m.phase='DWELL';m.goal=(2.4,2.)
    step(m,10.05)
    goal=m.local_setpoint()
    assert goal['horizontal_control']=='velocity'
    assert math.hypot(*goal['velocity_xy_m_s'])<=m.settings.speed_m_s
    with pytest.raises(ValueError,match='native_position_gain_bound_required'):
        GuardedLocalXYMission(m.settings,guard=m.guard,proportional_gain_s_inv=1.,
            resume_clear_s=.3,max_hold_s=2.,hover_position_hold=True)


def test_verified_native_speed_cap_avoids_control_mode_chatter_but_keeps_scan_gate():
    m=prepared();m.phase='DWELL';m.goal=(2.2,2.)
    m.native_position_gain_bound=4.;m.native_speed_limit=.25
    step(m,10.05)
    assert m.local_setpoint()['horizontal_control']=='position'
    assert m.local_setpoint()['xy']==m.goal
    for t in (10.2,10.3,10.4):step(m,t,scan=False)
    assert m.local_setpoint()['obstacle_braking_hold']
    assert m.stable_since is None


def test_mixed_command_alignment_rotates_velocity_without_translation():
    m=prepared();step(m,10.05)
    cfg=replace(m.settings, enu_yaw_deg=90., enu_offset_x_m=100., enu_offset_y_m=200.,map_y_axis_sign=-1)
    goal=dict(xy=(1.,2.), altitude_enu_m=1.7,yaw_deg=30.,horizontal_control='velocity',velocity_xy_m_s=(.2,.3))
    value=local_command_fields(goal,cfg)
    assert value['position_enu']==pytest.approx((102.,201.,1.7))
    assert value['velocity_enu']==pytest.approx((.3,.2,0.))
    assert value['yaw_enu_rad']==pytest.approx(math.radians(60))
