"""Actual C++ execution core, synthetic FC; no ROS or physical commands."""
import copy
import json
import math
import select
import subprocess
import sys
import unittest

BINARY = sys.argv.pop(1)
CFG = dict(command_frame='local_xy', execution_profile='anchor_trial', mission_ai='cpp_bt',
    execute=True, layout_id='synthetic-layout', drone_id='6', bounds_xy_m=[.5,5.3,.5,3.9],
    speed_m_s=.25, max_leg_m=1., pose_timeout_s=.2, takeoff_settle_s=2., leg_timeout_s=20.,
    takeoff_timeout_s=30., expected_mis_takeoff_alt_m=1.7, layout_confirmed=True,
    sensor_mount_confirmed=True, takeoff_settings_confirmed=True)
REQUEST = dict(action='start', request_id='native-hover', requested_at='2026-10-12T00:00:00Z',
    requested_s=10., ceiling_height_m=3., waypoints=[dict(type='hover',xy=[2.,2.],dwell_s=10.)])

def state(**changes):
    s=dict(connected=True, armed=False, landed=1, mode='POSCTL', xy=[2.,2.], uwb_xy=[2.,2.],
        velocity_xy=[0.,0.], vertical_speed_m_s=0., altitude_enu_m=.1, yaw_deg=0.,
        pose_age_s=0., uwb_age_s=0., state_age_s=0., landed_age_s=0., estimator_age_s=0.,
        estimator_valid=True, vertical_estimator_valid=True, bridge_ready=True, alignment_matches=True,
        bridge_observation_age_s=0., bridge_age_s=0., bridge_published=100, battery=.9, battery_age_s=0.,
        feedback_age_s=0., local_setpoint_ready=True, command_services_ready=True, ground_gate_ready=True,
        takeoff_param_age_s=0.,takeoff_alt_m=1.7,max_up_m_s=.5,max_up_age_s=0.,rc_override=3,rc_required=False,web_age_s=0., request_persisted=True)
    s.update(changes);return s

class Peer:
    def __init__(self,cfg=None):
        self.p=subprocess.Popen([BINARY],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self.t=10.;self.wall=10.;self.s=state();self.request=copy.deepcopy(REQUEST)
        self.call(dict(method='configure',settings=cfg or CFG))
        self.commands=[];self.out={};self.trace=[]
    def call(self,value):
        self.p.stdin.write(json.dumps(value).encode()+b'\n');self.p.stdin.flush()
        assert select.select([self.p.stdout],[],[],3)[0], 'native response timeout'
        r=json.loads(self.p.stdout.readline());assert r['ok'],r
        return r['payload']
    def tick(self,dt=.05,ack=True):
        self.t+=dt
        if self.s['armed'] and self.out.get('height_reference_m') is not None:
            target=self.out['height_reference_m'];step=max(-.5*dt,min(.5*dt,target-self.s['altitude_enu_m']));self.s['altitude_enu_m']+=step;self.s['vertical_speed_m_s']=step/dt
            if self.s['altitude_enu_m']>.15:self.s['landed']=2
        self.out=self.call(dict(method='tick',state=self.s,request=self.request,now_s=self.t,wall_s=self.wall))
        for a in self.out['actions']:
            self.commands.append(a['kind'])
            if ack:self.ack(a['token'],True)
            if a['kind']=='offboard_mode':self.s['mode']='OFFBOARD'
            elif a['kind']=='position_mode':self.s['mode']='POSCTL'
            elif a['kind']=='arm' and ack:self.s.update(armed=True)
            elif a['kind']=='land' and ack:self.s.update(mode='AUTO.LAND',armed=False,landed=1,altitude_enu_m=.1)
        self.trace.append(dict(elapsed_s=self.t-10,state=self.out['phase'],executor='C++',xy=self.s['xy'],height_m=self.s['altitude_enu_m'],uwb_age_s=self.s['uwb_age_s'],cpp_decision=self.out['cpp_decision']))
        return self.out
    def ack(self,token,accepted):return self.call(dict(method='ack',token=token,accepted=accepted,now_s=self.t))
    def until(self,phase,ack=True):
        for _ in range(700):
            if self.tick(ack=ack)['phase']==phase:return self.out
            assert self.out['phase'] not in ('FAILED','END','UNCONFIRMED','PILOT_OVERRIDE'),self.out
        raise AssertionError(('native timeout',phase,self.out))
    def close(self):self.p.stdin.close();self.p.wait(timeout=3);self.p.stdout.close()

class Contracts(unittest.TestCase):
    def setUp(self):self.peers=[]
    def tearDown(self):
        for p in self.peers:p.close()
    def peer(self,**kw):p=Peer(**kw);self.peers.append(p);return p
    def recovered_peer(self):
        p=self.separate();p.until('TAKING_OFF');p.s['mode']='ALTCTL';p.tick()
        self.assertEqual(p.out['phase'],'PILOT_OVERRIDE')
        p.s.update(armed=False,landed=1,mode='POSCTL',vertical_speed_m_s=0.,altitude_enu_m=.1,
            rc_valid=True,rc_mapping_valid=True,rc_neutral_valid=True,rc_sticks_centered=True,rc_age_s=0.,
            terminal_record_persisted=True,recording_ok=True)
        return p

    def test_pilot_ground_reset_requires_new_request_and_preserves_old_home(self):
        p=self.recovered_peer();old_request=copy.deepcopy(p.request);calls=list(p.commands)
        for _ in range(200):p.tick()
        self.assertEqual(p.out['phase'],'PILOT_OVERRIDE')
        for _ in range(4):p.tick()
        self.assertEqual(p.out['phase'],'IDLE');self.assertEqual(p.commands,calls)
        self.assertTrue(p.out['native_takeoff']['can_takeoff'])
        self.assertEqual(p.out['previous_flight']['home_xy_m'],[2.,2.])
        self.assertIsNone(p.out['home_xy_m']);self.assertFalse(p.out['flight_control_enabled'])
        p.wall=11.;p.request=dict(old_request,requested_s=11.);p.tick()
        self.assertEqual(p.out['phase'],'IDLE')
        p.request=dict(old_request,request_id='queued-before-reset',requested_s=10.);p.tick()
        self.assertEqual(p.out['phase'],'IDLE')
        p.request=dict(old_request,request_id='fresh-takeoff',requested_s=11.);p.tick()
        self.assertEqual(p.out['phase'],'PREPARING')
        self.assertEqual(p.out['home_xy_m'],[2.,2.]);self.assertIsNone(p.out['mission_start_xy_m'])
        p.until('TAKING_OFF');self.assertEqual(p.commands,calls+['offboard_mode','arm'])

    def test_pilot_recovery_never_reclaims_armed_or_unknown_ground(self):
        for change in (dict(armed=True,landed=2),dict(armed=True,landed=1),dict(landed=0),dict(connected=False),dict(state_age_s=3.),dict(landed_age_s=3.),dict(mode='ALTCTL')):
            p=self.recovered_peer();p.s.update(change);calls=list(p.commands)
            for _ in range(230):p.tick()
            self.assertEqual(p.out['phase'],'PILOT_OVERRIDE');self.assertEqual(p.commands,calls)

    def test_pilot_neutral_timer_restarts_for_sticks_rc_loss_or_record_failure(self):
        for change in (dict(rc_sticks_centered=False),dict(rc_age_s=1.1),dict(rc_valid=False),dict(rc_mapping_valid=False),dict(rc_neutral_valid=False),dict(terminal_record_persisted=False),dict(recording_ok=False),dict(pose_age_s=.3)):
            p=self.recovered_peer()
            for _ in range(180):p.tick()
            before=copy.deepcopy(p.s);p.s.update(change);p.tick();p.s=before
            self.assertEqual(p.out['pilot_recovery']['elapsed_s'],0.)
            for _ in range(180):p.tick()
            self.assertEqual(p.out['phase'],'PILOT_OVERRIDE')
            for _ in range(24):p.tick()
            self.assertEqual(p.out['phase'],'IDLE')

    def test_pilot_scheduler_gap_resets_neutral_timer(self):
        p=self.recovered_peer()
        for _ in range(180):p.tick()
        p.tick(dt=2.)
        self.assertEqual(p.out['pilot_recovery']['elapsed_s'],0.)
        for _ in range(180):p.tick()
        self.assertEqual(p.out['phase'],'PILOT_OVERRIDE')

    def test_pilot_reset_keeps_command_tokens_unique_and_second_result(self):
        p=self.recovered_peer();p.wall=20.
        for _ in range(204):p.tick()
        p.wall=21.;p.request=dict(p.request,request_id='second-takeoff',requested_s=21.)
        p.until('PREPARING')
        while not p.out['actions']:p.tick(ack=False)
        token=p.out['actions'][0]['token'];self.assertGreater(token,2)
        p.ack(1,True);p.tick(ack=False);self.assertEqual(p.out['phase'],'PREPARING')
        p.ack(token,True);p.until('TAKING_OFF');p.s['mode']='ALTCTL';p.tick()
        self.assertEqual(p.out['phase'],'PILOT_OVERRIDE');self.assertEqual(p.out['ground_reset_count'],1)
    def test_unsent_journal_wait_does_not_consume_fc_ack_deadline(self):
        p=self.peer();p.until('PREPARING')
        while not p.out['actions']:p.tick(ack=False)
        token=p.out['actions'][0]['token']
        p.call(dict(method='queued',token=token))
        p.s['mode']='POSCTL'
        for _ in range(140):p.tick(ack=False)
        self.assertEqual(p.out['phase'],'PREPARING')
        self.assertTrue(p.out['command_record_pending'])
        p.call(dict(method='dispatched',token=token,now_s=p.t))
        p.tick(dt=3.1,ack=False)
        self.assertEqual(p.out['phase'],'UNCONFIRMED')

    def test_land_cancels_unsent_position_request_and_ignores_late_ack(self):
        p=self.separate();p.until('TAKING_OFF')
        p.until('POSITION_HANDOFF',ack=False)
        token=p.out['actions'][0]['token']
        p.call(dict(method='queued',token=token));p.s['mode']='OFFBOARD'
        p.request=dict(action='land',request_id='cancel-during-journal')
        p.tick(ack=False)
        self.assertEqual(p.out['phase'],'LANDING')
        self.assertEqual(p.out['actions'][0]['kind'],'land')
        p.ack(token,True);p.request=None;p.tick(ack=False)
        self.assertEqual(p.out['phase'],'LANDING')
        self.assertFalse(p.out['landing_verified'])

    def test_complete_cpp_hover_ten_seconds_and_actual_ground_disarm(self):
        p=self.peer();p.until('DWELL');dwell=p.t;p.until('END')
        self.assertGreaterEqual(p.t-dwell,10.);self.assertEqual(p.commands,['offboard_mode','arm','land'])
        self.assertEqual(p.out['home_xy_m'],[2.,2.]);self.assertTrue(p.out['landing_verified'])
        self.assertEqual(p.out['completed_waypoints'],1)
    def test_drift_does_not_move_home(self):
        p=self.peer();p.until('TAKING_OFF');p.s['xy']=[2.7,2.];p.tick()
        self.assertEqual(p.out['phase'],'LANDING');self.assertEqual(p.out['home_xy_m'],[2.,2.])
    def test_missing_uwb_holds_target_without_completion_credit(self):
        p=self.peer();p.until('DWELL');p.s['uwb_age_s']=.26;p.tick()
        self.assertEqual(p.out['phase'],'DWELL');self.assertEqual(p.out['completed_waypoints'],0)
        self.assertTrue(p.out['cpp_decision']['observation_status']['progress_held']);self.assertNotIn('land',p.commands)
    def test_pilot_priority_precedes_expired_observation(self):
        p=self.peer();p.until('DWELL');p.s.update(mode='ALTCTL',uwb_age_s=1.);p.tick()
        self.assertEqual(p.out['phase'],'PILOT_OVERRIDE');self.assertNotIn('land',p.commands)
    def test_pending_arm_unknown_effect_never_emits_recovery_command(self):
        p=self.peer();p.until('PREPARING')
        for _ in range(100):
            p.tick(ack=False)
            if 'offboard_mode' in p.commands:
                a=p.out['actions'][0];p.ack(a['token'],True);break
        p.tick(ack=False);self.assertIn('arm',p.commands)
        p.tick(dt=3.1,ack=False);self.assertEqual(p.out['phase'],'UNCONFIRMED')
        p.ack(2,True);p.tick();self.assertEqual(p.out['phase'],'UNCONFIRMED');self.assertNotIn('land',p.commands)
    def test_scheduler_gap_does_not_cancel_approved_fixed_target(self):
        p=self.peer();p.until('PREPARING');p.tick(dt=.217)
        self.assertEqual(p.out['phase'],'PREPARING');self.assertFalse(p.commands)
    def test_start_is_blocked_by_unpersisted_request(self):
        p=self.peer();p.s['request_persisted']=False
        for _ in range(60):p.tick()
        p.t+=.1
        with self.assertRaises(AssertionError):p.tick()
        self.assertFalse(p.commands)
    def test_ground_gate_cannot_be_relaxed_by_configuration(self):
        p=self.peer();p.s['ground_gate_ready']=False
        for _ in range(100):p.tick()
        self.assertEqual(p.out['phase'],'IDLE');self.assertFalse(p.commands)
    def test_up_limit_above_point_five_blocks_START(self):
        p=self.peer();p.s['max_up_m_s']=1.
        for _ in range(100):p.tick()
        self.assertEqual(p.out['phase'],'IDLE');self.assertFalse(p.commands)
    def test_fc_boundary_is_checked_even_with_small_tracking_error(self):
        p=self.peer(cfg=dict(CFG,bounds_xy_m=[1.95,5.3,.5,3.9]));p.until('DWELL');p.s['xy']=[1.90,2.];p.tick()
        self.assertEqual(p.out['phase'],'LANDING');self.assertEqual(p.out['reason'],'px4_pose_outside_bounds')
    def test_command_rejection_never_reports_END(self):
        p=self.peer();p.until('PREPARING')
        for _ in range(40):
            p.tick(ack=False)
            if p.out['actions']:
                p.ack(p.out['actions'][0]['token'],False);break
        p.tick();self.assertNotEqual(p.out['phase'],'END');self.assertNotIn('arm',p.commands)
    def test_landing_ack_alone_is_not_completion(self):
        p=self.peer();p.until('DWELL');p.request=dict(action='land',request_id='cancel')
        p.tick(ack=False);a=p.out['actions'][0];p.ack(a['token'],True)
        p.request=None;p.s.update(armed=True,landed=2,mode='AUTO.LAND');p.tick()
        self.assertEqual(p.out['phase'],'LANDING');self.assertFalse(p.out['landing_verified'])
        self.assertEqual(p.out['goal_xy_m'],[2.,2.])
    def test_early_return_never_claims_completed_mission(self):
        p=self.peer();p.until('DWELL');p.request=dict(action='return_to_home',request_id='early-return')
        p.until('FAILED');self.assertTrue(p.out['landing_verified']);self.assertFalse(p.out['mission_complete'])
        self.assertEqual(p.out['completed_waypoints'],0)
    def test_failed_mission_still_records_confirmed_landing(self):
        p=self.peer();p.until('DWELL');p.s['estimator_valid']=False;p.until('FAILED')
        self.assertTrue(p.out['landing_verified']);self.assertFalse(p.out['mission_complete'])
    def test_configured_bridge_delivery_tolerance_uses_original_age(self):
        p=self.peer(cfg=dict(CFG,bridge_observation_timeout_s=.7));p.until('DWELL')
        p.s.update(uwb_age_s=.13,bridge_observation_age_s=.48);p.tick()
        self.assertEqual(p.out['phase'],'DWELL');self.assertNotIn('land',p.commands)
        self.assertEqual(p.out['bridge_observation_timeout_s'],.7)
        p.s['bridge_observation_age_s']=.700001;p.tick()
        self.assertEqual(p.out['phase'],'DWELL');self.assertNotIn('land',p.commands)
    def test_bridge_delivery_tolerance_never_relaxes_raw_uwb(self):
        p=self.peer(cfg=dict(CFG,bridge_observation_timeout_s=.7));p.until('DWELL')
        p.s.update(uwb_age_s=.250001,bridge_observation_age_s=.48);p.tick()
        self.assertEqual(p.out['phase'],'DWELL');self.assertTrue(p.out['cpp_decision']['observation_status']['progress_held'])

    def test_report_silence_and_web_loss_complete_nominal_hover(self):
        p=self.peer();p.until('DWELL')
        p.s.update(bridge_age_s=9.,bridge_observation_age_s=8.,bridge_ready=False,alignment_matches=False,web_age_s=9.)
        p.until('END');self.assertTrue(p.out['mission_complete']);self.assertEqual(p.commands,['offboard_mode','arm','land'])

    def test_raw_gap_never_rebases_target_and_restarts_ten_second_hover(self):
        p=self.peer();p.until('DWELL');target=p.out['goal_xy_m'];p.s['uwb_age_s']=3.
        for _ in range(80):p.tick();self.assertEqual(p.out['goal_xy_m'],target);self.assertEqual(p.out['completed_waypoints'],0)
        p.s['uwb_age_s']=0.;resumed=p.t;p.until('END');self.assertGreaterEqual(p.t-resumed,10.)
        self.assertEqual(p.out['home_xy_m'],[2.,2.])

    def test_candidate_rejection_freezes_before_old_pose_expires(self):
        p=self.peer();p.until('DWELL');target=p.out['goal_xy_m']
        p.s.update(uwb_age_s=.02,uwb_decision_rejected=True)
        for _ in range(40):
            p.tick();self.assertEqual(p.out['goal_xy_m'],target)
            self.assertEqual(p.out['completed_waypoints'],0)
            self.assertTrue(p.out['cpp_decision']['observation_status']['progress_held'])
        self.assertNotIn('land',p.commands)
        p.s['uwb_decision_rejected']=False;resumed=p.t;p.until('END')
        self.assertGreaterEqual(p.t-resumed,10.)

    def test_actual_fc_fault_and_contract_change_still_land(self):
        for change in (dict(estimator_valid=False),dict(bridge_contract_error='changed')):
            p=self.peer();p.until('DWELL');p.s.update(change);p.tick();self.assertEqual(p.out['phase'],'LANDING')

    def test_drift_guard_still_applies_during_observation_hold(self):
        p=self.peer();p.until('DWELL');p.s.update(uwb_age_s=3.,xy=[2.6,2.]);p.tick()
        self.assertEqual(p.out['phase'],'LANDING');self.assertEqual(p.out['home_xy_m'],[2.,2.])

    def test_offboard_start_does_not_require_auto_takeoff_altitude(self):
        p=self.peer();p.s.update(takeoff_alt_m=99.,takeoff_param_age_s=99.,rc_override=2)
        p.until('END');self.assertTrue(p.out['mission_complete'])

    def test_position_feedback_gap_holds_then_completes_after_recovery(self):
        p=self.peer();p.until('DWELL');target=p.out['goal_xy_m'];p.s['pose_age_s']=2.
        for _ in range(60):p.tick();self.assertEqual(p.out['goal_xy_m'],target);self.assertEqual(p.out['completed_waypoints'],0)
        self.assertNotIn('land',p.commands)
        p.s['pose_age_s']=0.;resumed=p.t;p.until('END');self.assertGreaterEqual(p.t-resumed,10.)

    def test_target_echo_loss_does_not_block_real_position_hover_completion(self):
        p=self.peer();p.until('DWELL');p.s['feedback_age_s']=20.;p.until('END')
        self.assertTrue(p.out['mission_complete'])

    def test_one_hertz_state_report_is_not_rejected_by_duplicate_age_gate(self):
        p=self.peer();p.s.update(state_age_s=1.2,landed_age_s=1.2);p.until('TAKING_OFF')
        self.assertIn('arm',p.commands)

    def separate(self):
        p=self.peer(cfg=dict(CFG,start_mode='takeoff_hold',takeoff_backend='local_position'))
        p.request=dict(REQUEST,action='takeoff',request_id='separate-takeoff')
        return p

    def lidar_takeoff(self):
        p=self.peer(cfg=dict(CFG,start_mode='takeoff_hold',takeoff_backend='local_position',
            takeoff_lidar_required=True,takeoff_lidar_fusion_ref='synthetic-fusion-qualification'))
        p.request=dict(REQUEST,action='takeoff',request_id='lidar-takeoff')
        p.s.update(lidar_ready=True,lidar_gate='ready',lidar_observation_age_s=.1,
            lidar_status_age_s=.01,lidar_speed_m_s=0.,lidar_stationary_window_ready=True)
        return p

    def test_lidar_takeoff_requires_continuous_fresh_stationary_sender_before_arm(self):
        for change in (dict(lidar_ready=False),dict(lidar_observation_age_s=.36),
                       dict(lidar_status_age_s=.51),dict(lidar_speed_m_s=.11),dict(lidar_stationary_window_ready=False)):
            p=self.lidar_takeoff();p.s.update(change)
            for _ in range(90):p.tick()
            self.assertFalse(p.commands);self.assertFalse(p.out['native_takeoff']['can_takeoff'])
        p=self.lidar_takeoff()
        for _ in range(50):p.tick()
        p.s['lidar_ready']=False;p.tick();p.s['lidar_ready']=True
        for _ in range(50):p.tick()
        self.assertFalse(p.commands);p.until('TAKING_OFF')

    def test_lidar_takeoff_uwb_gap_keeps_original_xy_and_recovers_without_rearm(self):
        p=self.lidar_takeoff();p.until('TAKING_OFF');p.s['uwb_age_s']=2.
        p.until('HOLD_READY')
        for _ in range(130):
            p.tick();self.assertEqual(p.out['phase'],'HOLD_READY');self.assertFalse(p.out['native_takeoff']['can_start'])
            self.assertEqual(p.out['home_xy_m'],[2.,2.]);self.assertFalse(p.out['streaming'])
        self.assertEqual(p.out['native_takeoff']['blocker'],'position_hover_wait_for_uwb_recovery')
        p.s['uwb_age_s']=0.
        for _ in range(90):p.tick();self.assertFalse(p.out['native_takeoff']['can_start'])
        for _ in range(15):p.tick()
        self.assertTrue(p.out['native_takeoff']['can_start'])
        self.assertEqual(p.commands,['offboard_mode','arm','position_mode'])
        self.assertEqual(p.out['mission_start_xy_m'],[2.,2.])

    def test_lidar_takeoff_uwb_gap_cannot_extend_unstable_ascent_deadline(self):
        p=self.lidar_takeoff();p.until('TAKING_OFF');began=p.t
        p.s.update(uwb_age_s=2.,velocity_xy=[.11,0.])
        p.until('LANDING')
        self.assertEqual(p.out['failure'],'takeoff_stability_timeout')
        self.assertGreaterEqual(p.t-began,30.)
        self.assertLess(p.t-began,30.2)
        self.assertEqual(p.out['home_xy_m'],[2.,2.])
        self.assertEqual(p.commands,['offboard_mode','arm','land'])

    def test_lidar_takeoff_does_not_recapture_drifted_position(self):
        p=self.lidar_takeoff();p.until('HOLD_READY');p.s.update(xy=[2.2,2.],uwb_xy=[2.2,2.])
        for _ in range(110):p.tick()
        self.assertIsNone(p.out['mission_start_xy_m']);self.assertFalse(p.out['native_takeoff']['can_start'])
        p.s.update(xy=[2.08,2.],uwb_xy=[2.08,2.]);self.hold_ready(p)
        self.assertEqual(p.out['mission_start_xy_m'],[2.,2.]);self.assertEqual(p.out['home_xy_m'],[2.,2.])

    def test_position_hover_waits_for_uwb_scatter_to_settle(self):
        p=self.lidar_takeoff();p.until('HOLD_READY')
        for i in range(150):
            p.s['uwb_xy']=[2.15 if i%2 else 1.85,2.];p.tick()
            self.assertEqual(p.out['phase'],'HOLD_READY');self.assertFalse(p.out['native_takeoff']['can_start'])
        p.s['uwb_xy']=[2.,2.]
        for _ in range(90):p.tick();self.assertFalse(p.out['native_takeoff']['can_start'])
        for _ in range(15):p.tick()
        self.assertTrue(p.out['native_takeoff']['can_start']);self.assertNotIn('land',p.commands)

    def test_lidar_loss_lands_after_grace_without_changing_xy_and_pilot_wins(self):
        p=self.lidar_takeoff();p.until('TAKING_OFF');p.s.update(lidar_ready=False,lidar_gate='lidar_observation_stale')
        for _ in range(9):p.tick();self.assertEqual(p.out['phase'],'TAKING_OFF');self.assertEqual(p.out['goal_xy_m'],[2.,2.])
        p.until('LANDING');self.assertIn('takeoff_lidar_lost',p.out['failure'])
        p=self.lidar_takeoff();p.until('TAKING_OFF');p.s.update(lidar_ready=False,mode='ALTCTL');p.tick()
        self.assertEqual(p.out['phase'],'PILOT_OVERRIDE');self.assertNotIn('land',p.commands)

    def test_lidar_drift_limit_and_short_gap_recovery(self):
        p=self.lidar_takeoff();p.until('TAKING_OFF');p.s['xy']=[2.26,2.];p.tick()
        self.assertEqual(p.out['failure'],'takeoff_hold_horizontal_drift')
        p=self.lidar_takeoff();p.until('TAKING_OFF');p.s['lidar_ready']=False
        for _ in range(5):p.tick()
        p.s['lidar_ready']=True;self.hold_ready(p);self.assertNotIn('land',p.commands)

    def test_lidar_status_rejects_stale_wrong_source_and_unpublished_rows(self):
        p=self.lidar_takeoff()
        report=dict(status_stamp_ns=10_000_000_000,last_sent_stamp_ns=9_900_000_000,
            gate='ready',geometry_gate='ready',mode_independent=True,z_observed=False,flight_commands_enabled=False,
            settings=dict(enabled=True,tag_id='6',floor_policy='metric_scan',vertical_structure_ref='synthetic-walls'),
            parameters=dict(EKF2_LID2_VER=2,EKF2_LID2_EN=1),
            last_published_observation=dict(accepted=True,published=True,z_observed=False,observed_axes=['x','y'],
                reference_point='fc_imu',frame_id='local_ned',representative_stamp_ns=9_900_000_000,reset_counter=0,velocity_ne_m_s=[0.,.01]),
            stationary_window=dict(ready=True,last_stamp_ns=9_900_000_000,reset_counter=0,samples=10,span_s=1.,observed_s=1.,
                purpose='stationary_readiness_only',fc_input_filtered=False,speed_m_s=.01))
        query=dict(method='lidar_status',report=report,ros_s=10.01,receipt_age_s=.01,publishers=1,tag='6')
        good=p.call(query);self.assertTrue(good['lidar_ready']);self.assertFalse(good['lidar_fusion_verified'])
        bad=copy.deepcopy(report);bad['parameters']['EKF2_LID2_VER']=1
        self.assertFalse(p.call(dict(query,report=bad))['lidar_ready'])
        for change in (dict(ros_s=10.5),dict(ros_s=9.),dict(receipt_age_s=.6),dict(publishers=2),dict(tag='5')):
            self.assertFalse(p.call({**query,**change})['lidar_ready'])
        for change in (dict(published=False),dict(z_observed=True),dict(representative_stamp_ns=9_800_000_000)):
            bad=copy.deepcopy(report);bad['last_published_observation'].update(change)
            self.assertFalse(p.call(dict(query,report=bad))['lidar_ready'])
        bad=copy.deepcopy(report);bad['last_observation']=dict(accepted=False,reasons=['insufficient_points'])
        self.assertTrue(p.call(dict(query,report=bad))['lidar_ready'])
        bad=copy.deepcopy(report);bad['stationary_window']['ready']=False
        out=p.call(dict(query,report=bad));self.assertTrue(out['lidar_ready']);self.assertFalse(out['lidar_stationary_window_ready'])

    def hold_ready(self,p):
        p.until('HOLD_READY')
        for _ in range(240):
            p.tick()
            if p.out['native_takeoff']['can_start']:return
        self.fail(p.out)

    def test_separate_takeoff_waits_without_stream_then_web_start_reenters_offboard(self):
        p=self.separate();self.hold_ready(p)
        self.assertEqual(p.commands,['offboard_mode','arm','position_mode'])
        for _ in range(100):
            p.tick();self.assertEqual(p.out['phase'],'HOLD_READY');self.assertFalse(p.out['streaming'])
            self.assertEqual(p.out['completed_waypoints'],0)
        p.s.update(xy=[2.06,2.],uwb_xy=[2.06,2.]);p.request=copy.deepcopy(REQUEST)
        p.until('RESUME_PREPARING');self.assertEqual(p.out['home_xy_m'],[2.,2.]);self.assertEqual(p.out['goal_xy_m'],[2.,2.])
        p.until('END');self.assertEqual(p.commands,['offboard_mode','arm','position_mode','offboard_mode','land'])
        self.assertTrue(p.out['landing_verified']);self.assertTrue(p.out['mission_complete'])

    def test_separate_start_cannot_arm_from_ground(self):
        p=self.separate();p.request=copy.deepcopy(REQUEST)
        for _ in range(100):p.tick()
        self.assertEqual(p.out['phase'],'IDLE');self.assertFalse(p.commands)

    def test_position_five_seconds_captures_once_and_keeps_original_return_home(self):
        p=self.separate();p.until('HOLD_READY');p.s.update(xy=[2.25,2.12],uwb_xy=[2.25,2.12])
        began=p.t
        for _ in range(100):
            p.tick();self.assertFalse(p.out['native_takeoff']['can_start']);self.assertIsNone(p.out['mission_start_xy_m'])
        p.tick();p.tick()
        self.assertGreaterEqual(p.t-began,5.)
        self.assertEqual(p.out['mission_start_xy_m'],[2.25,2.12]);self.assertEqual(p.out['home_xy_m'],[2.,2.])
        capture=copy.deepcopy(p.out['mission_start_capture'])
        p.s.update(xy=[2.27,2.12],uwb_xy=[2.27,2.12]);p.tick()
        self.assertEqual(p.out['mission_start_capture'],capture)
        p.request=dict(REQUEST,waypoints=[dict(type='hover',xy=[2.25,2.12],dwell_s=10.)])
        p.until('RESUME_PREPARING');self.assertEqual(p.out['goal_xy_m'],[2.25,2.12])
        self.assertEqual(p.out['home_xy_m'],[2.,2.]);self.assertEqual(p.commands.count('arm'),1)

    def test_position_gap_or_scheduler_pause_restarts_five_second_capture(self):
        for pause in ('uwb','scheduler'):
            p=self.separate();p.until('HOLD_READY')
            for _ in range(60):p.tick()
            if pause=='uwb':p.s['uwb_age_s']=2.;p.tick();p.s['uwb_age_s']=0.
            else:p.tick(dt=2.1)
            self.assertIsNone(p.out['mission_start_xy_m'])
            for _ in range(90):p.tick();self.assertFalse(p.out['native_takeoff']['can_start'])
            for _ in range(15):p.tick()
            self.assertTrue(p.out['native_takeoff']['can_start'])

    def test_position_continuous_motion_never_captures_origin(self):
        p=self.separate();p.until('HOLD_READY');p.s['velocity_xy']=[.11,0.]
        for _ in range(130):p.tick()
        self.assertIsNone(p.out['mission_start_xy_m']);self.assertFalse(p.out['native_takeoff']['can_start'])

    def test_takeoff_report_delay_does_not_land_or_block_position_handoff(self):
        p=self.separate();p.until('TAKING_OFF');p.s.update(bridge_age_s=9.,bridge_observation_age_s=8.,estimator_age_s=3.)
        self.hold_ready(p);self.assertNotIn('land',p.commands)

    def test_takeoff_actual_observation_gap_holds_original_goal_until_recovery(self):
        for key in ('uwb_age_s','pose_age_s'):
            p=self.separate();p.until('TAKING_OFF');p.s[key]=2.
            for _ in range(160):
                p.tick();self.assertEqual(p.out['phase'],'TAKING_OFF');self.assertEqual(p.out['goal_xy_m'],[2.,2.])
            self.assertNotIn('land',p.commands);p.s[key]=0.;self.hold_ready(p)

    def test_position_gap_revokes_start_and_requires_new_stability_window(self):
        p=self.separate();self.hold_ready(p);p.s['uwb_age_s']=2.
        for _ in range(80):p.tick();self.assertFalse(p.out['native_takeoff']['can_start']);self.assertFalse(p.out['streaming'])
        p.s['uwb_age_s']=0.;p.tick();self.assertFalse(p.out['native_takeoff']['can_start'])
        for _ in range(102):p.tick()
        self.assertTrue(p.out['native_takeoff']['can_start']);self.assertNotIn('land',p.commands)

    def test_position_land_without_mission_confirms_disarm_without_offboard_reentry(self):
        p=self.separate();self.hold_ready(p);p.request=dict(action='land',request_id='land-in-position')
        p.until('FAILED');self.assertTrue(p.out['landing_verified']);self.assertFalse(p.out['mission_complete'])
        self.assertEqual(p.commands,['offboard_mode','arm','position_mode','land'])

    def test_position_stick_movement_latches_pilot_and_blocks_later_start(self):
        p=self.separate();self.hold_ready(p)
        p.s.update(rc_required=True,rc_valid=True,rc_age_s=0.,rc_neutral_valid=True,rc_sticks_centered=False)
        p.tick();self.assertEqual(p.out['phase'],'PILOT_OVERRIDE');p.request=copy.deepcopy(REQUEST)
        p.s['rc_sticks_centered']=True;p.tick();self.assertEqual(p.out['phase'],'PILOT_OVERRIDE');self.assertNotIn('land',p.commands)

    def test_resume_gap_waits_with_fixed_target_without_second_arm(self):
        p=self.separate();self.hold_ready(p);p.request=copy.deepcopy(REQUEST);p.until('RESUME_PREPARING')
        p.s['uwb_age_s']=2.
        for _ in range(120):p.tick();self.assertEqual(p.out['phase'],'RESUME_PREPARING');self.assertEqual(p.out['goal_xy_m'],[2.,2.])
        self.assertEqual(p.commands.count('offboard_mode'),1);p.s['uwb_age_s']=0.;p.until('END');self.assertEqual(p.commands.count('arm'),1)

    def test_takeoff_hold_keeps_drift_and_estimator_fault_protection(self):
        for change in (dict(xy=[2.,2.7]),dict(estimator_valid=False)):
            p=self.separate();self.hold_ready(p);p.s.update(change);p.until('FAILED');self.assertTrue(p.out['landing_verified'])

    def test_position_ack_is_not_mode_confirmation_and_hold_stream_continues(self):
        p=self.separate();p.until('POSITION_HANDOFF');p.s['mode']='OFFBOARD'
        for _ in range(30):p.tick();self.assertEqual(p.out['phase'],'POSITION_HANDOFF');self.assertTrue(p.out['streaming'])
        p.s['mode']='POSCTL';self.hold_ready(p)

    def test_reentry_ack_is_not_mode_confirmation_and_never_rearms(self):
        p=self.separate();self.hold_ready(p);p.request=copy.deepcopy(REQUEST);p.until('RESUME_PREPARING')
        while p.commands.count('offboard_mode')<2:p.tick()
        p.s['mode']='POSCTL'
        for _ in range(30):p.tick();self.assertEqual(p.out['phase'],'RESUME_PREPARING')
        p.s['mode']='OFFBOARD';p.until('END');self.assertEqual(p.commands.count('arm'),1)

    def test_new_mission_battery_reserve_does_not_block_position_handoff(self):
        p=self.separate();p.until('TAKING_OFF');p.s['battery']=.29;p.until('HOLD_READY')
        for _ in range(110):p.tick();self.assertFalse(p.out['native_takeoff']['can_start'])
        self.assertNotIn('land',p.commands);self.assertFalse(p.out['streaming'])
        p.s['battery']=.9;p.tick();self.assertTrue(p.out['native_takeoff']['can_start'])

if __name__=='__main__':unittest.main()
