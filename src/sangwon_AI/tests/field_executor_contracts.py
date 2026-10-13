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
        self.t=10.;self.s=state();self.request=copy.deepcopy(REQUEST)
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
        self.out=self.call(dict(method='tick',state=self.s,request=self.request,now_s=self.t,wall_s=10.))
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

if __name__=='__main__':unittest.main()
