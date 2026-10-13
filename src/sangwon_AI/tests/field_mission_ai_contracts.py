"""Actual C++ BT contract replay. Synthetic input, no ROS or physical FC."""
import copy
import json
import math
from pathlib import Path
import select
import subprocess
import sys
import unittest

BINARY = sys.argv.pop(1)
SCHEMA = 'sangwon-field-ai/1'


def state(**changes):
    s = dict(connected=True, armed=True, landed=2, mode='OFFBOARD', xy=[2., 2.],
        uwb_xy=[2., 2.], velocity_xy=[0., 0.], vertical_speed_m_s=0., altitude_enu_m=1.8,
        pose_age_s=0., uwb_age_s=0., state_age_s=0., landed_age_s=0., estimator_age_s=0.,
        estimator_valid=True, vertical_estimator_valid=True, bridge_ready=True,
        alignment_matches=True, bridge_age_s=0., bridge_observation_age_s=0., battery=.9, battery_age_s=0., feedback_age_s=0.)
    s.update(changes)
    return s


class Peer:
    def __init__(self, points=None, lidar_config=None):
        self.p = subprocess.Popen([BINARY, '--stdio'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.seq = 0
        hello = self.call('hello', {})
        assert hello['tree'] == 'FieldMission'
        cfg = dict(command_frame='local_xy', execution_profile='anchor_trial',
            layout_id='synthetic-layout', drone_id='6', bounds_xy_m=[.5,5.3,.5,3.9],
            speed_m_s=.25, max_leg_m=1., pose_timeout_s=.2, takeoff_settle_s=2.,
            leg_timeout_s=20., takeoff_timeout_s=30., expected_mis_takeoff_alt_m=1.7)
        if lidar_config is not None:cfg['lidar_gap']=lidar_config
        self.preparation = dict(execution_id='fixture-request', settings=cfg, home_xy=[2.,2.],
            height_reference_m=1.8, ceiling_height_m=3., waypoints=points or [dict(type='hover',xy=[2.,2.],dwell_s=10.)])
        self.plan = self.call('prepare', self.preparation)['plan_ref']
        self.t = 10.

    def call(self, method, payload, **changes):
        self.seq += 1
        req = dict(schema=SCHEMA, session_id='fixture-session', sequence=self.seq, method=method, payload=payload)
        req.update(changes)
        self.p.stdin.write(json.dumps(req).encode()+b'\n'); self.p.stdin.flush()
        assert select.select([self.p.stdout],[],[],3)[0], 'C++ response timeout'
        result = json.loads(self.p.stdout.readline())
        if not result.get('ok'): raise ValueError(result.get('code'))
        assert result['backend']=='sangwon_cpp_bt' and result['fc_transport'] is False
        return result['payload']

    def step(self, values=None, phase='TAKING_OFF', **changes):
        self.t += .05
        payload = dict(execution_id='fixture-request',plan_ref=self.plan,sample_time_s=self.t,
            executor_phase=phase, executor_failure='', landing_confirmed=False, command='', state=values or state())
        payload.update(changes)
        return self.call('step', payload)

    def close(self):
        self.p.stdin.close(); self.p.wait(timeout=3); self.p.stdout.close()


class Contracts(unittest.TestCase):
    def setUp(self): self.peers=[]
    def tearDown(self):
        for peer in self.peers: peer.close()
    def peer(self, *a, **kw):
        p=Peer(*a,**kw); self.peers.append(p); return p

    def test_lidar_supported_gap_holds_target_and_restarts_dwell(self):
        from lidar_relative_contracts import CONFIG,scan
        p=self.peer(lidar_config=CONFIG);phase='TAKING_OFF';last_uwb=p.t;in_gap=False;after_gap=None
        for i in range(420):
            t=p.t+.05
            if i<100 or i>=112:last_uwb=t
            age=t-last_uwb
            s=state(uwb_stamp_ns=round(last_uwb*1e9),uwb_age_s=age,bridge_observation_age_s=age,
                    lidar_scan=scan(10+math.floor((t-10)*10+1e-6)/10-.005))
            s['lidar_scan']['age_s']=t-s['lidar_scan']['stamp_ns']/1e9
            r=p.step(s,phase)
            if r['gap_supported']:
                in_gap=True;self.assertEqual(r['stream_xy'],[2.,2.]);self.assertEqual(r['stable_s'],0.)
                self.assertEqual(r['completed_waypoints'],0)
            if i==112:after_gap=p.t
            if r['phase']=='LANDING':break
            phase='TAKING_OFF' if r['phase']=='WAIT_TAKEOFF' else r['phase']
        self.assertTrue(in_gap);self.assertEqual(r['phase'],'LANDING',r)
        self.assertGreaterEqual(p.t-after_gap,10.)
        self.assertTrue(r['home_verified'])

    def test_lidar_loss_or_long_gap_preserves_fc_hold_without_completion(self):
        from lidar_relative_contracts import CONFIG,scan
        for failed_sensor in (False,True):
            p=self.peer(lidar_config=CONFIG);last=p.t
            for i in range(100):
                t=p.t+.05
                if i<60:last=t
                age=t-last;sc=scan(10+math.floor((t-10)*10+1e-6)/10-.005);sc['age_s']=t-sc['stamp_ns']/1e9
                if failed_sensor and i>=60:sc['ranges']=[None]*720
                r=p.step(state(uwb_stamp_ns=round(last*1e9),uwb_age_s=age,bridge_observation_age_s=age,lidar_scan=sc))
                if r['phase']=='LANDING':break
            self.assertNotEqual(r['phase'],'LANDING');self.assertEqual(r['completed_waypoints'],0)
            self.assertEqual(r['stream_xy'],[2.,2.]);self.assertEqual(r['stable_s'],0.)

    def test_all_four_web_routes_use_actual_bt_and_retrace_home(self):
        for case, offsets, length in (
            ('hover',[(0,0)],0.),('x',[(0,0),(.5,0),(1,0),(.5,0),(0,0)],2.),
            ('y',[(0,0),(0,.5),(0,1),(0,.5),(0,0)],2.),
            ('xy',[(0,0),(.5,0),(1,0),(1,.5),(1,1),(1,.5),(1,0),(.5,0),(0,0)],4.)):
            with self.subTest(case=case):
                points=[dict(type='hover' if i==0 else 'waypoint',xy=[2+x,2+y],dwell_s=10. if case=='hover' else 0.) for i,(x,y) in enumerate(offsets)]
                p=self.peer(points); prior=[2.,2.]; phase='TAKING_OFF'; distance=0.; first_dwell=None
                for _ in range(2400):
                    s=state(xy=prior,uwb_xy=prior)
                    r=p.step(s,phase)
                    delta=math.dist(prior,r['stream_xy']);distance+=delta
                    self.assertLessEqual(delta,.25*.05+1e-8)
                    self.assertNotIn('z',r);self.assertNotIn('yaw',r)
                    if r['phase']=='DWELL' and first_dwell is None:first_dwell=p.t
                    if r['phase']=='LANDING':break
                    prior=r['stream_xy'];phase='TAKING_OFF' if r['phase']=='WAIT_TAKEOFF' else r['phase']
                self.assertEqual(r['phase'],'LANDING',r)
                self.assertTrue(r['home_verified']);self.assertEqual(r['home_xy'],[2.,2.])
                self.assertEqual(r['completed_waypoints'],len(points))
                self.assertAlmostEqual(distance,length,places=6)
                if case=='hover':self.assertGreaterEqual(p.t-first_dwell,10.)
                end=p.step(state(armed=False,landed=1,mode='AUTO.LAND'),'LANDING',landing_confirmed=True)
                self.assertEqual(end['phase'],'END')

    def test_direct_uwb_and_bridge_expiry_are_independent(self):
        for key,value in [('uwb_age_s',.251),('bridge_observation_age_s',.351)]:
            with self.subTest(key=key):
                r=self.peer().step(state(**{key:value}))
                self.assertNotEqual(r['phase'],'LANDING');self.assertEqual(r['completed_waypoints'],0)
                self.assertEqual(r['observation_status']['progress_held'],key=='uwb_age_s')
        r=self.peer().step(state(pose_age_s=.201))
        self.assertEqual(r['phase'],'LANDING');self.assertEqual(r['reason'],'PX4_POSE_UNAVAILABLE')

    def test_drift_resets_continuous_hover_but_never_home_or_goal(self):
        p=self.peer()
        for _ in range(90):r=p.step(phase='TAKING_OFF' if _<42 else r['phase'])
        self.assertEqual(r['phase'],'DWELL')
        r=p.step(state(xy=[2.2,2.]),'DWELL')
        self.assertEqual(r['stable_s'],0.)
        self.assertEqual(r['home_xy'],[2.,2.]);self.assertEqual(r['goal_xy'],[2.,2.])
        for _ in range(190):r=p.step(phase='DWELL')
        self.assertEqual(r['phase'],'DWELL');self.assertEqual(r['completed_waypoints'],0)

    def test_lateral_drift_is_not_authorized_as_a_new_target(self):
        r=self.peer().step(state(xy=[2.61,2.]))
        self.assertEqual(r['reason'],'TAKEOFF_HORIZONTAL_DRIFT')
        self.assertEqual(r['goal_xy'],[2.,2.])

    def test_rc_mode_exit_latches_yield_and_reentry_cannot_resume(self):
        p=self.peer();r=p.step(state(mode='ALTCTL'))
        self.assertEqual(r['phase'],'PILOT_OVERRIDE')
        r=p.step();self.assertEqual(r['phase'],'PILOT_OVERRIDE')

    def test_command_reply_unknown_latches_release(self):
        p=self.peer();r=p.step(phase='UNCONFIRMED')
        self.assertEqual(r['phase'],'PILOT_OVERRIDE')
        self.assertEqual(r['reason'],'COMMAND_EFFECT_UNKNOWN')

    def test_landing_and_disarm_without_executor_confirmation_cannot_end(self):
        p=self.peer()
        p.step(state(estimator_valid=False))
        r=p.step(state(armed=False,landed=1,mode='AUTO.LAND'),'LANDING')
        self.assertEqual(r['phase'],'LANDING')
        r=p.step(state(armed=False,landed=1,mode='AUTO.LAND'),'LANDING',landing_confirmed=True)
        self.assertEqual(r['phase'],'FAILED')

    def test_planner_gap_resets_dwell_without_large_position_step(self):
        p=self.peer([dict(type='waypoint',xy=[2.5,2.],dwell_s=0.)])
        for _ in range(45):r=p.step(phase='TAKING_OFF' if _<42 else 'MOVING')
        prior=r['stream_xy'];p.t+=.8
        r=p.step(state(xy=prior,uwb_xy=prior),'MOVING')
        self.assertNotEqual(r['phase'],'LANDING');self.assertLessEqual(math.dist(prior,r['stream_xy']),.0250001)
        self.assertTrue(r['planner_cycle_delayed']);self.assertEqual(r['stable_s'],0.)

    def test_plan_session_and_sequence_cannot_be_replayed_or_replaced(self):
        p=self.peer()
        with self.assertRaisesRegex(ValueError,'MISSION_CONTEXT_MISMATCH'):p.step(plan_ref='other')
        with self.assertRaisesRegex(ValueError,'SESSION_MISMATCH'):p.call('hello',{},session_id='other')
        with self.assertRaisesRegex(ValueError,'SESSION_OR_SEQUENCE_INVALID'):p.call('hello',{},sequence=1)
        with self.assertRaisesRegex(ValueError,'ONE_EXECUTION'):p.call('prepare',p.preparation)

    def test_executor_release_latches_and_cannot_forge_success(self):
        p=self.peer()
        r=p.call('release',dict(execution_id='fixture-request',plan_ref=p.plan,outcome='UNCONFIRMED',reason='unknown effect'))
        self.assertEqual(r['phase'],'UNCONFIRMED')
        self.assertIn(p.step()['phase'],('PILOT_OVERRIDE','UNCONFIRMED'))
        with self.assertRaisesRegex(ValueError,'NON_SUCCESS_RELEASE'):
            p.call('release',dict(execution_id='fixture-request',plan_ref=p.plan,outcome='END',reason='fake'))


if __name__=='__main__':unittest.main()
