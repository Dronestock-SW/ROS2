"""Actual C++ LiDAR geometry and bounded gap checks. No FC connection."""
import copy
import json
import math
import random
import subprocess
import sys
import unittest

BINARY=sys.argv[1] if len(sys.argv)>1 else None
CONFIG=dict(input_source='uniform_laserscan',enabled=True,mount_confirmed=True,timing_confirmed=True,vertical_structure_confirmed=True,
    mount_ref='SYNTHETIC_ONLY',timing_ref='SYNTHETIC_ONLY',scene_ref='SYNTHETIC_ONLY',frame_id='laser_frame',
    sensor_to_body_flu_m=[0,0,.15],sensor_to_body_yaw_rad=0,scan_plane_parallel_to_body_xy=True)

def scan(t,xy=(2.,2.),yaw=0.,noisy=False,corridor=False):
    rng=random.Random(round(t*1000));ranges=[]
    for i in range(720):
        a=-math.pi+i*math.pi/360+yaw;dx=math.cos(a);dy=math.sin(a)
        options=[]
        for axis,component,lower,upper in ((0,dx,0,5),(1,dy,0,4)):
            if corridor and axis==0:continue
            if abs(component)>1e-8:
                options.append(((upper if component>0 else lower)-xy[axis])/component)
        r=min(options) if options else 20.
        if noisy:r+=rng.gauss(0,.004)
        ranges.append(None if noisy and rng.random()<.025 else r)
    return dict(input_source='uniform_laserscan',stamp_ns=round(t*1e9)+719_000,first_beam_stamp_ns=round(t*1e9),first_beam_age_s=.005719,
        frame_id='laser_frame',age_s=.005,angle_min=-math.pi,
        angle_increment=math.pi/360,range_min=.03,range_max=12.,time_increment=.000001,scan_time=.1,
        ranges=ranges,roll_rad=0.,pitch_rad=0.,map_yaw_rad=yaw,attitude_age_s=.005,attitude_time_error_s=0.,
        reference_quaternion=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)],
        beam_quaternions=[[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]]*720)

def state(t,xy=(2.,2.),uwb_age=0.,**kw):
    return dict(xy=list(xy),velocity_xy=[0.,0.],uwb_xy=list(xy),uwb_age_s=uwb_age,
        uwb_stamp_ns=round((t-uwb_age)*1e9),lidar_scan=scan(t,xy,**kw))

def sdk_state(t,**kw):
    s=state(t,**kw);sc=s['lidar_scan'];order=list(range(283,720))+list(range(283))
    sc.update(input_source='sdk_pointcloud',point_times_s=[i*.0001 for i in range(720)],
        point_angles_rad=[-math.pi+i*math.pi/360 for i in order],
        ranges=[sc['ranges'][i] for i in order],
        beam_quaternions=[sc['beam_quaternions'][i] for i in order],
        stamp_ns=round(t*1e9)+71_900_000,first_beam_age_s=.0769)
    s['uwb_stamp_ns']=sc['stamp_ns']-round(s['uwb_age_s']*1e9)
    return s

class Peer:
    def __init__(self,config=CONFIG):
        self.p=subprocess.Popen([BINARY],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        self.p.stdin.write(json.dumps(config)+'\n');self.p.stdin.flush()
    def step(self,t,s):
        self.p.stdin.write(json.dumps(dict(sample_time_s=t,state=s),allow_nan=False)+'\n');self.p.stdin.flush()
        row=self.p.stdout.readline()
        assert row,self.p.stderr.read()
        return json.loads(row)
    def close(self):
        if self.p.stdin and not self.p.stdin.closed:self.p.stdin.close()
        self.p.wait(timeout=2)
        if not self.p.stdout.closed:self.p.stdout.close()
        if not self.p.stderr.closed:self.p.stderr.close()

class Contracts(unittest.TestCase):
    def setUp(self):self.peer=Peer()
    def tearDown(self):self.peer.close()
    def warm(self,**kw):
        for i in range(6):out=self.peer.step(10+i*.1,state(10+i*.1,**kw))
        return out
    def test_stationary_and_noisy_motion_sign(self):
        self.assertTrue(self.warm(noisy=True)['approved'])
        out=self.peer.step(10.6,state(10.6,xy=(2.02,1.99),uwb_age=.3,noisy=True))
        self.assertTrue(out['approved'],out)
        self.assertAlmostEqual(out['delta_xy'][0],.02,delta=.006)
        self.assertAlmostEqual(out['delta_xy'][1],-.01,delta=.006)
        self.assertFalse(out['fc_aiding'])
    def test_corridor_is_not_position_evidence(self):
        out=self.warm(corridor=True)
        self.assertFalse(out['approved']);self.assertEqual(out['reason'],'DEGENERATE_GEOMETRY')
    def test_gap_bound_without_restamping_UWB(self):
        self.warm()
        for i in range(1,8):
            t=10.5+i*.1;out=self.peer.step(t,state(t,uwb_age=i*.1))
            self.assertTrue(out['approved'],out)
            self.assertEqual(out['uwb_anchor_stamp_ns'],10_500_000_000)
        out=self.peer.step(11.4,state(11.4,uwb_age=.9))
        self.assertFalse(out['approved'])
    def test_tilt_timestamp_and_occlusion_revoke(self):
        self.warm();s=state(10.6);s['lidar_scan']['roll_rad']=.1
        self.assertFalse(self.peer.step(10.6,s)['approved'])
        s=state(10.7);s['lidar_scan']['stamp_ns']=10_000_000_000
        self.assertFalse(self.peer.step(10.7,s)['approved'])
        s=state(10.8);s['lidar_scan']['ranges']=[None]*720
        self.assertFalse(self.peer.step(10.8,s)['approved'])
    def test_NLOS_recovery_disagreement_is_latched(self):
        self.warm();s=state(10.6);s['uwb_xy']=[2.5,2.]
        out=self.peer.step(10.6,s)
        self.assertFalse(out['approved']);self.assertEqual(out['reason'],'UWB_LIDAR_RECOVERY_DISAGREEMENT')
        self.assertFalse(self.peer.step(10.7,state(10.7))['approved'])
    def test_unknown_mount_and_timing_cannot_enable(self):
        self.peer.close();cfg=copy.deepcopy(CONFIG);cfg['timing_confirmed']=False;self.peer=Peer(cfg)
        self.peer.p.stdin.close();self.assertEqual(self.peer.p.wait(timeout=2),2)
        self.peer.p.stdin=None
    def test_shadow_never_approves(self):
        self.peer.close();self.peer=Peer({});self.assertFalse(self.warm()['approved'])
    def test_reflected_anchor_map_does_not_reverse_physical_displacement(self):
        self.peer.close();self.peer=Peer(dict(CONFIG,map_y_axis_sign=-1));self.warm()
        s=state(10.6,xy=(2.02,1.99),uwb_age=.3);s['xy']=[2.02,2.01]
        out=self.peer.step(10.6,s)
        self.assertTrue(out['approved'],out)
        self.assertAlmostEqual(out['delta_xy'][0],.02,delta=.003)
        self.assertAlmostEqual(out['delta_xy'][1],.01,delta=.003)
    def test_rotation_within_scan_does_not_invent_body_translation(self):
        self.warm(yaw=.02);s=state(10.6,yaw=.02,uwb_age=.3);sc=s['lidar_scan']
        for i in range(720):
            yaw=.02*i/719;a=-math.pi+i*math.pi/360+yaw;dx=math.cos(a);dy=math.sin(a)
            sc['ranges'][i]=min([((5 if dx>0 else 0)-2)/dx if abs(dx)>1e-8 else 1e9,
                ((4 if dy>0 else 0)-2)/dy if abs(dy)>1e-8 else 1e9])
            sc['beam_quaternions'][i]=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]
        out=self.peer.step(10.6,s)
        self.assertTrue(out['approved'],out);self.assertLess(math.hypot(*out['delta_xy']),.003)
    def test_15cm_upper_mount_pitch_does_not_invent_body_translation(self):
        self.warm();s=state(10.6,uwb_age=.3);sc=s['lidar_scan'];pitch=.02;origin_x=2+.15*math.sin(pitch)
        sc['pitch_rad']=pitch;sc['reference_quaternion']=[math.cos(pitch/2),0.,math.sin(pitch/2),0.]
        sc['beam_quaternions']=[sc['reference_quaternion']]*720
        for i in range(720):
            a=-math.pi+i*math.pi/360;dx=math.cos(pitch)*math.cos(a);dy=math.sin(a)
            sc['ranges'][i]=min([((5 if dx>0 else 0)-origin_x)/dx if abs(dx)>1e-8 else 1e9,
                ((4 if dy>0 else 0)-2)/dy if abs(dy)>1e-8 else 1e9])
        out=self.peer.step(10.6,s)
        self.assertTrue(out['approved'],out);self.assertLess(math.hypot(*out['delta_xy']),.003)
    def test_sdk_acquisition_order_cloud_motion_and_timing_fault(self):
        self.peer.close();self.peer=Peer(dict(CONFIG,input_source='sdk_pointcloud'))
        for i in range(6):out=self.peer.step(10+i*.1,sdk_state(10+i*.1))
        self.assertTrue(out['approved'],out)
        out=self.peer.step(10.6,sdk_state(10.6,xy=(2.02,1.99),uwb_age=.3))
        self.assertTrue(out['approved'],out)
        self.assertAlmostEqual(out['delta_xy'][0],.02,delta=.003)
        self.assertAlmostEqual(out['delta_xy'][1],-.01,delta=.003)
        s=sdk_state(10.7,uwb_age=.4);s['lidar_scan']['point_times_s'][4]=0.
        out=self.peer.step(10.7,s);self.assertFalse(out['approved'])
        self.assertEqual(out['reason'],'LIDAR_BEAM_TIMING_INVALID')
    def test_wrong_input_source_cannot_modify_qualified_baseline(self):
        self.warm();out=self.peer.step(10.6,sdk_state(10.6))
        self.assertFalse(out['approved']);self.assertEqual(out['reason'],'LIDAR_INPUT_SOURCE_MISMATCH')
    def test_recent_but_unaligned_UWB_cannot_anchor(self):
        for i in range(6):
            s=state(10+i*.1);s['uwb_stamp_ns']-=100_000_000;out=self.peer.step(10+i*.1,s)
        self.assertFalse(out['approved']);self.assertEqual(out['uwb_anchor_stamp_ns'],0)

if __name__=='__main__':
    sys.argv=sys.argv[:1]
    unittest.main()
