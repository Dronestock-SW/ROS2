import math

import numpy as np
import pytest

from drone_uwb.processing.motion_reacquisition import MotionReacquisition
from drone_uwb.processing.observation_guard import ObservationGuard
from test_subset_reacquisition import RangeFixture
from drone_uwb.acquisition.validation import Cycle
from test_measured_btf import config, cycle, event, status
from drone_uwb.processing.measured_btf import MeasuredBtf


def seed(checker, velocity=(1., 0.)):
    for i in range(21):
        t=1.+i*.05
        checker.observe(round(t*1e9), [2.+velocity[0]*(t-1), 2.+velocity[1]*(t-1)])


@pytest.mark.parametrize('velocity',[(.25,0.),(1.,0.),(0.,-1.),(1.2,1.2)])
@pytest.mark.parametrize('gap',[.3,.5,.9])
def test_motion_recovery_uses_source_gap_and_measured_velocity(velocity,gap):
    p=MotionReacquisition();seed(p,velocity)
    original=list(p.history)
    for i in range(7):
        t=2.+gap+i*.025
        xy=[2.+v*(t-1) for v in velocity]
        ready,e=p.check(round(t*1e9),xy,eligible=True,key=1)
        assert ready==(i==6)
        assert list(p.history)==original  # Confirming never trains the reference.
    assert e['velocity_m_s']==pytest.approx(velocity)
    assert e['innovation_m']<1e-10
    assert e['age_s']==pytest.approx(gap+.15)


@pytest.mark.parametrize('bad',['stationary_takeoff','step','wrong_direction','old_time','long_gap','stopped_after_gap'])
def test_freeze_rejects_takeoff_jump_and_persistent_bias(bad):
    p=MotionReacquisition();seed(p,(0.,0.) if bad=='stationary_takeoff' else (1.,0.))
    original=list(p.history)
    for i in range(160):
        t=2.5+i*.025
        xy=[2.+(t-1),2.]
        if bad=='stationary_takeoff':xy=[3.,2.]
        if bad=='step':xy[0]+=.6
        if bad=='wrong_direction':xy[0]=3.-(t-2)
        if bad=='old_time':t=1.9
        if bad=='long_gap':t+=2.
        if bad=='stopped_after_gap':xy=[3.5,2.]
        ready,e=p.check(round(t*1e9),xy,eligible=True,key=1)
        assert not ready
        assert list(p.history)==original


def test_first_contradiction_cannot_be_waited_out_until_prediction_catches_up():
    p=MotionReacquisition();seed(p)
    assert not p.check(2_300_000_000,[4.,2.],eligible=True,key=1)[0]
    for i in range(20):
        t=2.8+i*.025
        ready,e=p.check(round(t*1e9),[2.+t-1,2.],eligible=True,key=1)
        assert not ready and e['reason']=='outside_frozen_motion_envelope'


@pytest.mark.parametrize('interrupt',['quality','subset','gap','duplicate'])
def test_recovery_needs_consecutive_same_subset_and_time(interrupt):
    p=MotionReacquisition();seed(p)
    for t in [2.5,2.55,2.6]:
        assert not p.check(round(t*1e9),[1.+t,2.],eligible=True,key=1)[0]
    t=2.625;key=1;eligible=True
    if interrupt=='quality':eligible=False
    if interrupt=='subset':key=2
    if interrupt=='gap':t=2.8
    if interrupt=='duplicate':t=2.6
    assert not p.check(round(t*1e9),[1.+t,2.],eligible=eligible,key=key)[0]
    assert not p.check(round((t+.025)*1e9),[1.+t+.025,2.],eligible=True,key=key)[0]


def test_speed_requires_sufficient_consistent_accepted_history():
    for speed in [0.,.1,3.]:
        p=MotionReacquisition();seed(p,(speed,0.))
        assert not p.check(2_500_000_000,[2.+speed*1.5,2.],eligible=True,key=1)[0]
    p=MotionReacquisition()
    for i in range(21):
        p.observe(1_000_000_000+i*50_000_000,[2.+(i*.05)**2,2.])
    assert not p.check(2_500_000_000,[4.25,2.],eligible=True,key=1)[0]


def test_raw_guard_recovers_motion_but_default_px4_bridge_guard_is_unchanged():
    moving=ObservationGuard(motion_recovery=True);legacy=ObservationGuard()
    for i in range(41):
        t=1.+i*.025
        for g in [moving,legacy]:g.check((t,2.),round(t*1e9))
    old=moving.prior_xy
    reasons=[]
    for i in range(7):
        t=2.5+i*.025
        reasons.append(moving.check((t,2.),round(t*1e9)))
        assert legacy.check((t,2.),round(t*1e9))=='observation_jump_quarantined'
        if i<6:assert moving.prior_xy==old
    assert reasons[-1]=='observation_motion_reacquired'
    assert moving.prior_xy==pytest.approx((2.65,2.))
    assert moving.motion_evidence['ready']


def test_raw_continuous_jump_remains_frozen_even_if_later_model_would_match():
    g=ObservationGuard(motion_recovery=True)
    for i in range(41):g.check((1.+i*.025,2.),1_000_000_000+i*25_000_000)
    old=g.prior_xy
    for i in range(120):
        assert g.check((3.,2.),2_025_000_000+i*25_000_000)=='observation_jump_quarantined'
        assert g.prior_xy==old
    assert g.motion_evidence['reason']=='continuous_step_freeze'
    assert g.check(old,6_000_000_000)=='observation_recovering'


class MovingRanges(RangeFixture):
    def step(self,seq,*,height=True,time_shift=0):
        end=1_000_000+seq*25_000+time_shift
        ts=[end-12_000+i*4_000 for i in range(4)]
        xy=lambda t:[1.+.8*(t-1_000_000)/1e6,2.]
        raw=np.array([np.linalg.norm(np.array([*xy(t),.85])-self.a[i]) for i,t in enumerate(ts)])
        if seq>=60:raw[1]+=.5
        cy=Cycle(seq,ts[0],end,15,raw,ts,['ok']*4,list(range(4)))
        row=dict(time_us=end,sample_time_us=ts,cal_slant_m=raw.tolist(),height_m=.85 if height else None,
                 sample_height_m=[.85 if height else None]*4,
                 height_source='measured_tof_imu' if height else None,models={'B':self.window.process(cy,seq)})
        return self.candidate.process(row),xy(end)


@pytest.mark.parametrize('outage',['height','input'])
def test_moving_b3_from_actual_ranges_recovers_original_measurement(outage):
    f=MovingRanges()
    for seq in range(140):d,xy=f.step(seq)
    assert d['ok'] and d['source']=='ToF_validated_B3'
    prior=f.candidate.recovery_reference.copy()
    if outage=='height':
        for seq in range(140,160):assert not f.step(seq,height=False)[0]['ok']
    recovered=[]
    for seq in range(160 if outage=='height' else 140,185):
        d,xy=f.step(seq,time_shift=500_000 if outage=='input' else 0)
        if d['reason']=='moving_subset_reacquired':
            recovered.append(d)
            assert d['xy_m']==pytest.approx(xy,abs=.01)
            assert math.dist(d['xy_m'],prior)>.35
        if not d['ok']:assert np.array_equal(f.candidate.recovery_reference,prior)
    assert recovered
    assert recovered[0]['motion_reacquisition']['elapsed_s']>=.15
    assert d['ok'] and d['xy_m']==pytest.approx(xy,abs=.01)


@pytest.mark.parametrize('outage',['radio','missing_anchor'])
def test_full_measured_pipeline_restores_moving_b4_after_gap(outage):
    c=config();p=MeasuredBtf(c);anchors=np.asarray(c['anchors_xyz_m'])
    p.process(event(status(),100_000));results=[]
    for seq in range(1,170):
        end=100_000+seq*25_000
        msg=cycle(seq,end,anchors)
        msg['raw_slant_m']=[float(np.linalg.norm(np.array([1.+.8*t/1e6,2.,1.12])-anchors[i]))
                            for i,t in enumerate(msg['sample_time_us'])]
        if 100<=seq<120 and outage=='radio':continue
        if 100<=seq<120:
            msg['valid_mask']=7;msg['raw_slant_m'][3]=None;msg['failure'][3]='timeout'
        for t in msg['sample_time_us']:
            stamp=1_790_000_020_002_000_000+t*1000
            p.height.add('imu',dict(stamp_ns=stamp,valid=True,quaternion_wxyz=[1,0,0,0]))
            p.height.add('tof',dict(stamp_ns=stamp,valid=True,range_m=1.))
        d=p.process(event(msg,end));results.append((seq,d))
    assert any(d.get('raw_motion_reacquisition',{}).get('ready') for _,d in results)
    final=results[-1][1]
    assert final['ok']
    assert final['xy_m']==pytest.approx([1.+.8*final['time_us']/1e6,2.],abs=.002)
    assert not final['flight_valid'] and not final['external_output_allowed']
    assert all(not d['ok'] for seq,d in results if 100<=seq<120)


def test_clock_reset_discards_motion_evidence_without_resetting_jump_origin():
    p=MeasuredBtf(config());seed(p.raw_guard.motion)
    for t in [1_000_000_000,1_025_000_000]:p.raw_guard.check((1.,2.),t)
    seed(p.raw_guard.motion)
    old=p.raw_guard.prior_xy
    p.reset_models()
    assert not p.raw_guard.motion.history
    assert p.raw_guard.prior_xy==old
