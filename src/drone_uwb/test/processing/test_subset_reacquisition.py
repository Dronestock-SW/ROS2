import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.acquisition.validation import Cycle
from drone_uwb.processing.experiments.h80_b import BSettings, H80Window
from drone_uwb.processing.experiments.tof_subset import NearPriorReacquisition, ToFTrackedSubsetCandidate


def qualified(x=2., anchor=1):
    return dict(ok=True,source='ToF_validated_B3',reason='unique_height_consistent_subset',
                xy_m=[x,2.],best_excluded_anchor=anchor)


@pytest.mark.parametrize('interruption',['gap','duplicate','backward','anchor','height','ambiguous','far','oscillation'])
def test_confirmation_requires_consecutive_fresh_unique_nearby_measurements(interruption):
    p=NearPriorReacquisition(.1,.1);reference=np.array([2.,2.])
    for t in [1_000_000,1_050_000,1_100_000]:
        assert not p.check(t,qualified(1.91 if interruption=='oscillation' else 2.),reference=reference,eligible=True)[0]
    stamp=1_125_000;d=qualified();eligible=True
    if interruption=='gap':stamp=1_250_000
    if interruption=='duplicate':stamp=1_100_000
    if interruption=='backward':stamp=1_090_000
    if interruption=='anchor':d=qualified(anchor=2)
    if interruption=='height':eligible=False
    if interruption=='ambiguous':d.update(ok=False,reason='ambiguous_subset')
    if interruption=='far':d=qualified(2.5)
    if interruption=='oscillation':d=qualified(2.09)
    assert not p.check(stamp,d,reference=reference,eligible=eligible)[0]
    assert not p.check(stamp+25_000,qualified(),reference=reference,eligible=True)[0]


def test_confirmation_never_bootstraps_or_accepts_a_persistent_step_with_elapsed_time():
    p=NearPriorReacquisition(.1,.1)
    for i in range(200):
        assert not p.check(1_000_000+i*25_000,qualified(),reference=None,eligible=True)[0]
    reference=np.array([2.,2.])
    for i in range(200):
        assert not p.check(10_000_000+i*25_000,qualified(2.5),reference=reference,eligible=True)[0]
    for i in range(7):
        ready,evidence=p.check(20_000_000+i*25_000,qualified(2.02),reference=reference,eligible=True)
        assert ready==(i==6)
    assert evidence['samples']==7 and evidence['distance_m']==pytest.approx(.02)


class RangeFixture:
    def __init__(self):
        self.c=json.loads((Path(__file__).parents[2]/'config/runtime/uwb_btf_real.json').read_text())
        self.c['anchors_xyz_m']=[[0,0,.15],[6.3,0,.15],[0,4.6,.15],[6.3,4.6,.15]]
        self.a=np.asarray(self.c['anchors_xyz_m'])
        self.window=H80Window(self.a,self.c['bias_m'],BSettings(**self.c['B']))
        self.candidate=ToFTrackedSubsetCandidate(self.c,**self.c['tof_subset'])

    def step(self,seq,*,height=True,x=2.,time_shift=0):
        end=1_000_000+seq*25_000+time_shift;ts=[end-12_000+i*4_000 for i in range(4)]
        raw=np.linalg.norm(np.array([x,2.,.85])-self.a,axis=1)
        if seq>=60:raw[1]+=.5
        cy=Cycle(seq,ts[0],end,15,raw,ts,['ok']*4,list(range(4)))
        row=dict(time_us=end,sample_time_us=ts,cal_slant_m=raw.tolist(),height_m=.85 if height else None,
                 sample_height_m=[.85 if height else None]*4,
                 height_source='measured_tof_imu' if height else None,models={'B':self.window.process(cy,seq)})
        return self.candidate.process(row)


@pytest.mark.parametrize('outage',['height','input'])
def test_real_ranges_reacquire_after_outage_without_changing_measurement_or_height_limits(outage):
    f=RangeFixture()
    for seq in range(140):last=f.step(seq)
    assert last['ok'] and last['source']=='ToF_validated_B3'
    old=f.candidate.recovery_reference.copy()
    if outage=='height':
        for seq in range(140,146):assert not f.step(seq,height=False)['ok']
    decisions=[]
    for seq in range(146,185):
        d=f.step(seq,x=2.02,time_shift=250_000 if outage=='input' else 0)
        decisions.append(d)
        if not d['ok']:assert np.array_equal(f.candidate.recovery_reference,old)
    recovered=[d for d in decisions if d['reason']=='near_prior_subset_reacquired']
    assert recovered and recovered[0]['reacquisition']['elapsed_s']>=.15
    assert recovered[0]['xy_m']==pytest.approx([2.02,2.],abs=.015)
    assert decisions[-1]['ok'] and decisions[-1]['xy_m']==pytest.approx([2.02,2.],abs=.005)


def test_real_b3_ranges_far_from_old_position_do_not_acquire_a_new_airborne_origin():
    f=RangeFixture()
    for seq in range(140):last=f.step(seq)
    assert last['ok']
    for seq in range(140,146):f.step(seq,height=False)
    old=f.candidate.recovery_reference.copy()
    for seq in range(146,300):
        d=f.step(seq,x=3.)
        assert not d['ok']
        assert np.array_equal(f.candidate.recovery_reference,old)
    assert d['reacquisition']['reason']=='outside_prior_radius'
