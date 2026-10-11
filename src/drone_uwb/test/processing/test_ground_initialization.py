import copy
import json
from pathlib import Path
import numpy as np
import pytest
from drone_uwb.processing.experiments.tof_subset import GroundSubsetInitialization, ToFTrackedSubsetCandidate
from drone_uwb.processing.experiments.h80_b import H80Window, BSettings
from drone_uwb.acquisition.validation import Cycle


def candidate(x=2., anchor=1):
    return dict(ok=True, source='ground_reference_B3', reason='unique_height_consistent_subset',
                xy_m=[x,2.], best_excluded_anchor=anchor)


@pytest.mark.parametrize('interruption', ['gap','movement','anchor','airborne','ambiguous','measured'])
def test_ground_acquisition_restarts_after_each_invalidating_event(interruption):
    p=GroundSubsetInitialization()
    for i in range(90):
        ready,_=p.check(1_000_000+i*22_000,candidate(),ground=True,already_initialized=False)
        assert not ready
    stamp=1_000_000+90*22_000; row=candidate();ground=True
    if interruption=='gap':stamp+=200_000
    if interruption=='movement':row=candidate(2.1)
    if interruption=='anchor':row=candidate(anchor=2)
    if interruption=='airborne':ground=False
    if interruption=='ambiguous':row.update(ok=False,reason='ambiguous_subset')
    if interruption=='measured':row['source']='ToF_validated_B3'
    assert not p.check(stamp,row,ground=ground,already_initialized=False)[0]
    assert not p.check(stamp+22_000,candidate(),ground=True,already_initialized=False)[0]


def test_ground_candidate_requires_full_span_and_never_reinitializes_tracking():
    p=GroundSubsetInitialization()
    for i in range(100):ready,proof=p.check(1_000_000+i*22_000,candidate(),ground=True,already_initialized=False)
    assert ready and proof['elapsed_s']>=2
    assert not p.check(3_200_000,candidate(),ground=True,already_initialized=True)[0]


@pytest.mark.parametrize('enabled,source,accept',[(False,'ground_antenna_reference',False),
    (True,'measured_tof_imu',False),(True,'ground_antenna_reference',True)])
def test_cold_start_with_one_bad_anchor_uses_ground_evidence_only(enabled,source,accept):
    c=json.loads((Path(__file__).parents[2]/'config/runtime/uwb_btf_real.json').read_text())
    c['ground_subset_initialization']=enabled
    c['anchors_xyz_m']=[[0,0,.15],[6.3,0,.15],[0,4.6,.15],[6.3,4.6,.15]]
    p=ToFTrackedSubsetCandidate(c,**c['tof_subset'])
    b=H80Window(c['anchors_xyz_m'],c['bias_m'],BSettings(**c['B']))
    a=np.asarray(c['anchors_xyz_m']);rows=[]
    for seq in range(180):
        end=1_000_000+seq*22_000;ts=[end-12_000+i*4000 for i in range(4)]
        raw=np.linalg.norm(np.array([5.,2.,.15])-a,axis=1);raw[0]-=.3
        cy=Cycle(seq,ts[0],end,15,raw,ts,['ok']*4,list(range(4)))
        row=dict(time_us=end,sample_time_us=ts,cal_slant_m=raw.tolist(),height_m=.15,
                 sample_height_m=[.15]*4,height_source=source,models={'B':b.process(cy,seq)})
        rows.append(p.process(row))
    assert bool(any(r['ok'] for r in rows)) is accept
    if accept:
        first=next(r for r in rows if r['ok'])
        assert first['ground_initialization']['elapsed_s']>=2
        assert rows[-1]['xy_m']==pytest.approx([5,2],abs=.01)
        # A loss of continuity after acquisition cannot silently create a new origin.
        row=copy.deepcopy(row);row['time_us']+=200_000
        row['sample_time_us']=[t+200_000 for t in ts]
        row['models']['B']['newest_sample_us']=row['time_us']
        assert not p.process(row)['ok']
