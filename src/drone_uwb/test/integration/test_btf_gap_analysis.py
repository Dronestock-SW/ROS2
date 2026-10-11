import importlib.util
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('btf_gap_tool',Path(__file__).parents[2]/'tools/analyze_btf_gaps.py')
tool=importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def test_gap_attribution_keeps_airborne_tail_and_rejects_stale_state(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps({'started':{'monotonic_ns':1_000_000_000}}))
    rows=[]
    def add(topic,t,data,string=False,age=0):
        ns=round(t*1e9)
        rows.append(dict(topic=topic,type='std_msgs/msg/String' if string else 'fixture',
            received_monotonic_ns=ns,received_ros_ns=ns,header_ns=ns-age,
            data={'data':json.dumps(data)} if string else data))
    add('/mavros/state',1,dict(armed=True,connected=True))
    add('/mavros/extended_state',1,dict(landed_state=2))
    add('/uwb/btf_pose',1.1,{})
    for t in [1.2,1.4,1.6]:
        add('/uwb/received',t,{'message':dict(type='uwb_raw_cycle',valid_mask=15,raw_slant_m=[1,2,3,4])},True)
        add('/uwb/btf_decision',t,dict(reason='no_fresh_continuity_prior',published=False),True)
    add('/mavros/state',1.7,dict(armed=True,connected=True),age=2_000_000_000)
    add('/uwb/received',2,{'message':dict(type='uwb_raw_cycle',valid_mask=15,raw_slant_m=[1,2,3,4])},True)
    (tmp_path/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    report=tool.analyze(tmp_path)
    gap=report['gaps_over_250ms'][0]
    assert gap['duration_s']==.5 and gap['boundary_censored']
    assert gap['raw_cycles']==2 and gap['reasons']=={'no_fresh_continuity_prior':2}
    assert report['topics']['/uwb/received']['samples']==3
    assert not report['fusion_verified']


def test_anchor_validity_does_not_count_invalid_flags_or_nan_as_receipt_success():
    assert tool.usable_range(dict(valid_mask=15,raw_slant_m=[1,2,3,4]),2)
    for d in [dict(valid_mask=3,raw_slant_m=[1,2,3,4]),
              dict(valid_mask=15,raw_slant_m=[1,2,'NaN',4]),
              dict(valid_mask=15,raw_slant_m=[1,2,float('nan'),4]),
              dict(valid_mask=15,raw_slant_m=[1,2])]:
        assert not tool.usable_range(d,2)


def test_missing_anchor_audit_separates_upstream_retry_from_anchor_timeout():
    cycles=[dict(valid_mask=3,raw_slant_m=[1,2,None,None],
                 failure=['ok','ok','slot_deadline','slot_deadline'],attempt_count=[2,1,0,0]),
            dict(valid_mask=11,raw_slant_m=[1,2,None,4],
                 failure=['ok','ok','response_timeout','ok'],attempt_count=[1,1,2,1]),
            dict(valid_mask=15,raw_slant_m=[1,2,3,4]),
            dict(valid_mask=0,raw_slant_m=[None]*4)]
    result=tool.missing_anchor_failures(cycles)
    assert result['by_anchor'][2]=={'slot_deadline':1,'response_timeout':1,'unrecorded':1}
    assert result['by_anchor'][3]=={'slot_deadline':1,'unrecorded':1}
    assert result['a3_a4_both_slot_deadline']==1
    assert result['a1_retried_before_both_skips']==1


def test_gap_timeline_preserves_trigger_and_frozen_guard_without_crossing_air_segments():
    def row(t,reason,published=False,**extra):
        return dict(received_monotonic_ns=round(t*1e9),seq=round(t*100),
                    reason=reason,replay_publish=published,**extra)
    segment=[row(1,'original_fit_consistent',True),
             row(1.025,'measured_height_unavailable'),
             row(1.1,'no_fresh_continuity_prior',stamp_ns=1000,
                 models={'B':{'fit':None},'B_TF':{'tracking_prior_age_s':.100042}}),
             row(2,'observation_jump_quarantined',replay_guard_prior={'stamp_ns':1000}),
             row(3,'observation_jump_quarantined',replay_guard_prior={'stamp_ns':1000}),
             row(3.025,'B_not_fresh_valid',models={'B_TF':{'reason':'B_not_fresh_valid'}}),
             row(4,'original_fit_consistent',True)]
    # Separate air segments and a long no-output tail cannot become a complete gap.
    result=tool.largest_complete_gap([segment,[row(100,'original_fit_consistent',True),
                                            row(200,'no_fresh_continuity_prior')]])
    assert result['duration_s']==3.
    assert result['first_failures'][1]['tracking_prior_age_s']==.100042
    assert result['dominant_guard_hold']['quarantined_decisions']==2
    assert result['dominant_guard_hold']['prior_update']['reason']=='no_fresh_continuity_prior'
    assert result['first_model_after_last_jump']['reason']=='B_not_fresh_valid'
    assert result['last_jump']['received_monotonic_ns']==3_000_000_000
    assert tool.largest_complete_gap([[row(1,'original_fit_consistent',True)],
                                    [row(20,'original_fit_consistent',True)]]) is None


def test_tof_limits_are_not_ground_height_or_hover_height():
    assert tool.valid_tof(dict(range=.3,min_range=.1,max_range=35.))
    for value in (.01,0.,-1.,36.,'NaN',None):
        assert not tool.valid_tof(dict(range=value,min_range=.1,max_range=35.))


def test_solver_comparison_reports_lost_or_changed_outputs_and_rejects_unaligned_runs(tmp_path):
    rows=[dict(received_monotonic_ns=i,seq=i,airborne=True,replay_publish=True,
               reason='original_fit_consistent',xy_m=[float(i),0.]) for i in range(1,4)]
    old=tmp_path/'before.jsonl';new=tmp_path/'after.jsonl'
    old.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    rows[0].update(replay_publish=False,reason='no_fresh_continuity_prior')
    rows[1]['xy_m']=[2.,.1]
    new.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    r=tool.compare_replays(old,new)
    assert r['removed_publications']==1 and r['removed'][0]['seq']==1
    assert r['shared_xy_delta_max_m']==pytest.approx(.1)
    assert r['upstream_changed_decisions']==0
    rows[2]['seq']=100
    new.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    with pytest.raises(ValueError,match='not aligned'):tool.compare_replays(old,new)


def test_replay_comparison_keeps_zero_output_segments_and_xy_without_z(tmp_path):
    rows=[]
    for t,air,old,new in [(1.,True,False,True),(1.5,True,False,True),
                          (2.,False,False,False),(3.,True,False,False),(4.,True,False,False)]:
        rows.append(dict(received_monotonic_ns=round(t*1e9),airborne=air,
            legacy_height_gate_publish=old,replay_publish=new,xyz_m=None,
            reason='original_fit_consistent',models={'B_TF':{'source':'unchanged_B4'}}))
    path=tmp_path/'replay.jsonl';path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    r=tool.replay_comparison(path)
    assert r['added_sources']=={'unchanged_B4':2} and r['added_with_xyz']==0
    assert r['removed_publications']==0
    assert r['comparison']['replay_publish']['published']==2
    assert r['comparison']['legacy_height_gate_publish']['published']==0
    assert r['comparison']['replay_publish']['gaps_over_250ms'][0]['duration_s']==1.
    assert r['comparison']['replay_publish']['gaps_over_250ms'][0]['boundary_censored']


def test_ordered_replay_distinguishes_queue_rejection_from_explicit_session_reset():
    from drone_uwb.processing.measured_btf import MeasuredBtf
    root=Path(__file__).parents[2]
    spec=importlib.util.spec_from_file_location('measured_replay',root/'tools/replay_measured_btf.py')
    replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
    p=MeasuredBtf(json.loads((root/'config/runtime/uwb_btf_real.json').read_text()))
    p.raw_guard.check((1.,1.),1_000_000_000)
    replay.apply_recorded_reset(p,dict(type='uwb_rejected',reset_scope='queued_models'))
    assert p.raw_guard.check((3.,3.),2_000_000_000)=='observation_jump_quarantined'
    with pytest.raises(ValueError,match='legacy rejection'):
        replay.apply_recorded_reset(p,dict(type='uwb_rejected'))
    replay.apply_recorded_reset(p,dict(type='ground_session_reset',accepted=False))
    assert p.raw_guard.prior_xy==(1.,1.)
    replay.apply_recorded_reset(p,dict(type='ground_session_reset',accepted=True,observation_session=1))
    assert p.raw_guard.prior_xy is None and p.observation_session==1
