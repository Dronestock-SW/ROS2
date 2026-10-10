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
