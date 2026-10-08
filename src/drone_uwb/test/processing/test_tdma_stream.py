"""Mixed serial streams must retain history and admit only fresh paired RAW."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.acquisition.tdma import TdmaGate
from drone_uwb.contracts.protocol import InvalidSample
from drone_uwb.processing.solvers.observations import Processor, Settings
from drone_uwb.processing.measured_btf import MeasuredBtf

ROOT=Path(__file__).parents[2]
LAYOUT=json.loads((ROOT/'config/anchors/anchors_20261004.json').read_text(encoding='utf-8'))
ANCHORS=np.asarray(LAYOUT['anchors_xyz_m'])


def status(tag='5', event='heartbeat'):
    return dict(type='uwb_raw_status',schema=1,tag_id=tag,event=event,
        firmware='uwb-tag-tdma40-v0.4.1-lora-sync1hz',uwb_ready=True,
        anchor_order=['A1','A2','A3','A4'],anchor_count=4,
        clock_domain='esp32_monotonic_boot_us',temporal_filter_applied=False)


def pair(seq=1, tag='5', boot=10, session=20, sf=None):
    offset=1000 if tag=='5' else 12500
    ref=2_000_000+seq*25_000
    times=[ref+offset+1250+i*2250 for i in range(4)]
    raw=dict(type='uwb_raw_cycle',schema=1,tag_id=tag,seq=seq,
        cycle_start_us=ref+offset+50,cycle_end_us=ref+offset+9050,
        valid_mask=15,raw_slant_m=np.linalg.norm(ANCHORS-[2.,2.,1.],axis=1).tolist(),
        sample_time_us=times,failure=['ok']*4,attempt_count=[1]*4,
        raw_xy_valid=True,raw_x_m=2.,raw_y_m=2.,raw_xy_anchor_mask=15)
    diag=dict(type='uwb_tdma_epoch',schema=1,tag_id=tag,boot_id=boot,session=session,
        sf_seq=seq if sf is None else sf,seq=seq,schedule_id=0x4001,
        start_offset_us=offset+50,end_offset_us=offset+9050,
        deadline_valid=True,local_overrun=False,epoch_span_us=6750,log_drops=0)
    return raw,diag


def event(msg, mono):
    return dict(message=msg,host_received_monotonic_ns=mono,
                host_received_ros_ns=mono+1_700_000_000_000_000_000)


@pytest.mark.parametrize('tag',['5','6'])
def test_mixed_stream_retains_history_and_has_one_output_per_valid_pair(tag):
    p=Processor(LAYOUT,Settings(tag_id=tag,tdma_mode='required',clock_warmup_samples=2,recovery_samples=2))
    p.process(status(tag),12_000_000_000,1_700_000_012_000_000_000)
    accepted=[]
    for seq in range(1,81):
        raw,diag=pair(seq,tag)
        mono=10_000_000_000+raw['cycle_end_us']*1000
        ros=mono+1_700_000_000_000_000_000
        assert p.process(raw,mono,ros).reason=='awaiting_tdma'
        for side in (
            dict(type='lora_rx',schema=1,tag_id=tag,rssi_dbm=-50,snr_db=10,payload='{}'),
            dict(type='jetson_pose_ack',schema=1,tag_id=tag,accepted=False,pose_seq=None,
                 fix=False,lora_eligible=False,reason='invalid_packet')):
            stable=p.stable
            assert p.process(side,mono+1,ros+1).reason.startswith('sideband_')
            assert p.stable==stable
        result=p.process(diag,mono+5_000_000,ros+5_000_000)
        if result.observation: accepted.append(result)
    assert len(accepted)==79
    assert all(r.details['tdma_verified'] for r in accepted)
    assert accepted[-1].observation.seq==80
    assert (accepted[-1].observation.x,accepted[-1].observation.y)==pytest.approx((2.,2.))


def test_measured_btf_uses_the_same_gate_without_diagnostic_history_resets():
    c=json.loads((ROOT/'config/runtime/uwb_btf_real.json').read_text(encoding='utf-8'))
    c['tdma_mode']='required'
    p=MeasuredBtf(c)
    p.process(event(status(),12_000_000_000))
    outputs=[]
    for seq in range(1,161):
        raw,diag=pair(seq)
        mono=10_000_000_000+raw['cycle_end_us']*1000
        assert p.process(event(raw,mono))['reason']=='awaiting_tdma'
        result=p.process(event(diag,mono+5_000_000))
        if result['ok']: outputs.append(result)
    assert len(outputs)>100
    assert all(r['tdma_verified'] for r in outputs)
    assert outputs[-1]['sample_time_us']==raw['sample_time_us']


@pytest.mark.parametrize('tag,role', [('5','a'), ('6','b')])
def test_tdma_and_same_time_measured_xyz_survive_integration(tag,role):
    c=json.loads((ROOT/f'config/runtime/uwb_btf_tag_{role}.json').read_text(encoding='utf-8'))
    c['height'].update(mount_confirmed=True,flat_floor_confirmed=True,
                       tof_to_tag_body_flu_m=[0,0,.12])  # Synthetic surveyed fixture only.
    p=MeasuredBtf(c)
    anchors=np.asarray(c['anchors_xyz_m'])
    p.process(event(status(tag),12_000_000_000))
    accepted=[]
    for seq in range(1,161):
        raw,diag=pair(seq,tag)
        raw['raw_slant_m']=np.linalg.norm(anchors-[2.,2.,1.],axis=1).tolist()
        mono=10_000_000_000+raw['cycle_end_us']*1000
        for sample in raw['sample_time_us']:
            stamp=1_700_000_010_000_000_000+sample*1000
            p.height.add('imu',dict(stamp_ns=stamp-1_000_000,valid=True,quaternion_wxyz=[1,0,0,0]))
            p.height.add('tof',dict(stamp_ns=stamp,valid=True,range_m=.88))
        assert p.process(event(raw,mono))['reason']=='awaiting_tdma'
        result=p.process(event(diag,mono+5_000_000))
        if result['ok']:
            accepted.append(result)
    assert len(accepted)>100
    assert accepted[-1]['xyz_m']==pytest.approx([2.,2.,1.],abs=.001)
    assert accepted[-1]['height_ready']
    assert accepted[-1]['tdma_verified']
    assert accepted[-1]['sample_time_us']==raw['sample_time_us']
    assert accepted[-1]['time_us']==max(raw['sample_time_us'])


@pytest.mark.parametrize('edit,reason',[
    ({'seq':2},'tdma_seq_mismatch'),
    ({'schedule_id':1},'unsupported_tdma_schedule'),
    ({'deadline_valid':False},'tdma_deadline_conflict'),
    ({'epoch_span_us':6751},'tdma_sample_span_mismatch'),
    ({'boot_id':True},'invalid_tdma_diagnostic'),
    ({'tag_id':'6'},'schema_or_tag_mismatch'),
])
def test_bad_pair_cannot_be_used(edit,reason):
    g=TdmaGate('5','required');raw,diag=pair()
    g.ingest(raw,100,1000)
    diag.update(edit)
    with pytest.raises(InvalidSample,match=reason): g.ingest(diag,101,1001)


def test_missing_late_and_duplicate_pairs_are_not_observations():
    g=TdmaGate('5','required')
    raw,diag=pair(1);g.ingest(raw,100,1000)
    raw2,diag2=pair(2);g.ingest(raw2,200,1100)
    assert g.counts['tdma_missing']==1
    with pytest.raises(InvalidSample,match='tdma_seq_mismatch'): g.ingest(diag,201,1101)
    g.ingest(raw2,300,1200)
    with pytest.raises(InvalidSample,match='tdma_pair_expired'): g.ingest(diag2,50_000_301,50_001_201)
    g.ingest(raw2,60_000_000,60_001_000)
    assert g.ingest(diag2,60_000_001,60_001_001).message is not None
    g.ingest(raw2,60_000_002,60_001_002)
    with pytest.raises(InvalidSample,match='duplicate_or_old_tdma_epoch'):
        g.ingest(diag2,60_000_003,60_001_003)


def test_session_and_boot_changes_clear_timing_and_old_session_cannot_return():
    g=TdmaGate('5','required')
    raw,diag=pair();g.ingest(raw,100,1000);g.ingest(diag,101,1001)
    raw,diag=pair(2,session=21,sf=0);g.ingest(raw,200,1100)
    changed=g.ingest(diag,201,1101)
    assert changed.reset_history and not changed.clear_status
    raw,diag=pair(3,session=20);g.ingest(raw,300,1200)
    with pytest.raises(InvalidSample,match='retired_tdma_session'): g.ingest(diag,301,1201)
    raw,diag=pair(1,boot=11,session=21);g.ingest(raw,400,1300)
    changed=g.ingest(diag,401,1301)
    assert changed.reset_history and changed.clear_status


def test_malformed_transition_does_not_hide_the_next_valid_reset():
    g=TdmaGate('5','required')
    raw,diag=pair();g.ingest(raw,100,1000);g.ingest(diag,101,1001)
    raw,diag=pair(2,session=21);g.ingest(raw,200,1100)
    invalid=copy.deepcopy(diag);invalid['epoch_span_us']=1
    with pytest.raises(InvalidSample):g.ingest(invalid,201,1101)
    g.ingest(raw,300,1200)
    assert g.ingest(diag,301,1201).reset_history


def test_late_slot_and_log_loss_reject_only_the_affected_pair():
    g=TdmaGate('5','required')
    raw,diag=pair();diag.update(end_offset_us=11700,deadline_valid=False)
    g.ingest(raw,100,1000)
    assert g.ingest(diag,101,1001).reason=='tdma_deadline_rejected'
    raw,diag=pair(2);g.ingest(raw,200,1100);assert g.ingest(diag,201,1101).message
    raw,diag=pair(3);diag['log_drops']=1;g.ingest(raw,300,1200)
    assert g.ingest(diag,301,1201).reason=='tdma_log_drop'


def test_auto_enables_for_tdma_firmware_but_preserves_legacy_raw_replay():
    g=TdmaGate('5')
    raw,_=pair();assert g.ingest(raw,100,1000).message
    g.ingest(status(),200,1100)
    assert g.ingest(raw,300,1200).reason=='awaiting_tdma'
    legacy=status();legacy['firmware']='old_raw'
    g.ingest(legacy,400,1300)
    assert g.required  # A later status cannot silently downgrade this connection.
