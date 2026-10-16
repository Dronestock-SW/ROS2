"""Contract tests: real timestamp conversion, replay gates and B_TF continuity."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.acquisition.single_tag import (
    DOMAIN, FourTimestampClock, SingleTagLink, MappedSingleTagInput, cycle_fields)
from drone_uwb.acquisition.validation import InvalidInput
from drone_uwb.contracts.protocol import decode_line, InvalidSample
from drone_uwb.processing.measured_btf import MeasuredBtf


def status(boot=31, session=47):
    return dict(type='uwb_raw_status', schema=2, tag_id='6', boot_id=boot, session=session,
                rf_protocol=2, clock_domain=DOMAIN, timestamp_method='rf_poll_final_midpoint',
                period_us=20000, range_bias_applied=False, temporal_filter_applied=False)


def raw(seq=1, end=2_000_000):
    return dict(type='uwb_raw_cycle', schema=2, tag_id='6', boot_id=31, session=47,
                seq=seq, cycle_start_us=end-12000, cycle_end_us=end, period_us=20000,
                valid_mask=15, raw_slant_m=[2.,3.,4.,5.], sample_time_us=[end-10000+i*2500 for i in range(4)],
                failure=[0]*4, attempts=[1]*4, rf_map_span_us=[30]*4, clock_domain=DOMAIN,
                clock_sync_valid=True, clock_generation=1, deadline_missed=False, uwb_ready=True)


def synced():
    link = SingleTagLink()
    link.ingest(status(), 21_000_000_000, 121_000_000_000)
    for i in range(5):
        t1 = 21_000_000+i*250000
        request = link.request(t1)
        reply = dict(request, type='time_sync_reply', t2_tag_us=t1-20_000_000+1000,
                     t3_tag_us=t1-20_000_000+1500)
        _, command = link.ingest(reply, (t1+2500)*1000, 100_000_000_000+(t1+2500)*1000)
    assert command['type'] == 'time_sync_commit'
    return link


def test_four_timestamps_fit_drift_and_expire_without_receive_time_substitution():
    clock = FourTimestampClock()
    alpha, offset = 1.00008, 40_000_000
    for i in range(110):
        tag = 1_000_000+i*250000
        t1 = round(offset+alpha*tag)-1000
        t4 = round(offset+alpha*(tag+500))+1000
        clock.add(t1, tag, tag+500, t4)
    assert clock.ready(t4)
    assert clock.alpha == pytest.approx(alpha, abs=1e-6)
    assert abs(clock.host_us(tag)-offset-alpha*tag) < 10
    before = clock.last_us
    with pytest.raises(InvalidInput, match='sync_rtt'):
        clock.add(t1+250000, tag+250000, tag+250500, t4+300000)
    assert clock.last_us == before
    assert not clock.ready(t4+2_000_001)


def test_raw_strict_null_mask_times_and_duplicate_json():
    for key, value in [('valid_mask', True), ('raw_slant_m',[None,3.,4.,5.]),
                       ('failure',[1,0,0,0]), ('sample_time_us',[1,2,3,4]),
                       ('rf_map_span_us',[501]*4), ('sf_seq',0)]:
        with pytest.raises(InvalidInput):
            cycle_fields(dict(raw(), **{key:value}), 1, 2)
    msg = raw(); msg.update(valid_mask=14, raw_slant_m=[None,3.,4.,5.],
                            sample_time_us=[None,*msg['sample_time_us'][1:]], failure=[4,0,0,0])
    assert cycle_fields(msg,1,2).indices == [1,2,3]
    msg['raw_slant_m'][0] = 0
    with pytest.raises(InvalidInput, match='missing_must_be_null'):
        cycle_fields(msg,1,2)
    with pytest.raises(InvalidSample):
        decode_line('{"schema":2,"schema":1}')


def test_link_rejects_old_sessions_duplicate_cycle_and_stale_pose():
    link = synced()
    msg = raw(end=2_010_000); msg['clock_generation'] = link.clock.generation
    event, _ = link.ingest(msg,22_020_000_000,122_020_000_000)
    assert event['single_tag']['mapped']
    assert event['single_tag']['cycle_end_host_monotonic_ns'] == 22_010_000_000
    with pytest.raises(InvalidInput, match='duplicate_or_reordered'):
        link.ingest(msg,22_030_000_000,122_030_000_000)
    result = dict(published=True,ok=True,source_identity=[31,47],host_session=link.host_session,
                  seq=1,time_us=msg['sample_time_us'][-1],xy_m=[1.,2.])
    pose = link.pose_command(result,22_040_000_000)
    assert pose['sample_time_us'] == result['time_us']
    with pytest.raises(InvalidInput, match='stale_or_duplicate'):
        link.pose_command(result,22_050_000_000)
    link.ingest(status(session=48),22_050_000_000,122_050_000_000)
    assert not link.clock.ready(22_050_000)
    with pytest.raises(InvalidInput, match='retired_session'):
        link.ingest(status(),22_060_000_000,122_060_000_000)


def test_uint32_cycle_wrap_and_disconnect_invalidate_clock():
    link = synced()
    for seq,end in [(0xffffffff,2_010_000),(0,2_030_000)]:
        msg=raw(seq,end);msg['clock_generation']=link.clock.generation
        link.ingest(msg,(20_000_000+end+10000)*1000,(120_000_000+end+10000)*1000)
    oldhost=link.host_session
    link.disconnect()
    assert not link.clock.ready(22_050_000) and link.host_session != oldhost
    with pytest.raises(InvalidInput, match='status_unavailable'):
        link.ingest(raw(1,2_050_000),22_060_000_000,122_060_000_000)


def test_additive_first_failure_diagnostics_preserve_raw_and_clock_mapping():
    original_link, diagnostic_link = synced(), synced()
    original = raw(end=2_010_000)
    original['clock_generation'] = original_link.clock.generation
    original['attempts'][3] = 2
    diagnostic = copy.deepcopy(original)
    diagnostic['clock_generation'] = diagnostic_link.clock.generation
    diagnostic.update(diag_version=1, transaction=[1, 2, 3, 5],
                      first_transaction=[1, 2, 3, 4], first_failure=[0, 0, 0, 4],
                      cause=[14, 14, 14, 14], first_cause=[14, 14, 14, 3])
    baseline, _ = original_link.ingest(original, 22_020_000_000, 122_020_000_000)
    added, _ = diagnostic_link.ingest(decode_line(json.dumps(diagnostic)),
                                      22_020_000_000, 122_020_000_000)
    assert added['message']['raw_slant_m'] == baseline['message']['raw_slant_m']
    assert added['message']['sample_time_us'] == baseline['message']['sample_time_us']
    for key in ('mapped', 'sample_host_monotonic_ns', 'cycle_end_host_monotonic_ns'):
        assert added['single_tag'][key] == baseline['single_tag'][key]
    assert added['message']['first_failure'][3] == 4
    assert added['message']['failure'][3] == 0


@pytest.mark.parametrize('kind', ['uwb_rf_attempt', 'uwb_rf_recovery'])
def test_delayed_diagnostic_is_retained_without_becoming_an_observation(kind):
    link = synced()
    identity_before, clock_before = link.source, link.clock
    pending_before, recent_before = link.pending, list(link.recent)
    # Diagnostic records deliberately retain their old session for attribution.
    diagnostic = dict(type=kind, schema=2, diag_version=1, tag_id='6',
                      boot_id=31, session=46, seq=1, cause=6)
    event, command = link.ingest(diagnostic, 22_020_000_000, 122_020_000_000)
    assert event['message'] == diagnostic
    assert event['single_tag']['mapped'] is False and command is None
    assert link.source == identity_before and link.clock is clock_before
    assert link.pending == pending_before and list(link.recent) == recent_before


def test_btf_schema2_never_updates_legacy_clock_or_waits_for_tdma(monkeypatch):
    config = json.loads((Path(__file__).parents[2]/'config/runtime/uwb_btf_real.json').read_text(encoding='utf-8'))
    config['tag_id']='6'
    model = MeasuredBtf(config)
    def forbidden(*args):
        pytest.fail('schema 2 called the legacy clock or TDMA gate')
    monkeypatch.setattr(model.clock,'update',forbidden)
    monkeypatch.setattr(model.clock,'host_s',forbidden)
    monkeypatch.setattr(model.tdma,'ingest',forbidden)
    anchors=np.array(config['anchors_xyz_m'])
    link=synced()
    good=[]
    for seq in range(1,81):
        end=2_010_000+seq*20000
        msg=raw(seq,end);msg['clock_generation']=link.clock.generation
        msg['raw_slant_m']=[float(np.linalg.norm(np.array([2.,2.,1.12])-a)) for a in anchors]
        mono=(20_000_000+end+10000)*1000
        if seq % 25 == 0:
            link.ingest(status(),mono,mono+100_000_000_000)
        event,_=link.ingest(msg,mono,mono+100_000_000_000)
        result=model.process(event)
        assert result['reason'] != 'awaiting_tdma'
        assert result.get('source_identity') == [31,47]
        if result.get('ok'): good.append(result)
    assert len(good)>40
    assert good[-1]['xy_m'] == pytest.approx([2.,2.],abs=.01)
    assert good[-1]['stamp_ns'] == 120_000_000_000+good[-1]['time_us']*1000
    assert model.process(event)['reason']=='single_tag_duplicate_or_reordered'


def test_missing_mapping_and_changed_generation_cannot_reuse_legacy_clock():
    gate=MappedSingleTagInput();link=synced()
    msg=raw(end=2_010_000);msg['clock_generation']=link.clock.generation
    event,_=link.ingest(msg,22_020_000_000,122_020_000_000)
    bad=copy.deepcopy(event);bad['single_tag']['mapped']=False
    with pytest.raises(InvalidInput,match='clock_unsynced'): gate.accept(bad)
    msg=raw(2,2_030_000);msg['clock_generation']=link.clock.generation
    event,_=link.ingest(msg,22_040_000_000,122_040_000_000)
    event['single_tag']['clock']['generation']+=1
    with pytest.raises(InvalidInput,match='clock_unsynced'): gate.accept(event)
