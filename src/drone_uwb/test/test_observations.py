"""Geometry, protocol, timing and frame gates used by the live path."""
import copy
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from drone_uwb.core import InvalidSample, LineFramer, Processor, Settings, decode_line, solve_xy
from drone_uwb.frames import BridgeSettings, gate, rotate_xy_covariance

LAYOUT = json.loads((Path(__file__).parents[1]/'config/anchors_20260906.json').read_text(encoding='utf-8'))
ANCHORS = np.array(LAYOUT['anchors_xyz_m'])


def status():
    return dict(type='uwb_raw_status', schema=1, tag_id='5', uwb_ready=True,
                anchor_order=['A1','A2','A3','A4'], anchor_count=4,
                clock_domain='esp32_monotonic_boot_us', temporal_filter_applied=False,
                anchor_layout_id='warehouse-5x4p5-z2p2-v2')


def cycle(seq=1, end=None, position=(2.09, 1.68, 1.19)):
    end = end if end is not None else 2_000_000 + seq*25_000
    ranges = np.linalg.norm(ANCHORS-np.array(position), axis=1).tolist()
    return dict(type='uwb_raw_cycle', schema=1, tag_id='5', seq=seq,
                cycle_start_us=end-20_000, cycle_end_us=end, valid_mask=15,
                raw_slant_m=ranges, failure=['ok']*4,
                sample_time_us=[end-16_000,end-12_000,end-8_000,end-4_000],
                raw_xy_valid=True, raw_x_m=position[0], raw_y_m=position[1], raw_xy_anchor_mask=15)


def process(p, msg, extra_ns=0, ros_extra_ns=0):
    mono = msg.get('cycle_end_us', 2_000_000)*1000 + 10_000_000_000 + extra_ns
    return p.process(msg, mono, mono+1_700_000_000_000_000_000+ros_extra_ns)


def warmed(settings=None):
    p = Processor(LAYOUT, settings or Settings(clock_warmup_samples=2, recovery_samples=2))
    process(p,status())
    process(p,cycle(1))
    assert process(p,cycle(2)).reason == 'accepted'
    return p


@pytest.mark.parametrize('height', [0.0,1.19,2.2,3.0])
def test_irregular_geometry_cancels_height(height):
    msg = cycle(position=(2.09,1.68,height))
    xy, residual = solve_xy(ANCHORS,np.array(msg['raw_slant_m']),[0,1,2,3])
    np.testing.assert_allclose(xy,[2.09,1.68],atol=1e-12)
    assert residual < 1e-12


def test_line_framer_split_multiple_and_overflow_recovery():
    framer=LineFramer(8)
    assert framer.feed(b'{"a"') == []
    assert framer.feed(b':1}\n{}\nxx') == [b'{"a":1}',b'{}']
    assert framer.feed(b'x'*50) == []
    assert len(framer.buffer) == 0
    assert framer.feed(b'junk\n{}\n') == [b'{}']
    assert framer.overflows == 1


@pytest.mark.parametrize('line',[b'[]',b'{"x":NaN}',b'{"x":Infinity}',b'\xff',b'{broken'])
def test_strict_json(line):
    with pytest.raises(InvalidSample):
        decode_line(line)


@pytest.mark.parametrize('field,value',[('schema',True),('valid_mask',True),('cycle_end_us',False),('seq',-1)])
def test_protocol_type_validation(field,value):
    p=warmed()
    msg=cycle(3)
    msg[field]=value
    assert process(p,msg).observation is None


def test_failures_and_missing_distances_never_invent_positions():
    p=warmed()
    msg=cycle(3)
    msg['raw_slant_m'][2]=None
    assert process(p,msg).reason == 'valid_mask_range_conflict'
    msg=cycle(4)
    msg['valid_mask']=11
    msg['raw_slant_m'][2]=None
    msg['failure'][2]='response_timeout'
    assert process(p,msg).reason == 'insufficient_anchors'
    # A timestamp may exist even when the Report subsequently fails validation.
    assert msg['sample_time_us'][2] is not None


def test_duplicate_and_sequence_wrap():
    p=warmed()
    assert process(p,cycle(2)).reason == 'duplicate_or_out_of_order'
    p.last_seq=0xffffffff
    assert process(p,cycle(0,end=2_075_000)).reason == 'warming_up'
    assert process(p,cycle(1,end=2_100_000)).reason == 'accepted'


def test_restart_requires_status_and_new_warmup():
    p=warmed()
    assert process(p,cycle(0,end=100_000),extra_ns=2_000_000_000).reason == 'source_restart_wait_status'
    assert process(p,cycle(1,end=125_000),extra_ns=2_000_000_000).reason == 'status_unavailable'
    process(p,status(),extra_ns=150_000_000)
    assert process(p,cycle(2,end=150_000),extra_ns=2_000_000_000).reason == 'warming_up'


def test_clock_jump_queue_and_stale_status():
    p=warmed()
    assert process(p,cycle(3),extra_ns=200_000_000).reason == 'queued_sample'
    assert process(p,cycle(4),ros_extra_ns=1_000_000_000).reason == 'warming_up'
    assert process(p,cycle(500)).reason == 'status_unavailable'


def test_obstruction_is_rejected_and_recovery_requires_new_good_samples():
    p=warmed(Settings(clock_warmup_samples=2,recovery_samples=2))
    msg=cycle(3)
    msg['raw_slant_m'][3]+=1.4
    decision=process(p,msg)
    assert decision.reason == 'inconsistent_ranges'
    assert decision.observation is None
    assert process(p,cycle(4)).reason == 'warming_up'
    assert process(p,cycle(5)).reason == 'accepted'


def test_coherent_but_impossible_range_jump_is_rejected():
    p=warmed()
    assert process(p,cycle(3,position=(3.09,1.68,1.19))).reason == 'range_jump'


def test_legacy_tag_xy_rejected_but_updated_tag_xy_accepted():
    p=warmed(Settings(source_mode='tag_xy',clock_warmup_samples=2,recovery_samples=2))
    msg=cycle(3)
    old=np.array([[0,0,2.2],[5,0,2.2],[0,4.5,2.2],[5,4.5,2.2]])
    xy,_=solve_xy(old,np.array(msg['raw_slant_m']),[0,1,2,3])
    msg['raw_x_m'],msg['raw_y_m']=map(float,xy)
    assert process(p,msg).reason == 'tag_layout_mismatch'
    assert process(p,cycle(4)).reason == 'warming_up'
    assert process(p,cycle(5)).reason == 'accepted'


def test_invalid_anchor_geometry_is_rejected():
    layout=copy.deepcopy(LAYOUT)
    layout['anchors_xyz_m'][2][2]+=0.1
    with pytest.raises(ValueError):
        Processor(layout)
    layout=copy.deepcopy(LAYOUT)
    layout['anchors_xyz_m']=[[0,0,2.2],[0,1,2.2],[0,2,2.2],[0,3,2.2]]
    with pytest.raises(ValueError):
        Processor(layout)


def test_rotation_changes_xy_and_covariance_only():
    settings=BridgeSettings(enu_yaw_deg=90,enu_offset_x_m=3,enu_offset_y_m=-1)
    xy,cov=rotate_xy_covariance(2,1,[[.04,.01],[.01,.09]],settings)
    np.testing.assert_allclose(xy,[2,1],atol=1e-12)
    np.testing.assert_allclose(cov,[[.09,-.01],[-.01,.04]],atol=1e-12)
    with pytest.raises(ValueError):
        rotate_xy_covariance(2,1,[[0,0],[0,0]],settings)


def test_bridge_requires_every_precondition_and_xy_only_fusion():
    params={'EKF2_EV_CTRL':1,'EKF2_EV_DELAY':0.,'EKF2_EV_NOISE_MD':0}
    defaults=BridgeSettings()
    assert gate(defaults,True,0,params,0) == 'disabled'
    ready=replace(defaults,enabled=True,alignment_confirmed=True,timing_confirmed=True,sensor_mount_confirmed=True)
    assert gate(ready,True,0,params,0) == 'ready'
    for field in ('alignment_confirmed','timing_confirmed','sensor_mount_confirmed'):
        assert gate(replace(ready,**{field:False}),True,0,params,0).endswith('_required')
    for mask in (0,3,5,9,15,None):
        assert gate(ready,True,0,{**params,'EKF2_EV_CTRL':mask},0) != 'ready'
    assert gate(ready,True,3,params,0) != 'ready'
    assert gate(ready,True,0,params,6) != 'ready'
    assert gate(ready,True,0,{**params,'EKF2_EV_DELAY':20.},0) != 'ready'
