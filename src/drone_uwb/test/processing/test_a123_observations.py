import copy
import pytest
from test_observations import LAYOUT, cycle, status, process
from drone_uwb.processing.solvers.observations import Processor, Settings


def make_processor():
    p=Processor(LAYOUT,Settings(min_anchors=3,active_anchor_mask=7,clock_warmup_samples=2,recovery_samples=2))
    process(p,status())
    return p


def test_missing_a4_computes_xyz_free_xy_and_preserves_wire():
    p=make_processor()
    for seq in (1,2):
        msg=cycle(seq);msg['valid_mask']=7;msg['raw_slant_m'][3]=None;msg['failure'][3]='timeout'
        original=copy.deepcopy(msg);result=process(p,msg)
        assert msg==original
    assert result.reason=='accepted'
    assert [result.observation.x,result.observation.y]==pytest.approx([2.09,1.68])
    assert result.observation.anchor_mask==7
    assert result.details['consistency_redundancy'] is False


def test_a4_recovery_is_not_silently_used():
    p=make_processor()
    process(p,cycle(1))
    msg=cycle(2);msg['raw_slant_m'][3]+=20
    result=process(p,msg)
    assert result.reason=='accepted'
    assert [result.observation.x,result.observation.y]==pytest.approx([2.09,1.68])
    assert result.observation.anchor_mask==7
    assert result.details['wire_anchor_mask']==15


def test_another_anchor_missing_cannot_be_replaced_by_a4():
    p=make_processor();process(p,cycle(1));process(p,cycle(2))
    msg=cycle(3);msg['valid_mask']=11
    assert process(p,msg).reason=='insufficient_anchors'
    assert process(p,cycle(4)).observation is None
    assert process(p,cycle(5)).reason=='accepted'


def test_default_still_requires_four_and_bad_profile_rejected():
    assert Settings().active_anchor_mask==15
    assert Settings().min_anchors==4
    with pytest.raises(ValueError):Settings(min_anchors=4,active_anchor_mask=7)
    with pytest.raises(ValueError):Settings(min_anchors=3,active_anchor_mask=3)
    with pytest.raises(ValueError):Settings(min_anchors=3,active_anchor_mask=7,source_mode='tag_xy')
