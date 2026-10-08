"""Fault injection through the real RAW adapter, independent of flight truth."""
import json
from pathlib import Path
import math
import pytest
from virtual_uwb import VirtualUwb
from drone_uwb.processing.measured_btf import MeasuredBtf
from drone_uwb.processing.observation_guard import ObservationGuard


@pytest.mark.parametrize('fault',['none','spike','nlos','coherent_step','phase_delay'])
@pytest.mark.parametrize('window_s',[.2,.8])
def test_sequential_tdma_raw_faults_recover_without_moving_accepted_xy(fault,window_s):
    root=Path(__file__).parents[2]/'drone_uwb/config/runtime/uwb_btf_tag_b_z2p2.json'
    c=json.loads(root.read_text(encoding='utf-8'))
    c['bias_m']=[.02,-.015,.01,.025]
    c['B']['window_s']=window_s
    c['height'].update(mount_confirmed=True,flat_floor_confirmed=True)
    model=VirtualUwb();adapter=MeasuredBtf(c);guard=ObservationGuard()
    accepted=[];reasons=[];decisions=[]
    for seq in range(240):
        end=100000+seq*25000
        stamp=1_790_000_020_000_000_000+end*1000+2_000_000
        active=100<=seq<132 and (fault!='spike' or seq==100)
        msgs=model.messages(end,[2+.05*end/1e6,2.,1.3],[.05,0,0],
                            fault=fault if active else 'none',elapsed=seq*.025)
        for t in msgs[1]['sample_time_us']:
            sample=stamp+(t-end)*1000
            adapter.height.add('imu',dict(stamp_ns=sample-1000000,valid=True,quaternion_wxyz=[1,0,0,0]))
            adapter.height.add('tof',dict(stamp_ns=sample,valid=True,range_m=1.3))
        for msg in msgs:
            out=adapter.process(dict(message=msg,host_received_monotonic_ns=20_000_000_000+end*1000+2_000_000,
                                     host_received_ros_ns=stamp))
        decisions.append(out);reasons.append(out['reason'])
        if out['ok'] and guard.check(tuple(map(float,out['xy_m'])),out['stamp_ns'])=='ready':
            expected=[2+.05*out['time_us']/1e6,2.]
            accepted.append((seq,math.dist(out['xy_m'],expected)))
    assert len(accepted)>120
    assert max(error for _,error in accepted)<.15
    assert accepted[-1][0]==239 and accepted[-1][1]<.02
    if fault=='coherent_step':
        assert 'observation_jump_quarantined' in reasons
        assert not any(100<=seq<132 for seq,_ in accepted)
    if fault=='nlos':
        assert any(d.get('models',{}).get('B_TF',{}).get('best_excluded_anchor')==2 for d in decisions)


def test_gradual_coherent_bias_is_not_observable_from_uwb_alone():
    # A physically plausible shared bias can pass a causal speed gate. This is
    # an explicit limitation, not a claim that all NLOS/bias has been solved.
    g=ObservationGuard()
    accepted=[]
    for i in range(100):
        xy=[2.+min(.12,i*.002),2.]
        if g.check(xy,1_000_000_000+i*25_000_000)=='ready':accepted.append(xy)
    assert accepted[-1][0]==pytest.approx(2.12)
