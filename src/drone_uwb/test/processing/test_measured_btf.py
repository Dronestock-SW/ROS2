import copy
import json
from pathlib import Path
import numpy as np
import pytest

from drone_uwb.processing.measured_btf import MeasuredBtf, MeasuredHeight
from drone_uwb.processing.experiments.h80_b import H80Window, BSettings
from drone_uwb.processing.experiments.tof_subset import ToFTrackedSubsetCandidate
from drone_uwb.acquisition.validation import Cycle

CONFIG = Path(__file__).parents[2]/'config/runtime/uwb_btf_real.json'


def config():
    return json.loads(CONFIG.read_text(encoding='utf-8'))


def test_mount_projects_to_antenna_height_and_never_uses_future_or_old_sensor():
    h = MeasuredHeight(config()['height'])
    h.add('imu',dict(stamp_ns=1_000_000_000,valid=True,quaternion_wxyz=[1,0,0,0]))
    h.add('tof',dict(stamp_ns=1_010_000_000,valid=True,range_m=.5))
    assert h.at(1_000_000_000)[1]['reason'] == 'tof_unavailable'
    assert h.at(1_020_000_000)[0] == pytest.approx(.62)
    assert h.at(1_120_000_000)[1]['reason'] == 'tof_stale'
    h.add('tof',dict(stamp_ns=1_030_000_000,valid=False,range_m=0.))
    assert h.at(1_040_000_000)[1]['reason'] == 'tof_invalid'


def test_mount_and_floor_must_be_confirmed():
    for flag in ('mount_confirmed','flat_floor_confirmed'):
        c = config()['height']; c[flag] = False
        h = MeasuredHeight(c)
        h.add('imu',dict(stamp_ns=100,valid=True,quaternion_wxyz=[1,0,0,0]))
        h.add('tof',dict(stamp_ns=100,valid=True,range_m=.5))
        assert h.at(100)[0] is None


def event(msg, source_us):
    mono = 20_000_000_000+source_us*1000+2_000_000
    return dict(message=msg,host_received_monotonic_ns=mono,
                host_received_ros_ns=1_790_000_000_000_000_000+mono)


def status():
    return dict(schema=1,type='uwb_raw_status',tag_id='5',clock_domain='esp32_monotonic_boot_us',
                anchor_order=['A1','A2','A3','A4'],anchor_count=4,uwb_ready=True,temporal_filter_applied=False)


def cycle(seq, end, anchors):
    times = [end-13000+i*4000 for i in range(4)]
    raw = []
    for i,t in enumerate(times):
        point = [2+.1*t/1e6,2,1.12]
        raw.append(float(np.linalg.norm(np.asarray(point)-anchors[i])))
    return dict(schema=1,type='uwb_raw_cycle',tag_id='5',seq=seq,
        cycle_start_us=end-16000,cycle_end_us=end,valid_mask=15,raw_slant_m=raw,
        sample_time_us=times,failure=['ok']*4)


def test_sequential_real_ranges_keep_sample_times_and_compute_moving_position():
    c = config(); p = MeasuredBtf(c); anchors=np.asarray(c['anchors_xyz_m'])
    assert p.process(event(status(),100_000))['reason']=='status'
    results=[]
    for seq in range(1,101):
        end=100_000+seq*22000
        msg=cycle(seq,end,anchors)
        base=1_790_000_020_000_000_000+2_000_000
        for t in msg['sample_time_us']:
            p.height.add('imu',dict(stamp_ns=base+t*1000-1000000,valid=True,quaternion_wxyz=[1,0,0,0]))
            p.height.add('tof',dict(stamp_ns=base+t*1000,valid=True,range_m=1.))
        r=p.process(event(msg,end));results.append(r)
    accepted=[r for r in results if r['ok']]
    assert len(accepted)>60
    r=accepted[-1]
    assert r['sample_time_us']==msg['sample_time_us']
    assert r['report_span_s']==pytest.approx(.012)
    assert r['time_us']==max(msg['sample_time_us'])
    assert r['xy_m']==pytest.approx([2+.1*r['time_us']/1e6,2],abs=.001)
    assert r['external_output_allowed'] is False
    assert r['flight_valid'] is False
    assert r['timestamp_calibrated'] is False


def test_repeated_cycle_and_clock_restart_cannot_repeat_last_good_position():
    c=config();p=MeasuredBtf(c);a=np.asarray(c['anchors_xyz_m'])
    p.process(event(status(),100_000))
    for seq in range(1,80):
        end=100_000+seq*22000;msg=cycle(seq,end,a);r=p.process(event(msg,end))
    assert r['ok']
    duplicate=p.process(event(msg,end+1000))
    assert not duplicate['ok'] and duplicate['reason']=='duplicate_or_out_of_order'
    restart=p.process(event(cycle(0,20000,a),end+2000))
    assert not restart['ok'] and restart['reason']=='source_restart_wait_status'
    assert p.process(event(cycle(1,42000,a),end+3000))['reason']=='status_unavailable'


def test_missing_anchor_does_not_return_a_position():
    p=MeasuredBtf(config());p.process(event(status(),100_000))
    msg=cycle(1,122000,np.asarray(config()['anchors_xyz_m']));msg['valid_mask']=7
    assert p.process(event(msg,122000))['reason']=='four_valid_ranges_required'


def test_async_subset_uses_per_anchor_heights_and_rejects_unavailable_height():
    c=config(); b=H80Window(c['anchors_xyz_m'],c['bias_m'],BSettings(**c['B']))
    candidate=ToFTrackedSubsetCandidate(c,**c['tof_subset'])
    a=np.asarray(c['anchors_xyz_m']); decisions=[]
    for seq in range(180):
        end=1_000_000+seq*22000;ts=[end-12000+i*4000 for i in range(4)]
        raw=np.array([np.linalg.norm(np.array([2+.12*t/1e6,2.,1.12])-a[i]) for i,t in enumerate(ts)])
        if seq>=80:raw[1]+=.5
        cy=Cycle(seq,ts[0],end,15,raw,ts,['ok']*4,list(range(4)))
        base=b.process(cy,seq)
        row=dict(time_us=end,sample_time_us=ts,cal_slant_m=raw.tolist(),height_m=1.12,
                 sample_height_m=[1.12]*4,height_source='measured_tof_imu',models={'B':base})
        decisions.append(candidate.process(row))
    used=[d for d in decisions[100:] if d['ok'] and d['source']!='unchanged_B4']
    assert used
    assert all(d['best_excluded_anchor']==2 for d in used)
    assert used[-1]['xy_m']==pytest.approx([2+.12*end/1e6,2.],abs=.005)
    row['time_us']+=22000; row['sample_time_us']=[t+22000 for t in ts]
    row['models']['B']['newest_sample_us']=row['time_us']
    row['height_m']=None;row['height_source']=None
    assert candidate.process(row)['reason']=='measured_height_unavailable'
