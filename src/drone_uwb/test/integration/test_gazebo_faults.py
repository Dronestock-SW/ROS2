"""Planned failures keep original ranges/truth and distinguish drops from errors."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from drone_uwb.integration.gazebo.gazebo_faults import RangeFaultPlan, EMPTY_PLAN
from drone_uwb.integration.gazebo.gazebo_ranges import main


def plan(kind='drop_all', start=.5, end=1.):
    event = dict(id='test_01',type=kind,start_s=start,end_s=end)
    if kind == 'range_bias':
        event.update(anchor_id='A2',bias_m=.5)
    return dict(EMPTY_PLAN,scenarios=[event])


def raw(elapsed):
    return dict(time_us=100_000_000+round(elapsed*1e6),seq=round(elapsed*1000000),
                clock_domain='gazebo_sim_us',raw_slant_m=[1.,2.,3.,4.],valid_mask=15)


def test_fault_clock_is_relative_to_first_sample_with_half_open_intervals():
    schedule = RangeFaultPlan(plan())
    flags = []
    for t in (0., .499999, .5, .999999, 1., 1.1):
        row = raw(t)
        result, info = schedule.apply(row)
        assert result == row
        flags.append(info['drop_requested'])
        assert info['origin_sim_us'] == 100_000_000
    assert flags == [False,False,True,True,False,False]


def test_bias_changes_only_declared_anchor_and_does_not_mutate_original_or_plan():
    config = plan('range_bias')
    original_config = deepcopy(config)
    schedule = RangeFaultPlan(config)
    schedule.apply(raw(0.))
    original = raw(.5)
    changed, info = schedule.apply(original)
    assert changed['raw_slant_m'] == [1.,2.5,3.,4.]
    assert original['raw_slant_m'] == [1.,2.,3.,4.]
    assert info['applied_bias_m'] == [0.,.5,0.,0.]
    assert changed['time_us'] == original['time_us']
    assert config == original_config
    recovered,_ = schedule.apply(raw(1.))
    assert recovered == raw(1.)


def test_out_of_range_bias_updates_validity_instead_of_clipping_it():
    config = plan('range_bias',0.,1.)
    config['scenarios'][0]['bias_m'] = 80.
    changed,_ = RangeFaultPlan(config).apply(raw(0.))
    assert changed['raw_slant_m'][1] == 82.
    assert changed['valid_mask'] == 13


@pytest.mark.parametrize('case',['duplicate','overlap','unknown','nan','tiny','wrong_clock'])
def test_invalid_fault_schedules_are_rejected(case):
    config = plan()
    if case in ('duplicate','overlap'):
        second = dict(config['scenarios'][0],id='second' if case=='overlap' else 'test_01')
        config['scenarios'].append(second)
    elif case == 'unknown':
        config['scenarios'][0]['type'] = 'clock_reversal'
    elif case == 'nan':
        config['scenarios'][0]['end_s'] = float('nan')
    elif case == 'tiny':
        config['scenarios'][0].update(start_s=0.,end_s=1e-8)
    else:
        config['clock_domain'] = 'wall_clock'
    with pytest.raises(ValueError):
        RangeFaultPlan(config)


def test_duplicate_or_reversed_simulation_time_is_not_a_second_injection():
    schedule = RangeFaultPlan()
    schedule.apply(raw(1.))
    with pytest.raises(ValueError,match='fault_clock_not_increasing'):
        schedule.apply(raw(1.))
    with pytest.raises(ValueError,match='fault_clock_not_increasing'):
        schedule.apply(raw(.9))


def cli_fixture(tmp_path, monkeypatch, *, behavior='success'):
    config = json.loads((Path(__file__).parents[2]/'config/gazebo_shadow.json').read_text(encoding='utf-8'))
    config['tag_offset_body_flu_m'] = [0.,0.,.3]
    config_path = tmp_path/'config.json'
    config_path.write_text(json.dumps(config),encoding='utf-8')
    calls,unsubscribed = [],[]
    class Packet:
        def __init__(self):
            self.header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nsec=0))
            self.data=''
    class Publisher:
        def publish(self, packet):
            calls.append(json.loads(packet.data))
            if behavior=='raise':
                raise OSError('test transport interrupted')
            return behavior!='false'
    class Node:
        def advertise(self,*args):
            return Publisher()
        def subscribe(self,kind,topic,callback):
            for i in range(9):
                stamp=100_000_000+i*250000
                callback(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                    sec=stamp//1000000,nsec=stamp%1000000*1000)),pose=[SimpleNamespace(
                    name='dronestock_x500_0',position=SimpleNamespace(x=2.,y=1.,z=1.),
                    orientation=SimpleNamespace(w=1.,x=0.,y=0.,z=0.))]))
            return True
        def unsubscribe(self,topic):
            unsubscribed.append(topic)
    for name,attr,value in (('gz.transport13','Node',Node),('gz.msgs10.pose_v_pb2','Pose_V',object),
                             ('gz.msgs10.stringmsg_pb2','StringMsg',Packet)):
        module=ModuleType(name)
        setattr(module,attr,value)
        monkeypatch.setitem(sys.modules,name,module)
    monkeypatch.setattr(sys,'stdout',SimpleNamespace(write=lambda text:len(text),flush=lambda:None,
                                                     reconfigure=lambda **kwargs:None))
    return config_path,calls,unsubscribed


@pytest.mark.parametrize('kind',['none','drop_all','range_bias'])
def test_live_cli_preserves_truth_originals_and_recovers_with_new_samples(tmp_path,monkeypatch,kind):
    config,calls,unsubscribed=cli_fixture(tmp_path,monkeypatch)
    output=tmp_path/'capture'
    args=['--config',str(config),'--output',str(output),'--rate-hz','4','--duration-s','1.5']
    if kind!='none':
        fault_file=tmp_path/'faults.json'
        fault_file.write_text(json.dumps(plan(kind)),encoding='utf-8')
        args+=['--fault-plan',str(fault_file)]
    main(args)
    def rows(name):
        return [json.loads(line) for line in (output/name).read_text(encoding='utf-8').splitlines()]
    before,after=rows('raw_before_fault.jsonl'),rows('raw_ranges.jsonl')
    emitted=rows('emission.jsonl')
    assert len(before)==len(after)==len(rows('truth.jsonl'))==len(rows('poses.jsonl'))==7
    assert len(calls)==(5 if kind=='drop_all' else 7)
    assert [r['seq'] for r in calls]==([0,1,4,5,6] if kind=='drop_all' else list(range(7)))
    for i,(original,changed) in enumerate(zip(before,after)):
        expected=.5 if kind=='range_bias' and i in (2,3) else 0.
        assert changed['raw_slant_m'][1]-original['raw_slant_m'][1]==pytest.approx(expected)
        assert all(changed['raw_slant_m'][k]==original['raw_slant_m'][k] for k in (0,2,3))
    assert sum(r['drop_requested'] for r in emitted)==(2 if kind=='drop_all' else 0)
    summary=json.loads((output/'capture.json').read_text(encoding='utf-8'))
    assert summary['stop_reason']=='duration_complete' and not summary['flight_valid']
    assert summary['counters'].get('publish_failed',0)==0
    assert len(unsubscribed)==1
    index=json.loads((output/'input_index.json').read_text(encoding='utf-8'))
    for name,record in index.items():
        assert hashlib.sha256((output/name).read_bytes()).hexdigest()==record['sha256']
    assert all('tag_xyz_m' not in row and 'body_xyz_m' not in row for row in calls)


@pytest.mark.parametrize('behavior',['false','raise'])
def test_publish_failure_is_not_mislabeled_as_planned_drop_and_evidence_survives(tmp_path,monkeypatch,behavior):
    config,calls,_=cli_fixture(tmp_path,monkeypatch,behavior=behavior)
    output=tmp_path/'capture'
    args=['--config',str(config),'--output',str(output),'--rate-hz','4','--duration-s','1.5']
    if behavior=='raise':
        with pytest.raises(OSError,match='test transport'):
            main(args)
    else:
        with pytest.raises(SystemExit) as error:
            main(args)
        assert error.value.code == 2
    emitted=[json.loads(line) for line in (output/'emission.jsonl').read_text(encoding='utf-8').splitlines()]
    assert len(emitted)==len(calls)
    assert all(not row['drop_requested'] and row['publish_attempted'] for row in emitted)
    assert all(row['reason']==('publish_exception' if behavior=='raise' else 'publish_failed') for row in emitted)
    assert emitted[0]['published'] is (None if behavior=='raise' else False)
    assert (output/'input_index.json').exists()
