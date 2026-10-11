import json
from pathlib import Path

import pytest

from drone_uwb.integration.boot_capture import ground_only, prune_idle
from drone_uwb.integration.manual_capture import GroundObservation


def evidence():
    proof = GroundObservation(1_000_000_000)
    for i in range(20):
        t = 1_000_000_000+i*100_000_000
        proof.observe('/mavros/state', dict(connected=True, armed=False), t, t, t)
        proof.observe('/mavros/extended_state', dict(landed_state=1), t, t, t)
    return dict(scope='manual_flight_receive_only', started=dict(wall_ns=1)), dict(
        stopped=True, error=None, writer_drained=True, ground_observation=proof.report(t)), proof


def test_only_continuously_observed_disarmed_ground_is_prunable():
    m,s,p = evidence()
    assert ground_only(m,s)
    p.observe('/mavros/state', dict(connected=True,armed=True),3_000_000_000,3_000_000_000,3_000_000_000)
    p.observe('/mavros/state', dict(connected=True,armed=False),3_100_000_000,3_100_000_000,3_100_000_000)
    s['ground_observation'] = p.report(3_100_000_000)
    assert not ground_only(m,s)  # A later DISARM cannot erase an earlier ARM.


@pytest.mark.parametrize('mutation', [
    {'stopped':False}, {'error':'disk_fault'}, {'ground_observation':{}}, {'writer_drained':False}])
def test_partial_and_failed_chunks_are_preserved(mutation):
    m,s,_ = evidence(); s.update(mutation)
    assert not ground_only(m,s)


@pytest.mark.parametrize('key,value', [('all_fresh',False),('all_connected',False),
    ('all_ground',False),('max_state_gap_s',2.),('last_landed_age_s',2.),
    ('first_state_delay_s',2.),('landed_count',0)])
def test_uncertain_state_is_preserved(key,value):
    m,s,_ = evidence(); s['ground_observation'][key]=value
    assert not ground_only(m,s)


def test_retention_keeps_flight_and_unknown_chunks(tmp_path):
    for i in range(7):
        p=tmp_path/f'capture-{i}'; p.mkdir()
        m,s,_=evidence(); m['started']['wall_ns']=i
        if i==0: s['ground_observation']['all_disarmed']=False
        if i==1: s.pop('ground_observation')
        (p/'manifest.json').write_text(json.dumps(m))
        (p/'summary.json').write_text(json.dumps(s))
    assert prune_idle(tmp_path)==['capture-2','capture-3']
    assert {p.name for p in tmp_path.iterdir()}=={'capture-0','capture-1','capture-4','capture-5','capture-6'}


def test_observation_launch_has_no_command_executor_or_ev_plugin():
    root=Path(__file__).resolve().parents[2]
    launch=(root/'launch/manual_observe.launch.py').read_text()
    config=(root/'config/runtime/mavros_btf_bench.yaml').read_text()
    assert "executable='flight_mission'" not in launch
    assert "executable='uwb_px4_bridge'" not in launch
    allow=config.split('plugin_allowlist:',1)[1].split('\n',1)[0]
    assert 'vision_pose' not in allow and 'setpoint' not in allow


def test_preflight_context_survives_later_idle_retention(tmp_path):
    for i in range(9):
        p=tmp_path/f'capture-{i}'; p.mkdir()
        m,s,_=evidence(); m['started']['wall_ns']=i
        if i==3: s['ground_observation']['all_disarmed']=False
        (p/'manifest.json').write_text(json.dumps(m))
        (p/'summary.json').write_text(json.dumps(s))
    removed=prune_idle(tmp_path)
    assert 'capture-2' not in removed and 'capture-3' not in removed
    assert removed==['capture-0','capture-1','capture-4','capture-5']
