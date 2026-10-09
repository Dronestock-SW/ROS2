"""Mission persistence and replay prevention; no aircraft or ROS publisher."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import sqlite3
import time

import pytest

from drone_mission.contracts import Settings
from drone_mission.local_web import LocalPlatform


ROUTE = [dict(id='P1', x=2., y=2.)]


def ready(p, **kwargs):
    p.telemetry=dict(fc_connected=True,fc_armed=False,fc_landed=1,flight_state='IDLE',**kwargs)
    p.telemetry_received_s=time.monotonic()


def save(p, name='호버', definition=None, **kwargs):
    return p.catalog_command(dict(action='save',name=name,definition=definition or {'trial_case':'hover'},**kwargs))


def test_drafts_offline_duplicates_revision_conflict_and_archive_survive_restart(tmp_path):
    path=tmp_path/'missions.sqlite3'
    p=LocalPlatform(mission_file=path)
    first=save(p)
    assert save(p,name='중복')['draft_id']==first['draft_id']
    assert save(p)['duplicate']
    assert p.assignment['control_action'] is None
    assert not p.store.listing()['runs']
    with pytest.raises(ValueError):save(p,definition={'trial_case':'x'})  # Same name, different path.
    save(p,definition={'trial_case':'x'},draft_id=first['draft_id'],revision=1)
    with pytest.raises(ValueError):save(p,definition={'trial_case':'y'},draft_id=first['draft_id'],revision=1)
    p.store.close()
    restored=LocalPlatform(mission_file=path)
    assert restored.store.draft(first['draft_id'])['revision']==2
    restored.catalog_command(dict(action='archive',draft_id=first['draft_id'],revision=2))
    assert restored.store.listing()['drafts']==[]
    assert restored.assignment['control_action'] is None
    with pytest.raises(ValueError):LocalPlatform(drone_id='6',mission_file=path)


@pytest.mark.parametrize('definition',[
    {'trial_case':'bad'}, {'route_tasks':[]}, {'route_tasks':[dict(id='P',x=float('nan'),y=2)]},
    {'route_tasks':[dict(id='P',x=99,y=2)]}, {'route_tasks':ROUTE*2},
    {'trial_case':'hover','route_tasks':ROUTE}, {'route_tasks':[dict(id='P',type='scan',x=2,y=2)]}])
def test_invalid_drafts_never_change_assignment(definition):
    p=LocalPlatform()
    with pytest.raises(ValueError):save(p,definition=definition)
    assert not p.store.listing()['drafts'] and p.assignment['control_action'] is None


def test_two_tabs_and_restart_do_not_replace_or_replay_start(tmp_path):
    path=tmp_path/'store.sqlite3';p=LocalPlatform(mission_file=path);ready(p)
    def start(key):
        try:return p.command('start',ROUTE,client_request_id=key)
        except ValueError:return None
    with ThreadPoolExecutor(max_workers=2) as pool:responses=list(pool.map(start,['click-one','click-two']))
    first=next(r for r in responses if r)
    assert sum(r is not None for r in responses)==1
    assert p.command('start',ROUTE,client_request_id=first['control_request_id'])==first
    with pytest.raises(ValueError):p.command('start',[dict(id='P2',x=2.,y=2.)],client_request_id=first['control_request_id'])
    p.store.close()
    restored=LocalPlatform(mission_file=path);ready(restored)
    assert restored.assignment['control_action'] is None
    assert restored.command('start',ROUTE,client_request_id=first['control_request_id'])==first
    assert restored.assignment['control_action'] is None  # Retry returns receipt; no dispatch.
    with pytest.raises(ValueError):restored.command('start',ROUTE)
    assert len(restored.store.listing()['runs'])==1


def test_active_draft_frozen_and_close_requires_matching_terminal_and_ground():
    p=LocalPlatform();ready(p);d=save(p,definition={'route_tasks':ROUTE})
    start=p.command('start',draft_id=d['draft_id'],revision=1)
    with pytest.raises(ValueError):save(p,name='수정',draft_id=d['draft_id'],revision=1)
    with pytest.raises(ValueError):p.catalog_command(dict(action='archive',draft_id=d['draft_id'],revision=1))
    close=dict(action='close_run',run_id=start['mission_db_id'])
    with pytest.raises(ValueError):p.catalog_command(close)
    p.telemetry.update({k:start[k] for k in ('mission_db_id','mission_code','route_revision')},flight_state='END',fc_armed=True)
    with pytest.raises(ValueError):p.catalog_command(close)
    p.telemetry['fc_armed']=False
    p.store.observe(dict(p.telemetry,mission_complete=True))
    assert p.catalog_command(close)['flight_command_sent'] is False
    assert not p.can_start()  # Closing history is not a companion reset.
    assert p.store.listing()['runs'][0]['state']=='END'
    ready(p)
    next_run=p.command('start',ROUTE)
    assert next_run['mission_db_id']>start['mission_db_id']
    assert next_run['mission_code']!=start['mission_code']


def test_stale_or_other_run_telemetry_cannot_complete_current_run():
    p=LocalPlatform();ready(p);a=p.command('start',ROUTE)
    p.store.observe(dict(mission_db_id=a['mission_db_id'],mission_code='older',route_revision=a['route_revision'],flight_state='END'))
    assert p.store.active()['state']=='REQUESTED'
    p.telemetry.update(a,flight_state='END');p.telemetry_received_s-=3
    with pytest.raises(ValueError):p.catalog_command(dict(action='close_run',run_id=a['mission_db_id']))


@pytest.mark.parametrize('change',[
    {'fc_armed':True},{'fc_landed':0},{'flight_state':'READY'},{'flight_state':'PILOT_OVERRIDE'}, {'fc_connected':False}])
def test_start_requires_idle_ground_without_overwriting_prior_assignment(change):
    p=LocalPlatform();ready(p);p.telemetry.update(change)
    with pytest.raises(ValueError):p.command('start',ROUTE)
    assert p.assignment['control_action'] is None and p.store.active() is None


def test_full_mission_server_gate_rejects_bypassing_disabled_start_button():
    p=LocalPlatform(settings=Settings(full_mission=True));ready(p)
    p.command('set_ceiling',ceiling=3)
    with pytest.raises(ValueError):p.command('start',ROUTE)
    assert not p.store.active()


def test_expired_unsent_run_can_close_only_in_fresh_idle_ground():
    p=LocalPlatform();ready(p);a=p.command('start',ROUTE)
    old=dict(a,control_requested_at=datetime.fromtimestamp(time.time()-35,timezone.utc).isoformat())
    p.store.acknowledge(old)
    assert p.catalog_command(dict(action='close_run',run_id=a['mission_db_id']))['flight_command_sent'] is False
    assert p.store.listing()['runs'][0]['state']=='REQUESTED'  # Never invent END.


def test_disk_failure_cannot_expose_a_start():
    p=LocalPlatform();ready(p)
    p.store.db.execute('PRAGMA query_only=ON')
    with pytest.raises(sqlite3.OperationalError):p.command('start',ROUTE)
    assert p.assignment['control_action'] is None


def test_pilot_takeover_blocks_land_and_return_without_new_receipts():
    p=LocalPlatform();ready(p);p.command('start',ROUTE)
    p.telemetry['flight_state']='PILOT_OVERRIDE'
    for action in ('land','return_to_home'):
        with pytest.raises(ValueError):p.command(action)
    assert p.store.db.execute('SELECT COUNT(*) FROM receipts').fetchone()[0]==1


@pytest.mark.parametrize('action',['land','return_to_home'])
def test_control_requires_current_run_and_connection_and_retries_do_not_reissue(action):
    p=LocalPlatform();ready(p);a=p.command('start',ROUTE)
    p.telemetry.update(flight_state='MOVING')
    with pytest.raises(ValueError):p.command(action)
    p.telemetry.update({k:a[k] for k in ('mission_db_id','mission_code','route_revision')},fc_connected=False)
    with pytest.raises(ValueError):p.command(action)
    p.telemetry['fc_connected']=True
    result=p.command(action,client_request_id='control-once')
    p.assignment['control_action']=None
    assert p.command(action,client_request_id='control-once')==result
    assert p.assignment['control_action'] is None
