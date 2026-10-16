"""Disk stalls must not borrow live telemetry or emergency command locks."""
from concurrent.futures import ThreadPoolExecutor
from http.server import ThreadingHTTPServer
import json
import threading
import time
from urllib.request import urlopen

import pytest

from drone_mission.local_web import LocalPlatform
from drone_mission.persistence import RequestJournal
from drone_mission.contracts import Settings


def flight(p, assignment=None):
    p.telemetry = dict(drone_id='5', fc_connected=True, fc_armed=True,
        fc_landed=2, fc_mode='POSCTL', flight_state='HOLD_READY', home_xy_m=[2.,2.], preflight={'checked_inputs_passed':True,'checks':[dict(code=k,passed=True) for k in ('layout_confirmed','transform','pose','estimator')]},
        native_takeoff=dict(can_land=True,can_start=True), operator_session_id='session')
    if assignment:
        p.telemetry.update({k:assignment[k] for k in ('mission_db_id','mission_code','route_revision')})
        p.telemetry['flight_state']='MOVING'
    p.telemetry_received_s=time.monotonic()


def test_blocked_database_keeps_http_telemetry_ack_and_land_live(tmp_path):
    p=LocalPlatform(mission_file=tmp_path/'missions.db')
    p.telemetry=dict(fc_connected=True,fc_armed=False,fc_landed=1,flight_state='IDLE')
    p.telemetry_received_s=time.monotonic()
    a=p.command('start',[dict(id='P1',x=2.,y=2.)])
    flight(p,a)
    entered=threading.Event();release=threading.Event()
    def slow():
        entered.set();assert release.wait(5)
    p.write_background('blocked-disk',slow)
    assert entered.wait(1)
    server=ThreadingHTTPServer(('127.0.0.1',0),p.http_handler())
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        t=time.monotonic()
        p.receive_telemetry(dict(p.telemetry, marker='current'))
        p.acknowledge(dict(request_id=a['control_request_id']))
        land=p.command('land',client_request_id='urgent-land')
        for path in ('/local/status','/local/missions','/api/drones/5/companion-mission/'):
            with urlopen(f'http://127.0.0.1:{server.server_port}'+path,timeout=.8) as response:
                value=json.load(response)
                if path.endswith('/status'):assert value['telemetry']['marker']=='current'
                if path.endswith('/companion-mission/'):assert value['control_action']=='land'
        assert time.monotonic()-t<.8
        assert not release.is_set()
        assert p.command('land',client_request_id='urgent-land')==land
    finally:
        release.set();server.shutdown();server.server_close();thread.join();p.close()
    # The delayed earlier ACK cannot erase the later LAND in the durable run.
    restored=LocalPlatform(mission_file=tmp_path/'missions.db')
    try:
        saved=json.loads(restored.store.active()['assignment'])
        assert saved['control_request_id']=='urgent-land'
        assert saved['control_action']=='land'
        assert restored.command('land',client_request_id='urgent-land')==land
        assert restored.assignment['control_action'] is None  # no restart replay
    finally:restored.close()


def test_land_supersedes_start_that_is_still_persisting():
    p=LocalPlatform(settings=Settings(speed_m_s=.25,command_frame='local_xy',execution_profile='anchor_trial',full_mission=True,start_mode='takeoff_hold',takeoff_backend='local_position'))
    flight(p);p.site['ceiling_height_m']=3.
    entered=threading.Event();release=threading.Event();start=p.store.start
    def slow(*args,**kwargs):
        entered.set();assert release.wait(5);return start(*args,**kwargs)
    p.store.start=slow
    try:
        with ThreadPoolExecutor(1) as pool:
            result=pool.submit(p.command,'start',trial_case='hover')
            assert entered.wait(1)
            land=p.takeoff(dict(action='takeoff_land',client_request_id='cancel-pending-start',operator_session_id='session'))
            assert land['control_action']=='takeoff_land'
            release.set()
            assert result.result(2)['dispatch_cancelled']
        assert p.assignment['control_action']=='takeoff_land'
        assert p.store.active() is None
    finally:release.set();p.close()


def test_failed_evidence_blocks_new_start_but_not_land():
    p=LocalPlatform(settings=Settings(speed_m_s=.25,command_frame='local_xy',execution_profile='anchor_trial',full_mission=True,start_mode='takeoff_hold',takeoff_backend='local_position'))
    flight(p);p.disk.error='disk_full'
    try:
        assert not p.can_start()
        p.takeoff(dict(action='takeoff_land',client_request_id='land-disk-full',operator_session_id='session'))
        assert p.assignment['control_action']=='takeoff_land'
    finally:p.close()


def test_request_journal_future_preserves_durability_and_deduplicates(tmp_path,monkeypatch):
    import drone_mission.persistence as storage
    original=storage.write_consumed;entered=threading.Event();release=threading.Event()
    def slow(*args):
        entered.set();assert release.wait(5);original(*args)
    monkeypatch.setattr(storage,'write_consumed',slow)
    journal=RequestJournal(tmp_path/'ledger.json')
    first=journal.submit('first');assert entered.wait(1)
    assert journal.submit('first') is first
    second=journal.submit('second')
    assert not first.done() and not second.done() and not journal.persisted
    release.set();assert second.result(2)=='second'
    assert journal.worker.close()
    assert json.loads(journal.path.read_text())['consumed']==['first','second']


def test_receipt_conflict_does_not_send_land():
    p=LocalPlatform();p.telemetry=dict(fc_connected=True,fc_armed=False,fc_landed=1,flight_state='IDLE');p.telemetry_received_s=time.monotonic()
    a=p.command('start',[dict(id='P1',x=2.,y=2.)],client_request_id='one-identifier');flight(p,a)
    try:
        with pytest.raises(ValueError):p.command('land',client_request_id='one-identifier')
        assert p.assignment['control_action']=='start'
    finally:p.close()
