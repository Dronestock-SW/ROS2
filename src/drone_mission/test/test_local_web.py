"""Local page freshness and HTTP command boundary checks."""

from http.server import ThreadingHTTPServer
import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from drone_mission.local_web import LocalPlatform
from drone_mission.contracts import Settings, parse_request


@pytest.fixture
def local_http():
    platform = LocalPlatform()
    server = ThreadingHTTPServer(('127.0.0.1', 0), platform.http_handler())
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        yield platform, 'http://127.0.0.1:'+str(server.server_port)
    finally:
        server.shutdown()
        server.server_close()
        worker.join(2)


def test_local_page_and_http_start_produce_a_new_explicit_request(local_http):
    _, origin = local_http
    with urlopen(origin+'/', timeout=2) as response:
        assert '이륙 · 경유지 이동 · 착륙' in response.read().decode('utf-8')
    body = json.dumps({'action':'start', 'route_tasks':[{'id':'P1', 'x':2.3, 'y':2.}]}).encode()
    request = Request(origin+'/local/command', data=body,
                      headers={'Content-Type':'application/json'}, method='POST')
    with urlopen(request, timeout=2) as response:
        first = json.load(response)
    with urlopen(request, timeout=2) as response:
        second = json.load(response)
    assert first['control_action'] == 'start'
    assert first['control_request_id'] != second['control_request_id']
    request.add_header('Origin', 'http://unrelated.example')
    with pytest.raises(HTTPError) as rejected:
        urlopen(request, timeout=2)
    assert rejected.value.code == 403


def test_browser_status_clears_flight_state_when_telemetry_expires(local_http):
    platform, origin = local_http
    platform.telemetry = {'flight_state':'MOVING', 'fc_armed':True, 'x':2.3}
    platform.telemetry_received_s = time.monotonic()
    with urlopen(origin+'/local/status', timeout=2) as response:
        current = json.load(response)
    assert current['telemetry_fresh'] is True
    assert current['telemetry']['fc_armed'] is True
    platform.telemetry_received_s = time.monotonic()-2.
    with urlopen(origin+'/local/status', timeout=2) as response:
        stale = json.load(response)
    assert stale['telemetry_fresh'] is False
    assert stale['telemetry'] == {}


def test_tag_b_web_request_uses_selected_layout_end_to_end():
    settings = Settings(drone_id='6',layout_id='warehouse-rectangle-6p3x4p6-z0p15-20261007')
    platform = LocalPlatform(settings.drone_id, settings.layout_id)
    request = platform.command('start',[dict(id='P1',x=2.3,y=2.)])
    assert request['drone_id'] == '6' and request['anchor_layout_id'] == settings.layout_id
    assert parse_request(request,settings,time.time())['action'] == 'start'


def test_preset_is_read_only_and_never_issues_start(local_http):
    platform, origin = local_http
    platform.telemetry=dict(flight_state='IDLE',fc_armed=False,fc_landed=1,px4_map_xy_m=[4.2,2.],
        preflight={'checks':[dict(code=k,passed=True) for k in ('layout_confirmed','alignment_confirmed','transform','pose','estimator')]})
    platform.telemetry_received_s=time.monotonic()
    with urlopen(origin+'/local/preset?case=xy',timeout=2) as response:
        route=json.load(response)['route_tasks']
    assert len(route)==9 and route[0]['dwell_s']==2
    assert platform.assignment['control_action'] is None
    platform.telemetry['px4_map_xy_m']=[float('nan'),2.]
    with pytest.raises(HTTPError) as error:
        urlopen(origin+'/local/preset?case=x',timeout=2)
    assert error.value.code==400


def test_ceiling_http_command_is_configuration_only(local_http):
    p,origin=local_http
    req=Request(origin+'/local/command',data=b'{"action":"set_ceiling","ceiling_height_m":3.0}',
                headers={'Content-Type':'application/json'},method='POST')
    with urlopen(req,timeout=2) as response:
        assert json.load(response)['flight_command_sent'] is False
    assert p.assignment['control_action'] is None
    with urlopen(origin+'/local/status',timeout=2) as response:
        assert json.load(response)['site']['ceiling_height_m']==3
