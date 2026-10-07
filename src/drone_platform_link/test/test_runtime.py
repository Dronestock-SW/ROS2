import asyncio
import hashlib
import hmac
import json
from pathlib import Path
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

import pytest
from websockets.asyncio.server import serve

from drone_platform_link.runtime import Config, Runtime, fetch_mission, presence_payload
from drone_platform_link.telemetry import Observations


def config(tmp_path, server='http://127.0.0.1:8001', **extra):
    return Config({'DRONESTOCK_SERVER_URL': server,
                   'DRONESTOCK_STATE_DIR': str(tmp_path), **extra})


def test_presence_never_fabricates_sensor_or_flight_state(tmp_path):
    first = presence_payload(config(tmp_path), 1)
    second = presence_payload(config(tmp_path), 2)
    assert first['companion_link'] is True
    assert first['fix'] is False and first['telemetry_verified'] is False
    assert first['flight_control_enabled'] is False
    for key in ('x', 'y', 'current_z_m', 'battery', 'fc_connected', 'fc_armed',
                'fc_mode', 'flight_state', 'active_waypoint_index'):
        assert first[key] is None
    assert 'mission_db_id' not in first  # Receiving an assignment isn't executing it.
    assert second['telemetry_seq'] > first['telemetry_seq']
    assert first['sent_at_ms'] == first['t']


def test_observations_are_fresh_and_flight_stays_disabled(tmp_path):
    import time
    observations = Observations()
    observations.receive_pose(1.2, 3.4, 'uwb_map', time.time_ns())
    observations.receive_battery(0.63, 15.2)
    sent = presence_payload(config(tmp_path), 1, observations)
    assert sent['x'] == 1.2 and sent['y'] == 3.4 and sent['fix'] is True
    assert sent['battery'] == 63 and sent['battery_voltage'] == 15.2
    assert sent['flight_control_enabled'] is False
    observations.pose = (1.2, 3.4, time.time_ns(), time.monotonic() - 1)
    observations.battery = (63, 15.2, time.monotonic() - 4)
    stale = presence_payload(config(tmp_path), 2, observations)
    assert stale['fix'] is False and stale['x'] is None and stale['battery'] is None
    observations.receive_pose(4, 5, 'wrong_frame', time.time_ns())
    assert observations.fields()['x'] is None


@pytest.mark.parametrize('origin', [
    'file:///etc/passwd', 'http://user:secret@127.0.0.1',
    'http://127.0.0.1/platform', 'http://127.0.0.1?token=secret',
])
def test_config_rejects_non_origin_addresses(tmp_path, origin):
    with pytest.raises(ValueError):
        config(tmp_path, origin)


def test_device_signature_matches_wire_get(tmp_path):
    cfg = config(tmp_path, DRONESTOCK_DEVICE_AUTH_ID='test-device',
                 DRONESTOCK_DEVICE_AUTH_SECRET='test-secret')
    headers = cfg.headers()
    canonical = '\n'.join(('GET', cfg.path, hashlib.sha256(b'').hexdigest(),
                           headers['X-DS-Timestamp'], headers['X-DS-Nonce']))
    expected = hmac.new(b'test-secret', canonical.encode('utf-8'), hashlib.sha256).hexdigest()
    assert headers['X-DS-Signature'] == expected
    assert cfg.headers()['X-DS-Nonce'] != headers['X-DS-Nonce']


def test_poll_failures_do_not_refresh_last_success(tmp_path):
    async def scenario():
        runtime = Runtime(config(tmp_path))
        response = {'ok': True, 'status': 'ACTIVE'}
        with patch('drone_platform_link.runtime.fetch_mission',
                   side_effect=[response, OSError('offline')]):
            task = asyncio.create_task(runtime.poll_loop())
            try:
                for _ in range(100):
                    if runtime.http_successes:
                        break
                    await asyncio.sleep(0.01)
                received = runtime.mission_at
                assert received is not None
                await asyncio.sleep(0.6)
                assert runtime.http_failures == 1
                assert runtime.http_ok is False
                assert runtime.mission_at == received
                assert runtime.status()['mission_age_ms'] >= 500
            finally:
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
    asyncio.run(scenario())


def test_http_contract_rejection_and_continuous_reconnection(tmp_path):
    requests = []
    responses = {'payload': {
        'ok': True, 'contract_version': '1.0', 'drone_id': '5',
        'status': 'ACTIVE', 'control_action': 'land', 'control_request_id': 'test-only',
        'route_tasks': [{'x': 1, 'y': 2, 'z': 3}],
    }}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            requests.append(('GET', self.path))
            body = json.dumps(responses['payload']).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            requests.append(('POST', self.path))
            self.send_error(405)

        def log_message(self, *args):
            pass

    http = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=http.serve_forever, daemon=True)
    thread.start()
    cfg = config(tmp_path, f'http://127.0.0.1:{http.server_port}')
    try:
        for invalid in (
            {'ok': False},
            {'ok': True, 'contract_version': '2.0', 'drone_id': '5'},
            {'ok': True, 'contract_version': '1.0', 'drone_id': '6'},
        ):
            original = responses['payload']
            responses['payload'] = invalid
            with pytest.raises(ValueError):
                fetch_mission(cfg)
            responses['payload'] = original

        async def scenario():
            messages = []
            connections = 0

            async def handler(ws):
                nonlocal connections
                connections += 1
                async for raw in ws:
                    messages.append(json.loads(raw))
                    await ws.send('{"type":"test_echo"}')
                    if connections == 1:
                        await ws.close()
                        return

            async with serve(handler, '127.0.0.1', 0) as server:
                cfg.ws_url = f'ws://127.0.0.1:{server.sockets[0].getsockname()[1]}/ws/drones/5/'
                runtime = Runtime(cfg)
                stop = asyncio.Event()
                task = asyncio.create_task(runtime.run(stop))
                try:
                    for _ in range(120):
                        if connections >= 2 and len(messages) >= 6:
                            break
                        await asyncio.sleep(0.05)
                    assert connections >= 2
                    assert len(messages) >= 6
                    assert runtime.http_successes >= 2
                    assert runtime.ws_received >= 1
                    assert runtime.mission['control_action'] == 'land'
                    assert all(m['flight_control_enabled'] is False for m in messages)
                    assert [m['telemetry_seq'] for m in messages] == sorted(
                        {m['telemetry_seq'] for m in messages})
                finally:
                    stop.set()
                    await asyncio.wait_for(task, 4)
            saved = json.loads((Path(tmp_path) / 'mission.json').read_text(encoding='utf-8'))
            assert saved['execution_enabled'] is False
            assert saved['payload']['status'] == 'ACTIVE'
            status = json.loads((Path(tmp_path) / 'status.json').read_text(encoding='utf-8'))
            assert status['websocket_connected'] is False
        asyncio.run(scenario())
        assert all(method == 'GET' and path == cfg.path for method, path in requests)
    finally:
        http.shutdown()
        http.server_close()
        thread.join(timeout=2)
