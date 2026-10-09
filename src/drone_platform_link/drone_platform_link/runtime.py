"""Poll assignments, optionally forward them to the ROS mission executor."""

import asyncio
import hashlib
import hmac
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
import queue
from pathlib import Path
import signal
import socket
import threading
import time
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener
import uuid

from websockets.asyncio.client import connect
from .telemetry import Observations, SOURCES


LOG = logging.getLogger('dronestock-companion')


class NoRedirect(HTTPRedirectHandler):
    """Keep device credentials on the configured endpoint."""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def interval(env, key, default, minimum, maximum):
    value = float(env.get(key, default))
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{key} must be in [{minimum}, {maximum}]')
    return value


class Config:
    def __init__(self, env=None):
        env = os.environ if env is None else env
        self.server = env.get('DRONESTOCK_SERVER_URL', '').rstrip('/')
        url = urlsplit(self.server)
        if (url.scheme not in ('http', 'https') or not url.hostname
                or url.username or url.password or url.path or url.query or url.fragment):
            raise ValueError('DRONESTOCK_SERVER_URL must be an HTTP(S) origin')
        self.pose_source = env.get('DRONESTOCK_POSE_SOURCE', 'btf_xy')
        self.ground_antenna_height_m = (interval(env, 'DRONESTOCK_GROUND_ANTENNA_HEIGHT_M', 0, .01, 1.)
            if env.get('DRONESTOCK_GROUND_ANTENNA_HEIGHT_M') else None)
        poll_enabled = env.get('DRONESTOCK_MISSION_POLL_ENABLED', 'false').lower()
        if poll_enabled not in ('true', 'false'):
            raise ValueError('DRONESTOCK_MISSION_POLL_ENABLED must be true or false')
        self.mission_poll_enabled = poll_enabled == 'true'
        if self.pose_source not in SOURCES:
            raise ValueError('unsupported_pose_source')
        self.session_id = uuid.uuid4().hex
        self.drone_id = env.get('DRONESTOCK_DRONE_ID', '5')
        if not self.drone_id.isascii() or not self.drone_id.isdecimal():
            raise ValueError('DRONESTOCK_DRONE_ID must be numeric')
        self.path = f'/api/drones/{self.drone_id}/companion-mission/'
        scheme = 'wss' if url.scheme == 'https' else 'ws'
        self.ws_url = f'{scheme}://{url.netloc}/ws/drones/{self.drone_id}/'
        if env.get('DRONESTOCK_WS_URL'):
            ws = urlsplit(env['DRONESTOCK_WS_URL'])
            if (ws.scheme not in ('ws', 'wss') or not ws.hostname or ws.username or ws.password
                    or ws.query or ws.fragment or ws.path != f'/ws/drones/{self.drone_id}/'):
                raise ValueError('invalid_DRONESTOCK_WS_URL')
            self.ws_url = env['DRONESTOCK_WS_URL']
        forwarding = env.get('DRONESTOCK_MISSION_FORWARDING', 'false').lower()
        if forwarding not in ('false', 'true', '0', '1'):
            raise ValueError('DRONESTOCK_MISSION_FORWARDING_must_be_boolean')
        self.mission_forwarding = forwarding in ('true', '1')
        self.mission_poll_enabled = self.mission_poll_enabled or self.mission_forwarding
        self.uwb_topic = env.get('DRONESTOCK_UWB_TOPIC', SOURCES[self.pose_source][0])
        if self.uwb_topic != SOURCES[self.pose_source][0]:
            raise ValueError('pose_source_topic_mismatch')
        self.companion_id = env.get('DRONESTOCK_COMPANION_ID', socket.gethostname())
        self.auth_id = env.get('DRONESTOCK_DEVICE_AUTH_ID', '')
        self.auth_secret = env.get('DRONESTOCK_DEVICE_AUTH_SECRET', '')
        if bool(self.auth_id) != bool(self.auth_secret):
            raise ValueError('Both device authentication settings are required together')
        self.poll_s = interval(env, 'DRONESTOCK_POLL_S', '0.5', 0.5, 60)
        self.telemetry_hz = interval(env, 'DRONESTOCK_TELEMETRY_HZ', '10', 0.1, 10)
        self.state_dir = Path(env.get(
            'DRONESTOCK_STATE_DIR',
            str(Path.home() / '.local/state/dronestock-companion'),
        ))

    def headers(self, method='GET', path=None, body=b''):
        headers = {'Accept': 'application/json', 'User-Agent': 'DroneStock-Link/0.1.0'}
        if self.auth_id:
            timestamp = str(int(time.time()))
            nonce = uuid.uuid4().hex
            canonical = '\n'.join((
                method, path or self.path, hashlib.sha256(body).hexdigest(), timestamp, nonce,
            ))
            signature = hmac.new(
                self.auth_secret.encode('utf-8'), canonical.encode('utf-8'), hashlib.sha256,
            ).hexdigest()
            headers.update({
                'X-DS-Device-ID': self.auth_id,
                'X-DS-Timestamp': timestamp,
                'X-DS-Nonce': nonce,
                'X-DS-Signature': signature,
            })
        return headers


def fetch_mission(config):
    opener = build_opener(ProxyHandler(), NoRedirect())
    request = Request(config.server + config.path, headers=config.headers(), method='GET')
    with opener.open(request, timeout=2) as response:
        body = response.read(1024 * 1024 + 1)
    if len(body) > 1024 * 1024:
        raise ValueError('mission_response_too_large')
    payload = json.loads(body)
    if not isinstance(payload, dict) or payload.get('ok') is not True:
        raise ValueError('mission_not_ok')
    if payload.get('contract_version') != '1.0':
        raise ValueError('unsupported_contract_version')
    if str(payload.get('drone_id')) != config.drone_id:
        raise ValueError('mission_drone_id_mismatch')
    return payload


def post_report(config, suffix, payload):
    path = f'/api/drones/{config.drone_id}/{suffix}/'
    body = json.dumps(payload, allow_nan=False, separators=(',', ':')).encode('utf-8')
    headers = config.headers('POST', path, body)
    headers['Content-Type'] = 'application/json'
    opener = build_opener(ProxyHandler(), NoRedirect())
    with opener.open(Request(config.server+path, data=body, headers=headers, method='POST'), timeout=2) as response:
        raw = response.read(65537)
    if len(raw) > 65536:
        raise ValueError('report_response_too_large')
    if raw:
        result = json.loads(raw)
        if not isinstance(result, dict) or result.get('ok') is False:
            raise ValueError('report_not_ok')


def presence_payload(config, sequence, observations=None):
    """Unknown sensor/flight values stay null; presence isn't flight readiness."""
    timestamp = int(time.time() * 1000)
    payload = {
        'observation_contract': '1.0',
        'drone_id': config.drone_id,
        'companion_session_id': config.session_id,
        'type': 'telemetry',
        'telemetry_source': 'companion',
        'companion_link': True,
        'companion_id': config.companion_id,
        'companion_version': '0.1.0-communication-only',
        'companion_mode': 'communication_only',
        'flight_control_enabled': False,
        'telemetry_seq': sequence,
        'sent_at_ms': timestamp,
        't': timestamp,
        'fix': False,
        'telemetry_verified': False,
        'x': None,
        'y': None,
        'current_z_m': None,
        'current_z_source': None,
        'battery': None,
        'battery_voltage': None,
        'fc_connected': None,
        'fc_armed': None,
        'fc_mode': None,
        'flight_state': None,
        'uwb_seq': None,
        'uwb_age_ms': None,
        'active_waypoint_index': None,
        'active_waypoint_id': None,
    }
    if observations is not None:
        payload.update(observations.fields())
    if config.mission_forwarding:
        payload.update(companion_mode='mission_link', companion_version='0.2.0-mission-link')
    return payload


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def error_code(exc):
    # Avoid logging credentials, arbitrary response text, or full request headers.
    return f'HTTP_{exc.code}' if isinstance(exc, HTTPError) else type(exc).__name__


class Runtime:
    def __init__(self, config):
        self.config = config
        self.observations = Observations(config.pose_source, config.ground_antenna_height_m)
        self.started = time.time()
        self.mission = None
        self.mission_at = None
        self.mission_monotonic = None
        self.http_ok = False
        self.http_error = None
        self.http_successes = 0
        self.http_failures = 0
        self.ws_connected = False
        self.ws_connections = 0
        self.ws_error = None
        self.ws_sent = 0
        self.ws_received = 0
        self.ws_accepted = 0
        self.ws_rejected = 0
        self.ws_sent_at = None
        self.sequence = 0
        self.mission_queue = None
        self.forwarded = 0
        self.acknowledged = set()
        self.report_error = None

    async def poll_loop(self):
        retry_s = 1
        previous_summary = None
        while True:
            started = time.monotonic()
            try:
                payload = await asyncio.to_thread(fetch_mission, self.config)
            except Exception as exc:
                error = error_code(exc)
                if error != self.http_error:
                    LOG.warning('mission_poll_failed error=%s', error)
                self.http_ok = False
                self.http_error = error
                self.http_failures += 1
                # Keep the last successful reception time across failures.
                await asyncio.sleep(retry_s)
                retry_s = min(30, retry_s * 2)
                continue
            self.mission = payload
            self.mission_at = time.time()
            self.mission_monotonic = time.monotonic()
            self.http_successes += 1
            self.http_ok = True
            self.http_error = None
            if self.config.mission_forwarding and self.mission_queue is not None:
                try:
                    self.mission_queue.put_nowait(payload)
                except queue.Full:
                    try:
                        self.mission_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self.mission_queue.put_nowait(payload)
                self.forwarded += 1
            retry_s = 1
            summary = {key: payload.get(key) for key in (
                'status', 'mission_db_id', 'mission_code', 'route_revision',
                'control_action', 'control_request_id', 'anchor_layout_id',
                'anchor_layout_valid',
            )}
            if summary != previous_summary:
                LOG.info('mission_received forwarding=%s data=%s',
                         self.config.mission_forwarding, json.dumps(summary))
                previous_summary = summary
            await asyncio.sleep(max(0, self.config.poll_s - (time.monotonic() - started)))

    async def receive_loop(self, ws):
        # Drain server frames so echoes cannot block WebSocket keepalive.
        # Incoming frames never become control commands.
        async for _message in ws:
            self.ws_received += 1
            try:
                ack = json.loads(_message)
            except (ValueError, TypeError):
                continue
            if isinstance(ack, dict) and ack.get('type') == 'observation.ack':
                if ack.get('accepted') is True:
                    self.ws_accepted += 1
                else:
                    self.ws_rejected += 1
                    raise ValueError('observation_report_rejected')

    async def report_loop(self):
        retry_s = 1
        previous_phase = None
        while True:
            fields = self.observations.fields()
            ack = fields.get('control_ack')
            try:
                if ack and ack['request_id'] not in self.acknowledged:
                    await asyncio.to_thread(post_report, self.config, 'control-action/ack',
                                            dict(contract_version='1.0', **ack))
                    self.acknowledged.add(ack['request_id'])
                phase = fields.get('flight_state')
                identity = (fields.get('mission_db_id'), fields.get('route_revision'), phase)
                if phase and identity != previous_phase and identity[0] is not None:
                    report_phase = ('completed' if fields.get('mission_complete') else
                                    'failed_returning' if phase in ('END', 'FAILED', 'UNCONFIRMED', 'PILOT_OVERRIDE') else
                                    'in_flight' if phase in ('TAKING_OFF', 'STABILIZING', 'MOVING', 'DWELL',
                                                           'ALIGNING', 'SCANNING', 'EGRESS', 'RECOVERING', 'RETURNING', 'LANDING') else None)
                    if report_phase:
                        await asyncio.to_thread(post_report, self.config, 'companion-phase', {
                            'contract_version': '1.0', 'phase': report_phase,
                            'notes': fields.get('flight_reason', ''),
                            'mission_db_id': fields['mission_db_id'],
                            'mission_code': fields.get('mission_code')})
                    previous_phase = identity
                self.report_error = None
                retry_s = 1
                await asyncio.sleep(.25)
            except Exception as exc:
                self.report_error = error_code(exc)
                await asyncio.sleep(retry_s)
                retry_s = min(30, retry_s*2)

    async def websocket_loop(self):
        retry_s = 1
        while True:
            connected_at = None
            try:
                async with connect(
                    self.config.ws_url, additional_headers={**self.config.headers(path=urlsplit(self.config.ws_url).path),
                        'X-DS-Observation-Version':'1.0'}, open_timeout=3, close_timeout=2,
                    ping_interval=10, ping_timeout=10, max_size=65536, max_queue=1,
                ) as ws:
                    connected_at = time.monotonic()
                    self.ws_connected = True
                    self.ws_connections += 1
                    self.ws_error = None
                    LOG.info('websocket_connected mode=%s',
                             'mission_link' if self.config.mission_forwarding else 'communication_only')
                    receiver = asyncio.create_task(self.receive_loop(ws))
                    try:
                        while True:
                            if receiver.done():
                                receiver.result()
                                raise ConnectionError('websocket_closed')
                            self.sequence += 1
                            payload = presence_payload(self.config, self.sequence, self.observations)
                            await asyncio.wait_for(ws.send(json.dumps(payload)), timeout=2)
                            self.ws_sent += 1
                            self.ws_sent_at = time.time()
                            await asyncio.sleep(1 / self.config.telemetry_hz)
                    finally:
                        receiver.cancel()
                        await asyncio.gather(receiver, return_exceptions=True)
            except Exception as exc:
                self.ws_error = error_code(exc)
                LOG.warning('websocket_disconnected error=%s retry_s=%s', self.ws_error, retry_s)
            finally:
                self.ws_connected = False
            if connected_at is not None and time.monotonic() - connected_at >= 10:
                retry_s = 1
            await asyncio.sleep(retry_s)
            retry_s = min(30, retry_s * 2)

    def status(self):
        return {
            'updated_at_unix': time.time(),
            'started_at_unix': self.started,
            'mode': 'mission_link' if self.config.mission_forwarding else 'communication_only',
            'mission_poll_enabled': self.config.mission_poll_enabled,
            'flight_control_enabled': self.observations.fields().get('flight_control_enabled', False),
            'mission_forwarding': self.config.mission_forwarding,
            'assignments_forwarded': self.forwarded,
            'control_requests_acknowledged': len(self.acknowledged),
            'report_last_error': self.report_error,
            'server': self.config.server,
            'drone_id': self.config.drone_id,
            'http_ok': self.http_ok,
            'http_successes': self.http_successes,
            'http_failures': self.http_failures,
            'http_last_error': self.http_error,
            'mission_received_at_unix': self.mission_at,
            'mission_age_ms': None if self.mission_monotonic is None else round(
                (time.monotonic() - self.mission_monotonic) * 1000),
            'websocket_connected': self.ws_connected,
            'websocket_connections': self.ws_connections,
            'websocket_messages_sent': self.ws_sent,
            'websocket_messages_received': self.ws_received,
            'websocket_observations_accepted': self.ws_accepted,
            'websocket_observations_rejected': self.ws_rejected,
            'websocket_last_sent_at_unix': self.ws_sent_at,
            'websocket_last_error': self.ws_error,
        }

    def save_state(self):
        self.config.state_dir.mkdir(parents=True, exist_ok=True)
        atomic_json(self.config.state_dir / 'status.json', self.status())
        if self.mission is not None:
            atomic_json(self.config.state_dir / 'mission.json', {
                'received_at_unix': self.mission_at,
                'execution_enabled': False,
                'assignment_forwarded': self.config.mission_forwarding,
                'payload': self.mission,
            })

    async def status_loop(self):
        count = 0
        while True:
            self.save_state()
            if count % 6 == 0:
                LOG.info('link_status http_ok=%s ws_connected=%s polls=%s sent=%s',
                         self.http_ok, self.ws_connected, self.http_successes, self.ws_sent)
            count += 1
            await asyncio.sleep(5)

    async def run(self, stop):
        LOG.info('starting server=%s drone_id=%s mode=communication_only',
                 self.config.server, self.config.drone_id)
        loops = [self.websocket_loop, self.status_loop]
        if self.config.mission_poll_enabled:
            loops.append(self.poll_loop)
        if self.config.mission_forwarding:
            loops.append(self.report_loop)
        tasks = [asyncio.create_task(loop()) for loop in loops]
        stopper = asyncio.create_task(stop.wait())
        try:
            done, _ = await asyncio.wait([*tasks, stopper], return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        finally:
            for task in [*tasks, stopper]:
                task.cancel()
            await asyncio.gather(*tasks, stopper, return_exceptions=True)
            self.ws_connected = False
            self.http_ok = False
            self.save_state()
            LOG.info('stopped')


async def async_main(config):
    from .ros_monitor import start
    observations = Observations(config.pose_source, config.ground_antenna_height_m)
    mission_queue = queue.Queue(maxsize=1) if config.mission_forwarding else None
    monitor_stop = threading.Event()
    monitor = start(observations, config, mission_queue, monitor_stop)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(signum, stop.set)
    runtime = Runtime(config)
    runtime.observations = observations
    runtime.mission_queue = mission_queue
    try:
        await runtime.run(stop)
    finally:
        monitor_stop.set()
        await asyncio.to_thread(monitor.join, 5.)


def main():
    config = Config()
    config.state_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s',
        handlers=[logging.StreamHandler(), RotatingFileHandler(
            config.state_dir / 'communication.log',
            maxBytes=1024 * 1024, backupCount=3, encoding='utf-8',
        )],
    )
    asyncio.run(async_main(config))


if __name__ == '__main__':
    main()
