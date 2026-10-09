"""Local flight-command page and the Platform v1 HTTP/WS test endpoints."""

import argparse
import asyncio
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import sqlite3
import threading
import time
import uuid
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import serve
from .contracts import LAYOUT, Settings, finite, parse_request
from .field_presets import field_route
from .mission_store import MissionStore, digest
from .site import ceiling_value


from .web_page import PAGE


class LocalPlatform:
    def __init__(self, drone_id='5', layout_id=LAYOUT, settings=None, site_file=None, mission_file=None):
        self.lock = threading.Lock()
        self.drone_id = drone_id
        self.settings = settings or Settings(drone_id=drone_id, layout_id=layout_id)
        self.store = MissionStore(mission_file, drone_id, layout_id)
        self.telemetry, self.reports, self.acks = {}, [], []
        self.telemetry_received_s = None
        self.site_file = Path(site_file) if site_file else None
        self.site = dict(drone_id=drone_id, anchor_layout_id=layout_id, ceiling_height_m=None)
        if self.site_file and self.site_file.exists():
            saved = json.loads(self.site_file.read_text(encoding='utf-8'))
            if saved.get('drone_id') != drone_id or saved.get('anchor_layout_id') != layout_id:
                raise ValueError('saved_site_device_or_layout_mismatch')
            self.site['ceiling_height_m'] = ceiling_value(saved['ceiling_height_m'])
        self.assignment = dict(ok=True, contract_version='1.0', drone_id=drone_id,
            status='IDLE', control_action=None, coordinate_frame='UWB_ANCHOR_LOCAL',
            origin='A1', x_axis='A1_TO_A2', y_axis='A1_TO_A3', z_axis='UP_FROM_FLOOR', unit='meter',
            anchor_layout_id=layout_id, route_tasks=[])
        active = self.store.active()
        if active:
            self.assignment = json.loads(active['assignment'])
            # Restore the route heartbeat but never replay an old flight command.
            self.assignment['control_action'] = None

    def ground(self):
        t = self.telemetry
        return (self.telemetry_received_s is not None
                and 0 <= time.monotonic()-self.telemetry_received_s <= 1
                and t.get('fc_connected') is True and t.get('fc_armed') is False
                and t.get('fc_landed') == 1)

    def can_start(self):
        return (not self.store.active() and self.ground()
                and self.telemetry.get('flight_state') == 'IDLE'
                and (not self.settings.full_mission or (
                    self.site['ceiling_height_m'] is not None
                    and self.telemetry.get('preflight', {}).get('checked_inputs_passed') is True)))

    def catalog_command(self, value):
        with self.lock:
            action = value.get('action')
            if action == 'save':
                definition = value.get('definition')
                if not isinstance(definition, dict):
                    raise ValueError('미션 내용을 확인하세요.')
                if definition.get('trial_case') in ('hover','x','y','xy') and set(definition) == {'trial_case'}:
                    definition = dict(definition)
                elif set(definition) == {'route_tasks'}:
                    self.validate_route(definition['route_tasks'])
                else:
                    raise ValueError('시험 종류 또는 경유지 중 하나를 지정하세요.')
                return self.store.save_draft(value.get('name'), definition, value.get('draft_id'), value.get('revision'))
            if action == 'archive':
                return self.store.archive_draft(value.get('draft_id'), value.get('revision'))
            if action == 'close_run':
                row = self.store.active()
                if not row or type(value.get('run_id')) is not int or row['id'] != value['run_id'] or not self.ground():
                    raise ValueError('최신 착륙·시동 꺼짐 상태와 현재 실행 번호가 필요합니다.')
                a = json.loads(row['assignment'])
                matched = all(self.telemetry.get(k) == a.get(k) for k in ('mission_db_id','mission_code','route_revision'))
                terminal = matched and self.telemetry.get('flight_state') in ('END','LANDED','FAILED','UNCONFIRMED','PILOT_OVERRIDE')
                expired = time.time()-datetime.fromisoformat(a['control_requested_at']).timestamp() > self.settings.request_ttl_s
                if not terminal and not (expired and self.telemetry.get('flight_state') == 'IDLE'):
                    raise ValueError('진행 중·수락 대기 미션은 기록을 닫을 수 없습니다.')
                self.store.close_run(row['id'])
                self.assignment = {k:v for k,v in self.assignment.items() if k in (
                    'ok','contract_version','drone_id','coordinate_frame','origin','x_axis','y_axis','z_axis','unit','anchor_layout_id')}
                self.assignment.update(status='IDLE', control_action=None, route_tasks=[])
                return dict(ok=True, flight_command_sent=False, requires_new_ground_session=True)
            raise ValueError('지원하지 않는 미션 관리 요청입니다.')

    def validate_route(self, route):
        now = datetime.now(timezone.utc).isoformat()
        payload = dict(self.assignment, control_action='start', control_request_id='draft-validation',
            control_requested_at=now, mission_db_id=1, mission_code='DRAFT', route_revision='draft',
            route_tasks=route, status='ACTIVE')
        parse_request(payload, self.settings, time.time())

    def site_editable(self):
        # A new ground session is required after every START. Expired telemetry
        # cannot unlock a mission, and a web restart cannot override an armed FC.
        if self.store.active():
            return False
        if self.telemetry_received_s is None:
            return True  # Offline planning only; no command is generated.
        return (0 <= time.monotonic()-self.telemetry_received_s <= 1
                and self.telemetry.get('fc_armed') is False
                and self.telemetry.get('fc_landed') == 1
                and self.telemetry.get('flight_state') in ('IDLE', 'READY'))

    def automatic_start(self):
        t = self.telemetry
        checks = {c['code']:c['passed'] for c in t.get('preflight',{}).get('checks',[])}
        xy = t.get('px4_map_xy_m')
        if (self.telemetry_received_s is None or not 0 <= time.monotonic()-self.telemetry_received_s <= .5
                or t.get('fc_armed') is not False or t.get('fc_landed') != 1
                or t.get('flight_state') not in ('IDLE','READY')
                or not all(checks.get(k) is True for k in ('layout_confirmed','alignment_confirmed','transform','pose','estimator'))
                or not isinstance(xy,(list,tuple)) or len(xy)!=2 or not finite(*xy)):
            raise ValueError('출발점 확인 대기: 최신 PX4 위치·지상 상태·좌표 변환 검증이 필요합니다.')
        return tuple(xy)

    def command(self, action, route=None, ceiling=None, trial_case=None, client_request_id=None, draft_id=None, revision=None):
        if action not in ('start', 'land', 'return_to_home', 'set_ceiling'):
            raise ValueError('지원하지 않는 요청입니다.')
        with self.lock:
            if action == 'set_ceiling':
                if not self.site_editable():
                    raise ValueError('천장 변경은 새 지상 세션에서만 가능합니다.')
                site = dict(self.site, ceiling_height_m=ceiling_value(ceiling))
                if self.site_file:
                    self.site_file.parent.mkdir(parents=True, exist_ok=True)
                    temporary = self.site_file.with_suffix('.tmp')
                    with temporary.open('w', encoding='utf-8') as stream:
                        json.dump(site, stream, allow_nan=False)
                        stream.flush()
                        os.fsync(stream.fileno())
                    temporary.replace(self.site_file)
                self.site = site
                return dict(ok=True, site=dict(site), flight_command_sent=False)
            body = dict(action=action, route_tasks=route, trial_case=trial_case, draft_id=draft_id, revision=revision)
            key = client_request_id or uuid.uuid4().hex
            previous = self.store.receipt(key, body)
            if previous is not None:
                return previous  # A lost HTTP response must not create/reissue a command.
            if action == 'start':
                if not self.can_start():
                    raise ValueError('START 차단: 진행 미션·최신 지상 상태·비행 준비 조건을 확인하세요.')
                name = '직접 입력 미션'
                if draft_id is not None:
                    draft = self.store.draft(draft_id)
                    if type(revision) is not int or draft['revision'] != revision:
                        raise ValueError('저장 미션 버전이 바뀌었습니다. 다시 선택하세요.')
                    if route is not None or trial_case is not None:
                        raise ValueError('저장 미션과 직접 경로를 동시에 실행할 수 없습니다.')
                    route, trial_case = draft['definition'].get('route_tasks'), draft['definition'].get('trial_case')
                    name = draft['name']
                start_xy = None
                if trial_case is not None:
                    start_xy = self.automatic_start()
                    route = field_route(trial_case, start_xy, self.settings)
                if not isinstance(route, list) or not 1 <= len(route) <= 20:
                    raise ValueError('경유지 1~20개를 입력하세요.')
                if self.settings.full_mission and self.site['ceiling_height_m'] is None:
                    raise ValueError('현장 천장 높이를 먼저 저장하세요.')
                self.validate_route(route)
                revision = hashlib.sha256(json.dumps(route, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]
                assignment = dict(self.assignment, route_revision=revision, route_tasks=route, status='ACTIVE')
                if self.site['ceiling_height_m'] is not None:
                    assignment['ceiling_height_m'] = self.site['ceiling_height_m']
                if start_xy is not None:
                    assignment['planned_launch_xy_m'] = list(start_xy)
                else:
                    assignment.pop('planned_launch_xy_m',None)
            else:
                if (not self.store.active() or self.telemetry_received_s is None
                        or not 0 <= time.monotonic()-self.telemetry_received_s <= 1
                        or self.telemetry.get('fc_connected') is not True
                        or any(self.telemetry.get(k) != self.assignment.get(k)
                               for k in ('mission_db_id','mission_code','route_revision'))
                        or self.telemetry.get('flight_state') in ('IDLE','END','LANDED','PILOT_OVERRIDE','UNCONFIRMED','FAILED')):
                    raise ValueError('제어 가능한 현재 미션이 없습니다. RC 인계를 덮어쓰지 않습니다.')
                assignment = dict(self.assignment)
            now = datetime.now(timezone.utc).isoformat()
            assignment.update(control_action=action, control_request_id=key, control_requested_at=now, generated_at=now)
            if action == 'start':
                assignment = self.store.start(key, body, assignment, name, draft_id)
            else:
                self.store.control(key, body, assignment)
            self.assignment = assignment
            return dict(self.assignment)

    def http_handler(self):
        platform = self

        class Handler(BaseHTTPRequestHandler):
            def send(self, body, status=200, content_type='application/json'):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def send_json(self, value, status=200):
                self.send(json.dumps(value, allow_nan=False).encode('utf-8'), status)

            def do_GET(self):
                if self.path == '/':
                    return self.send(PAGE, content_type='text/html; charset=utf-8')
                query = urlsplit(self.path)
                if query.path == '/local/preset':
                    try:
                        q = parse_qs(query.query)
                        with platform.lock:
                            xy = platform.automatic_start()
                            route = field_route(q['case'][0], xy, platform.settings)
                        return self.send_json(dict(route_tasks=route,start_xy_m=xy,start_source='px4_ekf2_to_uwb_map'))
                    except (KeyError, ValueError, OverflowError) as error:
                        return self.send_json({'error':str(error)[:256]},400)
                with platform.lock:
                    if self.path == '/local/missions':
                        return self.send_json(platform.store.listing())
                    if self.path == f'/api/drones/{platform.drone_id}/companion-mission/':
                        return self.send_json(platform.assignment)
                    if self.path == '/local/status':
                        age = (time.monotonic()-platform.telemetry_received_s
                               if platform.telemetry_received_s is not None else None)
                        fresh = age is not None and 0 <= age <= 1.
                        try:
                            automatic_xy = platform.automatic_start()
                        except ValueError:
                            automatic_xy = None
                        return self.send_json(dict(telemetry=platform.telemetry if fresh else {},
                            telemetry_fresh=fresh, telemetry_age_ms=round(age*1000) if age is not None else None,
                            site=platform.site, site_editable=platform.site_editable(),
                            can_start=platform.can_start(), active_run_id=(platform.store.active() or {}).get('id'),
                            ceiling_required=platform.settings.full_mission,
                            expected_takeoff_height_m=platform.settings.expected_mis_takeoff_alt_m,
                            automatic_start_xy_m=automatic_xy,
                            assignment=platform.assignment, phase_reports=platform.reports, acks=platform.acks))
                self.send_json({'error':'unknown_endpoint'},404)

            def do_POST(self):
                try:
                    if self.headers.get_content_type() != 'application/json':
                        raise ValueError('JSON 요청이 필요합니다.')
                    length = int(self.headers.get('Content-Length','0'))
                    if not 0 < length <= 65536:
                        raise ValueError('요청 크기를 확인하세요.')
                    raw = self.rfile.read(length)
                    origin = self.headers.get('Origin')
                    if origin and origin != 'http://'+self.headers.get('Host',''):
                        # Drain the bounded request body before closing. An
                        # unread body can reset TCP and hide the 403 on Windows.
                        return self.send_json({'error':'origin_rejected'},403)
                    value = json.loads(raw)
                    if not isinstance(value,dict):
                        raise ValueError('JSON 객체가 필요합니다.')
                    if self.path == '/local/command':
                        for key, expected in (('expected_drone_id',platform.drone_id), ('expected_layout_id',platform.settings.layout_id)):
                            if key in value and value[key] != expected:
                                raise ValueError('요청의 기체·배치가 현재 페이지와 다릅니다. 새로 불러오세요.')
                        return self.send_json(platform.command(value.get('action'),value.get('route_tasks'),value.get('ceiling_height_m'),value.get('trial_case'),value.get('client_request_id'),value.get('draft_id'),value.get('revision')))
                    if self.path == '/local/missions':
                        return self.send_json(platform.catalog_command(value))
                    with platform.lock:
                        if self.path == f'/api/drones/{platform.drone_id}/control-action/ack/':
                            platform.acks.append(value)
                            platform.acks = platform.acks[-200:]
                            if value.get('request_id') == platform.assignment.get('control_request_id'):
                                platform.assignment['control_action'] = None
                                platform.store.acknowledge(platform.assignment)
                        elif self.path == f'/api/drones/{platform.drone_id}/companion-phase/':
                            platform.reports.append(value)
                            platform.reports = platform.reports[-200:]
                        else:
                            return self.send_json({'error':'unknown_endpoint'},404)
                    self.send_json({'ok':True})
                except (ValueError, TypeError) as error:
                    self.send_json({'error':str(error)[:256]},400)
                except OSError:
                    self.send_json({'error':'현장 설정 저장 실패'},500)
                except sqlite3.Error:
                    self.send_json({'error':'미션 기록 저장 실패·명령 전송 중단'},503)

            def log_message(self, *args):
                pass

        return Handler

    async def websocket(self, ws):
        if ws.request.path != f'/ws/drones/{self.drone_id}/':
            await ws.close(code=1008)
            return
        async for raw in ws:
            try:
                value = json.loads(raw)
                if not isinstance(value,dict) or value.get('type') != 'telemetry':
                    continue
                with self.lock:
                    if str(value.get('drone_id')) != self.drone_id:
                        continue
                    self.store.observe(value)
                    self.telemetry = value
                    self.telemetry_received_s = time.monotonic()
            except (ValueError, TypeError, sqlite3.Error):
                continue


async def run(args):
    settings = (Settings(**json.loads(Path(args.config).read_text(encoding='utf-8')))
                if args.config else Settings(drone_id=args.drone_id or '5'))
    if args.drone_id is not None and args.drone_id != settings.drone_id:
        raise ValueError('web_config_drone_id_mismatch')
    site_file = args.site_config or Path.home()/'.local/state/dronestock-flight'/f'site-{settings.drone_id}.json'
    mission_file = args.mission_store or Path(site_file).with_name(f'missions-{settings.drone_id}-{digest(settings.layout_id)[:8]}.sqlite3')
    platform = LocalPlatform(settings.drone_id, settings.layout_id, settings=settings, site_file=site_file, mission_file=mission_file)
    http = ThreadingHTTPServer((args.host,args.http_port), platform.http_handler())
    worker = threading.Thread(target=http.serve_forever,daemon=True)
    worker.start()
    stop = asyncio.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(signum,stop.set)
    try:
        async with serve(platform.websocket,args.host,args.ws_port):
            print(f'Flight page: http://{args.host}:{http.server_port}',flush=True)
            print(f'WS endpoint: ws://{args.host}:{args.ws_port}/ws/drones/{settings.drone_id}/',flush=True)
            await stop.wait()
    finally:
        await asyncio.to_thread(http.shutdown)
        http.server_close()
        platform.store.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--http-port',type=int,default=8001)
    parser.add_argument('--ws-port',type=int,default=8002)
    parser.add_argument('--drone-id',choices=('5','6'))
    parser.add_argument('--config',default='',help='Same mission configuration as the flight launcher')
    parser.add_argument('--site-config',default='',help='Private persistent site settings; separate from FC parameters')
    parser.add_argument('--mission-store',default='',help='Private durable mission drafts and execution receipts')
    asyncio.run(run(parser.parse_args()))
