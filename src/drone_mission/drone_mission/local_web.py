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
import threading
import time
import uuid
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import serve
from .contracts import LAYOUT, Settings, finite
from .field_presets import field_route
from .site import ceiling_value


PAGE = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dronestock 비행 시험</title>
<style>body{max-width:850px;margin:40px auto;padding:0 20px;font:16px sans-serif;
background:#f6f7f9;color:#182332}textarea{box-sizing:border-box;width:100%;min-height:90px;
font:15px monospace;padding:12px}button{padding:12px 18px;margin:10px 8px 10px 0;
border:0;border-radius:6px;background:#164abd;color:white;cursor:pointer}
#land{background:#b92b26}pre{padding:18px;background:white;white-space:pre-wrap;border-radius:8px}</style>
<h1>Dronestock 비행 시험</h1>
<p>PX4 설정 고도로 이륙합니다. 전체 미션 설정에서는 안정화·이동·스캔·출발점 복귀 후 착륙합니다.</p>
<p>현장 천장 높이(바닥 기준 m): <input id="ceiling" type="number" min="0.5" max="100" step="0.1" placeholder="예: 3.0">
<button id="setceiling">천장 설정 저장</button><span id="site"></span></p>
<p>명령: <code>{"action":"set_ceiling","ceiling_height_m":3.0}</code><br>
천장은 지도 상한을 제한합니다. 이륙 높이·센서 실측 확인은 별도이며, START 후에는 변경할 수 없습니다.</p>
<label for="route">A1 기준 수평 경유지 · x/y 단위 m</label>
<p>출발점은 검증된 PX4 기체 위치로 자동 확인합니다. START 시 다시 읽고 복귀점으로 고정합니다.</p>
<select id="preset"><option value="hover">이륙·2초 호버·착륙</option><option value="x">X 1m 왕복</option><option value="y">Y 1m 왕복</option><option value="xy">X/Y 각 1m·역순 복귀</option></select>
<button id="make">시험 경로 만들기</button>
<textarea id="route">[]</textarea>
<div><button id="start" disabled>이륙 · 경유지 이동 · 착륙</button>
<button id="home">출발점 복귀 · 착륙</button><button id="land">착륙 요청</button></div>
<p id="notice">연결 상태를 확인하고 있습니다.</p><div id="checks"></div><pre id="state">상태 대기</pre>
<script>
const route=document.getElementById('route'),notice=document.getElementById('notice'),state=document.getElementById('state');
let autoCase=null;route.oninput=()=>{autoCase=null};
document.getElementById('setceiling').onclick=async()=>{try{
const r=await fetch('/local/command',{method:'POST',headers:{'Content-Type':'application/json'},
body:JSON.stringify({action:'set_ceiling',ceiling_height_m:Number(document.getElementById('ceiling').value)})});
const d=await r.json();notice.textContent=r.ok?'천장 설정 저장 완료 · 경로와 실측 지도는 START 때 함께 검사합니다.':d.error;
}catch(e){notice.textContent='천장 설정 저장 실패'}};
async function command(action){try{const body={action};if(action==='start'){if(autoCase)body.trial_case=autoCase;else body.route_tasks=JSON.parse(route.value);}
const r=await fetch('/local/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
const data=await r.json();notice.textContent=r.ok?'요청을 보냈습니다. 아래 수락·실행 상태를 확인하세요.':data.error;
}catch(e){notice.textContent='경유지 JSON과 서버 연결을 확인하세요.'}}
document.getElementById('start').onclick=()=>command('start');
document.getElementById('land').onclick=()=>command('land');
document.getElementById('home').onclick=()=>command('return_to_home');
document.getElementById('make').onclick=async()=>{try{const choice=document.getElementById('preset').value;
const q=new URLSearchParams({case:choice});const r=await fetch('/local/preset?'+q),d=await r.json();
if(!r.ok)throw Error(d.error);route.value=JSON.stringify(d.route_tasks,null,2);autoCase=choice;
notice.textContent='PX4 출발점 '+JSON.stringify(d.start_xy_m)+'에서 경로 생성 · START 시 재확인합니다.';}catch(e){notice.textContent=e.message;}};
async function update(){try{const r=await fetch('/local/status',{cache:'no-store'}),s=await r.json(),t=s.telemetry;
document.getElementById('start').disabled=!s.telemetry_fresh||(s.ceiling_required&&s.site.ceiling_height_m==null)||(t.preflight&&t.preflight.checked_inputs_passed!==true)||!['IDLE','READY'].includes(t.flight_state);
document.getElementById('setceiling').disabled=!s.site_editable;
document.getElementById('site').textContent=s.site.ceiling_height_m==null?'미설정':('저장값 '+s.site.ceiling_height_m+' m · 이륙 '+s.expected_takeoff_height_m+' m');
const checks=document.getElementById('checks');checks.replaceChildren();
for(const c of t.preflight?.checks??[]){const row=document.createElement('div');row.textContent=(c.passed?'✓ ':'대기 · ')+c.title;checks.appendChild(row);}
state.textContent=JSON.stringify({기체:s.assignment.drone_id,배치:s.assignment.anchor_layout_id,
텔레메트리수신:s.telemetry_fresh,상태:t.flight_state??'대기',사유:t.flight_reason??null,
기체연결:t.fc_connected??null,시동:t.fc_armed??null,PX4모드:t.fc_mode??null,
UWB_XYZ:[t.x??null,t.y??null,t.current_z_m??null],높이출처:t.current_z_source??null,
PX4_ENU:t.px4_position_enu_m??null,경유지:t.active_waypoint_id??null,
자동출발점_창고XY:s.automatic_start_xy_m??null,
고정복귀점_창고XY:t.home_xy_m??null,
목표적용:t.target_applied??false,착륙확인:t.landing_verified??false,
출발점복귀확인:t.home_verified??false,비행결과:t.flight_outcome??null,
작업결과:t.work_outcome??null,스캔결과:t.scan_results??[],
시도한작업:t.attempted_task_ids??[],남은작업:t.remaining_task_ids??[],
스캔실패작업:t.failed_scan_task_ids??[],전체경로수행:t.route_complete??false,
임무완료:t.mission_complete??false,요청검사:t.target_validation??null},null,2);
if(!s.telemetry_fresh)notice.textContent='기체 텔레메트리 수신 대기 · 현재 동작 상태를 확인할 수 없습니다.';
}catch(e){document.getElementById('start').disabled=true;notice.textContent='로컬 서버 응답 대기'}setTimeout(update,500)}update();
</script></html>'''.encode('utf-8')


class LocalPlatform:
    def __init__(self, drone_id='5', layout_id=LAYOUT, settings=None, site_file=None):
        self.lock = threading.Lock()
        self.drone_id = drone_id
        self.settings = settings or Settings(drone_id=drone_id, layout_id=layout_id)
        self.mission_number = 0
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

    def site_editable(self):
        # A new ground session is required after every START. Expired telemetry
        # cannot unlock a mission, and a web restart cannot override an armed FC.
        if self.assignment.get('mission_db_id') is not None:
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

    def command(self, action, route=None, ceiling=None, trial_case=None):
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
            if action == 'start':
                start_xy = None
                if trial_case is not None:
                    start_xy = self.automatic_start()
                    route = field_route(trial_case, start_xy, self.settings)
                if not isinstance(route, list) or not 1 <= len(route) <= 20:
                    raise ValueError('경유지 1~20개를 입력하세요.')
                if self.settings.full_mission and self.site['ceiling_height_m'] is None:
                    raise ValueError('현장 천장 높이를 먼저 저장하세요.')
                self.mission_number += 1
                revision = hashlib.sha256(json.dumps(route, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]
                self.assignment.update(mission_db_id=self.mission_number,
                    mission_code='LOCAL-'+str(self.mission_number), route_revision=revision,
                    route_tasks=route, status='ACTIVE')
                if self.site['ceiling_height_m'] is not None:
                    self.assignment['ceiling_height_m'] = self.site['ceiling_height_m']
                if start_xy is not None:
                    self.assignment['planned_launch_xy_m'] = list(start_xy)
                else:
                    self.assignment.pop('planned_launch_xy_m',None)
            now = datetime.now(timezone.utc).isoformat()
            self.assignment.update(control_action=action, control_request_id=uuid.uuid4().hex,
                                   control_requested_at=now, generated_at=now)
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
                        return self.send_json(platform.command(value.get('action'),value.get('route_tasks'),value.get('ceiling_height_m'),value.get('trial_case')))
                    with platform.lock:
                        if self.path == f'/api/drones/{platform.drone_id}/control-action/ack/':
                            platform.acks.append(value)
                            platform.acks = platform.acks[-200:]
                            if value.get('request_id') == platform.assignment.get('control_request_id'):
                                platform.assignment['control_action'] = None
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
                    self.telemetry = value
                    self.telemetry_received_s = time.monotonic()
            except (ValueError, TypeError):
                continue


async def run(args):
    settings = (Settings(**json.loads(Path(args.config).read_text(encoding='utf-8')))
                if args.config else Settings(drone_id=args.drone_id or '5'))
    if args.drone_id is not None and args.drone_id != settings.drone_id:
        raise ValueError('web_config_drone_id_mismatch')
    site_file = args.site_config or Path.home()/'.local/state/dronestock-flight'/f'site-{settings.drone_id}.json'
    platform = LocalPlatform(settings.drone_id, settings.layout_id, settings=settings, site_file=site_file)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--http-port',type=int,default=8001)
    parser.add_argument('--ws-port',type=int,default=8002)
    parser.add_argument('--drone-id',choices=('5','6'))
    parser.add_argument('--config',default='',help='Same mission configuration as the flight launcher')
    parser.add_argument('--site-config',default='',help='Private persistent site settings; separate from FC parameters')
    asyncio.run(run(parser.parse_args()))
