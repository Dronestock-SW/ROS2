"""Local flight-command page and the Platform v1 HTTP/WS test endpoints."""

import argparse
import asyncio
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import threading
import time
import uuid

from websockets.asyncio.server import serve
from .contracts import LAYOUT, Settings


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
<label for="route">A1 기준 수평 경유지 · x/y 단위 m</label>
<textarea id="route">[{"id":"P1","type":"waypoint","x":2.3,"y":2.0}]</textarea>
<div><button id="start">이륙 · 경유지 이동 · 착륙</button>
<button id="home">출발점 복귀 · 착륙</button><button id="land">착륙 요청</button></div>
<p id="notice">연결 상태를 확인하고 있습니다.</p><pre id="state">상태 대기</pre>
<script>
const route=document.getElementById('route'),notice=document.getElementById('notice'),state=document.getElementById('state');
async function command(action){try{const body={action};if(action==='start')body.route_tasks=JSON.parse(route.value);
const r=await fetch('/local/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
const data=await r.json();notice.textContent=r.ok?'요청을 보냈습니다. 아래 수락·실행 상태를 확인하세요.':data.error;
}catch(e){notice.textContent='경유지 JSON과 서버 연결을 확인하세요.'}}
document.getElementById('start').onclick=()=>command('start');
document.getElementById('land').onclick=()=>command('land');
document.getElementById('home').onclick=()=>command('return_to_home');
async function update(){try{const r=await fetch('/local/status',{cache:'no-store'}),s=await r.json(),t=s.telemetry;
state.textContent=JSON.stringify({기체:s.assignment.drone_id,배치:s.assignment.anchor_layout_id,
텔레메트리수신:s.telemetry_fresh,상태:t.flight_state??'대기',사유:t.flight_reason??null,
기체연결:t.fc_connected??null,시동:t.fc_armed??null,PX4모드:t.fc_mode??null,
UWB_XYZ:[t.x??null,t.y??null,t.current_z_m??null],높이출처:t.current_z_source??null,
PX4_ENU:t.px4_position_enu_m??null,경유지:t.active_waypoint_id??null,
목표적용:t.target_applied??false,착륙확인:t.landing_verified??false,
출발점복귀확인:t.home_verified??false,비행결과:t.flight_outcome??null,
작업결과:t.work_outcome??null,스캔결과:t.scan_results??[],
시도한작업:t.attempted_task_ids??[],남은작업:t.remaining_task_ids??[],
스캔실패작업:t.failed_scan_task_ids??[],전체경로수행:t.route_complete??false,
임무완료:t.mission_complete??false,요청검사:t.target_validation??null},null,2);
if(!s.telemetry_fresh)notice.textContent='기체 텔레메트리 수신 대기 · 현재 동작 상태를 확인할 수 없습니다.';
}catch(e){notice.textContent='로컬 서버 응답 대기'}setTimeout(update,500)}update();
</script></html>'''.encode('utf-8')


class LocalPlatform:
    def __init__(self, drone_id='5', layout_id=LAYOUT):
        self.lock = threading.Lock()
        self.drone_id = drone_id
        self.mission_number = 0
        self.telemetry, self.reports, self.acks = {}, [], []
        self.telemetry_received_s = None
        self.assignment = dict(ok=True, contract_version='1.0', drone_id=drone_id,
            status='IDLE', control_action=None, coordinate_frame='UWB_ANCHOR_LOCAL',
            origin='A1', x_axis='A1_TO_A2', y_axis='A1_TO_A3', z_axis='UP_FROM_FLOOR', unit='meter',
            anchor_layout_id=layout_id, route_tasks=[])

    def command(self, action, route=None):
        if action not in ('start', 'land', 'return_to_home'):
            raise ValueError('지원하지 않는 요청입니다.')
        with self.lock:
            if action == 'start':
                if not isinstance(route, list) or not 1 <= len(route) <= 20:
                    raise ValueError('경유지 1~20개를 입력하세요.')
                self.mission_number += 1
                revision = hashlib.sha256(json.dumps(route, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]
                self.assignment.update(mission_db_id=self.mission_number,
                    mission_code='LOCAL-'+str(self.mission_number), route_revision=revision,
                    route_tasks=route, status='ACTIVE')
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
                with platform.lock:
                    if self.path == f'/api/drones/{platform.drone_id}/companion-mission/':
                        return self.send_json(platform.assignment)
                    if self.path == '/local/status':
                        age = (time.monotonic()-platform.telemetry_received_s
                               if platform.telemetry_received_s is not None else None)
                        fresh = age is not None and 0 <= age <= 1.
                        return self.send_json(dict(telemetry=platform.telemetry if fresh else {},
                            telemetry_fresh=fresh, telemetry_age_ms=round(age*1000) if age is not None else None,
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
                        return self.send_json(platform.command(value.get('action'),value.get('route_tasks')))
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
                except (ValueError, TypeError):
                    self.send_json({'error':'요청 형식과 경유지 값을 확인하세요.'},400)

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
    platform = LocalPlatform(settings.drone_id, settings.layout_id)
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
    asyncio.run(run(parser.parse_args()))
