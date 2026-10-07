"""Local read-only viewer of Jetson monitoring results. No API/flight writes."""
import argparse
import datetime as dt
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
REMOTE="/home/arialhanho/Desktop/ROS2/src/sangwon_AI/.runtime/web_monitor/latest.json"
cache={"transport_ok":False,"report":None,"code":"LOADING"}
lock=threading.Lock()
last_fetch=0.0

def git_status():
    try:
        data=json.loads((ROOT/'.runtime/web_monitor/git_latest.json').read_text(encoding='utf-8'))
        return data if data.get('remote_read_only') is True else None
    except (OSError,ValueError): return None

def status():
    global cache,last_fetch
    with lock:
        if time.monotonic()-last_fetch<5:return dict(cache,git=git_status())
        try:
            result=subprocess.run(["ssh","-o","BatchMode=yes","-o","ConnectTimeout=6",
                "-o","StrictHostKeyChecking=yes","100.110.163.94","cat "+REMOTE],
                stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,timeout=9,check=True,
                creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
            if len(result.stdout)>2*1024*1024:raise ValueError("REPORT_TOO_LARGE")
            report=json.loads(result.stdout)
            if not isinstance(report,dict) or report.get("read_only") is not True:raise ValueError("INVALID_REPORT")
            cache={"transport_ok":True,"report":report,"code":"OK",
                   "received_at":dt.datetime.now(dt.timezone.utc).isoformat()}
        except (OSError,ValueError,subprocess.SubprocessError) as exc:
            cache=dict(cache,transport_ok=False,code=type(exc).__name__)
        last_fetch=time.monotonic()
        return dict(cache,git=git_status())

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--port",type=int,default=8877)
    args=parser.parse_args()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def do_GET(self):
            if self.headers.get("Host") not in (f"127.0.0.1:{args.port}",f"localhost:{args.port}"):
                self.send_error(403);return
            path=self.path.split("?",1)[0]
            if path=="/":body=(ROOT/"ops/web_monitor.html").read_bytes();mime="text/html; charset=utf-8"
            elif path=="/api/status":body=json.dumps(status(),ensure_ascii=False,allow_nan=False).encode();mime="application/json; charset=utf-8"
            else:self.send_error(404);return
            self.send_response(200)
            self.send_header("Content-Type",mime);self.send_header("Content-Length",str(len(body)))
            self.send_header("Cache-Control","no-store");self.send_header("X-Content-Type-Options","nosniff")
            self.send_header("Content-Security-Policy","default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
    server=ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
    print(f"Read-only monitor: http://127.0.0.1:{args.port}",flush=True)
    server.serve_forever()

if __name__=="__main__":main()
