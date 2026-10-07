"""Loopback-only bench plan preview. No ROS, serial, IPC or flight commands."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_web.bench_plan import audit_view, make_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8347)
    parser.add_argument('--audit-record', type=Path)
    args = parser.parse_args()
    record = json.loads(args.audit_record.read_text(encoding='utf-8')) if args.audit_record else None
    status = audit_view(record)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = urlsplit(self.path)
            code, kind = 200, 'application/json; charset=utf-8'
            try:
                if route.path == '/':
                    body = (ROOT / 'ops/bench_preview.html').read_bytes()
                    kind = 'text/html; charset=utf-8'
                elif route.path == '/api/status':
                    body = json.dumps(status, ensure_ascii=False).encode('utf-8')
                elif route.path == '/api/plan':
                    q = parse_qs(route.query)
                    plan = make_plan(q['case'][0], float(q['z'][0]))
                    body = json.dumps(plan, ensure_ascii=False, indent=2).encode('utf-8')
                else:
                    code, body = 404, b'{}'
            except (KeyError, ValueError, OverflowError):
                code, body = 400, b'{"error":"invalid_plan_request"}'
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            self.send_error(405, 'Flight commands are not available')

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print(f'http://127.0.0.1:{server.server_port} - REPLAY plans only', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
