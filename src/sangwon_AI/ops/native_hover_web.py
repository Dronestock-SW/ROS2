"""Loopback HTTP to private C++ IPC. No ROS or mission decisions in Python."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import socket
from urllib.parse import urlsplit


def exchange(path, request):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(2)
        client.connect(str(path))
        client.sendall(json.dumps(request, allow_nan=False).encode('utf-8'))
        return json.loads(client.recv(16384))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8350)
    args = parser.parse_args()
    token = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, body, kind='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(code)
            for key, value in {'Content-Type': kind, 'Content-Length': str(len(body)),
                               'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff',
                               'X-Frame-Options': 'DENY'}.items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def local_host(self):
            return urlsplit('http://' + self.headers.get('Host', '')).hostname in ('127.0.0.1', 'localhost')

        def do_GET(self):
            if not self.local_host():
                return self.reply(403, {'error': 'loopback_host_required'})
            if self.path == '/':
                return self.reply(200, Path(__file__).with_suffix('.html').read_bytes(), 'text/html; charset=utf-8')
            if self.path != '/api/status':
                return self.reply(404, {})
            try:
                result = exchange(args.socket, {'method': 'status'})
                result['csrf'] = token
                self.reply(200, result)
            except (OSError, ValueError):
                self.reply(503, {'error': 'controller_unavailable'})

        def do_POST(self):
            if (not self.local_host() or self.headers.get('Origin') != 'http://' + self.headers.get('Host', '')
                    or not secrets.compare_digest(self.headers.get('X-Bench-Token', ''), token)):
                return self.reply(403, {'error': 'same_origin_token_required'})
            if self.path not in ('/api/start', '/api/land'):
                return self.reply(404, {})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 512:
                    raise ValueError('invalid_length')
                self.connection.settimeout(2)
                data = json.loads(self.rfile.read(size))
                if set(data) != {'session', 'confirm'} or not isinstance(data['session'], str):
                    raise ValueError('invalid_request')
                result = exchange(args.socket, {'method': self.path.rsplit('/', 1)[-1], **data})
                self.reply(200 if result.get('ok') else 409, result)
            except (OSError, ValueError) as error:
                self.reply(503, {'error': str(error)})

        def log_message(self, *_):
            pass

    with ThreadingHTTPServer(('127.0.0.1', args.port), Handler) as server:
        print(f'http://127.0.0.1:{args.port} native hover transport', flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
