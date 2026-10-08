"""Bounded HTTP/WS transport. Redirects and cross-origin snapshots are refused."""
import hashlib
import hmac
import os
from pathlib import Path
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from .common import MAX_BYTES, encode, decode
from . import CONTRACT_VERSION

class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError("HTTP_REDIRECT_REFUSED")

class Transport:
    def __init__(self, config):
        self.config = config
        self.base = config["base_url"].rstrip("/")
        self.origin = urllib.parse.urlsplit(self.base)
        if self.origin.scheme not in ("http", "https") or self.origin.username or self.origin.password or self.origin.path:
            raise ValueError("INVALID_SERVER_ORIGIN")
        self.opener = urllib.request.build_opener(NoRedirect())
        self.timeout = float(config.get("timeout_s", 3))
        self.writable = config["mode"] == "contract"
        self.auth = config.get("auth", {"mode": "none"})
        if self.writable and self.auth["mode"] == "none" and self.origin.hostname not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError("DEVICE_AUTH_REGISTRATION_REQUIRED")
        self.ws = None
        self.control_session = None
        self.last_readiness_ack = None
        self.last_telemetry_ack = None

    def set_session(self, session):
        if session is not None and (not isinstance(session, str) or not session or len(session)>256 or
                                    any(ord(c)<33 or ord(c)>126 for c in session)):
            raise ValueError("INVALID_CONTROL_SESSION")
        if self.control_session != session:
            self.close()
        self.control_session = session

    def request_session(self, path, value):
        """W01 uses 5 lines; subsequent session-bound requests use 7 lines."""
        if not path.endswith('/control-sessions/') or value.get('type')!='control_session_request':
            raise ValueError("INVALID_SESSION_REQUEST")
        return decode(self.request_bytes("POST", path, value, session_request=True, expected_status=201))

    def headers(self, method, path, raw, session_request=False):
        headers = {"Accept": "application/json", "Accept-Encoding": "identity", "Content-Type": "application/json"}
        mode = self.auth["mode"]
        if mode == "hmac_session_v2" and session_request:
            mode = "hmac_v1"  # WEB_SERVER_W01_W02_AUTH_SESSION.md: W01 is always five lines.
        if mode in ("hmac_v1", "hmac_session_v2"):
            # Filled only after the web team registers this Jetson identity/key.
            device = os.environ.get("DRONESTOCK_DEVICE_AUTH_ID", "")
            secret = os.environ.get("DRONESTOCK_DEVICE_AUTH_SECRET", "")
            if not device or not secret:
                raise ValueError("DEVICE_AUTH_REGISTRATION_REQUIRED")
            stamp, nonce = str(int(time.time())), uuid.uuid4().hex
            parts = [method, path, hashlib.sha256(raw).hexdigest(), stamp, nonce]
            if mode != "hmac_v1":
                session = self.control_session
                if session is None:
                    raise ValueError("CONTROL_SESSION_REQUIRED")
                parts.extend((CONTRACT_VERSION, session))
                headers.update({"X-DS-Contract-Version": CONTRACT_VERSION, "X-DS-Control-Session": session})
            message = "\n".join(parts).encode()
            headers.update({"X-DS-Device-ID": device, "X-DS-Timestamp": stamp, "X-DS-Nonce": nonce,
                            "X-DS-Signature": hmac.new(secret.encode(), message, hashlib.sha256).hexdigest()})
        elif mode != "none":
            raise ValueError("UNSUPPORTED_AUTH_MODE")
        return headers

    def request_bytes(self, method, path, value=None, raw_body=None, key=None, session_request=False, expected_status=None):
        if not path.startswith("/") or path.startswith("//") or "\\" in path or urllib.parse.urlsplit(path).netloc:
            raise ValueError("INVALID_API_PATH")
        if method != "GET" and not self.writable:
            raise ValueError("READ_ONLY_TRANSPORT")
        raw = raw_body if raw_body is not None else (encode(value) if value is not None else b"")
        if len(raw) > MAX_BYTES:
            raise ValueError("MESSAGE_TOO_LARGE")
        headers = self.headers(method, path, raw, session_request=session_request)
        if key:
            headers["Idempotency-Key"] = key
        request = urllib.request.Request(self.base + path, data=raw if method != "GET" else None,
                                         method=method, headers=headers)
        with self.opener.open(request, timeout=self.timeout) as response:
            if expected_status is not None and response.status != expected_status:
                raise ValueError("UNEXPECTED_HTTP_STATUS")
            if "application/json" not in response.headers.get("Content-Type", ""):
                raise ValueError("NON_JSON_RESPONSE")
            body = response.read(MAX_BYTES + 1)
            if len(body) > MAX_BYTES:
                raise ValueError("MESSAGE_TOO_LARGE")
            return body

    def request(self, method, path, value=None, **kwargs):
        return decode(self.request_bytes(method, path, value, **kwargs))

    def plan(self, ref, drone_id):
        if (ref.get('content_url') != f'/api/drones/{drone_id}/mission-plan/' or
            type(ref.get('byte_length')) is not int or not 0 < ref['byte_length'] <= MAX_BYTES):
            raise ValueError('PLAN_LOCATION_REFUSED')
        body = self.request_bytes('GET', ref['content_url'])
        if len(body) != ref['byte_length'] or hashlib.sha256(body).hexdigest() != ref.get('sha256'):
            raise ValueError('PLAN_CONFLICT')
        decode(body)
        return body.decode('utf-8')

    def snapshot(self, ref):
        path = ref["content_url"]
        if not path.startswith("/api/mission-snapshots/") or not path.endswith("/content/"):
            raise ValueError("SNAPSHOT_LOCATION_REFUSED")
        body = self.request_bytes("GET", path)
        if len(body) != ref["byte_length"] or hashlib.sha256(body).hexdigest() != ref["sha256"]:
            raise ValueError("SNAPSHOT_CONFLICT")
        decode(body)
        return body.decode("utf-8")

    def publish(self, messages):
        if not self.writable:
            raise ValueError("READ_ONLY_TRANSPORT")
        from websockets.sync.client import connect
        url = self.config["websocket_url"]
        target = urllib.parse.urlsplit(url)
        if target.scheme not in ("ws", "wss") or target.hostname != self.origin.hostname or target.username or target.password:
            raise ValueError("WEBSOCKET_ORIGIN_REFUSED")
        if self.origin.scheme == "https" and target.scheme != "wss":
            raise ValueError("WEBSOCKET_TLS_DOWNGRADE")
        if self.ws is None:
            path = target.path + ("?" + target.query if target.query else "")
            self.ws = connect(url, additional_headers=self.headers("GET", path, b""),
                              open_timeout=self.timeout, close_timeout=1, max_size=MAX_BYTES)
        for message in messages:
            self.ws.send(encode(message).decode("utf-8"))
            kind = message.get("type")
            if kind in ("readiness", "telemetry"):
                try:
                    ack = decode(self.ws.recv(timeout=self.timeout))
                    self.validate_report_ack(message, ack, kind)
                    receipt = {"accepted":True, kind+"_seq":message[kind+"_seq"]}
                    if kind=="readiness": self.last_readiness_ack = receipt
                    else: self.last_telemetry_ack = receipt
                except Exception:
                    self.close()  # A late ACK cannot acknowledge the next report.
                    raise

    @staticmethod
    def validate_readiness_ack(message, ack):
        Transport.validate_report_ack(message, ack, "readiness")

    @staticmethod
    def validate_report_ack(message, ack, kind):
        if kind not in ("readiness", "telemetry"):
            raise ValueError("UNSUPPORTED_REPORT_ACK")
        prefix=kind.upper()
        if not isinstance(ack, dict) or ack.get("type")!=kind+".ack":
            raise ValueError(prefix+"_ACK_REQUIRED")
        if ack.get("accepted") is not True:
            code=ack.get("code", "UNSPECIFIED")
            # Only bounded machine codes enter logs; never remote free text.
            if not isinstance(code,str) or len(code)>80 or any(c not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for c in code):
                code="UNSPECIFIED"
            diagnostic=ack.get('diagnostic_code')
            if (isinstance(diagnostic,str) and 1<=len(diagnostic)<=80
                    and all(c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_' for c in diagnostic)):
                code+=':'+diagnostic
            raise ValueError(prefix+"_REJECTED:"+code)
        expected=message.get("context", {})
        received=ack.get("context", {})
        if not isinstance(received,dict):
            raise ValueError(prefix+"_ACK_CONTEXT_REQUIRED")
        for key in ("boot_id", "runtime_session_id", "control_session_id"):
            if not expected.get(key) or received.get(key)!=expected[key]:
                raise ValueError(prefix+"_ACK_CONTEXT_MISMATCH")
        sequence=kind+"_seq"
        if (ack.get("drone_id")!=message.get("drone_id") or ack.get("contract_version")!=CONTRACT_VERSION or
            type(ack.get(sequence)) is not int or ack[sequence]!=message.get(sequence)):
            raise ValueError(prefix+"_ACK_MISMATCH")

    def close(self):
        self.last_readiness_ack = None
        self.last_telemetry_ack = None
        if self.ws:
            self.ws.close()
            self.ws = None
