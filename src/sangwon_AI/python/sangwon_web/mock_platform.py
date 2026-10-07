"""Loopback-only draft.4 contract peer. No production credentials or aircraft."""
import argparse
import datetime as dt
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import signal
import sqlite3
import threading
import uuid
from . import CONTRACT_VERSION
from .common import MAX_BYTES, decode, encode, utc_now

class Platform:
    def __init__(self, snapshot, state):
        self.snapshot_bytes = Path(snapshot).read_bytes()
        self.snapshot = decode(self.snapshot_bytes)
        if self.snapshot["profile"] != "REPLAY" or not self.snapshot["drone_id"].startswith("TEST-"):
            raise ValueError("MOCK_REPLAY_ONLY")
        self.drone = self.snapshot["drone_id"]
        self.lock = threading.RLock()
        state = Path(state)
        state.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.db = sqlite3.connect(state / "mock.sqlite3", check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.executescript("CREATE TABLE IF NOT EXISTS inbox(key TEXT PRIMARY KEY,sha TEXT,raw BLOB);"
                             "CREATE TABLE IF NOT EXISTS commands(id TEXT PRIMARY KEY,client_key TEXT UNIQUE,raw BLOB);")
        self.latest = {}
        self.session = None
        self.sequence = 0
        self.closed = False

    def mission(self):
        known = set()
        for raw, in self.db.execute("SELECT raw FROM inbox"):
            value = decode(raw)
            if value.get("type") == "command_result":
                known.add(value["control_request_id"])
        requests = [decode(raw) for raw, in self.db.execute("SELECT raw FROM commands ORDER BY rowid")]
        requests = [r for r in requests if r["context"]["control_session_id"] == self.session and r["control_request_id"] not in known]
        return dict(contract_version=CONTRACT_VERSION, type="companion_mission", profile="REPLAY", drone_id=self.drone,
                    server_time=utc_now(), control_session_id=self.session,
                    assignment=None if self.closed else dict(assignment_id="assignment-replay-01", revision=1, state="QUEUED", snapshot_id=self.snapshot["snapshot_id"]),
                    snapshot_ref=dict(snapshot_id=self.snapshot["snapshot_id"], contract_version=CONTRACT_VERSION,
                        content_url="/api/mission-snapshots/"+self.snapshot["snapshot_id"]+"/content/",
                        sha256=hashlib.sha256(self.snapshot_bytes).hexdigest(), byte_length=len(self.snapshot_bytes)),
                    control_requests=requests, operator_attestations=[], recheck_requests=[])

    def state(self):
        return {"mode": "MOCK_REPLAY_ONLY", "physical_output_enabled": False, "latest": self.latest,
                "closed": self.closed, "command_count": self.db.execute("SELECT COUNT(*) FROM commands").fetchone()[0],
                "results": [decode(raw) for raw, in self.db.execute("SELECT raw FROM inbox")],
                "control_session_id": self.session}

    def submit(self, value):
        client_key = value["client_request_key"]
        prior = self.db.execute("SELECT raw FROM commands WHERE client_key=?", (client_key,)).fetchone()
        if prior:
            command = decode(prior[0])
            if command["action"] != value["action"]:
                raise ValueError("REQUEST_ID_CONFLICT")
            return command
        action = value["action"]
        if action not in ("START", "PAUSE", "RESUME", "CANCEL", "LAND_NOW"):
            raise ValueError("UNKNOWN_COMMAND")
        source = self.latest["readiness"] if action == "START" else self.latest["telemetry"]
        if action not in source["allowed_commands"]:
            raise ValueError("INVALID_PHASE")
        now = dt.datetime.now(dt.timezone.utc)
        self.sequence += 1
        command = dict(contract_version=CONTRACT_VERSION, type="control_request", profile="REPLAY", drone_id=self.drone,
                       control_request_id="request-"+uuid.uuid4().hex, command_seq=self.sequence, action=action,
                       created_at=now.isoformat().replace("+00:00","Z"),
                       expires_at=(now+dt.timedelta(seconds=10)).isoformat().replace("+00:00","Z"),
                       context=source["context"], payload={"start_mode":"AUTO_TAKEOFF"} if action=="START" else {})
        self.db.execute("INSERT INTO commands VALUES(?,?,?)", (command["control_request_id"],client_key,encode(command)))
        self.db.commit()
        return command

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--state-dir", required=True)
    parser.add_argument("--http-port", type=int, default=8878)
    parser.add_argument("--ws-port", type=int, default=8879)
    args = parser.parse_args()
    platform = Platform(args.snapshot, args.state_dir)
    prefix = "/api/drones/"+platform.drone+"/"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def respond(self, status, value, raw=False):
            body = value if raw else encode(value)
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def do_GET(self):
            with platform.lock:
                if self.path == prefix+"companion-mission/": self.respond(200,platform.mission())
                elif self.path == platform.mission()["snapshot_ref"]["content_url"]: self.respond(200,platform.snapshot_bytes,True)
                elif self.path == "/debug/state": self.respond(200,platform.state())
                else: self.respond(404,{"code":"NOT_FOUND"})
        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length","0"))
                if not 0 <= length <= MAX_BYTES: raise ValueError("MESSAGE_TOO_LARGE")
                raw = self.rfile.read(length)
                value = decode(raw)
                with platform.lock:
                    if self.path == "/debug/command":
                        self.respond(201,platform.submit(value)); return
                    if self.path == prefix+"control-sessions/":
                        if value.get("profile") != "REPLAY" or value.get("contract_version") != CONTRACT_VERSION or value.get("drone_id") != platform.drone:
                            raise ValueError("UNSUPPORTED_CONTRACT")
                        fields={"contract_version","type","drone_id","profile","boot_id","runtime_session_id","supported_contract_versions","capabilities"}
                        if set(value)!=fields or CONTRACT_VERSION not in value['supported_contract_versions']:
                            raise ValueError("INVALID_REQUEST")
                        if not {"command_session","readiness_binding"}.issubset(value['capabilities']):
                            raise ValueError("UNSUPPORTED_CAPABILITY")
                        platform.session = "session-"+uuid.uuid4().hex
                        platform.sequence = 0
                        self.respond(201,dict(contract_version=CONTRACT_VERSION,type="control_session",profile="REPLAY",
                            drone_id=platform.drone,boot_id=value["boot_id"],runtime_session_id=value["runtime_session_id"],
                            control_session_id=platform.session,negotiated_contract=CONTRACT_VERSION,
                            supported_capabilities=value['capabilities'],server_time=utc_now(),flight_authority=False,allowed_execution="REPLAY_ONLY")); return
                    if self.path not in {prefix+r+"/" for r in ("control-action/ack","companion-phase","preparation-reports","launch-snapshot","scan-task-results")}:
                        self.respond(404,{"code":"NOT_FOUND"}); return
                    key = self.headers.get("Idempotency-Key")
                    if not key: raise ValueError("IDEMPOTENCY_KEY_REQUIRED")
                    sha = hashlib.sha256(raw).hexdigest()
                    old = platform.db.execute("SELECT sha FROM inbox WHERE key=?",(key,)).fetchone()
                    if old and old[0] != sha: raise ValueError("RESULT_CONFLICT")
                    platform.db.execute("INSERT OR IGNORE INTO inbox VALUES(?,?,?)",(key,sha,raw)); platform.db.commit()
                    if value.get("event_type") == "EXECUTION_ENDED": platform.closed = True
                    self.respond(200,{"stored":True,"key":key,"sha256":sha,"duplicate":bool(old)})
            except (ValueError,KeyError,TypeError) as exc:
                self.respond(409,{"code":str(exc)})

    from websockets.sync.server import serve
    def receive(connection):
        try:
            for raw in connection:
                message = decode(raw)
                if message.get("drone_id") != platform.drone or message.get("profile") != "REPLAY": continue
                if message.get("flight_authority") is not False: continue
                if message.get("type") in ("telemetry","readiness"):
                    with platform.lock:
                        platform.latest[message["type"]] = message
                        if message["type"] == "readiness":
                            context=message.get("context",{})
                            accepted=(context.get("control_session_id")==platform.session and
                                      message.get("check_catalog_revision")=="BP-28-QS-05-draft4" and
                                      len(message.get("checks",[]))==33)
                            connection.send(encode(dict(type="readiness.ack",accepted=accepted,
                                code="OK" if accepted else "INVALID_READINESS",contract_version=CONTRACT_VERSION,
                                drone_id=platform.drone,context=context,readiness_seq=message.get("readiness_seq"))).decode())
                        else:
                            context=message.get("context",{})
                            connection.send(encode(dict(type="telemetry.ack",accepted=context.get("control_session_id")==platform.session,
                                contract_version=CONTRACT_VERSION,drone_id=platform.drone,
                                context=context,telemetry_seq=message.get("telemetry_seq"))).decode())
        except Exception: pass
    websocket = serve(receive,"127.0.0.1",args.ws_port,max_size=MAX_BYTES)
    threading.Thread(target=websocket.serve_forever,daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1",args.http_port),Handler)
    def stop(*_):
        threading.Thread(target=server.shutdown,daemon=True).start()
        websocket.shutdown()
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    print(f"MOCK_REPLAY_ONLY HTTP=127.0.0.1:{args.http_port} WS=127.0.0.1:{args.ws_port}",flush=True)
    server.serve_forever()
    server.server_close()

if __name__ == "__main__": main()
