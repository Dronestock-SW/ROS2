"""Supervised web adapter. Receives commands but never decides flight readiness."""
import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import time
import urllib.error
from . import CONTRACT_VERSION
from .common import atomic_json, decode, runtime_path, utc_now
from .ipc import CoreClient, CoreError
from .transport import Transport

ROUTES = {"control-action/ack", "companion-phase", "preparation-reports", "launch-snapshot", "scan-task-results", "mission-plan-receipts"}

class Adapter:
    def __init__(self, config, root):
        self.config, self.root = config, root
        self.state_dir = runtime_path(root, config["state_dir"])
        self.core = CoreClient(runtime_path(root, config["core_socket"]))
        self.transport = Transport(config)
        self.session = None
        self.runtime = None
        self.prepared = None
        self.planned = None
        self.preparation_error = None
        self.observed_contract = None
        self.poll_count = 0
        self.running = True
        self.stage = config.get("integration_stage", "full")
        if self.stage not in ("receive_only", "full", "planner_only"):
            raise ValueError("INVALID_INTEGRATION_STAGE")
        self.prefix = "/api/drones/" + config["drone_id"] + "/"

    def link(self, connected, code):
        self.core.call("link.update", {"connected": connected, "code": code,
                                      "observed_contract": self.observed_contract or ""})

    def connect_session(self, status):
        context = status["context"]
        request = {"contract_version": CONTRACT_VERSION, "type": "control_session_request",
                   "drone_id": self.config["drone_id"], "profile": status["profile"],
                   "boot_id": context["boot_id"], "runtime_session_id": context["runtime_session_id"],
                   "supported_contract_versions": [CONTRACT_VERSION],
                   "capabilities": status["supported_capabilities"]}
        self.transport.set_session(None)
        response = self.transport.request_session(self.prefix + "control-sessions/", request)
        negotiated=response.get("supported_capabilities")
        if (response.get("type")!="control_session" or response.get("negotiated_contract")!=CONTRACT_VERSION or
            response.get("profile") != status["profile"] or
            response.get("flight_authority") is not False or
            response.get("allowed_execution") != ("REPLAY_ONLY" if status["profile"] == "REPLAY" else "NONE") or
            not isinstance(negotiated,list) or any(not isinstance(c,str) for c in negotiated) or
            not {"command_session","readiness_binding"}.issubset(negotiated) or
            not set(negotiated).issubset(request["capabilities"])):
            raise ValueError("INVALID_SESSION_NEGOTIATION")
        self.core.call("session.set", response)
        self.session = response["control_session_id"]
        self.transport.set_session(self.session)
        self.runtime = context["runtime_session_id"]
        self.prepared = None
        self.planned = None

    def poll(self):
        health_path = runtime_path(self.root, self.config.get("host_health_path", ".runtime/host_health.json"))
        try:
            self.core.call("host.update", decode(health_path.read_bytes()))
        except (OSError, ValueError, CoreError):
            # Absence/staleness stays UNKNOWN in C++; never replace it with a new timestamp.
            pass
        status = self.core.call("status")
        if status["drone_id"] != self.config["drone_id"]:
            raise ValueError("CORE_DRONE_MISMATCH")
        if self.config["mode"] == "observe":
            data = self.transport.request("GET", self.prefix + "companion-mission/")
            self.observed_contract = data.get("contract_version")
            code = "OBSERVE_ONLY" if self.observed_contract == CONTRACT_VERSION else "WEB_CONTRACT_MISMATCH"
            self.link(False, code)
            self.poll_count += 1
            self.write_status("CONNECTED_READ_ONLY", code)
            return
        if self.session is None or self.runtime != status["context"]["runtime_session_id"]:
            self.transport.close()
            self.connect_session(status)
        data = self.transport.request("GET", self.prefix + "companion-mission/")
        self.observed_contract = data.get("contract_version")
        if self.observed_contract != CONTRACT_VERSION:
            raise ValueError("UNSUPPORTED_CONTRACT")
        if data.get("profile") != status["profile"]:
            raise ValueError("PROFILE_MISMATCH")
        if status["profile"] == "HOST_OBSERVE" and (data.get("assignment") is not None or data.get("control_requests")):
            raise ValueError("HOST_EXECUTION_REFUSED")
        if data.get("drone_id") != self.config["drone_id"] or data.get("control_session_id") != self.session:
            raise ValueError("STALE_CONTROL_SESSION")
        self.link(True, "OK")
        if self.stage == "planner_only" and status['profile'] != 'HOST_OBSERVE':
            raise ValueError('PLANNER_ONLY_REQUIRES_HOST_OBSERVE')
        plan_ref = data.get('mission_plan_ref')
        if plan_ref:
            plan_key = (self.runtime, self.session, plan_ref.get('plan_id'), plan_ref.get('sha256'))
            if self.planned != plan_key:
                text = self.transport.plan(plan_ref, self.config['drone_id'])
                context = {k: status['context'][k] for k in ('boot_id', 'runtime_session_id')}
                context['control_session_id'] = self.session
                self.core.call('plan.receive', {'context': context, 'plan_ref': plan_ref, 'plan_text': text})
                self.planned = plan_key
        elif self.planned:
            self.core.call('plan.clear'); self.planned = None
        if data.get("assignment"):
            ref = data["snapshot_ref"]
            key = (self.runtime, self.session, data["assignment"]["assignment_id"], ref["sha256"])
            needs_recheck = (not status["context"]["preparation_id"] and
                             status["telemetry"]["phase"] == "IDLE" and not self.preparation_error)
            if key != self.prepared or needs_recheck:
                text = self.transport.snapshot(ref)
                # Cache is keyed by content hash, never by an untrusted filename.
                atomic_json(self.state_dir / "snapshot_cache.json", {"ref": ref, "snapshot_text": text})
                try:
                    self.core.call("prepare", {"assignment_id": key[2], "snapshot_ref": ref, "snapshot_text": text})
                    self.preparation_error = None
                except CoreError as exc:
                    self.preparation_error = str(exc)
                self.prepared = key
        else:
            self.core.call("prepare.clear")
            self.prepared = None
        if self.stage == "receive_only":
            # W01/W02 can be commissioned before report/command/result APIs are ready.
            # No command processing, WS status injection, or durable-outbox POST here.
            self.poll_count += 1
            self.write_status("SNAPSHOT_RECEIVED" if self.prepared else "CONNECTED_NO_ASSIGNMENT",
                              self.preparation_error or "OK")
            return
        commands = data.get("control_requests", [])
        if not isinstance(commands, list) or len(commands) > 64:
            raise ValueError("INVALID_COMMAND_BATCH")
        for command in commands:
            self.core.call("command", command)
        for item in self.core.call("outbox.list"):
            if self.stage == 'planner_only' and item['route'] != 'mission-plan-receipts':
                continue
            if item["route"] not in ROUTES:
                raise ValueError("OUTBOX_ROUTE_REFUSED")
            body = item["raw_body"].encode("utf-8")
            if hashlib.sha256(body).hexdigest() != item["sha256"]:
                raise ValueError("OUTBOX_CORRUPTED")
            receipt = self.transport.request("POST", self.prefix + item["route"] + "/",
                                             raw_body=body, key=item["key"])
            if receipt.get("stored") is not True or receipt.get("key") != item["key"] or receipt.get("sha256") != item["sha256"]:
                raise ValueError("DURABLE_RECEIPT_REQUIRED")
            self.core.call("outbox.ack", {"key": item["key"], "sha256": item["sha256"]})
        status = self.core.call("status")
        if self.stage == 'planner_only':
            self.poll_count += 1
            self.write_status('PLANNER_RECEIVED' if self.planned else 'CONNECTED_NO_MISSION', 'OK')
            return
        self.transport.publish([status["readiness"], status["telemetry"]])
        self.poll_count += 1
        self.write_status("CONNECTED", self.preparation_error or "OK")

    def write_status(self, state, code):
        atomic_json(self.state_dir / "adapter_status.json", {
            "scope": "WEB_TRANSPORT_ONLY", "generated_at": utc_now(), "pid": os.getpid(),
            "state": state, "code": code, "mode": self.config["mode"],
            "integration_stage": self.stage,
            "base_url": self.config["base_url"], "drone_id": self.config["drone_id"],
            "requested_contract": CONTRACT_VERSION, "observed_contract": self.observed_contract,
            "control_session_id": self.session, "poll_count": self.poll_count,
            "readiness_ack": self.transport.last_readiness_ack,
            "telemetry_ack": self.transport.last_telemetry_ack,
            "physical_output_enabled": False, "preparation_error": self.preparation_error})

    def run(self):
        delay = self.config.get("poll_interval_s", .5)
        while self.running:
            started = time.monotonic()
            try:
                self.poll()
                delay = self.config.get("poll_interval_s", .5)
            except Exception as exc:
                code = "HTTP_" + str(exc.code) if isinstance(exc, urllib.error.HTTPError) else type(exc).__name__ + ":" + str(exc)[:160]
                # No response bodies, authentication headers or secrets enter diagnostics.
                self.session = None
                self.prepared = None
                self.transport.close()
                self.transport.set_session(None)
                try: self.link(False, code)
                except (OSError, CoreError): pass
                self.write_status("DISCONNECTED", code)
                delay = min(max(delay * 2, .5), 10)
            end = started + delay
            while self.running and time.monotonic() < end:
                time.sleep(min(.1, max(0, end - time.monotonic())))
        self.transport.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--identity", help="Private identity file; never passed as shell assignments")
    args = parser.parse_args()
    path = Path(args.config).resolve()
    root = path.parent.parent
    config = decode(path.read_bytes())
    if args.identity:
        from .identity import load_identity
        os.environ.update(load_identity(args.identity, config['drone_id']))
    if config["mode"] not in ("observe", "contract"):
        raise ValueError("INVALID_ADAPTER_MODE")
    if not config["drone_id"] or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for c in config["drone_id"]):
        raise ValueError("INVALID_DRONE_ID")
    os.umask(0o077)
    adapter = Adapter(config, root)
    adapter.state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (adapter.state_dir / "adapter.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        def stop(*_): adapter.running = False
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        adapter.run()

if __name__ == "__main__":
    main()
