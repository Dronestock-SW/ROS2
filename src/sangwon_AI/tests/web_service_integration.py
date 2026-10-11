"""Cross-process HTTP/WS -> C++ Runtime -> FakePx4 tests. Loopback only."""
import copy
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import socket
import sqlite3
import subprocess
import sys
import tempfile
import time
import urllib.request
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"python"))
from sangwon_web.common import decode, encode
from sangwon_web.ipc import CoreClient, CoreError

DAEMON=Path(sys.argv[1]).resolve()

def wait_for(fn, timeout=20):
    until=time.monotonic()+timeout
    last=None
    while time.monotonic()<until:
        try:
            value=fn()
            if value: return value
        except (OSError,CoreError,KeyError) as exc: last=exc
        time.sleep(.05)
    raise AssertionError("Timed out: "+str(last))

def stop(process):
    if process.poll() is None:
        process.terminate()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=3)

class Rig:
    def __init__(self, steps=10, profile="REPLAY", snapshot=None, overrides=None):
        # Short /tmp path keeps AF_UNIX under 108 bytes; disposable test state only.
        self.temp=tempfile.TemporaryDirectory(prefix="sangwon-it-")
        self.root=Path(self.temp.name); (self.root/"config").mkdir()
        self.procs=[]; self.handles=[]
        self.env=dict(os.environ,PYTHONPATH=str(ROOT/"python"),PYTHONDONTWRITEBYTECODE="1")
        self.snapshot=snapshot if snapshot is not None else (ROOT/"contracts/runtime_replay/waypoint_snapshot.json").read_bytes()
        self.snapshot_ref=dict(snapshot_id=decode(self.snapshot)["snapshot_id"],sha256=hashlib.sha256(self.snapshot).hexdigest(),byte_length=len(self.snapshot))
        self.config=decode((ROOT/"config/companion.replay.json").read_bytes())
        self.config.update(state_dir=".runtime/c",replay_steps_per_tick=steps)
        self.config["profile"] = profile
        self.config.update(overrides or {})
        self.path=self.root/"config/core.json"; self.path.write_bytes(encode(self.config))
        self.core=CoreClient(self.root/".runtime/c/core.sock")
        self.daemon=self.launch("core",[str(DAEMON),"--config",str(self.path)])
        wait_for(lambda:self.core.call("status"))

    def launch(self,name,args):
        handle=(self.root/(name+".log")).open("ab"); self.handles.append(handle)
        proc=subprocess.Popen(args,env=self.env,stdout=handle,stderr=subprocess.STDOUT)
        self.procs.append(proc); return proc

    def connect(self):
        status=self.core.call("status"); context=status["context"]
        self.core.call("session.set",dict(contract_version="1.1-draft.4",profile="REPLAY",drone_id="TEST-DRONE-01",
            flight_authority=False,allowed_execution="REPLAY_ONLY",
            boot_id=context["boot_id"],runtime_session_id=context["runtime_session_id"],
            server_time=dt.datetime.now(dt.timezone.utc).isoformat(),control_session_id="test-session-"+uuid.uuid4().hex))

    def prepare(self, assignment="test-assignment"):
        return self.core.call("prepare",dict(assignment_id=assignment,snapshot_ref=self.snapshot_ref,snapshot_text=self.snapshot.decode()))

    def command(self, action="START", seq=1):
        now=dt.datetime.now(dt.timezone.utc)
        return dict(contract_version="1.1-draft.4",profile="REPLAY",drone_id="TEST-DRONE-01",type="control_request",
            control_request_id="req-"+uuid.uuid4().hex,command_seq=seq,action=action,
            created_at=now.isoformat(),expires_at=(now+dt.timedelta(seconds=10)).isoformat(),
            context=self.core.call("status")["context"],payload={"start_mode":"AUTO_TAKEOFF"} if action=="START" else {})

    def web(self, stage="full"):
        holders=[]; ports=[]
        for _ in range(2):
            sock=socket.socket(); sock.bind(("127.0.0.1",0)); ports.append(sock.getsockname()[1]); holders.append(sock)
        for sock in holders:sock.close()
        self.url=f"http://127.0.0.1:{ports[0]}"
        self.mock=self.launch("mock",[sys.executable,"-m","sangwon_web.mock_platform","--snapshot",str(ROOT/"contracts/runtime_replay/waypoint_snapshot.json"),
            "--state-dir",str(self.root/"mock"),"--http-port",str(ports[0]),"--ws-port",str(ports[1])])
        wait_for(lambda:self.http("/debug/state"))
        conf=decode((ROOT/"config/web.replay.json").read_bytes())
        conf.update(base_url=self.url,websocket_url=f"ws://127.0.0.1:{ports[1]}/ws/drones/TEST-DRONE-01/",
                    state_dir=".runtime/w",core_socket=".runtime/c/core.sock",integration_stage=stage)
        self.adapter_path=self.root/"config/web.json"; self.adapter_path.write_bytes(encode(conf))
        self.start_adapter()
        if stage=="receive_only":
            wait_for(lambda:decode((self.root/".runtime/w/adapter_status.json").read_bytes()).get("state")=="SNAPSHOT_RECEIVED")
        else:
            wait_for(lambda:self.http("/debug/state")["latest"].get("readiness",{}).get("can_start"))

    def start_adapter(self):
        self.adapter=self.launch("adapter",[sys.executable,"-m","sangwon_web.adapter","--config",str(self.adapter_path)])

    def http(self,path,body=None):
        req=urllib.request.Request(self.url+path,data=encode(body) if body else None,headers={"Content-Type":"application/json"})
        with urllib.request.urlopen(req,timeout=2) as response:return decode(response.read())

    def pending_reports(self):
        rows=[]
        while True:
            page=self.core.call('outbox.list',{'after_key':rows[-1]['key']} if rows else {})
            if not page:return rows
            assert len(page)<=32 and not ({r['key'] for r in page}&{r['key'] for r in rows})
            rows.extend(page)

    def __enter__(self):return self
    def __exit__(self,kind,value,tb):
        for proc in reversed(self.procs):stop(proc)
        for handle in self.handles:handle.close()
        if kind:
            for log in self.root.glob("*.log"):print(log.name,log.read_text()[-6000:])
        self.temp.cleanup()

def boundary_test():
    with Rig() as rig:
        initial=rig.core.call("status")["readiness"]
        assert not initial["can_start"]
        assert len(initial["checks"])==33
        assert any(c["status"]=="UNKNOWN" and c["blocking"] for c in initial["checks"])
        rig.connect(); ready=rig.prepare()
        assert ready["can_start"] and ready["flight_authority"] is False
        assert ready["readiness_seq"]>initial["readiness_seq"]
        assert len({c['check_id'] for c in ready['checks']})==33
        assert all(not c['applicable'] for c in ready['checks'] if c['check_id'].startswith('QS-'))
        assert ready['checks'][9]['source']=='APPROVED_REPLAY_FIXTURE'
        telemetry=rig.core.call('status')['telemetry']
        assert telemetry['position_transport']=='JETSON_WIFI'
        assert telemetry['uwb_map_position']['valid'] is False and telemetry['uwb_map_position']['position_m'] is None
        parsed=lambda value:dt.datetime.fromisoformat(value.replace('Z','+00:00'))
        for _ in range(50):
            sample=rig.core.call('status')
            assert parsed(sample['telemetry']['pose_fused']['observed_at'])<=parsed(sample['telemetry']['generated_at'])
            for check in sample['readiness']['checks']:
                if check['observed_at']:
                    assert parsed(check['observed_at'])<=parsed(sample['readiness']['generated_at'])
        assert rig.core.call("status")["telemetry"]["px4"]["armed"] is False
        expired=rig.command(); old=dt.datetime.now(dt.timezone.utc)-dt.timedelta(seconds=20)
        expired.update(created_at=old.isoformat(),expires_at=(old+dt.timedelta(seconds=10)).isoformat())
        assert rig.core.call("command",expired)["code"]=="COMMAND_EXPIRED"
        wrong=rig.command(); wrong["context"]["runtime_session_id"]="stale"
        assert rig.core.call("command",wrong)["code"]=="STALE_RUNTIME_SESSION"
        stale=rig.command(); stale["context"]["preparation_id"]="stale"
        assert rig.core.call("command",stale)["code"]=="PREPARATION_STALE"
        scan=decode(rig.snapshot); scan["route_tasks"][0]["type"]="scan"; raw=encode(scan)
        ref=dict(rig.snapshot_ref,sha256=hashlib.sha256(raw).hexdigest(),byte_length=len(raw))
        rig.core.call("link.update", {"connected":True,"code":"OK","observed_contract":"1.1-draft.4"})
        try:rig.core.call("prepare",dict(assignment_id="scan",snapshot_ref=ref,snapshot_text=raw.decode()))
        except CoreError as exc:assert "UNSUPPORTED_CAPABILITY" in str(exc)
        else:raise AssertionError("Scan was silently executed")
        rig.prepare()
        start=rig.command(); accepted=rig.core.call("command",start)
        assert accepted["status"]=="ACCEPTED"
        assert rig.core.call("command",start)==accepted
        changed=copy.deepcopy(start); changed["payload"]["start_mode"]="RC_HANDOVER"
        assert rig.core.call("command",changed)["code"]=="REQUEST_ID_CONFLICT"
        result=wait_for(lambda:(r if (r:=rig.core.call("command.get",dict(control_request_id=start["control_request_id"]))).get("result_revision")==2 else None))
        assert result["flight_outcome"]=="SUCCEEDED" and result["visited"]==2,result
        assert rig.core.call("command",start)==result
        rig.connect()
        try:rig.prepare()
        except CoreError as exc:assert "ASSIGNMENT_CLOSED" in str(exc)
        else:raise AssertionError("Completed assignment rearmed")
        rig.prepare("next-assignment")
    print("PASS command TTL/context/deduplication, unsupported scan, no auto-start, terminal ledger")

def web_test():
    with Rig() as rig:
        rig.web()
        initial=rig.http("/debug/state")
        wait_for(lambda:decode((rig.root/'.runtime/w/adapter_status.json').read_bytes()).get('readiness_ack',{}).get('accepted'))
        assert initial["latest"]["telemetry"]["px4"]["armed"] is False
        body=dict(action="START",client_request_key="operator-click-1")
        request=rig.http("/debug/command",body)
        assert rig.http("/debug/command",body)["control_request_id"]==request["control_request_id"]
        wait_for(lambda:rig.core.call("status")["telemetry"]["phase"] not in ("IDLE","PREPARING"))
        stop(rig.adapter)
        result=wait_for(lambda:(r if (r:=rig.core.call("command.get",dict(control_request_id=request["control_request_id"]))).get("result_revision")==2 else None))
        assert result["flight_outcome"]=="SUCCEEDED",result
        assert rig.core.call("outbox.list"),"Offline results must be durable"
        rig.start_adapter()
        final=wait_for(lambda:(s if (s:=rig.http("/debug/state"))["closed"] else None))
        wait_for(lambda:not rig.core.call("outbox.list"))
        assert final["command_count"]==1
        final_results=[r for r in final["results"] if r.get("type")=="command_result" and r["result_revision"]==2]
        assert len(final_results)==1 and final_results[0]["flight_outcome"]=="SUCCEEDED"
        latest=wait_for(lambda:(s if (s:=rig.http("/debug/state"))["latest"].get("telemetry",{}).get("phase")=="ENDED" else None))
        assert latest["latest"]["readiness"]["can_start"] is False
    print("PASS HTTP assignment/START + WS status + offline continuation + durable outbox replay")

def restart_test():
    with Rig(steps=1) as rig:
        rig.connect();rig.prepare();cmd=rig.command();accepted=rig.core.call("command",cmd)
        old=rig.core.call("status")["context"]["runtime_session_id"]
        rig.daemon.kill();rig.daemon.wait(timeout=3)
        rig.daemon=rig.launch("restarted",[str(DAEMON),"--config",str(rig.path)])
        current=wait_for(lambda:(s if (s:=rig.core.call("status"))["context"]["runtime_session_id"]!=old else None))
        assert current["recovery_locked"] and not current["readiness"]["can_start"]
        assert current["telemetry"]["px4"]["armed"] is False
        assert rig.core.call("command",cmd)==accepted
        rig.connect();fresh=rig.command()
        assert rig.core.call("command",fresh)["code"]=="RECOVERY_LOCK"
    print("PASS SIGKILL durable ledger + fresh runtime epoch + recovery lock, no resume")

def receive_only_test():
    with Rig() as rig:
        rig.web(stage="receive_only")
        cached=decode((rig.root/".runtime/w/snapshot_cache.json").read_bytes())
        assert cached["snapshot_text"].encode()==rig.snapshot
        command=rig.command()
        # Queue a valid START at the mock peer: commissioning must never consume it.
        with sqlite3.connect(rig.root/"mock/mock.sqlite3") as db:
            db.execute("INSERT INTO commands VALUES(?,?,?)",(command["control_request_id"],"receive-only",encode(command)))
        path=rig.root/".runtime/w/adapter_status.json"
        before=decode(path.read_bytes())["poll_count"]
        wait_for(lambda:decode(path.read_bytes())["poll_count"]>=before+2)
        assert rig.core.call("command.get",dict(control_request_id=command["control_request_id"])) is None
        assert rig.core.call("status")["telemetry"]["px4"]["armed"] is False
        assert rig.core.call("outbox.list"),"Preparation outbox must remain unsent"
        peer=rig.http("/debug/state")
        assert peer["latest"]=={} and peer["results"]==[] and not peer["closed"]
    print("PASS W01/W02 receive-only: verified bytes, queued START ignored, no WS or result POST")

def control_test():
    with Rig(steps=4) as rig:
        rig.connect();rig.prepare();start=rig.command()
        assert rig.core.call("command",start)["status"]=="ACCEPTED"
        def fresh():
            rig.core.call("link.update",dict(connected=True,code="OK",observed_contract="1.1-draft.4"))
            return rig.core.call("status")
        wait_for(lambda:fresh()["telemetry"]["phase"]=="NAVIGATING")
        pause=rig.command("PAUSE",2)
        assert rig.core.call("command",pause)["status"]=="ACCEPTED"
        def completed(command):
            fresh()
            value=rig.core.call("command.get",dict(control_request_id=command["control_request_id"]))
            return value if value.get("result_revision")==2 else None
        assert wait_for(lambda:completed(pause))["status"]=="COMPLETED"
        resume=rig.command("RESUME",3)
        assert rig.core.call("command",resume)["status"]=="ACCEPTED"
        assert wait_for(lambda:completed(resume))["status"]=="COMPLETED"
        cancel=rig.command("CANCEL",4)
        assert rig.core.call("command",cancel)["status"]=="ACCEPTED"
        wait_for(lambda:fresh()["telemetry"]["phase"] in ("RETURNING","LANDING"))
        assert rig.core.call("command",rig.command("PAUSE",5))["code"]=="INVALID_PHASE"
        stale_seq=rig.command("CANCEL",3)
        assert rig.core.call("command",stale_seq)["code"]=="STALE_COMMAND_SEQUENCE"
        assert wait_for(lambda:completed(cancel))["status"]=="COMPLETED"
        assert rig.core.call("command.get",dict(control_request_id=start["control_request_id"]))["flight_outcome"]=="CANCELED"
    print("PASS PAUSE/RESUME/CANCEL completion, return phase lock, stale sequence rejection")

def host_test():
    with Rig(profile="HOST_OBSERVE") as rig:
        status = rig.core.call("status")
        assert set(status['supported_capabilities']) == {'command_session', 'readiness_binding', 'planner_plan_v1'}
        context = status['context']
        session = dict(contract_version='1.1-draft.4', profile='HOST_OBSERVE', drone_id='TEST-DRONE-01',
            boot_id=context['boot_id'], runtime_session_id=context['runtime_session_id'],
            server_time=dt.datetime.now(dt.timezone.utc).isoformat(), control_session_id='host-test',
            flight_authority=False, allowed_execution='NONE')
        rig.core.call('session.set', session)
        now = dt.datetime.now(dt.timezone.utc)
        report = dict(schema_version='sangwon-host-health/1', boot_id=context['boot_id'],
            scope='HOST_DIAGNOSTICS_ONLY', can_start=False, flight_authority=False,
            monitor_session_id='monitor-test', monitor_seq=1, generated_monotonic_s=time.monotonic()-9,
            generated_at=(now-dt.timedelta(seconds=9)).isoformat(), checks=[dict(id='HOST_DISK',
                status='PASS', detail='Synthetic disk observation', operator_action='Check recorder separately',
                required_for_flight=True)])
        rig.core.call('host.update', report)
        rig.core.call('host.update', report)
        ready = rig.core.call('status')['readiness']
        assert ready['host_diagnostics']['source_age_ms'] >= 9000
        assert not ready['can_start'] and ready['allowed_commands'] == []
        assert ready['host_diagnostics']['state'] == 'LIVE'
        for bad in (dict(report, boot_id='previous-boot'), dict(report, can_start=True),
                    dict(report, generated_monotonic_s=time.monotonic()+100)):
            try: rig.core.call('host.update', bad)
            except CoreError: pass
            else: raise AssertionError('Invalid host evidence accepted')
        wait_for(lambda:rig.core.call('status')['readiness']['host_diagnostics']['state']=='STALE', timeout=3)
        try: rig.prepare()
        except CoreError as exc: assert 'PROFILE_NOT_READY' in str(exc)
        else: raise AssertionError('HOST executed a mission')
        assert not rig.core.call('status')['readiness']['can_start']
    print('PASS authenticated HOST diagnostics, cached-evidence expiry, forbidden execution')

if __name__=="__main__":
    boundary_test();web_test();restart_test();control_test();receive_only_test();host_test()
