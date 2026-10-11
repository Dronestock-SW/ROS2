"""Web START -> C++ native takeoff transactions -> FakePx4 -> durable results."""
import copy
import hashlib
from web_service_integration import Rig, wait_for, ROOT, decode, encode
from sangwon_web.ipc import CoreError


def native_web_mission():
    with Rig(overrides={"replay_takeoff_policy": "PX4_AUTO_TAKEOFF",
                        "approved_replay_takeoff_reference": "WAREHOUSE_MAP_SYNTHETIC_ONLY"}) as rig:
        rig.web()
        request = rig.http("/debug/command", dict(action="START", client_request_key="native-takeoff-1"))
        def completed_result():
            result = rig.core.call("command.get", dict(control_request_id=request["control_request_id"]))
            return result if result and result.get("result_revision") == 2 else None
        result = wait_for(completed_result)
        assert result["flight_outcome"] == "SUCCEEDED", result
        state = rig.core.call("status")
        assert not state["physical_output_enabled"] and not state["readiness"]["flight_authority"]
        control = state["mode_control"]
        assert control["takeoff_policy"] == "PX4_AUTO_TAKEOFF" and control["observed_mode"] == "AUTO.LAND"
        assert control["hardware_transport_implemented"] is False and control["parameter_writer_implemented"] is False
        final = wait_for(lambda: (s if (s := rig.http("/debug/state"))["closed"] else None))
        wait_for(lambda: not rig.core.call("outbox.list"))
        final = rig.http("/debug/state")
        events = [r["details"] for r in final["results"] if r.get("event_type") == "PX4_CONTROL_TRANSITION"]
        for operation in ("TAKEOFF", "ARM", "OFFBOARD", "LAND"):
            assert any(e["operation"] == operation and e["state"] == "CONFIRMED" for e in events), (operation, events)
        assert rig.http("/debug/command", dict(action="START", client_request_key="native-takeoff-1"))["control_request_id"] == request["control_request_id"]
        wait_for(lambda: not rig.core.call("outbox.list"))
    print("PASS authenticated mock web START/native takeoff/OFFBOARD/LAND, durable transition receipts; no physical output")


def reference_is_required():
    with Rig(overrides={"replay_takeoff_policy": "PX4_AUTO_TAKEOFF"}) as rig:
        rig.connect()
        try:
            rig.prepare()
        except CoreError as exc:
            assert "NATIVE_TAKEOFF_REFERENCE_REQUIRED" in str(exc), exc
        else:
            raise AssertionError("Unvalidated native takeoff reference was accepted")
        assert rig.core.call("status")["readiness"]["can_start"] is False
    print("PASS native takeoff target/reference missing blocks preparation")


def offline_transition_pagination():
    snapshot=decode((ROOT/"contracts/runtime_replay/waypoint_snapshot.json").read_bytes())
    extra=copy.deepcopy(snapshot["route_tasks"][-1]);extra["task_id"]="pagination-extra-waypoint"
    extra["position_m"]["x"]+=.4;snapshot["route_tasks"].append(extra)
    raw=encode(snapshot)
    with Rig(snapshot=raw,overrides={"replay_takeoff_policy": "PX4_AUTO_TAKEOFF",
                        "approved_replay_takeoff_reference": "WAREHOUSE_MAP_SYNTHETIC_ONLY",
                        "approved_replay_snapshot_sha256":[hashlib.sha256(raw).hexdigest()]}) as rig:
        rig.connect();rig.prepare();command=rig.command()
        assert rig.core.call("command", command)["status"] == "ACCEPTED"
        def ended():
            result=rig.core.call("command.get", dict(control_request_id=command["control_request_id"]))
            return result if result and result.get("result_revision") == 2 else None
        assert wait_for(ended)["flight_outcome"] == "SUCCEEDED"
        first=rig.core.call("outbox.list");rows=rig.pending_reports()
        assert len(first) == 32 and len(rows) > 32, (len(first),len(rows))
        assert any(row["body"].get("type") == "execution_result" for row in rows)
        assert rig.core.call("outbox.list") == first, "Read cursor must not acknowledge unsent results"
        for cursor in ("missing-key", "x"*257):
            try:
                rig.core.call("outbox.list", {"after_key":cursor})
            except CoreError as exc:
                assert "INVALID_OUTBOX_CURSOR" in str(exc), exc
            else:
                raise AssertionError("Unknown/oversized outbox cursor was accepted")
    print("PASS offline mode/result journal spans bounded pages; reads do not ACK or drop unsent records")


if __name__ == "__main__":
    reference_is_required()
    offline_transition_pagination()
    native_web_mission()
