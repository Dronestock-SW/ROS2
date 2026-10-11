#!/usr/bin/env python3
"""Read-only host diagnostics. Never grants flight readiness or opens a vehicle port."""
import argparse
import datetime as dt
import fcntl
import grp
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid

SCHEMA = "sangwon-host-health/1"
TTL_S = 10.0  # > 3 command timeouts (4.5s) + loop sleep (2s); host display only.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from sangwon_sensors.perception import host_checks as perception_host_checks
from sangwon_sensors.px4 import host_checks as px4_host_checks, read_report as read_px4_report


def command(args):
    try:
        run = subprocess.run(args, capture_output=True, text=True, encoding="utf-8",
                             timeout=1.5, check=False, env={**os.environ, "LC_ALL": "C"})
        return run.returncode, run.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return -1, ""


def check(identifier, status, detail, action, required=True):
    if status not in ("PASS", "FAIL", "UNKNOWN", "WARN"):
        raise ValueError("Invalid check state")
    return {"id": identifier, "status": status, "required_for_flight": required,
            "detail": detail, "operator_action": action}


def boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()


def collect(root):
    checks = []
    software = Path("/opt/ros/humble/setup.bash").is_file()
    checks.append(check("HOST_ROS_INSTALLED", "PASS" if software else "FAIL",
                        "ROS Humble setup file present" if software else "ROS setup missing",
                        "Validate ROS packages and build; file presence does not prove node health"))
    code, value = command(["timedatectl", "show", "-p", "NTPSynchronized", "--value"])
    checks.append(check("HOST_UTC_SYNC", "PASS" if code == 0 and value == "yes" else "UNKNOWN",
                        "NTP synchronization=" + (value or "unavailable"),
                        "Verify UTC synchronization before accepting expiring web commands"))
    free = shutil.disk_usage(root).free
    checks.append(check("HOST_DISK", "PASS" if free >= 2 * 1024**3 else "FAIL",
                        f"Available bytes={free}; 2GiB is a host-only trial floor",
                        "Flight recording capacity still needs mission-rate sizing"))
    groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
    checks.append(check("HOST_SERIAL_PERMISSION", "PASS" if "dialout" in groups else "FAIL",
                        "dialout effective=" + str("dialout" in groups),
                        "Administrator adds dialout after device ownership review; start a new user session"))
    code, value = command(["nmcli", "-t", "-f", "TYPE,STATE", "device"])
    wifi = code == 0 and "wifi:connected" in value.splitlines()
    checks.append(check("HOST_WIFI", "PASS" if wifi else "WARN",
                        "Wi-Fi link connected" if wifi else "Wi-Fi link not confirmed",
                        "Check saved access point; this is not API authentication or server reachability", False))
    code, value = command(["loginctl", "show-user", str(os.getuid()), "-p", "Linger", "--value"])
    checks.append(check("HOST_BOOT_PERSISTENCE", "PASS" if code == 0 and value == "yes" else "WARN",
                        "User manager linger=" + (value or "unknown"),
                        "Enable linger or deploy a reviewed system service, then verify a cold boot", False))
    for name, path in (("PX4", "/dev/pixhawk"), ("LIDAR", "/dev/lidar")):
        exists = Path(path).exists()
        checks.append(check("DEVICE_" + name, "WARN" if exists else "FAIL",
                            f"{path}: {'present, identity and data unverified' if exists else 'not detected'}",
                            "Connect device, verify identity/permissions and fresh telemetry"))
    for identifier, detail, action in (
        ("UWB_FIELD_ACCEPTANCE_PENDING", "ROS2 UWB/B_TF adapter integrated; field calibration/fusion unverified", "Verify Tag profile, anchors, mounts, timing and PX4 fusion evidence"),
        ("PX4_PREFLIGHT_PENDING", "No authoritative PX4 preflight feed", "Integrate fresh PX4 status and test arming checks"),
        ("CPP_RUNTIME_PENDING", "Only REPLAY runtime is implemented", "Implement ROS production nodes and watchdog transport"),
        ("FLIGHT_RECORDING_PENDING", "No production flight recorder health feed", "Verify recorder progress, retention and capacity"),
        ("FLIGHT_PROFILE_PENDING", "No approved FLIGHT profile", "Finish UWB, yaw, altitude and failsafe validation"),
    ):
        checks.append(check(identifier, "UNKNOWN", detail, action))
    checks.append(check("RC_STATUS_PENDING", "UNKNOWN", "RC telemetry and existing switch configuration unverified",
                        "Verify Offboard handover; preserve existing manual RC/PX4 settings", False))
    try:
        report = json.loads((root / '.runtime/perception/health.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        report = {}
    checks.extend(perception_host_checks(report, boot_id(), time.monotonic()))
    checks.extend(px4_host_checks(read_px4_report(root / '.runtime/px4/health.json'), boot_id(), time.monotonic()))
    return checks


def snapshot(checks, device_boot, session, sequence, now):
    blockers = [c["id"] for c in checks if c["required_for_flight"] and c["status"] != "PASS"]
    # Structural guard: this helper can never publish an actual preflight approval.
    if "HOST_MONITOR_NOT_FLIGHT_AUTHORITY" not in blockers:
        blockers.append("HOST_MONITOR_NOT_FLIGHT_AUTHORITY")
    return {
        "schema_version": SCHEMA, "boot_id": device_boot, "monitor_session_id": session,
        "monitor_seq": sequence, "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "generated_monotonic_s": now, "display_ttl_s": TTL_S,
        "scope": "HOST_DIAGNOSTICS_ONLY", "state": "BLOCKED",
        "system_ready": False, "mission_ready": False, "can_start": False,
        "flight_authority": False, "companion_session_id": None, "preparation_id": None,
        "blockers": blockers, "checks": checks,
    }


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, name = tempfile.mkstemp(prefix=".health-", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, allow_nan=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def notify(message):
    address = os.environ.get("NOTIFY_SOCKET")
    if address:
        if address.startswith("@"):
            address = "\0" + address[1:]
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.sendto(message.encode("utf-8"), address)


def is_fresh(data, current_boot, now):
    try:
        stamp = data["generated_monotonic_s"]
        age = now - stamp
        return (data["schema_version"] == SCHEMA and data["boot_id"] == current_boot
                and type(stamp) in (int, float) and 0 <= age <= TTL_S)
    except (KeyError, TypeError, ValueError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--status", action="store_true")
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    output = root / ".runtime/host_health.json"
    current_boot = boot_id()
    if args.status:
        try:
            data = json.loads(output.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            print("UNKNOWN: host monitor has no readable report")
            return 2
        if not is_fresh(data, current_boot, time.monotonic()):
            print("STALE: host report expired or belongs to another boot")
            return 2
        print(json.dumps(data, ensure_ascii=False, indent=2))
        return 0
    output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (output.parent / "host_health.lock").open("a", encoding="utf-8") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("Another host monitor owns the report", file=sys.stderr)
            return 3
        running = True

        def stop(_number, _frame):
            nonlocal running
            running = False

        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        session, sequence, last_signature = str(uuid.uuid4()), 0, None
        while running:
            checks = collect(root)
            sequence += 1
            data = snapshot(checks, current_boot, session, sequence, time.monotonic())
            atomic_write(output, data)
            signature = [(c["id"], c["status"]) for c in checks]
            if signature != last_signature:
                print(json.dumps({"state": data["state"], "blockers": data["blockers"]}), flush=True)
                last_signature = signature
            # systemd READY means the DIAGNOSTIC SERVICE is started, never flight readiness.
            notify(("READY=1\n" if sequence == 1 else "") + "WATCHDOG=1\nSTATUS=Host diagnostics active; flight BLOCKED")
            if args.once:
                return 0
            for _ in range(20):
                if not running:
                    break
                time.sleep(0.1)
        notify("STOPPING=1\nSTATUS=Host diagnostics stopping")
    return 0


if __name__ == "__main__":
    sys.exit(main())
