"""Read-only software environment checks. Never starts vehicle nodes."""
import grp
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]


def main():
    checks = {}
    for module in ("rclpy", "cv2", "cv_bridge", "mavros_msgs.msg", "sensor_msgs.msg",
                   "evdev", "pyzbar.pyzbar", "requests", "websockets"):
        try:
            loaded = importlib.import_module(module)
            checks[module] = {"ok": True, "path": str(getattr(loaded, "__file__", ""))}
        except Exception as exc:
            checks[module] = {"ok": False, "error": str(exc)}
    try:
        import numpy as np
        from cv_bridge import CvBridge
        pixels = np.zeros((4, 4, 3), dtype=np.uint8)
        bridge = CvBridge()
        message = bridge.cv2_to_imgmsg(pixels, encoding="bgr8")
        assert np.array_equal(bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"), pixels)
        checks["image_roundtrip"] = {"ok": True, "numpy": np.__version__}
    except Exception as exc:
        checks["image_roundtrip"] = {"ok": False, "error": str(exc)}
    log_python = ROOT / ".venv-log/bin/python"
    try:
        log_env = os.environ.copy()
        log_env.pop("PYTHONPATH", None)
        log_env.pop("PYTHONHOME", None)
        proc = subprocess.run([str(log_python), "-c",
            "import json,numpy,pyulog,importlib.metadata; print(json.dumps({"
            "'numpy':numpy.__version__,'pyulog':importlib.metadata.version('pyulog')}))"],
            env=log_env, capture_output=True, text=True, check=True, timeout=15)
        checks["log_environment"] = {"ok": True, **json.loads(proc.stdout)}
    except Exception as exc:
        checks["log_environment"] = {"ok": False, "error": str(exc)}
    checks["python_environment"] = {"ok": Path(sys.prefix) == ROOT / ".venv",
                                    "executable": sys.executable, "prefix": sys.prefix}
    checks["ros_domain"] = {"ok": os.environ.get("ROS_DOMAIN_ID") == "1",
                            "value": os.environ.get("ROS_DOMAIN_ID")}
    for command in ("colcon", "cmake", "g++", "sangwon_replay"):
        path = shutil.which(command)
        checks[command] = {"ok": path is not None, "path": path}
    groups = [grp.getgrgid(gid).gr_name for gid in os.getgroups()]
    devices = {path: Path(path).exists() for path in
               ("/dev/pixhawk", "/dev/lidar", "/dev/uwb", "/dev/input/qr_reader")}
    versions = {}
    for package in ("numpy", "websockets", "requests", "pip", "pyzbar", "evdev"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    result = {"schema": "sangwon-dev-environment/1",
              "checked_at": datetime.now(timezone.utc).isoformat(),
              "checks": checks, "versions": versions,
              "development_checks_pass": all(check["ok"] for check in checks.values()),
              "groups": groups, "devices": devices,
              "flight_ready": False, "flight_authority": False,
              "remaining": ["PX4/UWB/web/ArUco integration and flight preflight are unverified",
                            "Full vehicle services and cold boot require integration tests"]}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["development_checks_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
