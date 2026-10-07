import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

source = Path(__file__).resolve().parents[1] / "ops/health_monitor.py"
spec = importlib.util.spec_from_file_location("health_monitor", source)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class HostMonitorTests(unittest.TestCase):
    def test_even_all_pass_cannot_grant_flight_readiness(self):
        item = monitor.check("FAKE", "PASS", "ok", "none")
        report = monitor.snapshot([item], "boot", "session", 1, 100)
        self.assertFalse(report["can_start"])
        self.assertFalse(report["system_ready"])
        self.assertFalse(report["flight_authority"])
        self.assertEqual(report["state"], "BLOCKED")
        self.assertIsNone(report["preparation_id"])

    def test_unknown_required_blocks(self):
        report = monitor.snapshot([monitor.check("PX4", "UNKNOWN", "", "")], "b", "s", 1, 1)
        self.assertIn("PX4", report["blockers"])

    def test_optional_rc_unknown_is_not_mandatory_connection(self):
        report = monitor.snapshot([monitor.check("RC", "UNKNOWN", "", "", False)], "b", "s", 1, 1)
        self.assertNotIn("RC", report["blockers"])

    def test_old_boot_or_stale_is_invalid(self):
        report = monitor.snapshot([], "boot1", "session", 1, 100)
        self.assertTrue(monitor.is_fresh(report, "boot1", 101))
        self.assertFalse(monitor.is_fresh(report, "boot2", 101))
        self.assertFalse(monitor.is_fresh(report, "boot1", 111))
        self.assertFalse(monitor.is_fresh(report, "boot1", 99))

    def test_bad_times_are_invalid(self):
        for value in ("100", None, float("nan"), float("inf")):
            report = monitor.snapshot([], "b", "s", 1, value)
            self.assertFalse(monitor.is_fresh(report, "b", 100))

    def test_atomic_report_replaces_old_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "health.json"
            monitor.atomic_write(path, {"monitor_seq": 1})
            monitor.atomic_write(path, {"monitor_seq": 2})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["monitor_seq"], 2)
            self.assertEqual(list(Path(directory).iterdir()), [path])


if __name__ == "__main__":
    unittest.main()
