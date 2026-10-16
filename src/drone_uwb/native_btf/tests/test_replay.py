"""Exercise observation and FREEZE through the compiled JSONL executable."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import unittest


class ReplayContract(unittest.TestCase):
    def demo(self):
        return [json.loads(x) for x in (EXAMPLES/"freeze.jsonl").read_text(encoding="utf-8").splitlines()]

    def run_rows(self, rows):
        with tempfile.TemporaryDirectory(prefix="btf-replay-") as directory:
            source = Path(directory)/"input.jsonl"
            source.write_text("".join(json.dumps(x)+"\n" for x in rows), encoding="utf-8")
            run = subprocess.run([str(BINARY), str(EXAMPLES/"config.json"), str(source)],
                                 capture_output=True, text=True, encoding="utf-8")
        output = [json.loads(x) for x in run.stdout.splitlines()]
        self.assertEqual(len(output), len(rows))
        self.assertTrue(all(x["external_output_allowed"] is False for x in output))
        return run.returncode, output

    def test_silence_freezes_target_until_qualified_recovery(self):
        code, result = self.run_rows(self.demo())
        self.assertEqual(code, 0)
        self.assertEqual(result[11]["freeze"]["state"], "RUNNING")
        stamp = result[11]["freeze"]["last_observation_monotonic_us"]
        for row in result[12:15]:
            self.assertNotIn("measurement", row)
            self.assertEqual(row["freeze"]["state"], "FREEZE")
            self.assertEqual(row["freeze"]["target_xy_m"], [2, 2])
            self.assertEqual(row["freeze"]["last_observation_monotonic_us"], stamp)
            self.assertTrue(row["freeze"]["pause_mission"])
        self.assertEqual(result[-1]["freeze"]["state"], "RUNNING")
        self.assertTrue(any(x.get("freeze", {}).get("reason") == "observation_recovered_resume_next_tick" for x in result[15:]))

    def test_future_mapped_measurement_does_not_release_freeze(self):
        rows = self.demo()[:12]
        rows[-1]["source_monotonic_us"] = rows[-1]["now_monotonic_us"]+1
        code, result = self.run_rows(rows)
        self.assertEqual(code, 0)
        self.assertTrue(result[-1]["measurement"]["ok"])
        self.assertEqual(result[-1]["freeze"]["state"], "FREEZE")
        self.assertEqual(result[-1]["freeze"]["reason"], "stale_or_repeated_observation")

    def test_unknown_fc_report_keeps_stream_but_explicit_fault_defers(self):
        for health, streams, defer in (("unknown", True, False), ("invalid", False, True)):
            rows = self.demo()[:12]
            rows.append(dict(type="tick", now_monotonic_us=1400000, fc_health=health, approved_target_xy_m=[9, 9]))
            code, result = self.run_rows(rows)
            self.assertEqual(code, 0)
            hold = result[-1]["freeze"]
            self.assertEqual(hold["target_xy_m"], [2, 2])
            self.assertEqual(hold["stream_target"], streams)
            self.assertEqual(hold["defer_to_fc_failsafe"], defer)
            self.assertTrue(hold["pause_mission"])

    def test_clock_regression_does_not_reanchor_target(self):
        rows = self.demo()[:12]
        rows.append(dict(type="tick", now_monotonic_us=1, fc_health="healthy", approved_target_xy_m=[9, 9]))
        code, result = self.run_rows(rows)
        self.assertEqual(code, 0)
        self.assertEqual(result[-1]["freeze"]["target_xy_m"], [2, 2])
        self.assertEqual(result[-1]["freeze"]["reason"], "host_clock_reset")

    def test_malformed_row_is_rejected_and_does_not_reuse_old_output(self):
        rows = self.demo()[:12]
        rows.append(dict(type="frame", cycle_start_us=True))
        rows.append(dict(type="tick", now_monotonic_us=1400000, fc_health="healthy"))
        code, result = self.run_rows(rows)
        self.assertEqual(code, 1)
        self.assertIn("error", result[-2])
        self.assertFalse(result[-1]["freeze"]["stream_target"])
        self.assertIsNone(result[-1]["freeze"]["last_observation_xy_m"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--examples", type=Path, required=True)
    args, remaining = parser.parse_known_args()
    BINARY, EXAMPLES = args.binary.resolve(), args.examples.resolve()
    unittest.main(argv=[__file__, *remaining])
