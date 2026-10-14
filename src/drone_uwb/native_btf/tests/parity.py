"""Compare compiled C++ with the unchanged Python B_TF kernel on identical input.

UTF-8 files; no ROS, serial port, FC, or network access. --python-root points to
src/drone_uwb (the directory containing the drone_uwb Python package).
"""
from __future__ import annotations
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


def dumps(obj):
    return json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--binary", required=True, type=Path)
    p.add_argument("--python-root", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    args = p.parse_args()
    sys.path.insert(0, str(args.python_root.resolve()))
    from drone_uwb.acquisition.validation import Cycle
    from drone_uwb.processing.experiments.h80_b import BSettings, H80Window
    from drone_uwb.processing.experiments.tof_subset import ToFTrackedSubsetCandidate

    anchors = np.array([[0, 0, 2.2], [6.3, 0, 2.2], [0, 4.6, 2.2], [6.3, 4.6, 2.2]])
    bias = np.array([.01, -.02, .03, -.01])
    config = dict(anchors_xyz_m=anchors.tolist(), bias_m=bias.tolist(), B=asdict(BSettings()),
                  enforce_b4_height_integrity=False, ground_subset_initialization=False)
    cases = ["stationary", "moving", "noisy", "one_spike", "persistent_a3",
             "two_bad_anchors", "b4_without_tof", "b3_without_tof", "height_conflict",
             "height_veto", "height_veto_recovery", "ground_acquisition", "packet_gap",
             "clock_restart", "coherent_step", "large_source_clock", "accelerating",
             "partial_height", "near_reacquisition", "moving_reacquisition", "moving_input_gap", "far_rejection"]
    args.output.mkdir(parents=True, exist_ok=True)
    summary = dict(cases=[], max_xy_difference_m=0., max_score_difference_m=0., failures=[])
    all_py_ms, all_cpp_ms = [], []
    for name in cases:
        cfg = dict(config)
        cfg["enforce_b4_height_integrity"] = name in ("height_veto", "height_veto_recovery")
        cfg["ground_subset_initialization"] = name == "ground_acquisition"
        folder = args.output / name
        folder.mkdir(exist_ok=True)
        (folder / "config.json").write_text(dumps(cfg), encoding="utf-8")
        window = H80Window(anchors, bias, BSettings(**cfg["B"]))
        candidate = ToFTrackedSubsetCandidate(cfg)
        rng = np.random.default_rng(20261014)
        rows, expected = [], []
        for k in range(220 if "reacquisition" in name or name in ("moving_input_gap", "far_rejection") else 160):
            start = 1_000_000 + k * 25_000
            if name == "packet_gap" and k >= 75:
                start += 300_000
            if name == "moving_input_gap" and k >= 140:
                start += 500_000
            if name == "clock_restart" and k >= 75:
                start -= 1_500_000
            if name == "large_source_clock":
                start += 1_000_000_000_000
            stamps = [start + 4000 * (i + 1) for i in range(4)]
            # Ground truth uses relative scenario time, never the fitted output.
            t = (np.array(stamps) - start + k * 25_000) / 1e6
            if name == "moving_input_gap" and k >= 140:
                t += .5
            moving = name in ("moving", "noisy", "packet_gap", "large_source_clock", "accelerating")
            x = 2 + .3*t if moving else np.full(4, 2.)
            y = 2 - .1*t if moving else np.full(4, 2.)
            if name in ("moving_reacquisition", "moving_input_gap"):
                x = 1. + .8*t
            if name == "near_reacquisition" and k >= 160:
                x += .02
            if name == "far_rejection" and k >= 160:
                x += 1.
            if name == "accelerating":
                x += .05*t*t
            if name == "coherent_step" and 65 <= k < 95:
                x += .5
            heights = .8 + .02*t
            raw = np.linalg.norm(np.column_stack([x, y, heights]) - anchors, axis=1) + bias
            if name == "noisy":
                raw += rng.normal(0, .012, 4)
            if name == "one_spike" and k == 65:
                raw[2] -= 1.95
            if name in ("persistent_a3", "b3_without_tof", "ground_acquisition") and (k >= 45 or name == "ground_acquisition"):
                raw[2] += .6
            if name in ("near_reacquisition", "moving_reacquisition", "moving_input_gap", "far_rejection") and k >= 60:
                raw[1] += .5
            if name == "two_bad_anchors" and 45 <= k < 110:
                raw[1] += .8
                raw[2] -= .6
            height_list = heights.tolist()
            source = "measured_tof_imu"
            if name == "ground_acquisition":
                source = "ground_antenna_reference"
            if name in ("b4_without_tof", "b3_without_tof") and k >= 45:
                height_list = [None] * 4
                source = "missing"
            if name in ("near_reacquisition", "moving_reacquisition", "far_rejection") and 140 <= k < 160:
                height_list = [None] * 4
                source = "missing"
            if name in ("height_conflict", "height_veto") or (name == "height_veto_recovery" and 45 <= k < 85):
                height_list = (heights + .5).tolist()
            if name == "partial_height" and 45 <= k < 90:
                height_list[1] = None
                raw[2] += .6
            row = dict(type="frame", cycle_start_us=start, time_us=max(stamps), sample_time_us=stamps,
                       raw_slant_m=raw.tolist(), sample_height_m=height_list, height_source=source)
            rows.append(row)
            cycle = Cycle(k, start, max(stamps), 15, raw, stamps, ["ok"]*4, list(range(4)))
            before = time.perf_counter_ns()
            b = window.process(cycle, k)
            d = candidate.process(dict(time_us=max(stamps), cal_slant_m=(raw-bias).tolist(),
                sample_time_us=stamps, sample_height_m=height_list, height_m=height_list[-1],
                height_source=source if source != "missing" else None, models={"B": b}))
            all_py_ms.append((time.perf_counter_ns()-before)/1e6)
            expected.append(dict(ok=d["ok"], source=d["source"], reason=d["reason"],
                xy_m=d.get("xy_m"), best_score_m=d.get("best_score_m"),
                best_excluded_anchor=d.get("best_excluded_anchor", 0),
                b4_ok=b["ok"], b4_xy_m=b.get("xy_m")))
        input_path = folder / "frames.jsonl"
        input_path.write_text("".join(dumps(row)+"\n" for row in rows), encoding="utf-8")
        (folder / "python.jsonl").write_text("".join(dumps(row)+"\n" for row in expected), encoding="utf-8")
        run = subprocess.run([str(args.binary.resolve()), str(folder/"config.json"), str(input_path)],
                             capture_output=True, text=True, encoding="utf-8", check=True)
        (folder / "cpp.jsonl").write_text(run.stdout, encoding="utf-8")
        actual = [json.loads(line) for line in run.stdout.splitlines()]
        if len(actual) != len(expected):
            raise AssertionError(f"{name}: output row count differs")
        for i, (want, record) in enumerate(zip(expected, actual)):
            got = record["measurement"]
            all_cpp_ms.append(record["compute_ms"])
            for key in ("ok", "source", "reason", "best_excluded_anchor"):
                if want[key] != got[key]:
                    summary["failures"].append(dict(case=name, row=i, field=key, python=want[key], cpp=got[key]))
            for key, max_key in (("xy_m", "max_xy_difference_m"), ("best_score_m", "max_score_difference_m")):
                if want[key] is None or got[key] is None:
                    if (want[key] is None) != (got[key] is None):
                        summary["failures"].append(dict(case=name, row=i, field=key, reason="null mismatch"))
                    continue
                error = float(np.max(np.abs(np.asarray(want[key])-np.asarray(got[key]))))
                summary[max_key] = max(summary[max_key], error)
                if error > 1e-7:
                    summary["failures"].append(dict(case=name, row=i, field=key, difference=error))
            if got["B"]["ok"] != want["b4_ok"]:
                summary["failures"].append(dict(case=name, row=i, field="B.ok"))
            if want["b4_xy_m"] is not None and got["B"]["xy_m"] is not None:
                if np.max(np.abs(np.asarray(want["b4_xy_m"])-got["B"]["xy_m"])) > 1e-7:
                    summary["failures"].append(dict(case=name, row=i, field="B.xy_m"))
        summary["cases"].append(dict(name=name, rows=len(rows), accepted=sum(x["ok"] for x in expected),
            reasons=dict(Counter(x["reason"] for x in expected)),
            input_sha256=hashlib.sha256(input_path.read_bytes()).hexdigest()))
    summary["timing"] = {name: dict(median_ms=float(np.median(values)), p95_ms=float(np.percentile(values, 95)),
                                    max_ms=max(values)) for name, values in (("python", all_py_ms), ("cpp", all_cpp_ms))}
    summary["timing_scope"] = "local PC synthetic kernel only; excludes acquisition/ROS/FC; not Jetson speed"
    summary["rows"] = len(all_py_ms)
    summary["passed"] = not summary["failures"]
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    print(dumps({k: v for k, v in summary.items() if k != "cases"}))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
