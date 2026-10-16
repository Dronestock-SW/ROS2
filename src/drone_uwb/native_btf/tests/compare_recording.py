"""Compare normalized recorded B_TF inputs, without replaying ROS or FC output.

The selected capture may begin mid-flight. Both kernels start with EMPTY
history; agreement does not reconstruct the original live output or certify XY.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("binary", "python-root", "config", "recording", "output"):
        p.add_argument("--"+name, type=Path, required=True)
    args = p.parse_args()
    sys.path.insert(0, str(args.python_root.resolve()))
    from drone_uwb.acquisition.validation import Cycle
    from drone_uwb.processing.experiments.h80_b import BSettings, H80Window
    from drone_uwb.processing.experiments.tof_subset import ToFTrackedSubsetCandidate
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    window = H80Window(cfg["anchors_xyz_m"], cfg["bias_m"], BSettings(**cfg["B"]))
    candidate = ToFTrackedSubsetCandidate(cfg, **cfg.get("tof_subset", {}))
    normalized, expected = [], []
    skipped = 0
    for line in args.recording.read_text(encoding="utf-8").splitlines():
        envelope = json.loads(line)
        if envelope.get("topic") != "/uwb/btf_decision":
            continue
        d = json.loads(envelope["data"]["data"])
        required = ("time_us", "sample_time_us", "raw_slant_m", "sample_height_m", "models")
        if not all(k in d for k in required):
            skipped += 1
            continue
        if d.get("layout_id") != cfg.get("layout_id"):
            raise ValueError("capture/config layout mismatch")
        times = d["sample_time_us"]
        source = d.get("height_source") or "missing"
        frame = dict(type="frame", cycle_start_us=min(times), time_us=d["time_us"],
                     sample_time_us=times, raw_slant_m=d["raw_slant_m"],
                     sample_height_m=d["sample_height_m"], height_source=source)
        normalized.append(frame)
        cycle = Cycle(len(normalized), min(times), d["time_us"], 15,
                      np.array(d["raw_slant_m"]), times, ["ok"]*4, list(range(4)))
        b = window.process(cycle, len(normalized))
        latest = times.index(max(times))
        result = candidate.process(dict(time_us=d["time_us"], sample_time_us=times,
            cal_slant_m=(cycle.raw-np.array(cfg["bias_m"])).tolist(),
            sample_height_m=d["sample_height_m"], height_m=d["sample_height_m"][latest],
            height_source=None if source == "missing" else source, models={"B": b}))
        expected.append(result)
    if not normalized:
        raise ValueError("no normalized B_TF inputs in capture")
    args.output.mkdir(parents=True, exist_ok=True)
    src = args.output/"normalized.jsonl"
    src.write_text("".join(json.dumps(x, allow_nan=False)+"\n" for x in normalized), encoding="utf-8")
    run = subprocess.run([str(args.binary.resolve()), str(args.config.resolve()), str(src.resolve())],
                         check=True, capture_output=True, text=True, encoding="utf-8")
    (args.output/"cpp.jsonl").write_text(run.stdout, encoding="utf-8")
    actual = [json.loads(x)["measurement"] for x in run.stdout.splitlines()]
    failures, delta = [], 0.
    if len(actual) != len(expected):
        raise AssertionError("row count mismatch")
    for i, (a, b) in enumerate(zip(expected, actual)):
        for key in ("ok", "reason", "source"):
            if a[key] != b[key]:
                failures.append(dict(row=i, field=key, python=a[key], cpp=b[key]))
        if a["xy_m"] is not None and b["xy_m"] is not None:
            error = float(np.max(np.abs(np.array(a["xy_m"])-b["xy_m"])))
            delta = max(delta, error)
            if error > 1e-7:
                failures.append(dict(row=i, field="xy_m", difference=error))
        elif (a["xy_m"] is None) != (b["xy_m"] is None):
            failures.append(dict(row=i, field="xy_m", reason="null mismatch"))
    summary = dict(passed=not failures, rows=len(actual), skipped_decisions=skipped,
        max_xy_difference_m=delta, failures=failures,
        source_sha256=hashlib.sha256(args.recording.read_bytes()).hexdigest(),
        normalized_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
        python_reasons=dict(Counter(x["reason"] for x in expected)),
        scope="Recorded normalized inputs, empty initial histories; not original live output or physical accuracy")
    (args.output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(summary))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
