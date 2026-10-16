#!/usr/bin/env python3
"""Compile and run production tag RF and USB runtime with host-only adapters.

Every case runs in a fresh process, including production static state. Windows
uses WSL g++ when no native compiler is available. No serial/network/board API
is imported or accessed. Passing proves host control-flow contracts only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def run(command: list[str], *, timeout: int = 30) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, capture_output=True, text=True, encoding="utf-8",
                          errors="replace", timeout=timeout, check=False)


def wsl_path(path: Path) -> str:
    result = run(["wsl.exe", "-e", "wslpath", "-a", str(path.resolve())])
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "WSL path conversion failed")
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="g++", help="host C++17 compiler")
    parser.add_argument("--suite", choices=("all", "main", "runtime"), default="all")
    parser.add_argument("--report", type=Path, help="optional UTF-8 JSON result")
    args = parser.parse_args()
    if os.name == "nt" and not shutil.which(args.compiler):
        command = ["wsl.exe", "-e", "python3", wsl_path(Path(__file__)),
                   "--compiler", args.compiler, "--suite", args.suite]
        if args.report is not None:
            command += ["--report", wsl_path(args.report)]
        return subprocess.run(command, check=False).returncode

    package = Path(__file__).resolve().parents[1]
    suites = {"main": ("native", "test_tag_main.cpp"),
              "runtime": ("runtime_native", "test_tag_runtime.cpp")}
    if args.suite != "all":
        suites = {args.suite: suites[args.suite]}
    files = [package / "tag/src/main.cpp", package / "tag/src/SingleTagRuntime.cpp",
             package / "tag/include/SingleTagRuntime.h", package / "tag/include/SingleTagDiagnostics.h",
             package / "tag/include/SingleTagDiagnosticJson.h",
             package / "common/SingleTagProtocol.h"]
    for folder, source in suites.values():
        native = package / "tag/tests" / folder
        files += sorted(native.rglob("*.h")) + [native / source]
    result: dict = {
        "schema": 1,
        "evidence_type": "host_cpp_production_tag_fault_injection",
        "limits": "Fake SPI, RF and MCU clocks; no real-board, RF, UART or flight validation.",
        "compiler": run([args.compiler, "--version"]).stdout.splitlines()[0],
        "source_sha256": {str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in files},
        "cases": [],
        "compiles": {},
    }
    with tempfile.TemporaryDirectory(prefix="uwb-tag-native-") as directory:
      for suite, (folder, filename) in suites.items():
        native = package / "tag/tests" / folder
        source = native / filename
        binary = Path(directory) / (suite + (".exe" if os.name == "nt" else ""))
        command = [args.compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
                   "-DSINGLE_TAG_RATE_HZ=50", "-DSINGLE_TAG_BENCH_CONTROLS=0",
                   "-I", str(native / "fakes"), "-I", str(package / "tag/include"),
                   str(source), "-o", str(binary)]
        compiled = run(command, timeout=60)
        result["compiles"][suite] = {"command": command, "returncode": compiled.returncode,
                                    "stdout": compiled.stdout, "stderr": compiled.stderr}
        if compiled.returncode == 0:
            listing = run([str(binary), "--list"])
            if listing.returncode:
                raise RuntimeError(listing.stderr)
            for name in listing.stdout.splitlines():
                executed = run([str(binary), name], timeout=10)
                case = {"suite": suite, "name": name, "passed": executed.returncode == 0,
                        "returncode": executed.returncode, "stdout": executed.stdout.strip(),
                        "stderr": executed.stderr.strip()}
                if name == "diagnostic_serialization" and case["passed"]:
                    try:
                        messages = [json.loads(line) for line in executed.stdout.splitlines() if line.startswith("{")]
                        assert len(messages) == 4, "expected attempt, recovery, RAW fields and maximum-width JSON"
                        attempt, recovery, raw, wide = messages
                        assert attempt["type"] == "uwb_rf_attempt" and attempt["cause"] == 3 and attempt["failure"] == 4
                        assert attempt["poll_tx_confirmed"] is True
                        assert recovery["type"] == "uwb_rf_recovery" and recovery["event"] == "init_success"
                        assert raw["first_failure"][3] == 4 and raw["first_cause"][3] == 3 and raw["cause"][3] == 14
                        assert raw["first_transaction"][3] != raw["transaction"][3]
                        assert wide["start_us"] == (1 << 64) - 1 and wide["transaction"] == (1 << 32) - 1
                        case["parsed_messages"] = messages
                    except (ValueError, KeyError, AssertionError) as error:
                        case["passed"] = False
                        case["stderr"] = f"JSON contract validation failed: {error}"
                if suite == "runtime" and case["passed"]:
                    try:
                        messages = [json.loads(line) for line in executed.stdout.splitlines() if line.startswith("{")]
                        if name == "runtime_json_sizes":
                            assert len(messages) == 4
                            assert messages[0]["failure"][3] == 0 and messages[0]["first_failure"][3] == 4
                            assert messages[1]["raw_slant_m"][3] is None and messages[1]["sample_time_us"][3] is None
                            assert messages[3]["diag_attempts"] == 0xffffffff and messages[3]["diag_failures"][14] == 0
                        elif name == "runtime_raw_priority":
                            assert [m["type"] for m in messages] == ["uwb_raw_status", "uwb_raw_cycle", "uwb_rf_attempt"]
                        elif name == "runtime_stale_and_old_session":
                            assert [m["session"] for m in messages if m["type"] == "uwb_rf_attempt"] == [22]
                        elif name == "runtime_stalled_usb":
                            raw = [m for m in messages if m["type"] == "uwb_raw_cycle"]
                            assert len(raw) == 1 and raw[0]["seq"] == 24
                        case["parsed_messages"] = messages
                    except (ValueError, KeyError, AssertionError) as error:
                        case["passed"] = False
                        case["stderr"] = f"runtime JSON contract validation failed: {error}"
                result["cases"].append(case)
                print(case["stdout"].splitlines()[-1] if case["passed"] else case["stderr"], flush=True)
        else:
            print(compiled.stderr, file=sys.stderr)
    result["passed"] = (bool(result["cases"]) and all(c["passed"] for c in result["cases"])
                        and all(c["returncode"] == 0 for c in result["compiles"].values()))
    result["passed_count"] = sum(c["passed"] for c in result["cases"])
    result["case_count"] = len(result["cases"])
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{result['passed_count']}/{result['case_count']} host cases passed; real-board validation not performed.")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    raise SystemExit(main())
