#!/usr/bin/env python3
"""Validate UWB tag JSONL packets received over USB serial on Jetson."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from typing import Any


SCHEMA_VERSION = 1
DEFAULT_ANCHOR_COUNT = 4
EXPECTED_ORDER = ["A1", "A2", "A3", "A4"]


class ContractError(ValueError):
    pass


def _require(packet: dict[str, Any], name: str, expected_type: type) -> Any:
    if name not in packet:
        raise ContractError(f"missing field: {name}")
    value = packet[name]
    if expected_type is int and isinstance(value, bool):
        raise ContractError(f"{name} must be int, not bool")
    if not isinstance(value, expected_type):
        raise ContractError(f"{name} must be {expected_type.__name__}")
    return value


def _require_array(packet: dict[str, Any], name: str, length: int) -> list[Any]:
    value = _require(packet, name, list)
    if len(value) != length:
        raise ContractError(f"{name} length must be {length}, got {len(value)}")
    return value


def validate_status(packet: dict[str, Any]) -> dict[str, Any]:
    if _require(packet, "type", str) != "uwb_raw_status":
        raise ContractError("not a uwb_raw_status packet")
    if _require(packet, "schema", int) != SCHEMA_VERSION:
        raise ContractError("unsupported status schema")
    _require(packet, "event", str)
    _require(packet, "tag_id", str)
    _require(packet, "firmware", str)
    _require(packet, "uwb_ready", bool)
    if _require(packet, "serial_baud", int) != 921600:
        raise ContractError("unexpected serial baud")
    anchor_count = _require(packet, "anchor_count", int)
    anchor_order = _require_array(packet, "anchor_order", anchor_count)
    if anchor_count != DEFAULT_ANCHOR_COUNT or anchor_order != EXPECTED_ORDER:
        raise ContractError(
            f"unsupported anchor contract: count={anchor_count}, order={anchor_order}"
        )
    if _require(packet, "anchor_layout_id", str) != "warehouse-rectangle-6p3x4p6-z2p2-20261004":
        raise ContractError("unexpected anchor layout")
    if _require(packet, "clock_domain", str) != "esp32_monotonic_boot_us":
        raise ContractError("unsupported tag clock domain")
    if _require(packet, "temporal_filter_applied", bool):
        raise ContractError("tag temporal filter must remain disabled")
    if _require(packet, "raw_xy_method", str) != "linear_ls_unfiltered_equal_anchor_z":
        raise ContractError("unexpected raw XY method")
    _require(packet, "display_ready", bool)
    _require(packet, "lora_ready", bool)
    if _require(packet, "lora_frequency_hz", int) != 915_000_000:
        raise ContractError("unexpected LoRa carrier frequency")
    if _require(packet, "lora_good_fix_period_ms", int) != 500:
        raise ContractError("unexpected good-fix LoRa period")
    if _require(packet, "lora_bad_fix_period_ms", int) != 250:
        raise ContractError("unexpected bad-fix LoRa period")
    return packet


def validate_cycle(
    packet: dict[str, Any], anchor_count: int = DEFAULT_ANCHOR_COUNT
) -> dict[str, Any]:
    if _require(packet, "type", str) != "uwb_raw_cycle":
        raise ContractError("not a uwb_raw_cycle packet")
    if _require(packet, "schema", int) != SCHEMA_VERSION:
        raise ContractError("unsupported cycle schema")
    _require(packet, "tag_id", str)
    seq = _require(packet, "seq", int)
    if not 0 <= seq <= 0xFFFFFFFF:
        raise ContractError("seq outside uint32 range")
    cycle_start = _require(packet, "cycle_start_us", int)
    cycle_end = _require(packet, "cycle_end_us", int)
    if cycle_start < 0 or cycle_end < cycle_start:
        raise ContractError("invalid cycle time interval")

    valid_mask = _require(packet, "valid_mask", int)
    if not 0 <= valid_mask < (1 << anchor_count):
        raise ContractError("valid_mask outside anchor bit range")

    ranges = _require_array(packet, "raw_slant_m", anchor_count)
    sample_times = _require_array(packet, "sample_time_us", anchor_count)
    failures = _require_array(packet, "failure", anchor_count)
    attempts = _require_array(packet, "attempt_count", anchor_count)
    raw_xy_valid = _require(packet, "raw_xy_valid", bool)
    raw_x = packet.get("raw_x_m")
    raw_y = packet.get("raw_y_m")
    raw_xy_anchor_mask = _require(packet, "raw_xy_anchor_mask", int)
    if not 0 <= raw_xy_anchor_mask < (1 << anchor_count):
        raise ContractError("raw_xy_anchor_mask outside anchor bit range")
    if raw_xy_anchor_mask & ~valid_mask:
        raise ContractError("raw_xy_anchor_mask cannot include invalid anchors")
    if raw_xy_valid:
        if bin(raw_xy_anchor_mask).count("1") < 3:
            raise ContractError("valid raw XY requires at least three anchors")
        for name, value in (("raw_x_m", raw_x), ("raw_y_m", raw_y)):
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ContractError(f"{name} must be finite for valid raw XY")
    elif raw_x is not None or raw_y is not None:
        raise ContractError("invalid raw XY requires null coordinates")
    for index in range(anchor_count):
        raw_range = ranges[index]
        sample_time = sample_times[index]
        failure = failures[index]
        if raw_range is not None and (
            not isinstance(raw_range, (int, float))
            or isinstance(raw_range, bool)
            or not math.isfinite(raw_range)
        ):
            raise ContractError(f"raw_slant_m[{index}] must be finite or null")
        if sample_time is not None and (
            not isinstance(sample_time, int) or isinstance(sample_time, bool)
        ):
            raise ContractError(f"sample_time_us[{index}] must be uint64 or null")
        if not isinstance(failure, str):
            raise ContractError(f"failure[{index}] must be string")
        if (
            not isinstance(attempts[index], int)
            or isinstance(attempts[index], bool)
            or not 0 <= attempts[index] <= 2
        ):
            raise ContractError(f"attempt_count[{index}] must be from 0 to 2")
        if valid_mask & (1 << index):
            if raw_range is None or sample_time is None or failure != "ok":
                raise ContractError(
                    f"valid anchor A{index + 1} requires range, time, and failure=ok"
                )
    return packet


def validate_lora_rx(packet: dict[str, Any]) -> dict[str, Any]:
    """Validate an SX1262 downlink forwarded by the tag over USB serial."""
    if _require(packet, "type", str) != "lora_rx":
        raise ContractError("not a lora_rx packet")
    if _require(packet, "schema", int) != SCHEMA_VERSION:
        raise ContractError("unsupported LoRa RX schema")
    _require(packet, "tag_id", str)
    rssi = _require(packet, "rssi_dbm", int)
    if not -32768 <= rssi <= 32767:
        raise ContractError("rssi_dbm outside int16 range")
    snr = _require(packet, "snr_db", int)
    if not -128 <= snr <= 127:
        raise ContractError("snr_db outside int8 range")
    _require(packet, "payload", str)
    return packet


def validate_pose_ack(packet: dict[str, Any]) -> dict[str, Any]:
    if _require(packet, "type", str) != "jetson_pose_ack":
        raise ContractError("not a jetson_pose_ack packet")
    if _require(packet, "schema", int) != SCHEMA_VERSION:
        raise ContractError("unsupported pose ACK schema")
    _require(packet, "tag_id", str)
    accepted = _require(packet, "accepted", bool)
    pose_seq = packet.get("pose_seq")
    if accepted:
        if not isinstance(pose_seq, int) or isinstance(pose_seq, bool):
            raise ContractError("accepted pose ACK requires integer pose_seq")
        if not 0 <= pose_seq <= 0xFFFFFFFF:
            raise ContractError("pose_seq outside uint32 range")
    elif pose_seq is not None:
        raise ContractError("rejected pose ACK requires null pose_seq")
    fix = _require(packet, "fix", bool)
    lora_eligible = _require(packet, "lora_eligible", bool)
    if lora_eligible and (not accepted or not fix):
        raise ContractError("lora_eligible requires an accepted valid fix")
    _require(packet, "reason", str)
    return packet


def encode_jetson_pose(
    *,
    tag_id: str,
    seq: int,
    sample_time_us: int,
    fix: bool,
    quality: int,
    x_m: float | None = None,
    y_m: float | None = None,
    reason: str = "ok",
) -> bytes:
    """Encode one Jetson result for stream.write() on the same serial link."""
    if not isinstance(tag_id, str) or not tag_id or len(tag_id) > 15:
        raise ContractError("tag_id must contain 1-15 characters")
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq <= 0xFFFFFFFF:
        raise ContractError("seq outside uint32 range")
    if (
        isinstance(sample_time_us, bool)
        or not isinstance(sample_time_us, int)
        or sample_time_us < 0
    ):
        raise ContractError("sample_time_us must be uint64")
    if not isinstance(fix, bool):
        raise ContractError("fix must be bool")
    if isinstance(quality, bool) or not isinstance(quality, int) or not 0 <= quality <= 100:
        raise ContractError("quality must be an integer from 0 to 100")
    if (
        not isinstance(reason, str)
        or len(reason) > 31
        or "\\" in reason
        or '"' in reason
    ):
        raise ContractError("reason must be at most 31 characters without JSON escapes")
    packet: dict[str, Any] = {
        "type": "jetson_pose",
        "schema": SCHEMA_VERSION,
        "tag_id": tag_id,
        "seq": seq,
        "sample_time_us": sample_time_us,
        "fix": fix,
        "quality": quality,
        "reason": reason,
    }
    if fix:
        if (
            not isinstance(x_m, (int, float))
            or isinstance(x_m, bool)
            or not math.isfinite(x_m)
            or not isinstance(y_m, (int, float))
            or isinstance(y_m, bool)
            or not math.isfinite(y_m)
        ):
            raise ContractError("valid fix requires finite x_m and y_m")
        packet["x_m"] = x_m
        packet["y_m"] = y_m
    encoded = (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")
    if len(encoded) > 255:
        raise ContractError("encoded jetson_pose exceeds tag line buffer")
    return encoded


@dataclass
class StreamState:
    status_ready: bool = False
    tag_id: str | None = None
    last_seq: int | None = None
    cycles: int = 0
    missing_cycles: int = 0

    def consume(self, packet: dict[str, Any]) -> str:
        packet_type = packet.get("type")
        if packet_type == "uwb_tdma_epoch":
            # Sideband metadata does not create a range or reset RAW history.
            # Full deadline/RAW pairing is checked by analyze_capture.py.
            if _require(packet, "schema", int) != SCHEMA_VERSION:
                raise ContractError("unsupported TDMA schema")
            tag_id = _require(packet, "tag_id", str)
            if self.tag_id is not None and tag_id != self.tag_id:
                raise ContractError("TDMA tag_id differs from status tag_id")
            return "tdma_epoch"
        if packet_type == "uwb_raw_status":
            validate_status(packet)
            previous_tag_id = self.tag_id
            self.status_ready = packet["uwb_ready"]
            self.tag_id = packet["tag_id"]
            if packet["event"] == "boot" or (
                previous_tag_id is not None and previous_tag_id != self.tag_id
            ):
                self.last_seq = None
            return "status"
        if packet_type == "lora_rx":
            validate_lora_rx(packet)
            if self.tag_id is not None and packet["tag_id"] != self.tag_id:
                raise ContractError("LoRa RX tag_id differs from status tag_id")
            return "lora_rx"
        if packet_type == "jetson_pose_ack":
            validate_pose_ack(packet)
            if self.tag_id is not None and packet["tag_id"] != self.tag_id:
                raise ContractError("pose ACK tag_id differs from status tag_id")
            return "pose_ack"
        if packet_type != "uwb_raw_cycle":
            raise ContractError(f"unsupported packet type: {packet_type!r}")
        if not self.status_ready:
            raise ContractError("cycle received before ready status")
        validate_cycle(packet)
        if packet["tag_id"] != self.tag_id:
            raise ContractError("cycle tag_id differs from status tag_id")
        seq = packet["seq"]
        if self.last_seq is not None:
            delta = (seq - self.last_seq) & 0xFFFFFFFF
            if delta == 0:
                raise ContractError("duplicate cycle sequence")
            if delta > 0x7FFFFFFF:
                raise ContractError("cycle sequence moved backwards")
            self.missing_cycles += delta - 1
        self.last_seq = seq
        self.cycles += 1
        return "cycle"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--stats-every", type=int, default=100)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import serial
    except ImportError:
        print("pyserial is required: python3 -m pip install pyserial", file=sys.stderr)
        return 2

    state = StreamState()
    with serial.Serial(args.port, args.baud, timeout=1.0) as stream:
        while True:
            raw_line = stream.readline()
            if not raw_line:
                continue
            try:
                packet = json.loads(raw_line.decode("utf-8"))
                kind = state.consume(packet)
                if kind == "cycle" and state.cycles % args.stats_every == 0:
                    print(
                        json.dumps(
                            {
                                "cycles": state.cycles,
                                "missing_cycles": state.missing_cycles,
                                "last_seq": state.last_seq,
                            }
                        ),
                        flush=True,
                    )
            except (UnicodeDecodeError, json.JSONDecodeError, ContractError) as exc:
                print(f"UWB packet rejected: {exc}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
