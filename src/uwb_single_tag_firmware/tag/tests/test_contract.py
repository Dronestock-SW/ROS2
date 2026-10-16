import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "jetson" / "uwb_serial_receiver.py"
SPEC = importlib.util.spec_from_file_location("uwb_serial_receiver", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


STATUS = {
    "type": "uwb_raw_status",
    "schema": 1,
    "event": "boot",
    "tag_id": "5",
    "firmware": "uwb-tag-for-jetson-v1",
    "uwb_ready": True,
    "serial_baud": 921600,
    "anchor_count": 4,
    "anchor_order": ["A1", "A2", "A3", "A4"],
    "anchor_layout_id": "warehouse-rectangle-6p3x4p6-z2p2-20261004",
    "clock_domain": "esp32_monotonic_boot_us",
    "temporal_filter_applied": False,
    "raw_xy_method": "linear_ls_unfiltered_equal_anchor_z",
    "display_ready": True,
    "lora_ready": True,
    "lora_frequency_hz": 915000000,
    "lora_good_fix_period_ms": 500,
    "lora_bad_fix_period_ms": 250,
}

CYCLE = {
    "type": "uwb_raw_cycle",
    "schema": 1,
    "tag_id": "5",
    "seq": 1,
    "cycle_start_us": 1_000_000,
    "cycle_end_us": 1_020_000,
    "valid_mask": 15,
    "raw_slant_m": [1.0, 2.0, 3.0, 4.0],
    "sample_time_us": [1_005_000, 1_010_000, 1_015_000, 1_020_000],
    "failure": ["ok", "ok", "ok", "ok"],
    "attempt_count": [1, 1, 1, 1],
    "raw_xy_valid": True,
    "raw_x_m": 0.7,
    "raw_y_m": 1.8,
    "raw_xy_anchor_mask": 15,
}

LORA_RX = {
    "type": "lora_rx",
    "schema": 1,
    "tag_id": "5",
    "rssi_dbm": -73,
    "snr_db": 8,
    "payload": '{"type":"command","action":"hold"}',
}

POSE_ACK = {
    "type": "jetson_pose_ack",
    "schema": 1,
    "tag_id": "5",
    "accepted": True,
    "pose_seq": 120,
    "fix": True,
    "lora_eligible": True,
    "reason": "ok",
}


class ContractTest(unittest.TestCase):
    def test_tdma_sideband_preserves_raw_sequence_and_identity(self):
        state = MODULE.StreamState()
        state.consume(dict(STATUS))
        state.consume(dict(CYCLE))
        epoch = dict(type="uwb_tdma_epoch", schema=1, tag_id="5")
        self.assertEqual(state.consume(epoch), "tdma_epoch")
        self.assertEqual(state.consume(dict(CYCLE, seq=2)), "cycle")
        self.assertEqual((state.cycles, state.missing_cycles), (2, 0))
        with self.assertRaises(MODULE.ContractError):
            state.consume(dict(epoch, tag_id="6"))
        with self.assertRaises(MODULE.ContractError):
            state.consume(dict(epoch, schema=2))

    def test_old_layout_is_rejected(self):
        with self.assertRaises(MODULE.ContractError):
            MODULE.validate_status(dict(STATUS, anchor_layout_id="warehouse-5x4p5-z2p2-v2"))

    def test_lora_health_extension_is_backward_compatible(self):
        packet = dict(STATUS, firmware="uwb-tag-for-jetson-v1.9-lora-health",
                      lora_hw_verified=True, lora_tx_busy=False,
                      lora_tx_attempts=20, lora_tx_done=20,
                      lora_tx_timeouts=0, lora_watchdog_timeouts=0,
                      lora_recoveries=0, lora_rx_packets=0, lora_rx_errors=0,
                      lora_last_tx_done_ms=5000, lora_last_rx_ms=0)
        state = MODULE.StreamState()
        self.assertEqual(state.consume(packet), "status")
        self.assertEqual(state.consume(dict(CYCLE)), "cycle")

    def test_lora_failure_does_not_disable_uwb_contract(self):
        packet = dict(STATUS, lora_ready=False, lora_hw_verified=False,
                      lora_tx_done=0, lora_tx_timeouts=1, lora_recoveries=1)
        state = MODULE.StreamState()
        self.assertEqual(state.consume(packet), "status")
        self.assertEqual(state.consume(dict(CYCLE)), "cycle")

    def test_valid_stream(self):
        state = MODULE.StreamState()
        self.assertEqual(state.consume(dict(STATUS)), "status")
        self.assertEqual(state.consume(dict(CYCLE)), "cycle")
        self.assertEqual(state.cycles, 1)

    def test_cycle_before_status_is_rejected(self):
        with self.assertRaises(MODULE.ContractError):
            MODULE.StreamState().consume(dict(CYCLE))

    def test_valid_bit_requires_complete_sample(self):
        packet = dict(CYCLE)
        packet["raw_slant_m"] = [None, 2.0, 3.0, 4.0]
        with self.assertRaises(MODULE.ContractError):
            MODULE.validate_cycle(packet)

    def test_null_is_allowed_for_invalid_anchor(self):
        packet = dict(CYCLE)
        packet["valid_mask"] = 14
        packet["raw_slant_m"] = [None, 2.0, 3.0, 4.0]
        packet["sample_time_us"] = [None, 1_010_000, 1_015_000, 1_020_000]
        packet["failure"] = ["response_timeout", "ok", "ok", "ok"]
        packet["raw_xy_anchor_mask"] = 14
        MODULE.validate_cycle(packet)

    def test_invalid_raw_xy_requires_null_coordinates(self):
        packet = dict(CYCLE)
        packet["raw_xy_valid"] = False
        with self.assertRaises(MODULE.ContractError):
            MODULE.validate_cycle(packet)

        packet["raw_x_m"] = None
        packet["raw_y_m"] = None
        packet["raw_xy_anchor_mask"] = 3
        MODULE.validate_cycle(packet)

    def test_sequence_gap_is_counted(self):
        state = MODULE.StreamState()
        state.consume(dict(STATUS))
        state.consume(dict(CYCLE))
        packet = dict(CYCLE)
        packet["seq"] = 4
        state.consume(packet)
        self.assertEqual(state.missing_cycles, 2)

    def test_status_heartbeat_preserves_sequence_tracking(self):
        state = MODULE.StreamState()
        state.consume(dict(STATUS))
        state.consume(dict(CYCLE))
        heartbeat = dict(STATUS)
        heartbeat["event"] = "heartbeat"
        state.consume(heartbeat)
        packet = dict(CYCLE)
        packet["seq"] = 3
        state.consume(packet)
        self.assertEqual(state.missing_cycles, 1)

    def test_boot_status_resets_sequence_after_tag_restart(self):
        state = MODULE.StreamState()
        state.consume(dict(STATUS))
        prior = dict(CYCLE)
        prior["seq"] = 500
        state.consume(prior)
        state.consume(dict(STATUS))
        state.consume(dict(CYCLE))
        self.assertEqual(state.last_seq, 1)

    def test_duplicate_sequence_is_rejected(self):
        state = MODULE.StreamState()
        state.consume(dict(STATUS))
        state.consume(dict(CYCLE))
        with self.assertRaises(MODULE.ContractError):
            state.consume(dict(CYCLE))

    def test_lora_downlink_is_accepted_independently(self):
        state = MODULE.StreamState()
        self.assertEqual(state.consume(dict(LORA_RX)), "lora_rx")

    def test_lora_downlink_tag_must_match_known_status(self):
        state = MODULE.StreamState()
        state.consume(dict(STATUS))
        packet = dict(LORA_RX)
        packet["tag_id"] = "other"
        with self.assertRaises(MODULE.ContractError):
            state.consume(packet)

    def test_jetson_pose_encoding_and_ack(self):
        encoded = MODULE.encode_jetson_pose(
            tag_id="5",
            seq=120,
            sample_time_us=8_451_000,
            fix=True,
            x_m=0.701,
            y_m=1.803,
            quality=92,
        )
        self.assertTrue(encoded.endswith(b"\n"))
        packet = __import__("json").loads(encoded)
        self.assertEqual(packet["type"], "jetson_pose")
        self.assertEqual(packet["x_m"], 0.701)

        state = MODULE.StreamState()
        state.consume(dict(STATUS))
        self.assertEqual(state.consume(dict(POSE_ACK)), "pose_ack")

    def test_valid_fix_requires_finite_xy(self):
        with self.assertRaises(MODULE.ContractError):
            MODULE.encode_jetson_pose(
                tag_id="5",
                seq=1,
                sample_time_us=1,
                fix=True,
                x_m=None,
                y_m=1.0,
                quality=80,
            )


if __name__ == "__main__":
    unittest.main()
