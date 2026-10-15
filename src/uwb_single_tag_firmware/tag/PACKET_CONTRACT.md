# TDMA radiofix3 receiver compatibility (2026-10-06)

The packaged Jetson example accepts the current layout ID
`warehouse-rectangle-6p3x4p6-z2p2-20261004` and dispatches `uwb_tdma_epoch`
separately from RAW. TDMA metadata cannot create a measurement or reset RAW
sequence history. Use `scripts/analyze_capture.py` for paired deadline checks.
The stored layout is not a survey of the current bench placement.

# TDMA v0.3 display update (2026-10-06)

Display mode is tag_name_link_status_async. A separate core initializes and
updates the OLED. Fresh deadline-valid 4/4 ranging shows TAG A or TAG B;
no such sample within 1500ms shows LOST CONNECT. XY quality is independent
of the connection display, including arbitrary anchor placement tests.

## TDMA v0.2 additions (2026-10-06)

The original RAW cycle field names and unmodified slant ranges are preserved.
In v0.2, raw_xy_valid additionally requires all four anchors and all six
squared-range pair residuals <=0.15 m. Rejected XY coordinates are null.
The 0.15 m value is a consistency threshold, not a position accuracy bound.
Raw range validity remains independent of this derived-coordinate gate.

Each RAW cycle is followed by uwb_tdma_epoch with the same tag_id and seq.
It includes xy_quality_valid, xy_quality_reason and xy_pair_residual_m.
Missing/invalid geometry has a null residual. Consumers must dispatch these
messages separately; they are not malformed RAW cycles or new measurements.
A passed quality flag is not proof of LOS or accurate position. Consistent
multi-link biases can pass. ToF is connected to Pixhawk, not directly to Tag.

# UWB Tag <-> Jetson and LoRa packet contract

## Transport

- Link: ESP32-S3 USB serial
- Format: UTF-8 JSON Lines; one JSON object per line
- Baud rate: 921600 with a 4096-byte asynchronous tag TX buffer
- Schema: 1
- Clock: ESP32 monotonic boot-time microseconds
- Packet order: a valid `uwb_raw_status` must be received before UWB cycles are used
- Direction: tag-to-Jetson UWB RAW and LoRa downlink; Jetson-to-tag filtered pose

## `uwb_raw_status`

Sent at boot, after UWB recovery, and every 5 seconds as a heartbeat. A Jetson
process that starts after the tag therefore becomes synchronized without a
manual tag reset.

| Field | Type | Meaning |
|---|---|---|
| `type` | string | Always `uwb_raw_status` |
| `schema` | integer | Contract version; currently 1 |
| `event` | string | `boot`, `heartbeat`, `recovered`, `recovery_failed`, or `recovery_requested` |
| `tag_id` | string | Tag identity |
| `firmware` | string | Tag firmware version |
| `uwb_ready` | boolean | Whether following cycle data may be consumed |
| `serial_baud` | integer | Serial link speed |
| `anchor_count` | integer | Must be 4 |
| `anchor_order` | string[4] | Must be `[A1,A2,A3,A4]` |
| `anchor_layout_id` | string | Layout used by the tag RAW X/Y solver; currently `warehouse-rectangle-6p3x4p6-z2p2-20261004` |
| `clock_domain` | string | `esp32_monotonic_boot_us` |
| `temporal_filter_applied` | boolean | Must remain false |
| `raw_xy_method` | string | `linear_ls_unfiltered_equal_anchor_z` |
| `display_ready` | boolean | Whether the onboard LCD initialized |
| `display_mode` | string, optional (v1.10+) | `tag_name_link_status_async` in v0.3; independent-core UWB connection display |
| `lora_ready` | boolean | Whether the independent SX1262 link initialized |
| `lora_frequency_hz` | integer | Configured LoRa carrier frequency |
| `lora_good_fix_period_ms` | integer | LoRa period for a fresh valid Jetson pose |
| `lora_bad_fix_period_ms` | integer | LoRa period for a stale/invalid/missing pose |

### Optional LoRa health fields (firmware v1.9+)

These fields extend the existing `uwb_raw_status` packet; the schema and RAW
cycle contract are unchanged. Older firmware need not provide these fields.

| Field | Meaning |
|---|---|
| `lora_hw_verified` | SX1262 private sync-word register readback passed at the last initialization; not RF/antenna verification |
| `lora_tx_busy` | Waiting for the local radio's completion callback |
| `lora_tx_attempts` | Calls made to `Radio.Send` since tag boot |
| `lora_tx_done` | Local SX1262 TX-done callbacks while a send was pending; **not ground-station ACKs** |
| `lora_tx_timeouts` | Pending transmissions timed out by the radio/library or independent watchdog |
| `lora_watchdog_timeouts` | Subset detected by the independent 3.5-second watchdog |
| `lora_recoveries` | LoRa reinitialization attempts after timeout or hardware verification failure |
| `lora_rx_packets` | LoRa payloads actually received by the tag |
| `lora_rx_errors` | LoRa receive-error callbacks |
| `lora_last_tx_done_ms` | Last local TX-done time, tag-boot milliseconds; meaningful only after a TX-done event |
| `lora_last_rx_ms` | Last RX time, tag-boot milliseconds; meaningful only after an RX event |

Counters restart on reboot; millisecond timestamps wrap after about 49.7 days.
LoRa settings, power, periodic `drone.status` payload, and Jetson freshness rules
are unchanged. A missing Jetson pose still produces `fix:false` and null XY;
this does not prevent transmission. The ground station has no automatic ACK,
so `lora_rx_packets:0` does not prove a broken uplink.

The bridge services Heltec's cooperative timer handler after hardware IRQs.
A separate 3.5-second millis-based guard recovers a stuck pending TX even if
the timer/IRQ path fails. Timers are unlinked before radio reinitialization.
Recovery can briefly delay a UWB cycle; it does not modify measured ranges.

## `uwb_raw_cycle`

Sent once per A1-A4 ranging cycle.

| Field | Type | Meaning |
|---|---|---|
| `type` | string | Always `uwb_raw_cycle` |
| `schema` | integer | Contract version; currently 1 |
| `tag_id` | string | Must match the status packet |
| `seq` | uint32 | Cycle continuity and loss detection |
| `cycle_start_us` | uint64 | Cycle start in ESP32 boot-time microseconds |
| `cycle_end_us` | uint64 | Cycle end in ESP32 boot-time microseconds |
| `valid_mask` | 4-bit integer | Bit 0-3 correspond to A1-A4 |
| `raw_slant_m` | number-or-null[4] | Unfiltered DS-TWR 3D slant ranges in metres |
| `sample_time_us` | uint64-or-null[4] | Per-anchor sample times |
| `failure` | string[4] | Protocol, ToF, and physical validity result |
| `attempt_count` | integer[4] | Per-anchor radio attempts: 1 normally, 2 after one protocol receive retry, or 0 if recovery stopped the cycle before this anchor |
| `raw_xy_valid` | boolean | Whether at least three valid, non-collinear, equal-height anchors produced an X/Y solution |
| `raw_x_m`, `raw_y_m` | number or null | Unfiltered linear least-squares position derived directly from this cycle's RAW ranges |
| `raw_xy_anchor_mask` | 4-bit integer | Anchors supplied to the RAW X/Y solution; always a subset of `valid_mask` |

`failure` does not identify a plausible but noisy spike inside the physical
range. Jetson must detect that from the time series. A clear valid bit means the
corresponding range and time are present and `failure` is `ok`.
The tag may retry a failed Response/Report reception once. This is radio
transaction recovery, not value filtering: no ranges are averaged or altered,
and the emitted value is the untouched DS-TWR result from the successful try.

`raw_x_m` and `raw_y_m` are derived diagnostics, not sensor-native RAW values.
They use spatial linear least squares only; there is no time history, Median,
Kalman, height correction, bias correction, or previous-position constraint.
For reference anchor 1 and another anchor `i`, each equation is:

```text
2(x_i-x_1)x + 2(y_i-y_1)y
  = r_1^2-r_i^2 + |A_i|^2-|A_1|^2
```

The unknown tag Z disappears only because every configured anchor has the
same surveyed height of 2.2 m. If fewer than three usable anchors remain, the
geometry is singular, or anchor heights differ, `raw_xy_valid=false` and both
coordinates are `null`. Jetson must still calculate its own filtered pose from
`raw_slant_m`; this diagnostic X/Y is for comparison and logging.

## `jetson_pose` (Jetson -> tag)

Jetson sends this after filtering and position solving. The line must fit in
255 bytes and end with `\n`. The tag does not filter or recompute this result.

| Field | Type | Meaning |
|---|---|---|
| `type` | string | Always `jetson_pose` |
| `schema` | integer | Currently 1 |
| `tag_id` | string | Must match the firmware tag ID |
| `seq` | uint32 | Jetson result sequence |
| `sample_time_us` | uint64 | Source sample time used by Jetson |
| `fix` | boolean | Whether XY is valid |
| `x_m`, `y_m` | finite number | Required only when `fix=true` |
| `quality` | integer | 0-100 Jetson quality score |
| `reason` | string | Short result reason; optional, no escapes |

Example:

```json
{"type":"jetson_pose","schema":1,"tag_id":"5","seq":120,"sample_time_us":8451000,"fix":true,"x_m":0.701,"y_m":1.803,"quality":92,"reason":"ok"}
```

The tag accepts this packet and uses it for LoRa. It never alters the supplied
filtered X/Y. The OLED shows only UWB ranging status: `TAG A` / `TAG B` requires a full
valid A1-A4 cycle within the last 1.5 seconds and an initialized UWB radio;
otherwise it shows `LOST CONNECT`. It does not show ranges or coordinates. A pose
older than 500 ms is not transmitted as a valid LoRa fix.

## `jetson_pose_ack` (tag -> Jetson)

The tag emits one ACK for every complete input line so Jetson can distinguish
transport success from a malformed or wrong-tag packet.

```json
{"type":"jetson_pose_ack","schema":1,"tag_id":"5","accepted":true,"pose_seq":120,"fix":true,"lora_eligible":true,"reason":"ok"}
```

On rejection, `accepted=false`, `pose_seq=null`, and `reason=invalid_packet`.
`lora_eligible=true` means the accepted packet contains finite X/Y and may be
used for LoRa while it remains inside the normal 500 ms freshness window.

## `drone.status` (tag -> remote LoRa receiver)

This carries the latest Jetson result, not UWB RAW ranges. A valid result is
fresh for 500 ms. Fresh fixes transmit at 2 Hz; invalid/missing/stale results
transmit at 4 Hz so a failure reaches the receiver faster.

```json
{"type":"drone.status","id":"5","seq":120,"t":8500,"fix":true,"x":0.701,"y":1.803,"q":92,"src":"jetson","uwb_seq":121,"reason":"ok"}
```

When no usable Jetson result exists, `fix=false`, `x=null`, `y=null`, `q=0`,
and `reason` explains the state, for example `jetson_pose_stale`.

## `lora_rx` (tag -> Jetson)

Every LoRa downlink received by the SX1262 is forwarded to Jetson over USB:

```json
{"type":"lora_rx","schema":1,"tag_id":"5","rssi_dbm":-73,"snr_db":8,"payload":"command text"}
```

`payload` is printable ASCII; unsupported binary bytes are replaced with `?`.

## LoRa radio settings

| Item | Value |
|---|---|
| Carrier | 915 MHz |
| Bandwidth | 125 kHz |
| Spreading factor | SF7 |
| Coding rate | 4/5 |
| Preamble | 8 symbols |
| TX power | 10 dBm |

The tag services LoRa IRQ/RX work every 100 ms at most, with an earlier service
when a 250/500 ms status transmission is due. This keeps the independent radio
responsive without switching the shared SPI bus on every UWB cycle.

## Data deliberately not emitted

- RF diagnostics and `rf_valid_mask`
- `twr_seq`
- Detailed `uwb_raw_timing`
- Median, EMA, jump, range-bias, height-corrected, Kalman, or AI results

This is the minimum preprocessing contract requested by the Jetson team. If
NLOS/AI filtering is added, a later schema must add the RF fields explicitly.

## External Jetson configuration

`config/jetson_layout.json` contains the confirmed A1-A4 deployment coordinates.
Its zero range biases mean no correction is applied; they are explicitly marked
uncalibrated and must not be interpreted as completed calibration results.

`config/jetson_layout.example.json` remains an empty template for a different
deployment. Every `null` coordinate and range bias in the template must be
replaced with surveyed/calibrated values before solving is enabled.
