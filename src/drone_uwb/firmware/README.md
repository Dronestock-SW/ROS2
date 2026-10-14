# Preserved PX4 Firmware
This index identifies firmware artifacts saved for field-work recovery.
Read the linked field report before selecting or installing an image.

## Prepared R3 Candidate, 2026-10-14

R3 adds independent horizontal LiDAR velocity aiding, protocol marker 2.
It retains CRSF and observes only north/east velocity. It is not installed.
The actual FC still has the R2 image below, with protocol marker 1.

| Field | Value |
|---|---|
| Candidate | `PX4_v1.17.0_Pixhawk6C_CRSF_LIDAR2D_INDEPENDENT_R3.px4` |
| SHA256 | `d3be11c1ba53243d283784dfe0a0348146f434a090bd878312a66e326f86d075` |
| Image size / board limit | 1,957,756 / 1,966,080 bytes |
| Hardware patch | [R3 source](../tools/px4_lidar2d_independent_hardware.patch) |
| Results | [Independent aiding candidate](../../../docs/report/lidar_independent_aiding_20261014.md) |
| Reproduction and physical checks | [Runbook](../../../docs/runbooks/lidar_independent_aiding.md) |

The binary is preserved outside Git in the task's `.review` artifact directory.
Simulation and estimator tests do not qualify physical fusion or flight.

## Installed R2 Image, 2026-10-13

| Field | Value |
|---|---|
| File | `PX4_v1.17.0_Pixhawk6C_CRSF_LIDAR2D_R2.px4` |
| Target | Pixhawk6C / FMUv6C, board ID 56 |
| Base PX4 commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Changes | CRSF driver, LiDAR2D hardware patch, used parameter version marker |
| SHA256 | `d52d045bdc8f474237a376f584c6c2a58fddc0b1b6ef12e98c773d0750bf628b` |
| Installation evidence | CRC verified on the field FC; not flight qualification |

The patch is [px4_lidar2d_hardware.patch](../tools/px4_lidar2d_hardware.patch).
The build, parameter preservation and recovery evidence are in the
[field report](../../../docs/report/lidar2d_crsf_flash_20261013.md).
The [day-end handoff](../../../docs/report/day_end_handoff_20261013.md)
states what remains unverified.

The older `px4_fmu-v6c_lidar2d_20261012.px4` is retained as historical evidence.
It is not the R2 image installed on 2026-10-13.
Neither image implies that LiDAR observations are being fused.
Do not restore a complete parameter file merely to match an image:
vehicle calibrations and current safety settings must be preserved.
