# Preserved PX4 Firmware
This index identifies firmware artifacts saved for field-work recovery.
Read the linked field report before selecting or installing an image.

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
