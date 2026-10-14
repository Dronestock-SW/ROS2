# First Takeoff and Ground Reset, 2026-10-13

## Actual Flight

Running source remains `b5ea7a46d6106fdd86d211ee016d537366542a7c`, recorded in `position-origin-release.json`. Main, FC parameters, 0.5 m/s ascent limit, and 1.7 m goal were not changed. No LiDAR SLAM was enabled. The assistant submitted no flight commands.

Boot: `46e75066-3284-4e69-b373-8f389246d320`; fresh alignment used operator-reported +Y heading. The operator submitted web TAKEOFF at `2026-10-13T18:42:06.334395+09:00`, observed physical drift, and took RC control.

The execution reached TAKING_OFF, not POSITION_HANDOFF/HOLD_READY. No separate START or ten-second mission hover occurred. `mission_start_xy_m` remained null; original `home_xy_m` remained `[2.2155842954380938, 2.516216905889667]`.

ULog timestamps below are FC boot time, not Jetson monotonic time:

- Primary EKF instance remained 1. Optical-flow fusion became active at 442.822 s.
- EV position fusion stopped at 447.165 s and resumed at 447.642 s.
- At 447.692 s XY reset count changed 13 -> 14, NED delta `[-0.436585, -0.270902]` m. This is an estimator reset, not a measured 51 cm physical translation.
- At reset, FC height relative to its preflight reference was approximately 1.27 m; upward speed was approximately 0.41 m/s.
- Before abort, ROS B_TF pose receipt gap reached 0.738 s, versus 0.121 s for raw UWB. FC visual-odometry receipt gap over 438-448 s reached 0.842 s; external reset counter remained 0.
- B_TF decisions included unavailable measured height and no height-consistent subset. Physical drift root cause remains unresolved. Do not infer UWB radio loss as its principal cause.
- C++ triggered `px4_local_frame_or_pose_jump`, then PILOT_OVERRIDE on pilot intervention. Sender count stopped at 201; maximum gap at abort was 0.0545 s. Report latency was not the LAND trigger.
- FC subsequently confirmed landed/DISARM. Handheld video confirms visible motion, but not a calibrated metric displacement.

## Preserved Evidence

Local root: `C:/Users/ASWPC_NTBOOK/Desktop/Drone5/outputs/ground_web_20261013`. Closed raw capture and ULog hashes match Jetson receipts. No original logs were deleted.

| File | SHA256 |
| --- | --- |
| `capture-1791884513951202644-a5b6e3e7/events.jsonl` | `829d1055c8d7e49b3c4e019560cb5bf4b278b885496b06c030412da5cc30c91e` |
| `20261013-1837-plus-y/ulog-flight-download/log-31.ulg` | `39b5a1c1073eb495e4931f205e7a0465486f060cc91e18ad2b3811bd32982166` |
| `KakaoTalk_261013_184315.mp4` | `78730370723eaf643df2d07919ecf03b6b42902278d5db6ed02eb9c240b303b2` |

## Requested Web Reset

The latest request narrowed scope to clearing web PILOT_OVERRIDE. Fresh landed/DISARM/POSCTL, no active web run, unchanged 201 setpoints, and fresh neutral RC were checked continuously for over ten seconds. Only executor, observer, local web, and telemetry children were restarted; MAVROS PID 2724, EV bridge PID 4353, and sensor services were retained.

New session: `/home/arialhanho/ROS2-review-20261008-codex/.review/fc-owned-runtime-20261013/sessions/20261013-pilot-reset-01`. Original logs and consumed-request ledger are preserved. Initial port-bind failure after shutdown had no listening process; subsequent launch succeeded without bypassing checks.

Readback confirmed `IDLE`, `can_takeoff=true`, `fc_armed=false`, `fc_landed=1`, `setpoints_sent=0`, no active run. Raw capture was briefly paused for disk headroom and restarted before the new session. This is a latch reset, not drift resolution or physical-flight validation. Optical-only automatic takeoff, Altitude takeoff, and faster ascent were discussed but NOT configured.

## Undeployed Recovery Candidate

The working tree contains ground-only PILOT_OVERRIDE recovery after ten continuous seconds of fresh landed/DISARM/POSCTL, calibrated neutral RC, stationary pose, and durable prior completion. Gaps reset the timer. Recovery itself sends no commands; a new TAKEOFF issued after reset is mandatory. Old request IDs and command tokens remain unusable. Previous outcome/home and the request ledger are preserved; RC baseline and observer intent are renewed.

The candidate passed 46 C++ replay contracts on Jetson. ROS integration validation is not yet complete. It is NOT the running field binary. Airborne override remains terminal. Do not deploy this candidate merely because the web was reset successfully.
