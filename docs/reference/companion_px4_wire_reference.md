# Companion과 PX4 wire 상세 명세

층별 연결과 수정 위치는 [로직 분석 명세](companion_px4_api.md)부터 읽는다.
이 부록은 최초 조사 시점의 필드·상태 코드 사전이다.
아래 `c66d7dd` 기준과 당시 작업 파일 조사 범위를 보존한다.
개정 본문의 `20bd603` 기준과 native/local 이륙 절차는 본문이 우선한다.
companion이 PX4의 어느 단계에 값을 넣는지 정의한다.
센서 관측·임무 명령·제어 책임을 구분할 때 읽는다.

**UWB·LiDAR는 현재 상태 추정에, 임무 목표는 제어 목표에 들어간다.**
ARM·모드 요청은 그 두 경로를 사용할 비행 상태를 바꾼다.
자세·각속도 제어와 모터별 출력 계산은 PX4가 수행한다.

| 문서 기준 | 값 |
|---|---|
| 확인일 | 2026-10-13, Asia/Seoul |
| companion 기준 | `FINAL-TEST1-SANGWON`, `c66d7dd934975c645ac1d50400e6da909990d0f0` |
| 추가 확인 | 현재 작업 폴더의 미커밋 `path_heading` 변경. 별도 표시 |
| PX4 기준 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 전용 LiDAR 확장 | `src/drone_uwb/tools/px4_lidar2d_hardware.patch` |
| 실물 대상 | PX4 FMUv6C, board ID 56 |
| 실제 배포 상태 | [10월 13일 확인 기록](../report/companion_px4_audit_20261013.md) |
| 범위 | 실물 공통 관측 경로와 FINAL 임무 경로 |

이 명세는 표에 적힌 revision의 확인 기록이다.
후속 `99a4a9b`에 포함된 native 이륙·웹 시작 변경은 검토 범위 밖이다.
`feature_uwb`에도 이 FINAL 기준 명세를 공유한다.
두 브랜치의 전체 임무 구현이 같다는 뜻은 아니다.

일반 PX4 전체 API 목록이 아니다.
이 저장소가 실제로 만드는 입력을 기준으로 한다.
실험·레거시 경로는 10절에서 구분한다.

## 1. 어느 단계에 관여하는가

companion 출력은 세 종류의 입구를 사용한다.

```text
companion                                       PX4 FC

UWB 거리 -> B_TF XY -> UWB bridge -------+--> mavlink_receiver
                                        |      -> vehicle_visual_odometry
                                        |      -> EKF2 EV 수평 위치 융합 ----+
T-mini Pro + FC IMU -> LiDAR 속도 -------+--> 전용 LiDAR 수신 분기           |
                                               -> lidar_velocity2d         |
                                               -> EKF2 aux 수평 속도 융합 -+
                                                                           v
FC 센서 IMU/기압/ToF/광류 --------------------> EKF2 ----> 현재 위치·속도·자세
                                                                           |
C++ 임무 판단 -> Python 실행기 -> XYZ/yaw 목표 -> trajectory_setpoint        |
                                                    |                      |
                                                    v                      v
                                             위치 제어 -> 속도 제어
                                                       -> 자세 목표·추력
                                                       -> 자세 제어
                                                       -> 각속도 제어
                                                       -> 제어 할당
                                                       -> 모터 출력

ARM·모드·이착륙 요청 -> vehicle_command -> Commander / Navigator
                                              -> 비행 상태·제어 활성 경로
```

| companion 요소 | 직접 넣는 값 | PX4의 첫 실질 사용 단계 | 이후 영향 |
|---|---|---|---|
| B_TF + UWB bridge | 안테나 수평 위치·분산 | EKF2 외부 위치 관측 | 추정 위치 변화 → 위치 오차와 제어 출력 변화 |
| LiDAR 속도 bridge | IMU 원점 N/E 속도·분산 | EKF2 보조 속도 관측 | 추정 속도 변화 → 속도 오차와 제어 출력 변화 |
| C++ 임무 + Python 실행기 | 목표 X/Y/Z와 yaw | OFFBOARD 목표 입력 | 위치·속도 제어 → 자세·추력 목표 |
| 모드·ARM 실행기 | 상태 전환 요청 | Commander | 어떤 제어 경로가 활성인지 변경 |
| 이착륙·재배치 실행기 | 상위 비행 동작 요청 | Commander / Navigator | FC의 자동 비행 목표 생성 |
| telemetry 요청기 | 출력할 메시지·주기 | MAVLink stream 관리 | 관측 지연·대역폭에 영향 |
| 카메라·ArUco 제안 | 판독 결과·정렬 제안 | 현재 직접 FC 입력 없음 | 연결된 임무 판단을 거쳐야 목표에 영향 |
| `/flight_state`, `/target_pose` | 상태·목표 표시 | 현재 직접 FC 입력 없음 | 기록·화면·검증용 |

센서 관측을 넣으면 모터가 곧바로 그 값을 따른다는 뜻은 아니다.
먼저 EKF가 현재 상태를 바꾼다.
활성 제어기가 그 상태와 목표의 차이를 계산한다.

## 2. 공통 통신과 좌표 계약

관측 두 경로는 raw MAVLink를 사용한다.
OFFBOARD 목표는 MAVROS `setpoint_raw` 플러그인을 사용한다.

| 항목 | 계약 |
|---|---|
| ROS 격리 | tag 5 → domain 1, tag 6 → domain 2 |
| FC 연결 소유자 | MAVROS 한 인스턴스 |
| launch 기본 연결 | `/dev/pixhawk:921600`, MAVLink 2 |
| FC 대상 | system 1 / component 1 |
| raw 관측 송신 | `/uas1/mavlink_sink`, `mavros_msgs/msg/Mavlink` |
| raw 수신 관찰 | `/uas1/mavlink_source` |
| UWB 송신 식별 | system 255 / component 196 |
| LiDAR 송신 식별 | system 255 / component 198 |
| ROS 센서·raw QoS | `qos_profile_sensor_data` |
| 공통 상태 주기 | bridge monitor 0.2초, 5Hz |
| 단위 | 위치 m, 속도 m/s, 각도 rad, 분산 해당 단위의 제곱 |

| 좌표 | 축 | 적용 위치 |
|---|---|---|
| `uwb_map` | 앵커 배치 좌표 | B_TF 원래 XY |
| ROS ENU | 동·북·위 | MAVROS 위치와 목표 입력 |
| PX4 NED | 북·동·아래 | wire 관측, PX4 추정·목표 |
| body FLU | 전·좌·위 | ROS IMU, LiDAR 장착 설정 |
| body FRD | 전·우·아래 | PX4 센서 장착 파라미터 |

UWB raw 관측은 companion에서 ENU→NED 변환을 끝낸다.
LiDAR raw 관측도 이미 N/E 값으로 만들어 보낸다.
MAVROS raw 경로에서 좌표를 다시 바꾸지 않는다.

OFFBOARD의 `PositionTarget` 수치는 ROS ENU다.
`coordinate_frame=FRAME_LOCAL_NED`라도 이 수치는 ENU다.
MAVROS `local_cb`가 wire NED로 한 번 변환한다.
미리 NED로 바꾸면 축을 두 번 변환하게 된다.
[MAVROS 변환 구현](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/src/plugins/setpoint_raw.cpp)의 `local_cb`에 해당한다.
이 외부 참조는 ROS2 상위 소스다.
실물에 설치된 MAVROS 바이너리의 동일성까지 인증한 것은 아니다.

시각은 측정 시각과 수신 시각을 구분한다.
raw 관측의 `time_usec`는 ROS 측정 시각의 μs 표현이다.
PX4 `sync_stamp`가 FC 시각으로 대응시킨다.
EKF 버퍼는 설정 지연과 내부 반 주기 보정을 적용한다.
새 수신 시각을 붙여 오래된 관측을 갱신하지 않는다.

## 3. OBS UWB 수평 위치

UWB는 PX4의 외부 위치 관측 입구를 사용한다.
이 경로의 `VISION`은 전송 분류다.
센서가 카메라라는 뜻이 아니다.

```text
/uwb/btf_pose : PoseWithCovarianceStamped
  -> LocalBridge.pose / guarded_horizontal_odometry
  -> /uas1/mavlink_sink : ODOMETRY(331), VISION(2)
  -> MavlinkReceiver::handle_message_odometry
  -> vehicle_visual_odometry
  -> EKF2::UpdateExtVisionSample -> setExtVisionData
  -> controlExternalVisionFusion -> controlEvPosFusion
  -> fuseHorizontalPosition
```

| 필드 | ROS·companion 값 | wire / PX4 계약 |
|---|---|---|
| 원본 위치 | `btf_pose.pose.position.x/y` | 앵커 지도상의 안테나 XY |
| 원본 분산 | ROS 6×6의 `[0,1,6,7]` | XY 2×2 공분산 |
| `frame_id` | 지도 확인·정렬 수행 | `MAV_FRAME_LOCAL_NED=1` |
| `child_frame_id` | 속도는 미관측 | `MAV_FRAME_BODY_FRD=12` |
| `x/y` | 지도→ENU→NED | N/E 안테나 위치 |
| `z` | 필수 3D 위치 자리값 | `0`; `EKF2_EV_CTRL=1`로 높이 융합 제외 |
| `q[4]` | 자세 미관측 | 모두 NaN |
| `vx/vy/vz` | 속도 미관측 | 모두 NaN |
| 각속도 | 미관측 | 모두 NaN |
| `pose_covariance` | NED 변환 후 포장 | `[0]=varN`, `[1]=covNE`, `[6]=varE` |
| 나머지 pose 대각 | 미사용 축 | `[11,15,18,20]=1e6` |
| `velocity_covariance` | 미관측 표시 | 첫 항 NaN |
| `quality` | 별도 점수 없음 | 0, `EKF2_EV_QMIN=0` 요구 |
| `reset_counter` | 세션 연속성 식별 | uint8 |
| `time_usec` | 원래 pose stamp | 수신 시각으로 교체하지 않음 |

표준 PX4 수신기는 위치 분산의 대각 성분을 사용한다.
wire의 `covNE`가 EKF까지 그대로 보존되지는 않는다.
LiDAR 전용 경로의 보수적 대각 변환과 차이가 있다.

| 송신 허용 조건 | 현재 구현 |
|---|---|
| 설정 | `enabled=true`, 배치·장착 확인 |
| 정렬 | 지정 method, host boot ID, FC 기준 시각 |
| 시작 | 최신 연결·DISARM·ON_GROUND |
| 관측 수명 | 기본 최대 150ms, 중복 stamp 거부 |
| 파라미터 | GPS_CTRL=0, EV_CTRL=1, EV_NOISE_MD=0, EV_QMIN=0 |
| 지연·장착 | EV_DELAY 및 EV_POS_X/Y/Z가 설정과 일치 |
| clock | 30개 유효 timesync, RTT≤20ms, 최근 값 <0.5초 |
| 단일 입력 | B_TF·상태 publisher, router 각각 하나 |
| 경쟁 관측 | vision_pose·odometry/in 별도 publisher가 있으면 차단 |
| 동작 모드 | 시작 후에는 수동·OFFBOARD 여부로 끄지 않음 |

`EKF2_EV_POS_*`는 안테나와 IMU의 차이를 보정한다.
PX4는 자세로 장착 벡터를 회전해 위치에서 뺀다.
companion이 같은 보정을 중복 적용하면 안 된다.
새 예시의 태그 FRD는 `(-0.10, 0, -0.07)m`다.
기존 core 기본값과 이전 FC 기록에는 Z=-0.10m가 남아 있다.
실제 설정과 파라미터를 일치시켜야 gate를 통과한다.
이번 문서 작업에서 FC 값을 변경하지 않았다.

EV 위치 융합은 조건에 따라 수평 위치를 초기화할 수 있다.
따라서 이 입력은 단순 화면 표시보다 영향이 크다.
입력이 사라지면 companion은 재전송을 중단한다.
PX4는 자체 관측 timeout·보조원 전환을 판단한다.

소스: [관측 계약](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/drone_uwb/integration/local_odometry.py),
[ROS bridge](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/drone_uwb/integration/ros/local_odometry_bridge.py).

## 4. OBS LiDAR 수평 속도

LiDAR는 별도의 2D 보조 속도 입구를 사용한다.
전용 펌웨어가 있어야 이 경로가 생긴다.
기존 PX4의 `LIDAR` enum만으로 활성화되지 않는다.

```text
/point_cloud + /mavros/imu/data + /mavros/downward_0
  -> MeasuredLidarMotion.observe
  -> /uas1/mavlink_sink : ODOMETRY(331), LIDAR(7)
  -> handle_message_odometry의 전용 분기
  -> lidar_velocity2d
  -> EKF2::UpdateAuxVelSample의 검사
  -> setAuxVelData -> auxvel buffer
  -> controlAuxVelFusion -> fuseHorizontalVelocity
```

| 필드 | 값 | PX4에서의 의미 |
|---|---|---|
| source | 255 / 198 | 전용 송신자 검사 |
| `estimator_type` | LIDAR=7 | 표준 EV 입구와 분리 |
| 두 frame | LOCAL_NED=1 | 속도 N/E 기준 |
| `x/y/z`, `q` | 모두 NaN | 위치·자세 미관측 |
| `vx/vy` | 유한한 N/E 속도 | FC IMU 원점에서의 수평 속도 |
| `vz` | NaN 필수 | 수직 속도 미관측 |
| 각속도 | 모두 NaN | 관측으로 보내지 않음 |
| `velocity_covariance[0,1,6]` | varN, covNE, varE | 양의 정부호 2×2 공분산 |
| 나머지 속도 분산 | NaN | 미관측 성분 |
| pose 분산 | 모두 NaN | 위치·자세 관측 없음 |
| `quality` | 1 | 통과 표시, 정량적 정확도 점수가 아님 |
| `time_usec` | 속도 계산 구간 중점 | 마지막 광선 수신 시각과 다름 |
| `reset_counter` | 0–255 | 정합 연속성 변경 통지 |

전용 uORB 메시지 `lidar_velocity2d`는 다음을 갖는다.

| 필드 | 형식·단위 |
|---|---|
| `timestamp`, `timestamp_sample` | uint64, FC μs |
| `velocity_ne` | float32[2], m/s |
| `covariance_ne` | float32[3], `[varN,covNE,varE]`, m²/s² |
| `reset_counter` | uint8 |
| `quality` | int8 |
| `source_system`, `source_component` | uint8 |
| `frame_valid` | bool |

PX4 기존 aux 버퍼는 두 개의 분산만 받는다.
전용 입력은 `D=diag(varN+abs(covNE), varE+abs(covNE))`로 바꾼다.
교차항을 조용히 버리지 않고 분산을 보수적으로 늘린다.

### 4.1 속도가 만들어지는 과정

SDK `stamps`의 광선 취득 순서를 보존한다.
FC IMU 자세·각속도로 점별 회전 영향을 보정한다.
LiDAR 장착 yaw와 레버암을 적용한다.
점군 정합 변위와 시간 간격으로 속도를 계산한다.
최종 기준점은 LiDAR 중심이 아닌 FC IMU다.

FC 위치·속도와 UWB 위치는 이 속도 추정의 입력이 아니다.
그러나 FC 자세·heading을 사용한다.
따라서 FC와 통계적으로 완전히 독립인 관측은 아니다.
현재 공분산은 임시 공학 모델이다.
독립 동적 정확도·상관관계 보정은 미검증이다.

하방 ToF는 유효 높이·기하 조건에 사용한다.
이 경로에서 ToF로 수직 속도를 만들지 않는다.
수평 관측을 `vz=0`으로 확장하지 않는다.

### 4.2 송신과 수신 조건

| 검사 단계 | 요구 조건 |
|---|---|
| companion 설정 | enabled, 장착·수평면·취득 시각 근거 |
| 현재 저장 설정 | yaw≈−91.5°, 위치 FLU=(0,0,+0.10)m; enabled=false |
| 처리 범위 | 거리 0.20–4.0m, 속도≤0.8m/s, tilt≤10° |
| 높이 범위 | 기본 0.10–2.25m, 신선한 유효 ToF |
| 신선도 | 관측≤180ms, receipt≤150ms |
| 파라미터 | VER=1, EN=1, AGE=180–200ms, AVEL_DELAY=5ms |
| 시작·clock | UWB와 같은 지상 시작, timesync 확인 |
| 실제 출력 | 새 점군에 종속; 처리 timer 50Hz는 출력 보장 주기가 아님 |
| FC source/frame | 255/198, NED/NED, vz=NaN |
| FC 값 검사 | finite NE, 양의 정부호 공분산, quality 1–100 |
| FC 시간 검사 | 미래·중복·0·AGE 초과 stamp 거부 |
| 연속성 | 첫 표본·reset·gap에는 buffer 초기화 후 준비 |
| EKF 사용 | 다른 수평 보조 활성 + innovation 검사 통과 |

`EKF2_LID2_EN=1`이면 기존 landing-target aux 입력을 대체한다.
두 입력을 같은 버퍼에서 섞지 않는다.
UWB EV 위치 경로는 별도이므로 함께 사용할 수 있다.
현재 aux 경로는 다른 수평 보조가 활성일 때만 융합한다.
LiDAR 단독으로 위치 유지가 보장되는 구조는 아니다.

### 4.3 FC 검사 상태 코드

`lidar_velocity2d_status.accepted`는 수신 검사 통과 누계다.
EKF의 실제 융합 횟수가 아니다.

| reason | 의미 |
|---|---|
| 0 | 정의상 disabled. 현재 비활성 분기는 주기적으로 0을 발행하지 않음 |
| 1 | 첫 유효 표본, 준비 |
| 2 | aux 입력 함수에 전달 |
| 3 | 값·공분산 오류 |
| 4 | quality 오류 |
| 5 | 시각 오류·만료 |
| 6 | 중복·역순 stamp |
| 7 | 송신자 오류 |
| 8 | reset counter 변경, 다시 준비 |
| 9 | 수신 공백, 다시 준비 |
| 10 | frame 또는 vz 계약 오류 |

`reason=2`라도 버퍼의 최소 간격 제한을 받을 수 있다.
EKF가 아직 초기화되지 않았거나 다른 수평 보조가 없을 수도 있다.
innovation이 거부되면 실제 융합도 되지 않는다.

확인은 `estimator_aid_src_aux_vel`에서 한다.
최신 sample, `fused`, `time_last_fuse` 증가를 함께 본다.
`observation`, `innovation`, `test_ratio`도 대조한다.
한 번 찍힌 오래된 `fused=true`만으로 지속 융합을 판정하지 않는다.

소스: [속도 계산](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/drone_uwb/processing/lidar_motion.py),
[전송 계약](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/drone_uwb/integration/lidar_velocity.py),
[ROS bridge](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/drone_uwb/integration/ros/lidar_velocity_bridge.py),
[FC 확장 patch](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/tools/px4_lidar2d_hardware.patch).

## 5. CMD OFFBOARD 위치와 yaw 목표

이 경로는 추정치를 고치는 관측 API가 아니다.
FC가 따라갈 목표를 제공한다.

```text
C++ FieldMission -> JSON 결정 -> LocalXYMission
  -> FlightNode.local_target_message
  -> /mavros/setpoint_raw/local : PositionTarget
  -> MAVROS setpoint_raw::local_cb
  -> SET_POSITION_TARGET_LOCAL_NED(84)
  -> mavlink_receiver
  -> offboard_control_mode + trajectory_setpoint
  -> mc_pos_control
```

| 필드 | ROS 입력 | wire / FC |
|---|---|---|
| `header.stamp` | 현재 발행 시각 | time_boot_ms로 변환 |
| `coordinate_frame` | FRAME_LOCAL_NED=1 | LOCAL_NED=1 |
| `position.x/y` | 지도→ENU 목표 | N/E 목표 |
| `position.z` | `altitude_enu_m` | 부호 변환한 D 목표 |
| `yaw` | ENU yaw rad | NED yaw rad |
| `type_mask` | 2552 | 위치 XYZ·yaw 사용 |
| velocity 3축 | ignore bits 8/16/32 | 목표로 사용하지 않음 |
| acceleration 3축 | ignore bits 64/128/256 | 목표로 사용하지 않음 |
| yaw_rate | ignore bit 2048 | 목표로 사용하지 않음 |
| 발행 주기 | tick 0.05초 | 목표 20Hz, 실제 gap 별도 검사 |

`type_mask=2552`에는 Z_IGNORE가 없다.
현재 코드가 Z 목표를 명시적으로 보낸다는 뜻이다.
ARM 확인 뒤 목표 높이는 다음처럼 만든다.

```text
height_target = ground_height + expected_mis_takeoff_alt_m
```

예시 설정은 상승량 1.7m다.
이 값은 LiDAR의 높이 관측이나 추력 명령이 아니다.
companion이 고도 목표를 정하고, PX4가 고도 제어를 한다.
따라서 “companion은 z를 보내지 않는다”는 표현은 부정확하다.
별도 companion 고도 PID를 구현한 것은 아니다.
[고도 정책](../altitude_policy.md)의 책임 구분을 이 수준에서 읽어야 한다.

PX4는 유효 목표를 받으면 `offboard_control_mode`를 발행한다.
실제 `trajectory_setpoint`는 OFFBOARD 상태에서만 발행한다.
준비 중 목표 송신은 OFFBOARD 진입 준비에 사용된다.
RC 수동 상태에서 보내기만 했다고 목표가 즉시 적용되지는 않는다.

`setpoint_raw` 플러그인이 실행돼야 이 API가 연결된다.
수동 launch 기본값에는 포함되지 않는다.
`local_setpoints=true`일 때 명시적으로 추가한다.
플러그인 로딩 자체는 임무 시작 명령이 아니다.

FC 이후의 계산 단계는 다음과 같다.

| PX4 단계 | 입력 → 출력 |
|---|---|
| 위치 제어 | 목표 위치−추정 위치 → 속도 목표 |
| 속도 제어 | 목표 속도−추정 속도 → 가속도·추력 요구 |
| 자세 목표 생성 | 추력 방향·yaw → 자세 quaternion·추력 |
| 자세 제어 | 목표 자세−추정 자세 → 각속도 목표 |
| 각속도 제어 | 목표 각속도−gyro → 토크·추력 |
| 제어 할당 | 토크·추력 → 모터별 출력 |

현재 임무 API는 roll/pitch나 모터별 값을 직접 보내지 않는다.
yaw 목표는 보낸다.
“자세 제어는 PX4 전담”과 “yaw 목표를 보냄”은 양립한다.

소스: [메시지 생성·발행](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_mission/drone_mission/node.py),
[임무 실행 상태](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_mission/drone_mission/local_xy.py),
[C++ 연결](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_mission/drone_mission/cpp_mission_ai.py).

## 6. CMD 모드와 비행 동작

이 서비스들은 FC의 상태 전환을 요청한다.
서비스 성공 응답만으로 실제 전환을 확정하지 않는다.

| ROS API / 형식 | MAVLink | 주요 필드 | PX4 처리 |
|---|---|---|---|
| `/mavros/set_mode` / SetMode | SET_MODE(11) | base_mode=0, custom_mode | DO_SET_MODE `vehicle_command` → Commander |
| `/mavros/cmd/arming` / CommandBool | COMMAND_LONG(76), ARM_DISARM=400 | value true/false, 강제 ARM 값 없음 | Commander ARM/DISARM 검사 |
| `/mavros/cmd/takeoff` / CommandTOL | COMMAND_LONG, NAV_TAKEOFF=22 | yaw/위도/경도/altitude NaN | FC 자동 이륙 경로 |
| `/mavros/cmd/land` / CommandTOL | COMMAND_LONG, NAV_LAND=21 | 같은 필드 NaN | FC 착륙 경로 |
| `/mavros/cmd/command_int` / CommandInt | COMMAND_INT(75), DO_REPOSITION=192 | speed, yaw, x/y, z=NaN | Navigator 재배치 경로 |

| 코드의 kind | SetMode 값 |
|---|---|
| `takeoff_mode` | AUTO.TAKEOFF |
| `offboard_mode` | OFFBOARD |
| `position_mode` | POSCTL |

DO_REPOSITION은 기존 global/native 경로에 해당한다.
`frame=0`, x/y는 위도·경도 정수 표현이다.
지금 FINAL `local_xy`의 연속 목표는 5절을 쓴다.
두 방식의 좌표와 높이 처리를 섞지 않는다.

ACK는 요청을 수락했는지 보여준다.
실제 상태는 `/mavros/state`, 착륙 상태는 `extended_state`로 확인한다.
목표 피드백은 `/mavros/setpoint_raw/target_local`에서 확인한다.
이 피드백도 목표 도달이나 비행 성공을 뜻하지 않는다.

`native_hover_node.cpp`도 별도 ARM·모드 client를 갖는다.
AUTO.TAKEOFF/AUTO.LAND/POSCTL 요청을 보낼 수 있다.
FINAL Python 실행기와 동시에 쓰는 명령 소유자로 취급하면 안 된다.
같은 FC에 두 임무 실행기가 명령을 줄 수 있기 때문이다.

소스: [명령 dispatch](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_mission/drone_mission/node.py),
[대체 native hover 실행기](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/sangwon_AI/src/native_hover_node.cpp).

## 7. AUX 상태 조회와 시각 동기화

상태 조회도 FC와 통신하지만 비행 목표는 아니다.

| API | 전송 또는 처리 | 관여 단계 |
|---|---|---|
| `/mavros/param/get_parameters` | ROS GetParameters, MAVROS 파라미터 cache 조회 | 실행 조건 대조 |
| `/mavros/cmd/command`, command=511 | MAV_CMD_SET_MESSAGE_INTERVAL | FC telemetry stream 주기 |
| 같은 서비스, command=512 | MAV_CMD_REQUEST_MESSAGE | 지정 상태 메시지 요청 |
| MAVROS sys_time | TIMESYNC(111), 설정 10Hz | companion↔FC 시각 대응 |
| 고정 readback 도구 | SERIAL_CONTROL(126)의 허용된 NSH 조회 | 버전·파라미터·uORB 관찰 |

GetParameters 성공은 MAVROS cache를 읽었다는 뜻이다.
동시에 FC 원본을 새로 읽었다는 보장은 아니다.
최종 적용값 확인에는 FC 직접 readback을 구분해 사용한다.
관측 bridge는 파라미터를 자동으로 쓰지 않는다.

| 수동 관측 요청 | 목표 Hz |
|---|---:|
| ESTIMATOR_STATUS(230) | 10 |
| EXTENDED_SYS_STATE(245) | 5 |
| ATTITUDE_QUATERNION(31) | 100 |
| HIGHRES_IMU(105) | 100 |
| DISTANCE_SENSOR(132) | 40 |
| LOCAL_POSITION_NED(32) | 30 |
| RC_CHANNELS(65) | 10 |
| OPTICAL_FLOW_RAD(106) | 10 |
| ODOMETRY(331), FC→companion 상태 | 30 |

요청 주기는 실측 도착 주기가 아니다.
FC 부하·링크 대역폭·실행 설정에 따라 달라진다.
ODOMETRY 331은 양방향에서 쓰일 수 있다.
메시지 번호와 함께 방향·sysid·compid·estimator_type을 봐야 한다.

clock gate는 RTT≤20ms인 30개 표본으로 준비한다.
마지막 유효 표본은 0.5초 이내여야 한다.
offset이 5ms 넘게 바뀌면 다시 준비한다.
이 검사는 센서 취득 지연 교정의 대체물이 아니다.

소스: [stream 요청](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/drone_uwb/integration/ros/manual_streams.py),
[clock 검사](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/drone_uwb/integration/clock_readiness.py),
[readback](https://github.com/Dronestock-SW/ROS2/blob/c66d7dd934975c645ac1d50400e6da909990d0f0/src/drone_uwb/tools/lidar_hardware_readback.py).

## 8. FC 피드백과 companion 내부 경계

FC 피드백은 주로 관측 보정·임무 판단에 쓰인다.

| FC→companion API | companion 소비자 | 다시 FC에 영향을 주는 경로 |
|---|---|---|
| `/mavros/imu/data` | B_TF 높이 기하, LiDAR 점별 보정·heading | 정제된 위치·속도 관측 |
| `/mavros/downward_0` | B_TF 높이 기하, LiDAR 유효 조건 | XY 관측 허용·거부. 높이 관측 재주입 없음 |
| `/mavros/local_position/odom` | 임무 현재 위치·속도·도달 검사 | 다음 위치·yaw 목표 |
| `/mavros/state` | 연결·armed·mode 확인 | 송신·명령 허용 여부 |
| `/mavros/extended_state` | ON_GROUND·착륙 확인 | 시작 및 임무 전환 |
| `/mavros/rc/in` | RC 인계 검사 | 임무 명령 중단·상태 전환 |
| `/mavros/setpoint_raw/target_local` | 실제 FC 목표 대조 | 불일치·만료 시 임무 중단 |
| `/mavros/timesync_status` | 관측 시계 검증 | 관측 허용·시각 불연속 처리 |

TFmini Plus·optical flow의 FC 센서 융합 경로는 별도다.
companion이 그 메시지를 구독한다고 센서 소유자가 바뀌지 않는다.

C++ 임무 판단은 `sangwon-field-ai/1` JSON을 반환한다.
Python이 응답 시각·세션·결정을 검증하고 FC API로 바꾼다.
현재 작업 폴더의 `path_heading` 변경은 yaw 목표와 회전 대기를 확장한다.
이는 미커밋 작업이며 새 EKF 관측 입구를 만들지 않는다.

LiDAR에는 두 사용처가 있다.

| 사용처 | 입력이 바꾸는 것 | FC 도달 경로 |
|---|---|---|
| 새 `lidar_velocity_bridge` | EKF 보조 속도 | OBS LiDAR |
| 기존 C++ `lidar_gap`·경로 검사 | 임무 진행·정지·회복 판단 | Python 목표·명령 경로 |

두 기능은 별도 설정과 검증을 갖는다.
한쪽 활성화가 다른 쪽의 융합 성공을 뜻하지 않는다.

## 9. 수동과 OFFBOARD에서의 영향

관측 입력은 모드와 독립적으로 유지할 수 있다.
제어 목표는 활성 모드의 권한을 따른다.

| 입력 | 수동 POSCTL | OFFBOARD |
|---|---|---|
| UWB 위치 | 조건 통과 시 EKF 관측, RC 목표의 위치 제어에 영향 | 같은 EKF 관측, 임무 목표의 위치 제어에 영향 |
| LiDAR 속도 | 조건 통과 시 aux 관측 | 같은 aux 관측 |
| companion XYZ/yaw 목표 | OFFBOARD 제어 목표로 적용되지 않음 | 유효 목표가 제어기에 전달 |
| RC | 조종 목표 제공 | 설정·인계 조건에 따라 제어권 회수 |
| ARM·모드 요청 | Commander 검사 | Commander 검사 |

수동 STABILIZED와 POSCTL은 다르다.
수평 위치 제어가 꺼진 모드에서는 위치 유지 효과를 보장하지 않는다.
관측이 계속 들어간다는 사실과 위치 제어 활성은 별개다.

수평 관측은 수직 관측이 아니다.
그러나 EKF 상태들은 공분산으로 연결돼 있다.
`fuseDirectStateMeasurement`는 전체 상태의 gain을 계산한다.
수평 관측도 상관된 자세·bias·다른 상태에 간접 영향을 줄 수 있다.
“z/vz를 관측으로 넣지 않는다”는 계약은 유효하다.
“z·자세에 수학적으로 아무 영향도 없다”는 주장은 하지 않는다.

## 10. 대체 경로와 직접 보내지 않는 항목

동일 센서의 여러 송신기를 동시에 사용하지 않는다.
같은 관측의 중복 입력을 막기 위해서다.

| 항목 | 현재 위치 |
|---|---|
| `uwb_px4_bridge` | 기존 `/mavros/vision_pose/pose_cov` → VISION_POSITION_ESTIMATE(102) 대체 경로 |
| `uwb_local_odometry_bridge` | FINAL 공통 경로. 위 legacy publisher와 동시 사용 차단 |
| `/mavros/odometry/in` | 경쟁 외부 관측 검사 대상. 이번 두 bridge의 출력 토픽은 아님 |
| GPS_INPUT·SET_GPS_GLOBAL_ORIGIN | 현재 공통 UWB/LiDAR 경로에서 발행하지 않음 |
| SET_ATTITUDE_TARGET·직접 thrust | 현재 FINAL 임무 실행기에서 발행하지 않음 |
| RC_OVERRIDE·MANUAL_CONTROL | 현재 FINAL 임무 실행기에서 발행하지 않음 |
| `/aruco_alignment/velocity_suggestion` | 내부 제안. MAVROS 직접 송신 없음 |
| `/uwb/btf_xyz` | 상태 표시·기록. EV 높이 입력으로 연결하지 않음 |
| `/target_pose` | 관측용 목표 표시. 실제 목표 API는 5절 |
| 웹·LoRa 상태 전송 | FC EKF·모터 API와 별도 |

legacy vision pose는 자리값 z와 identity quaternion을 담는다.
EV_CTRL 설정으로 미관측 축을 제외해야 한다.
현재 raw ODOMETRY 경로는 미관측 자세·속도를 NaN으로 명시한다.
legacy 구성을 현재 배포와 혼동하지 않는다.

## 11. 설치 수신 융합의 판정 기준

각 단계는 다른 증거를 요구한다.

| 단계 | 충족 증거 | 아직 뜻하지 않는 것 |
|---|---|---|
| 소스 준비 | 코드·patch·설정 존재 | 실행 중임 |
| 빌드 | 해당 board의 `.px4` 생성·해시 | FC 설치 |
| FC 설치 | 실제 FC 버전·전용 파라미터 readback | EN 활성·관측 수신 |
| companion 송신 | 증가하는 published, 실제 packet | FC 도착 |
| FC 수신 | 최신 `lidar_velocity2d` | 검사 통과 |
| 검사 통과 | reason=2, accepted 증가 | EKF 융합 |
| EKF 융합 | 최신 aid_src, fused, last_fuse 증가 | 독립 정확도 |
| 정확도 | 외부 기준과 동기화한 오차 측정 | 비행 안정성 |
| 비행 검증 | 실제 비행·인계·센서 손실 기록 | 모든 조건에서 안전 |

UWB는 `vehicle_visual_odometry` 수신부터 확인한다.
이어 `estimator_aid_src_ev_pos`를 확인한다.
LiDAR는 `lidar_velocity2d`와 `estimator_aid_src_aux_vel`를 사용한다.

2026-10-12 기록은 LiDAR 전용 FC 미설치 상태였다.
2026-10-13 조회는 FC 상태 응답이 없어 완료하지 못했다.
오늘 원격 프로세스 목록에 LiDAR/UWB raw 송신기는 없었다.
이는 [확인 기록](../report/companion_px4_audit_20261013.md)에 분리했다.

## 12. PX4 소스에서 확인할 위치

아래는 실제 검토한 고정 revision 기준이다.
LiDAR 확장 부분은 저장소 patch를 추가로 본다.

| 단계 | 파일 / 함수 |
|---|---|
| wire 관측 분기 | `src/modules/mavlink/mavlink_receiver.cpp` / `handle_message_odometry` |
| EV 변환 | `src/modules/ekf2/EKF2.cpp` / `UpdateExtVisionSample` |
| LiDAR 검사 | 같은 파일 / `UpdateAuxVelSample` + hardware patch |
| 시각·버퍼 | `src/modules/ekf2/EKF/estimator_interface.cpp` / `setExtVisionData`, `setAuxVelData` |
| UWB 수평 융합 | `EKF/aid_sources/external_vision/ev_pos_control.cpp` / `controlEvPosFusion` |
| LiDAR 수평 융합 | `EKF/aid_sources/auxvel/auxvel_fusion.cpp` / `controlAuxVelFusion` |
| 2D 속도 보정 | `EKF/velocity_fusion.cpp` / `fuseHorizontalVelocity` |
| 상태 결합 영향 | `EKF/ekf_helper.cpp` / `fuseDirectStateMeasurement` |
| OFFBOARD 입구 | `mavlink_receiver.cpp` / `handle_message_set_position_target_local_ned` |
| 위치·속도 루프 | `mc_pos_control/PositionControl/PositionControl.cpp` |
| 자세 루프 | `mc_att_control/mc_att_control_main.cpp` |
| 각속도 루프 | `mc_rate_control/MulticopterRateControl.cpp` |
| 모터 할당 | `control_allocator/ControlAllocator.cpp` |

[PX4 수신기 고정 소스](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_receiver.cpp),
[PX4 EKF 고정 소스](https://github.com/PX4/PX4-Autopilot/tree/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2),
[MAVLink 메시지 정의](https://mavlink.io/en/messages/common.html#ODOMETRY)를 함께 참조한다.
코드와 문서 기준 파일의 SHA256은
[source manifest](../report/evidence/companion_px4_api_20261013/source_manifest.json)에 있다.
