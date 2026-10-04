# 공용 XY 데모 기준

XY 시험 입력과 실측 ToF 표시의 기준이다.
데모 또는 센서 표시를 연결할 때 읽는다.

## 범위

공용 데모에서 드론의 가상 고도를 제거했다.
2026-10-04 사용자가 ToF 실측 가능을 확인했다.
실행은 [데모 절차](../runbooks/demo_procedure.md)를 따른다.
도착 판정은 [임무 상태 기준](demo_mission_design.md)을 따른다.

```text
config/demo.json
    |
    +-- XY 경로·목표 ---------> /demo_pose, /target_pose
    |
    +-- XY 잡음·공백 ---------> /uwb_pose (demo_xy)

실측 ToF 토픽 (sensor_msgs/Range)
    |
    +-- 거리·시각·수신 나이 ---> /demo_status.tof
                                  |
                          미수신·노후: null
```

XY 관측은 표시·도착·수신 공백 시험용이다.
실제 RAW 처리기를 실행한 결과가 아니다.
가상 z가 필요했던 사선거리 생성도 제거했다.
`/uwb/raw` 발행과 `received.jsonl` 생성은 종료했다.
실제 RAW 검증은 [UWB 재생 절차](../runbooks/uwb_data_runbook.md)를 따른다.
별도 Gazebo·수식 단위시험은 재현용으로 유지한다.

## 설정

[설정 파일](../../src/drone_demo/config/demo.json)에서 XY를 조정한다.

| 항목 | 기본값 | 의미 |
|---|---|---|
| `start_xy_m` | [2.09, 1.68]m | 시작 위치 |
| `target_xy_m` | [3.09, 2.68]m | 시험 목표 |
| `rate_hz` | 40Hz | 데모 입력 주기 |
| `duration_s` | 12초 | 자동 종료 시간 |
| `start_hold_s` | 2초 | 이동 전 대기 |
| `speed_m_s` | 0.25m/s | 지정 경로의 이동 속도 |
| `xy_noise_stddev_m` | 0.02m | XY 각 축의 예시 잡음 |
| `seed` | 7 | 입력 재현용 난수 seed |
| `real_tof_topic` | `/tof/range` | 실측 Range 구독 인자 |
| `tof_timeout_s` | 0.2초 | 거리 표시의 만료 시간 |

`z_min_m`, `z_max_m`, `z_period_s`는 제거했다.
`synthetic_z_enabled` 실행 인자도 제거했다.
기존 설정의 `range_noise_stddev_m`은 XY 잡음으로 교체한다.
새 설정의 숫자는 센서 교정값이 아니다.

앵커 XY는 [6.3×4.6m 직사각형](../reference/uwb_anchor_layout.md)이다.
앵커 설치 높이 2.2m와 드론의 고도는 별개의 값이다.
데모는 배치의 좌표계만 공유하고 거리를 합성하지 않는다.

## 입력과 출력

DOMAIN_ID 99와 `ROS_LOCALHOST_ONLY=1`을 사용한다.
이유: 데모 목표와 실기 임무 입력을 분리해야 한다.

| 경계 | 메시지 | 의미 |
|---|---|---|
| `/demo_pose` | `PoseStamped` | 지정한 XY 경로. z=0 자리값 |
| `/target_pose` | `PoseStamped` | XY 목표. z=0 자리값 |
| `/uwb_pose` | `PoseWithCovarianceStamped` | 잡음·공백을 넣은 XY 시험 관측 |
| `real_tof_topic` | `sensor_msgs/Range` | 원래 센서 시각·frame·거리·측정 범위 |
| `/demo_status` | `String` JSON | schema 2. XY 출처와 ToF 상태 |

`/uwb_pose`의 z·회전 분산은 1e6이다.
위치 메시지의 좌표계는 `uwb_map`이다.
상태는 `z_m=null`, `z_source=unobserved`를 기록한다.
ToF 거리와 지도 높이를 같은 숫자로 채우지 않는다.
이유: 자세·바닥·장착 오프셋 보정이 필요하다.

ToF가 유효하면 `tof.available=true`와 `range_m`을 기록한다.
측정 시각은 0보다 크고 미래가 아니어야 한다.
센서 frame과 유한한 측정 범위를 요구한다.
중복·과거 표본은 신선도를 갱신하지 않는다.
미수신·잘못된 입력·시간 초과에는 값을 null로 둔다.
신선도는 측정 시각과 수신 후 단조 시계로 확인한다.
ToF 수신은 XY 데모를 종료하지 않는다.

`/target_pose`는 transient local QoS로 저장 발행한다.
`/uwb_pose`와 ToF 구독은 sensor data QoS다.
MAVROS·PX4 제어기는 이 데모에서 실행하지 않는다.

## 시나리오와 파일 계약

| 시나리오 | 경로 | XY 시험 관측 |
|---|---|---|
| `stationary` | 시작점 유지, 목표도 시작점 | 연속 |
| `move` | 2초 뒤 직선 이동 | 연속 |
| `gap` | move와 같음 | 4초 이상, 5초 미만 공백 |

JSONL은 `truth_xy_m`과 `target_xy_m`을 사용한다.
과거 `truth_xyz_m`·`target_xyz_m`은 새 출력에 없다.
CSV도 XY만 기록한다.
`demo_export`는 센서에 접속하지 않는다.
실측 ToF를 포함한 상태는 ROS의 `/demo_status`에서 읽는다.
과거 시험 파일은 당시 계약으로 보존한다.

고도 제어와 후보정 입력은 [고도 원칙](../altitude_policy.md)을 따른다.
