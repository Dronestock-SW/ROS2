# PX4 OFFBOARD & MAVLink 파라미터 정리 (HW 파트 수신본)

> **수신 자료 전사본이다.** 2026-08-12 HW 파트에서 받은 PDF
> `PX4_ROS2_MAVLink_OFFBOARD_정리.pdf` (12쪽, 작성일 2026-08-09)의 본문을 옮겼다.
> 원본 PDF 바이너리는 이 저장소에 없다. 필요하면 HW 파트에 재요청한다.
> 이 문서는 **기준 문서가 아니다.** 우리 쪽 판단 기준은 docs/roadmap.md 다.

**문서 기준**: PX4 v1.17 Stable / Jetson ROS 2 + MAVROS2 + MAVLink + Pixhawk 6C Mini
**작성 목적**: 파라미터 역할과 데이터 흐름을 쉽게 확인하기 위한 내부 참고 문서

---

## 핵심 구성

```text
UWB / 거리 센서      현재 위치 측정
        ↓
Jetson              ROS 2에서 좌표 처리 · 미션/웹 목표 처리
        ↓
MAVROS2             ROS 2 메시지 ↔ MAVLink 변환
        ↓
MAVLink Serial      TELEM1 또는 TELEM2
        ↓
Pixhawk 6C Mini / PX4   EKF2 위치 추정 + OFFBOARD 제어 + 모터 제어
```

---

## 1. 가장 중요한 수정 사항

> **중요**
> Jetson에서 ROS 2를 사용하더라도 Pixhawk와의 실제 통신을 MAVLink로 할 경우, PX4의 ROS 2 전용 `OffboardControlMode` / `TrajectorySetpoint`를 직접 보내는 구조가 아니다. Jetson의 ROS 2 노드가 MAVROS2를 거쳐 MAVLink setpoint 메시지를 PX4로 전송하는 구조다.

### 1.1 전체 통신 구조

```text
UWB                 현재 X/Y(/Z) 좌표 생성
        ↓
Jetson ROS 2 노드    UWB 좌표 처리 · 목표 좌표 관리
        ↓
MAVROS2             ROS 2 ↔ MAVLink Gateway
        ↓
MAVLink             Serial: TELEM1 또는 TELEM2
        ↓
PX4                 External Position → EKF2 / Setpoint → OFFBOARD Controller
```

MAVROS 공식 ROS 2 브랜치는 MAVLink를 ROS 2에 연결하는 Gateway이며 Serial/UDP/TCP 연결과 OFFBOARD 지원을 제공한다.

### 1.2 현재 위치와 목표 위치는 반드시 분리

| 구분 | 의미 | PX4로 전달되는 경로 |
|---|---|---|
| UWB 현재 좌표 | "드론이 지금 어디에 있는가" | MAVROS2 vision/odometry plugin → MAVLink `VISION_POSITION_ESTIMATE` 또는 `ODOMETRY` → EKF2 |
| 웹/미션 목표 좌표 | "드론이 어디로 가야 하는가" | MAVROS2 setpoint plugin → MAVLink `SET_POSITION_TARGET_LOCAL_NED` → PX4 Position Controller |

> **잘못하면 안 되는 부분**
> UWB가 측정한 현재 위치를 목표 setpoint에 그대로 넣으면 PX4 입장에서는 계속 "현재 위치로 가라"는 명령을 받는 셈이다. UWB 좌표는 EKF의 현재 위치 입력, 웹 좌표는 OFFBOARD 목표값으로 사용한다.

---

## 2. OFFBOARD — MAVLink 방식에서의 정확한 의미

OFFBOARD는 PX4 외부의 컴퓨터가 위치, 속도, 자세 등의 목표값(Setpoint)을 보내 드론을 제어하는 비행 모드다.

| 항목 | MAVLink 방식 |
|---|---|
| 외부 제어 주체 | Jetson의 ROS 2 프로그램 |
| 통신 변환 | MAVROS2 |
| PX4로 실제 전달 | MAVLink Setpoint 메시지 |
| Position 제어 예 | `SET_POSITION_TARGET_LOCAL_NED` |
| Attitude 제어 예 | `SET_ATTITUDE_TARGET` |
| OFFBOARD 생존 신호 | MAVLink setpoint 스트림 자체가 proof-of-life 역할 |

### 2.1 OffboardControlMode / TrajectorySetpoint는 어떻게 되는가

| 메시지 | 직접 ROS 2(uXRCE-DDS) 방식 | 이번 프로젝트(MAVLink) |
|---|---|---|
| `OffboardControlMode` | 사용함 — heartbeat + 제어계층 선택 | 직접 사용하지 않음 |
| `TrajectorySetpoint` | Position/Velocity/Acceleration 목표 uORB 메시지 | 직접 사용하지 않음 |
| MAVLink Setpoint | 선택 사항 | 사용함 — setpoint + heartbeat 역할 |

기존 메모의 position / velocity / acceleration 개념 자체는 유효하지만, 이번 구성에서는 `OffboardControlMode` 필드를 켜는 것이 아니라 MAVLink setpoint의 type mask와 유효 필드로 제어 종류가 결정된다.

### 2.2 OFFBOARD 전송 주기

> **PX4 v1.17 요구사항**
> 지원되는 MAVLink setpoint 메시지를 2 Hz보다 빠르게 계속 보내야 하며, OFFBOARD 진입 전에 최소 1초 이상 스트림이 들어오고 있어야 한다. MAVLink 방식에서는 setpoint 자체가 생존 신호이므로, 위치를 유지할 때도 현재 목표 위치 setpoint를 계속 전송해야 한다.

실사용에서는 통신 여유를 두기 위해 10 Hz 정도 이상의 주기를 사용하는 설계가 흔하지만, PX4의 공식 최소 조건은 > 2 Hz다.

---

## 3. UWB External Position / EKF2 파라미터

### 3.1 EKF2_EV_CTRL

외부 위치추정(External Vision 계열) 데이터 중 어떤 항목을 EKF2가 융합할지 정하는 비트마스크다.

| 비트 값 | 기능 | 설명 |
|---|---|---|
| 1 (bit 0) | Horizontal position | X/Y 위치 융합 |
| 2 (bit 1) | Vertical position | Z 위치 융합 |
| 4 (bit 2) | 3D velocity | 외부 3축 속도 융합 |
| 8 (bit 3) | Yaw | 외부 Yaw 융합 |

| 사용 상황 | EKF2_EV_CTRL | 설명 |
|---|---|---|
| UWB가 X/Y만 제공, Z는 별도 센서 | 1 | X/Y만 UWB 사용 |
| UWB가 X/Y/Z 모두 신뢰 가능 | 3 | 1 + 2 = X/Y/Z 위치 사용 |
| XYZ + 외부 속도까지 사용 | 7 | 1 + 2 + 4 |
| XYZ + 속도 + Yaw 모두 사용 | 15 | 1 + 2 + 4 + 8 |

> **주의**
> `EKF2_EV_CTRL = 3`으로 설정한다고 Z 값이 자동으로 생기는 것은 아니다. MAVLink 외부 위치 메시지에 실제로 신뢰할 수 있는 Z가 들어오는 경우에만 Vertical position fusion을 활성화해야 한다.

### 3.2 EKF2_EV_DELAY

| 항목 | 내용 |
|---|---|
| 역할 | 외부 위치 측정값과 PX4 IMU 측정값 사이의 시간 지연 보정 |
| 범위 | 0 ~ 300 ms |
| 기본값 | 0 ms |
| 발생 가능한 지연 | UWB 측정 → 좌표 계산 → Jetson 처리 → MAVROS2 → UART/MAVLink → Pixhawk |
| 설정 원칙 | 임의로 크게 잡지 말고 로그/측정으로 실제 지연을 확인한 뒤 보정 |

### 3.3 EKF2_GPS_CTRL

GNSS/GPS 정보를 EKF2에 융합할지 결정하는 비트마스크다. GPS를 사용하지 않는 실내 UWB 구성이라면 0으로 설정해 GPS aiding을 비활성화할 수 있다.

| 값 | 의미 |
|---|---|
| 0 | GPS/GNSS aiding 사용 안 함 |
| 기본값 7 | 위치 + 고도 + 3D 속도 융합 |

### 3.4 EKF2_HGT_REF — 추가 확인 권장

Z축/고도 기준을 무엇으로 삼을지 결정한다. GPS를 끈 상태라면 기본 높이 기준이 현재 센서 구성과 맞는지 반드시 확인하는 것이 좋다.

| 값 | 높이 기준 |
|---|---|
| 0 | Barometer |
| 1 | GPS |
| 2 | Range sensor |
| 3 | Vision / External position |

> **프로젝트 적용 예**
> UWB는 X/Y만 사용하고 거리센서로 지면 높이를 잡는다면 `EKF2_EV_CTRL=1`을 우선 검토하고, 높이 기준은 Range sensor 구성에 맞춘다. 반대로 UWB Z까지 외부 위치로 융합한다면 `EKF2_EV_CTRL=3`과 Height Reference 구성을 함께 검토해야 한다.

### 3.5 좌표계 주의 — 매우 중요

| 구분 | 일반 좌표계 |
|---|---|
| ROS | ENU / FLU (Z Up) |
| PX4 | NED / FRD (Z Down) |
| UWB | 설치 시 정의한 자체 좌표계일 수 있음 |

MAVROS의 표준 plugin을 사용하면 ROS↔PX4 좌표계 변환을 처리할 수 있지만, UWB 자체 좌표축이 ROS ENU와 다르면 먼저 UWB 좌표계를 ROS map/odom 기준에 맞춰야 한다. 직접 변환을 추가하면서 MAVROS 변환까지 중복 적용하면 축이 뒤집힐 수 있다.

---

## 4. OFFBOARD Failsafe 파라미터

### 4.1 COM_OF_LOSS_T

| 항목 | 내용 |
|---|---|
| 역할 | OFFBOARD setpoint 스트림이 끊긴 뒤 Offboard Loss Failsafe를 발생시키기까지 기다리는 시간 |
| 범위 | 0 ~ 60 s |
| 기본값 | 1.0 s |
| 다음 단계 | 시간 초과 후 `COM_OBL_RC_ACT`에 설정된 동작으로 전환 |

### 4.2 COM_OBL_RC_ACT

OFFBOARD loss failsafe가 발생했을 때 어떤 비행 모드/조치를 사용할지 결정한다.

| 값 | 동작 | 한 줄 설명 |
|---|---|---|
| 0 | Position | 위치 보조 모드로 전환; 조종사가 스틱으로 이어서 제어 가능 |
| 1 | Altitude | 고도 유지 보조; X/Y 위치 고정은 하지 않음 |
| 2 | Stabilized | 자세 안정화 중심; 위치/고도 자동 유지가 아님 |
| 3 | Return | Home 또는 설정된 안전 복귀 지점으로 Return |
| 4 | Land | 착륙 수행 |
| 5 | Hold | 현재 위치와 고도를 유지하며 자동 대기 |
| 6 | Terminate | 비행 제어를 강제 종료하는 최종 비상조치 |
| 7 | Disarm | 즉시 시동 해제/모터 정지 상태로 전환 |

> **0 Position vs 5 Hold**
> 둘 다 위치가 유지될 수 있지만 목적이 다르다. Position은 조종사가 수동으로 이어받을 수 있는 위치 보조 모드이고, Hold는 PX4가 자동으로 그 자리에서 대기하는 모드다.

> **6 Terminate vs 7 Disarm**
> Terminate는 비행 제어를 강제 종료하는 Flight Termination 개념이고, Disarm은 시동 해제/모터 정지다. 두 옵션 모두 비행 중에는 매우 강한 조치이므로 일반적인 통신 끊김 대응에는 신중해야 한다.

### 4.3 COM_FAIL_ACT_T

| 항목 | 내용 |
|---|---|
| 역할 | Failsafe 조건이 발생한 뒤 RTL/Land/Hold 같은 실제 Failsafe 반응 전에 Hold 상태로 기다리는 시간 |
| 기본값 | 5.0 s |
| 범위 | 0 ~ 25 s |
| 0 설정 | 추가 지연 비활성화 |

> **정확한 해석**
> `COM_FAIL_ACT_T`는 모든 OFFBOARD 동작에 무조건 붙는 "두 번째 OFFBOARD 타이머"가 아니다. PX4의 일반 failsafe reaction delay이며, 문서상 RTL/Land/Hold 진입 전에 Hold 시간을 제공하는 용도다.

### 4.4 Failsafe 흐름

```text
Jetson MAVLink setpoint 정상      > 2 Hz 스트림 유지
        ↓
Setpoint 송신 중단                Jetson 프로그램/통신 문제
        ↓
COM_OF_LOSS_T                    기본 1.0 s 대기
        ↓
Offboard Loss Failsafe 발생       COM_OBL_RC_ACT 확인
        ↓
필요한 경우 COM_FAIL_ACT_T        RTL/Land/Hold 등의 reaction 전 Hold 지연
        ↓
설정된 Failsafe Action 실행       Position / Return / Land / Hold 등
```

---

## 5. Jetson ↔ Pixhawk MAVLink 포트 파라미터

### 5.1 MAV_1_CONFIG

MAVLink Instance 1을 어느 Serial Port에 연결할지 지정한다. "1번"은 물리 포트 번호가 아니라 MAVLink 인스턴스 번호다.

| 값 | 포트 |
|---|---|
| 0 | Disabled |
| 6 | UART 6 |
| 101 | TELEM1 |
| 102 | TELEM2 |
| 103 | TELEM3 |
| 104 | TELEM/SERIAL4 |
| 201 | GPS1 |
| 202 | GPS2 |
| 203 | GPS3 |
| 300 | Radio Controller |
| 301 | Wi-Fi Port |
| 401 | EXT2 |
| 1000 | Ethernet |

> **현재 메모 기준**
> `MAV_1_CONFIG = 101`로 설정했다면 MAVLink Instance 1을 TELEM1에 배치한 것이다. 설정 변경 후에는 재부팅이 필요하다.

> **TELEM1 사용 시 확인**
> PX4 기본 구성에서 TELEM1은 보통 GCS/Normal 용도이고 TELEM2는 Companion/Onboard 용도로 구성된다. Jetson을 TELEM1에 연결해도 되지만, 기존 MAVLink 인스턴스가 TELEM1을 사용 중인지 확인해 포트 중복을 피해야 한다.

### 5.2 MAV_1_MODE

| 값 | 모드 | 용도 |
|---|---|---|
| 0 | Normal | 일반/GCS용 메시지 프로파일 |
| 1 | Custom | 사용자 정의 |
| 2 | Onboard | Companion Computer용 — 이번 프로젝트 권장 |
| 3 | OSD | OSD용 |
| 7 | Minimal | 최소 메시지 |
| 8 | External Vision | 외부 비전용 프로파일 |
| 10 | Gimbal | 짐벌용 |
| 11 | Onboard Low Bandwidth | 저대역폭 Companion용 |

> **수정 사항**
> `MAV_1_MODE`는 Jetson이 물리적으로 연결되어 있지 않아도 설정할 수 있다. `MAV_1_CONFIG`로 포트를 지정한 뒤 PX4를 재부팅하면 관련 `MAV_1_*` 파라미터가 QGroundControl에 나타나는 구조다.

### 5.3 같이 확인할 파라미터

| 파라미터 | 권장/의미 |
|---|---|
| `MAV_1_RATE` | 0이면 해당 링크 이론상 최대 대역폭의 절반을 기준으로 stream rate 제한 |
| `MAV_1_FORWARD` | 필요한 경우 다른 MAVLink 포트로 메시지 forwarding. Companion 단독 링크면 보통 Disabled |
| `SER_TEL1_BAUD` | TELEM1 사용 시 Jetson의 serial baud와 동일하게 설정 |
| `SER_TEL2_BAUD` | TELEM2 사용 시 Jetson의 serial baud와 동일하게 설정; PX4 기본 Companion 예시는 921600 |

### 5.4 권장 포트 구성 예

| 용도 | 포트 | 예시 설정 |
|---|---|---|
| GCS Telemetry | TELEM1 | `MAV_0_CONFIG=TELEM1` / `MAV_0_MODE=Normal` |
| Jetson Companion | TELEM2 | `MAV_1_CONFIG=TELEM2` / `MAV_1_MODE=Onboard` / `SER_TEL2_BAUD=921600` |

현재 TELEM1(101)을 사용할 계획이라면 GCS는 USB 또는 다른 포트로 사용하고, TELEM1의 기존 할당이 겹치지 않는지 먼저 확인하는 방식이 안전하다.

---

## 6. 프로젝트 데이터 흐름 — 최종 정리

### 6.1 현재 위치(UWB) 경로

```text
UWB Tag/Anchors                 현재 좌표 X/Y(/Z)
        ↓
Jetson ROS 2                    좌표 계산 · 필터링 · 좌표축 정렬
        ↓
MAVROS2 Vision/Odometry Plugin  ROS pose/odometry를 MAVLink로 변환
        ↓
MAVLink                         VISION_POSITION_ESTIMATE 또는 ODOMETRY
        ↓
PX4 EKF2                        EKF2_EV_CTRL 설정에 따라 외부 위치 융합
        ↓
PX4 Local Position              드론의 현재 위치 추정값
```

### 6.2 목표 위치(웹/미션) 경로

```text
웹/미션                          목표 좌표 예: X=3.0, Y=2.0, Z=1.0
        ↓
Jetson ROS 2                    목표 좌표 수신 · 비행 로직
        ↓
MAVROS2 Setpoint Plugin         Position Setpoint 전송
        ↓
MAVLink SET_POSITION_TARGET_LOCAL_NED   Position 필드 활성화
        ↓
PX4 OFFBOARD Position Controller        현재 위치와 목표 위치 오차 계산
        ↓
Attitude/Rate/Control Allocation        PX4가 모터 출력을 자동 계산
```

### 6.3 한 문장씩 기억하기

| 항목 | 한 줄 설명 |
|---|---|
| UWB | 드론이 지금 어디 있는지 알려주는 현재 위치 센서 |
| EKF2 | UWB/IMU/고도센서 등의 정보를 융합해 현재 상태를 추정 |
| Jetson | 어디로 갈지 결정하는 외부 컴퓨터 |
| MAVROS2 | ROS 2 데이터와 MAVLink 메시지를 연결하는 Gateway |
| MAVLink | Jetson과 Pixhawk 사이의 실제 통신 프로토콜 |
| OFFBOARD | 외부 컴퓨터의 setpoint를 받아 PX4가 비행하는 모드 |
| PX4 | 목표까지 어떻게 날지 계산하고 모터를 제어 |

---

## 7. 설정/시험 체크리스트

| No. | 확인 항목 |
|---|---|
| 1 | 프로펠러 제거 상태에서 Bench Test 먼저 수행 |
| 2 | `MAV_1_CONFIG`가 실제 Jetson 연결 포트와 일치하는지 확인 |
| 3 | `MAV_1_CONFIG` 변경 후 PX4 재부팅 |
| 4 | `MAV_1_MODE = Onboard` 확인 |
| 5 | Serial Baud가 Jetson과 Pixhawk 양쪽에서 동일한지 확인 |
| 6 | MAVROS2에서 FCU 연결 상태 확인 |
| 7 | UWB 현재 좌표가 MAVLink external position으로 PX4에 들어오는지 확인 |
| 8 | EKF2에서 External Position fusion이 정상인지 확인 |
| 9 | UWB 좌표축과 ROS/PX4 좌표축 방향 확인 |
| 10 | OFFBOARD 진입 전 Setpoint를 >2 Hz로 1초 이상 전송 |
| 11 | OFFBOARD 진입 후 setpoint 스트림이 계속 유지되는지 확인 |
| 12 | `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` failsafe를 지상에서 단계적으로 시험 |

> **비행 전 권장**
> 실내 쿼드콥터에서는 Return보다 Hold 또는 Land가 더 예측 가능한 경우가 많다. 단, Hold는 현재 위치 추정이 정상이어야 하므로 UWB/EKF 상실 상황과 Jetson 통신 상실 상황을 별도의 failsafe로 구분해 시험해야 한다.

---

## 8. 기존 메모에서 수정된 핵심 문장

| 기존 표현 | 수정된 표현 |
|---|---|
| `OffboardControlMode`에서 position=true로 사용 | MAVLink 방식에서는 `OffboardControlMode`를 직접 사용하지 않고 MAVLink setpoint의 유효 필드/type mask로 제어 종류를 정함 |
| `TrajectorySetpoint`로 웹 목표 좌표 전송 | MAVLink 방식에서는 MAVROS2를 통해 `SET_POSITION_TARGET_LOCAL_NED` 등의 MAVLink 메시지로 목표 좌표 전송 |
| `MAV_1_MODE`는 Jetson 연결 후 설정 가능 | Jetson 연결 여부와 무관하게 설정 가능. `MAV_1_CONFIG` 지정 후 재부팅 필요 |
| `EKF2_EV_CTRL=3`이면 XYZ 사용 | 실제 외부 메시지에 신뢰 가능한 XYZ가 들어오는 경우에만 3 사용. UWB가 XY만 제공하면 1이 맞음 |
| `COM_FAIL_ACT_T`는 모든 failsafe의 추가 5초 | 일반 failsafe reaction delay이며 문서상 RTL/Land/Hold 전 Hold 대기 시간 |

---

## 9. 참고 자료

- [1] PX4 v1.17 Offboard Mode — https://docs.px4.io/v1.17/en/flight_modes/offboard
- [2] PX4 Parameter Reference — https://docs.px4.io/main/en/advanced_config/parameter_reference
- [3] PX4 External Position Estimation — https://docs.px4.io/main/en/ros/external_position_estimation
- [4] PX4 MAVLink Peripherals — https://docs.px4.io/main/en/peripherals/mavlink_peripherals
- [5] MAVROS ROS 2 official repository — https://github.com/mavlink/mavros/tree/ros2

> **최종 요약**
> 이번 프로젝트의 핵심은 "ROS 2를 쓰지만 Pixhawk와는 MAVLink로 통신"한다는 점이다. 따라서 UWB 현재 위치는 MAVROS2를 통해 External Position MAVLink 메시지로 EKF2에 넣고, 웹 목표 위치는 MAVLink Position Setpoint로 OFFBOARD에 넣는다. `OffboardControlMode`/`TrajectorySetpoint`는 직접 uXRCE-DDS 방식의 개념이므로 이번 통신 구조에서는 직접 사용하지 않는다.
