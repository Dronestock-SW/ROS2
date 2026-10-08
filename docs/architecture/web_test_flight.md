# 로컬 웹 비행 시험 구조

센서 관측과 비행 명령의 연결 설명이다. 좌표·제어 책임을 검토할 때 읽는다.

PX4 EKF2가 비행 위치를 추정한다.
companion은 관측 전달과 임무 순서를 맡는다.
별도 비행 추정기·자세·고도 제어기를 두지 않는다.

로컬 통합판은 Tag A/B·TDMA 입력을 함께 지원한다.
설정·수신기·B_TF의 태그와 배치를 시작 전에 대조한다.
기본 bridge·비행 출력과 물리 확인값은 false다.
[통합 기록](../report/local_jetson_integration_20261008.md)에 검증 범위를 적었다.

실물 domain 1/2의 PX4 명령 잠금을 공유한다.
Python 웹 FSM과 C++ native hover 중 하나만 획득한다.
C++ BT 서비스의 실물 출력 금지는 유지한다.
이전 배포본·외부 GCS는 이 잠금에 참여하지 않는다.
따라서 기존 서비스 확인과 실제 명령 주체 조사가 필요하다.

## 관측과 명령 흐름

현재 위치와 목표 위치는 서로 다른 입력이다.

```text
UWB RAW UART -> uwb_node -> /uwb/received ----------+
                                                  |
TFmini Plus -> PX4 -> MAVROS ToF / IMU / TIMESYNC --+
                                                  v
                                       uwb_btf_node (B_TF)
                                         |            |
                            /uwb/btf_pose XY     /uwb/btf_xyz
                                         |       태그 높이 표시
                                    uwb_px4_bridge
                                         |
                            /mavros/vision_pose/pose_cov
                                         |
                                    PX4 EKF2
                                         |
                                PX4 위치·속도·상태
                                         v
로컬 웹 -> GET mission -> /mission/assignment -> flight_mission
                                                   |
                                arm -> takeoff -> reposition -> land
                                                   |
                       /flight_state -> WebSocket -> 로컬 웹
                                +-> HTTP 요청 ACK·단계 보고
```

통합 launch는 MAVROS를 하나만 시작한다.
실물 센서·브리지·미션·플랫폼을 함께 관리한다.
기존 bench launch의 센서 전용 플러그인 목록과 구분한다.
새 목록에는 `vision_pose`, `param`, `command`가 있다.
`local_position`, `global_position`, `setpoint_raw`도 포함한다.

## XY 보정과 XYZ의 의미

높이는 XY 풀이의 기하와 관측 품질에 쓰인다.
항상 XY를 두 번 보정하는 구조는 아니다.

| 단계 | 처리 | 결과 |
|---|---|---|
| B4 | 동일 높이 4앵커의 H80 시간창 풀이 | 기본 XY 후보 |
| ToF 검사 | 자세·장착 위치로 태그 높이 계산 | 각 거리의 3D 일관성 검사 |
| B3 | B4 적합 RMS가 0.06m 이상일 때 검사 | 일관된 3앵커 재풀이 또는 거부 |
| XYZ 표시 | 승인한 XY와 같은 시각의 태그 높이 | `/uwb/btf_xyz` |
| FC 관측 전달 | 고정 회전·원점과 XY 공분산 변환 | MAVROS ENU 수평 관측 |
| 비행 융합 | PX4 EKF2가 수평 관측을 융합 | PX4의 비행 위치 |

동일 높이 앵커에서는 공통 높이 항을 분리한다.
좋은 B4는 그대로 사용한다.
ToF가 없는 B4에는 XYZ를 생성하지 않는다.
비행 launch는 `require_height_for_pose=true`다.
이 경로는 높이 누락 때 XY 발행도 중지한다.
이유: 이번 시험의 기하 검증 입력이 사라졌기 때문이다.

태그 높이는 바닥 기준 UWB 안테나 높이다.
PX4의 기체 기준 고도와 동일하지 않다.
현재 실물 B_TF 설정은 ToF 렌즈 위 0.12m를 사용한다.
이는 2026-10-05 사용자 확인값이다.
FC에서 태그까지의 장착 위치는 별도로 측정한다.

| 입력 | 기준 | 사용 |
|---|---|---|
| `tof_to_tag_body_flu_m` | ROS FLU: 전방·좌측·위 | 렌즈에서 태그까지 높이 계산 |
| `EKF2_EV_POS_X/Y/Z` | PX4 FRD: 전방·우측·아래 | FC에서 태그까지 위치 보상 |
| `enu_yaw_deg`, `enu_offset_*` | 창고 XY → PX4 ENU | 관측과 목표에 같은 값 적용 |

FLU의 +Z와 FRD의 +Z는 반대다.
렌즈→태그 0.12m를 EV_POS_Z에 복사하지 않는다.
이유: 좌표축과 기준점이 모두 다르기 때문이다.

`/uwb/btf_pose`와 PX4 입력의 z=0은 자리값이다.
Z·회전 공분산은 1e6으로 둔다.
브리지는 `EKF2_EV_CTRL=1`을 요구한다.
`EKF2_EV_NOISE_MD=0`도 요구한다.
실측 XYZ를 고도 목표로 전송하지 않는다.

## 웹 요청 계약

v1 HTTP 수신·텔레메트리 경계를 유지한다.
전체 순서 시작을 위해 로컬 `start`를 추가했다.

| 경로·필드 | 의미 |
|---|---|
| GET `companion-mission/` | 유효한 요청·활성 경로 수신. 기본 2Hz |
| `control_action=start` | 새 요청 ID로 이륙·경유지·착륙 시작 |
| `control_action=land` | PX4 착륙 요청 |
| `control_action=return_to_home` | 기록한 출발 XY 복귀 후 착륙 |
| POST `control-action/ack/` | FSM 수락. 물리적 완료와 별개 |
| POST `companion-phase/` | v1 단계 보고 |
| WebSocket `/ws/drones/5/` | 최신 텔레메트리 송신. 수신 프레임은 명령이 아님 |

좌표 계약은 `UWB_ANCHOR_LOCAL`이다.
A1 원점, A1→A2가 +X, A1→A3가 +Y다.
단위는 meter다. 배치 ID도 일치해야 한다.
`mission_db_id`, `mission_code`, `route_revision`을 검사한다.
경유지 1~20개와 고유 ID를 요구한다.
`waypoint`, `hover`만 받는다. `scan`은 거부한다.
경유지의 z는 고도 명령에 사용하지 않는다.

요청 ID를 디스크에 먼저 기록한다.
기본 보관 파일은 아래 경로다.

```text
~/.local/state/dronestock-flight/requests.json
```

같은 시작 요청과 프로세스 재시작은 재이륙하지 않는다.
수락한 경로의 반복 조회는 수신 시간을 갱신한다.
비행 중 경로·revision 교체는 거부한다.
웹 단절·새 좌표 입력 오류는 자동 복구로 재개하지 않는다.

## PX4 명령과 도착 판정

PX4가 이륙·고도 유지·착륙을 수행한다.

```text
IDLE -> READY -> ARMING -> TAKING_OFF -> MOVING -> LANDING -> LANDED
                                     경유지 반복 --^
                   입력 상실·명령 실패 ----------> LANDING -> FAILED
                   수동 모드 전환 --------------> PILOT_OVERRIDE
```

| 단계 | PX4 요청·증거 |
|---|---|
| arm | 일반 `CommandBool`. 강제 arm 미사용 |
| takeoff | `CommandTOL`, 위치·yaw·altitude=NaN |
| 이륙 완료 | armed, IN_AIR, AUTO.LOITER 확인 |
| 수평 이동 | `COMMAND_INT/DO_REPOSITION`, E7 위경도, z=NaN |
| 목표 반영 | 새 `/mavros/setpoint_raw/target_global`의 XY 일치 |
| 도착 | PX4 위치 반경 0.15m, 속도 ≤0.1m/s, 1초 유지 |
| land | PX4 `CommandTOL` 착륙 |
| 착륙 확인 | ON_GROUND와 disarmed를 함께 확인 |

MAVROS `CommandInt.success`는 송신 성공이다.
실제 목표 적용과 도착은 별도로 확인한다.
위경도는 FC가 보고한 전역 원점에서 계산한다.
전역 원점이 없으면 시작을 막는다.
가짜 GNSS·원점을 주입해 통과시키지 않는다.
이유: FC 좌표와 목표의 관계를 검증할 수 없기 때문이다.
실내 FC의 전역 목표 지원도 현장에서 확인해야 한다.

이륙 높이는 PX4 `MIS_TAKEOFF_ALT`를 사용한다.
시험 기대값은 상승 높이 0.6m다.
바닥 기준 태그 높이 0.6m와는 다르다.
FC 파라미터를 자동 변경하지 않는다.
실제 펌웨어의 NaN 처리와 고도 설정을 확인해야 한다.

UWB, 높이, PX4 위치와 브리지 전달의 시각을 검사한다.
브리지의 과거 누적 송신 횟수만으로 시작하지 않는다.
유효한 웹 조회가 2초간 없으면 착륙을 요청한다.
적용된 목표가 바뀌면 착륙을 요청한다.
목표 피드백이 0.5초 끊겨도 같은 처리를 한다.
수동 모드에서는 조종권을 넘기고 추가 명령을 멈춘다.
출발점 복귀가 시험 한도를 넘으면 현재 위치 착륙을 요청한다.

arm 중 웹 착륙 요청은 후속 이륙을 취소한다.
강제 기능 없이 일반 disarm을 요청한다.
그 수락과 이후의 새 지상 상태를 함께 확인한다.
거부·무응답 때 취소 완료를 확정하지 않는다.
같은 XY 경유지를 반복해도 대기 시간을 새로 시작한다.

공중에서 실행기가 재시작되면 경로를 복원하지 않는다.
`execute=true`의 주체 없는 자동 모드는 착륙을 요청한다.
수동 모드와 `execute=false`는 조종권을 가져오지 않는다.
원래 미션 ID를 복원하지 못하면 null로 표시한다.
상실된 정보로 성공·자동 재개를 추측하지 않는다.

companion 프로세스가 죽으면 이 FSM도 실행되지 않는다.
그 경우의 착륙은 PX4 datalink failsafe에 달려 있다.
QGroundControl 등 다른 연결이 상실 감지를 가릴 수 있다.
현재 모사 시험은 이 FC failsafe를 검증하지 않는다.

## 검증 범위

ROS·HTTP·WebSocket 연결과 명령 순서를 검사했다.
RAW UWB·ToF·IMU·TIMESYNC도 ROS로 통과시켰다.
FC는 시험용 프로토콜 노드다.
정상·UWB 단절·웹 단절·공중 재시작을 검사했다.
실제 EKF2·공력·충돌·실물 비행은 검증하지 않았다.
확인 기록은 [2026-10-08 결과](../report/web_test_flight_20261008.md)다.
