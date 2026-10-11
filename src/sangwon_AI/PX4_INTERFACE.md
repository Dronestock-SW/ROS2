# Jetson↔PX4 연결 상세 명세
이 문서는 목표 메시지·좌표 변환·제어권·실패 처리의 내부 계약이다.
MAVROS 연결 구현과 SITL 검수를 준비할 때 읽는다.

상태: 내부 구현 명세 v1.0, 실장 검증 전 / 2026-10-02.
근거: Jetson 설치 MAVROS 2.14.0, 제공 ULog의 PX4 v1.17.0.
현재 연결된 PX4의 펌웨어·설정은 별도 재확인이 필요하다.
아래 메시지를 내부 구현 기준으로 사용한다. 실기체 시험 완료를 뜻하지 않는다.

## 2026-10-05: C++ 모드 전환 구현

`ModeManager`와 `FlightGuard`가 요청·실제 모드 확인을 관리한다. 하드웨어 전송기는 아직 없고 HOST_OBSERVE/SITL/FLIGHT 권한은 변경하지 않았다. REPLAY의 기존 OFFBOARD 이륙은 기본값으로 유지하며, 승인된 합성 설정에서 다음 흐름을 선택할 수 있다.

```text
새 START → TAKEOFF 요청 → 실제 AUTO.TAKEOFF 확인 → ARM 요청/armed 확인
 → PX4 이륙 완료 관측 + 목표 높이·속도 안정 확인
 → 현재 pose를 고정해 1.5초 prestream
 → OFFBOARD 요청/실제 확인 → 호버·이동·회전·스캔·복귀
 → AUTO.LAND 요청/확인 → landed + disarmed 확인 → 종료
```

- `config`의 `replay_takeoff_policy=PX4_AUTO_TAKEOFF`와 `approved_replay_takeoff_reference=WAREHOUSE_MAP_SYNTHETIC_ONLY`를 모두 요구한다. 웹의 START `start_mode=AUTO_TAKEOFF`는 출발 방식이며 FC 모드 정책을 직접 변경하지 않는다.
- `ControlRequest`는 execution/boot/request ID, frame epoch, 변환 개정, 원래 target, 발급 시각과 절대 deadline을 가진다. 응답은 원래 요청과 일치하고 최신이어야 한다. ACK 수락만으로 완료하지 않고 실제 mode/armed 상태를 확인한다. 현재 시험값은 전환 6초, 1초 간격 최대 3회이며 재시도로 deadline을 연장하지 않는다.
- `Output.request`와 `Output.stream_position`은 별개다. OFFBOARD/ARM 확인을 기다리면서도 고정 목표 stream을 유지하고, AUTO.LAND 확인 뒤에는 stream을 중단한다. 위치/yaw/좌표계가 무효이면 Land 대기 중에도 위치 stream을 보내지 않는다. prestream 공백은 누적 시간을 초기화한다.
- `IntentKind::Hold`는 OFFBOARD 고정 위치 목표다. PX4 AUTO.LOITER로 전환하는 명령이 아니다. POSITION은 RC 수동 인계 모드이며 정상 자율 호버/스캔에 사용하지 않는다.
- TAKEOFF 완료 후 AUTO.LOITER가 관측되면 원래 TAKEOFF 요청에 결합된 최신 완료 근거와 유효한 전역 위치가 있어야 예상된 자동 전환으로 인정한다. 단순 HOLD 관측은 인계 허가가 아니다. RC/PX4 인계·예상 밖 이탈 시 출력을 반환하고 같은 비행에서 재획득하지 않는다.
- native 이륙 중에는 목표 위치 stream으로 상승하지 않는다. 완료 신호 누락/취소/위치·경로 유효성 상실/기록 실패/저잔량은 Land 처리한다. 이륙·인계 중 PAUSE/RESUME는 거부한다. 비행 중 disarm이나 파라미터 복원은 하지 않는다.
- 확인 실패는 지상에서는 이륙 중단/필요한 지상 disarm, 공중에서는 Land 요청으로 처리한다. Land도 거부되거나 확인 기한을 넘으면 stream을 중단하고 제어권을 반환한다. 상태 소실·RC 인계에서는 새 명령을 보내지 않는다.

실제 FC 연결 전 추가 검수: 웹 바닥 z→PX4 native TAKEOFF 고도 기준(AMSL/로컬 원점)의 변환과 실제 목표, TAKEOFF 완료 신호와 후속 mode, 단일 MAVROS 요청 채널의 FC/boot/시각 결합, ACK 지연·순서 및 독립 failsafe. native TAKEOFF 구간은 OFFBOARD가 아니므로 Jetson 전체 상실을 Offboard-loss 설정만으로 처리할 수 있다고 가정하지 않는다. FC의 native 구간 독립 대응도 시험해야 한다. 내부 request ID는 MAVLink ACK가 직접 회신하는 ID가 아니므로 수신 ACK에 현재 ID를 임의로 붙이지 않는다. `MIS_TAKEOFF_ALT`를 임무 도중 덮어써 목표 높이를 맞추지 않는다.

근거: [현재 펌웨어 commit의 모드 요구사항](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/ModeUtil/mode_requirements.cpp), [native TAKEOFF 고도·완료 처리](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/navigator/takeoff.cpp). 로터 기체의 AUTO.LOITER는 전역 위치를 요구한다. UWB 로컬 위치만으로 진입 가능하다고 가정하지 않는다.

## 1. 출력 경계

```text
BT/경로계획 -> 승인된 목표 의도 -> 단일 명령 중재·송신기
                                         |
                       MAVROS PositionTarget (로컬 ENU)
                                         |
                            MAVLink 목표 (로컬 NED)
                                         |
                              PX4 위치·자세 제어
```

출력 모듈만 위치 목표·모드·arm 요청을 보낸다.
웹·BT 노드는 MAVROS에 직접 경쟁 송신하지 않는다.
직접 모터·정규화 추력 명령은 이 명세의 출력이 아니다.
UWB 관측의 PX4 입력 경로는 전체 사양 수령 뒤 별도로 결정한다.
MAVROS 목표 송신과 PX4 ROS2 직접 Offboard 입력을 동시에 사용하지 않는다.

## 2. 위치·yaw 목표

설계 기준은 `/mavros/setpoint_raw/local`, `mavros_msgs/msg/PositionTarget`이다.
실제 launch의 namespace·remap·QoS·플러그인 활성 여부는 J05에서 확인한다.

| 필드 | 구현 값·의미 |
|---|---|
| header.stamp | 현재 송신 시각. 원래 행동 목표 생성 시각은 별도 보존 |
| header.frame_id | 검증된 PX4 로컬 ENU 프레임의 이름 |
| coordinate_frame | FRAME_LOCAL_NED=1. MAVLink 대상 프레임 선택값 |
| type_mask | IGNORE_VX/VY/VZ, IGNORE_AFX/AFY/AFZ, IGNORE_YAW_RATE의 OR = 2552 |
| position.x/y/z | 지도에서 **PX4 로컬 ENU**로 변환한 m 단위 목표 |
| yaw | 같은 로컬 ENU 기준 rad, +x=0, 위에서 본 반시계 방향 양수 |
| velocity·acceleration_or_force·yaw_rate | 0으로 초기화하되 위 마스크로 무시 |
| IGNORE_PX/PY/PZ·IGNORE_YAW·FORCE | 설정하지 않음 |

웹 yaw가 없더라도 내부 방향 선택을 거쳐 유한한 yaw 목표를 보낸다.
수평 이동은 진행 방향, 수직 이동은 기존 방향, 도착 yaw는 안전 회전 뒤 적용한다.
제자리 정지 시 선택한 위치·yaw를 고정한다. 매번 센서 위치로 목표를 옮기지 않는다.
필드명에 NED가 있어도 이 MAVROS 입력의 위치 숫자를 미리 NED로 바꾸지 않는다.
MAVROS 2.14.0의 local_cb가 ENU→NED 및 yaw 변환을 수행한다.
header.frame_id만 바꿔도 임의의 지도 좌표가 자동 변환되는 구조는 아니다.
근거: [설치 버전의 setpoint_raw 구현](https://raw.githubusercontent.com/mavlink/mavros/2.14.0/mavros/src/plugins/setpoint_raw.cpp).

이 마스크는 위치·yaw 제어용이다. 속도 제한을 velocity 필드에 넣어도 적용되지 않는다.
시간에 따른 목표 이동과 실제 PX4 제한값을 함께 검증한다.
경유점을 갑자기 멀리 점프시키는 방식의 제동 성능을 가정하지 않는다.

## 3. 좌표 변환

| 이름 | 의미 | 결정 상태 |
|---|---|---|
| warehouse_map | 직교 지도 XY, 공통 바닥 기준 +z | 실제 원점·yaw 표기는 웹 협의 |
| px4_local_enu | MAVROS가 노출하는 PX4 로컬 좌표의 내부 이름 | ROS header 이름과 물리 의미를 대조 |
| px4_local_ned | PX4/MAVLink 로컬 좌표 | MAVROS에서 변환 |
| vehicle_reference | 일반 이동점 z가 가리키는 기체/PX4 기준점 | 장착 오프셋 확정 필요. scan 라벨 기준점과 분리 |

같은 기준점·수평면을 쓰는 경우의 변환식은 다음과 같다.

```text
p_enu = Rz(theta_map_to_enu) * p_map + t_map_to_enu
yaw_enu = wrap(yaw_map_rad + theta_map_to_enu)
p_ned = (p_enu.y, p_enu.x, -p_enu.z)       # MAVROS 측 변환 확인용
yaw_ned = wrap(pi/2 - yaw_enu)            # 평면 heading 확인용
```

웹 각도는 먼저 합의한 0방향·부호·단위를 내부 rad로 정규화한다.
theta는 **지도 축과 실제 PX4 로컬 축의 관계**다.
웹 시작 방향 화살표를 theta 보정값으로 덮어쓰지 않는다.
태그·PX4·기체 중심의 측정점이 다르면 자세에 따른 장착 변환을 먼저 반영한다.
기울어진 바닥·비직교 UWB 축을 위 단순 회전으로 억지로 맞추지 않는다.

웹 z를 PX4 z에 바로 대입하지 않는다.
scan XYZ는 라벨 위치이므로 먼저 QR_SCAN_SPEC의 라벨 방향·판독 거리·장착 변환으로 기체 pose를 산출한다.
출력 경계에는 검증된 기체 pose만 전달한다. 라벨 위치를 PositionTarget에 직접 넣지 않는다.
바닥 기준과 PX4 로컬 원점 사이 높이·기체 기준점 차이를 반영한다.
ToF 거리 한 개를 변환 오프셋으로 자동 확정하지 않는다.
PX4 재부팅·로컬 원점 변경 시 이전 비행의 변환을 재사용하지 않는다.
내부 v1은 비행 중 수평/고도 reset의 자동 변환 보정을 지원하지 않는다.
유효 변환을 잃으면 이전 경로 목표를 폐기하고 Land한다. 지상에서는 다시 정렬·검증한다.
비행 중 yaw reset·확인된 yaw 품질 상실은 사용자 결정대로 이동 중단·Land다.

변환본에는 지도 개정·기체 설정·PX4 부팅 문맥·측량 근거·검증 상태를 연결한다.
정방향/역방향 왕복 시험과 실제 +x/+y/+z/회전 방향 시험을 모두 한다.
수학적 왕복 성공만으로 현장 축 방향이 옳다고 승인하지 않는다.

## 4. 관측과 품질

아래 경로는 기본 namespace 기준의 연결 후보다.

| 입력 | 타입·핵심 값 | 사용하는 판정 |
|---|---|---|
| /mavros/state | mavros_msgs/State: connected, mode, armed | 연결·Offboard 전환·시동 확인 |
| /mavros/extended_state | mavros_msgs/ExtendedState: landed_state | 지상/공중·착륙 진행 |
| /mavros/local_position/pose | geometry_msgs/PoseStamped | 로컬 ENU 위치·자세 |
| /mavros/local_position/velocity_local | geometry_msgs/TwistStamped | 도착·정지 속도 |
| /mavros/battery | sensor_msgs/BatteryState: percentage 등 | 유효한 잔량의 30% 경계 |
| estimator 상태 입력 | mavros_msgs/EstimatorStatus 등, 실제 발행 경로 확인 | 위치·속도 상태의 보조 판정 |
| PX4 전용 품질 입력 | uXRCE-DDS 수신 어댑터, 아래 필드 대응 | yaw reset·yaw 품질·로컬 reset·배터리 센서 상실·failsafe 원인 |

설치된 EstimatorStatus에는 위치/속도 등의 flag는 있으나 yaw reset counter는 없다.
State.manual_input만으로 실제 RC 스틱 인계를 판정하지 않는다.
유한한 PoseStamped 수신만으로 EKF 위치 유효·정확성을 승인하지 않는다.
추가 품질 수신은 PX4 uXRCE-DDS→C++ 어댑터 경로로 설계한다.
MAVROS가 제어 출력의 유일 경로이며 DDS의 /fmu/in 제어 토픽은 사용하지 않는다.
실제 보드의 연결 채널·메시지 버전·품질 판정 매핑을 검증하기 전에는 FLIGHT를 승인하지 않는다.

| DDS 논리 토픽 | 읽을 필드 | 내부 사용 |
|---|---|---|
| vehicle_local_position | xy/z/v_xy/v_z_valid, reset counters, heading_good_for_control, heading_var | 위치·속도 유효성, 프레임 reset, yaw 품질 근거 |
| vehicle_status | nav_state, nav_state_user_intention, failsafe, failsafe_and_user_took_over, arming_state | 실제 모드와 명령 문맥 대조·제어권 반환 |
| failsafe_flags | local_position_invalid, local_altitude_invalid, local_velocity_invalid, battery_unhealthy, manual_control_signal_lost | 기체 내부의 명시적 이상, RC 링크 상실 구분 |
| battery_status | id/source, connected, remaining, timestamp, faults | 운용 중인 배터리 인스턴스 상태·잔량·실패 근거 |

해당 ULog 펌웨어 소스의 [DDS publication 목록](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml)에 위 토픽이 있다.
실제 /fmu/out 토픽의 버전 접미사·형식은 실행할 펌웨어와 px4_msgs를 맞춰 확인한다.
[로컬 위치 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleLocalPosition.msg),
[기체 상태 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleStatus.msg),
[실패 상태 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/FailsafeFlags.msg)를 기준으로 구현한다.

DDS 좌표는 NED다. MAVROS ENU와 숫자를 직접 비교하지 않고 어댑터에서 한 번 변환한다.
같은 기체·같은 PX4 부팅·시간 정렬 여부를 확인하고 오래된 다른 소스와 혼합하지 않는다.
reset counter는 같은 부팅 문맥의 이전 관측과 비교한다. 최초 수신 자체를 reset으로 판정하지 않는다.
전송 공백으로 reset 발생 여부를 알 수 없으면 품질 UNKNOWN으로 처리한다.

heading_good_for_control 하나의 의미를 기존 로그만으로 확정하지 않는다.
FLIGHT 품질 프로파일에 사용할 PX4 필드·유효 나이·정상/이상 판정·시험 근거를 지정한다.
프로파일이 없거나 필요한 필드가 없으면 yaw 품질은 UNKNOWN이며 시작할 수 없다.
운용 중 승인된 판정이 INVALID이거나 프레임 reset이면 이동 중단·Land다.
필수 제어 품질 자체가 오래되어 판단 불가하면 좌표 유지를 장담하지 않고 Land 경로로 넘긴다.

배터리 토픽 미수신은 Jetson 통신 문제일 수 있어 센서 상실로 단정하지 않는다.
현재 전원 인스턴스와 최신 실패 상태를 함께 확인한다.
connected는 전원 모듈의 전압 기준일 수 있으므로 이 값 하나로 고장 원인을 확정하지 않는다.
명시적인 PX4 배터리 이상은 Land, Jetson 잔량만 미수신이면 기존 PX4 독립 대응 정책을 따른다.
[배터리 상태 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/BatteryStatus.msg).

RC 인계의 정확한 원인이 확인되지 않아도 예상 밖 Offboard 이탈 시 자율 출력을 반환한다.
스틱 인계 원인의 분류 정확도와 제어권 반환 동작은 별도 검수한다.

각 값은 원천 시각·수신 단조 시각·나이·유효 여부를 보관한다.
배터리 percentage는 ROS 메시지의 유효 범위와 NaN/미상 표현을 확인한 뒤 정규화한다.
Jetson 텔레메트리 누락과 PX4 센서 자체 상실을 같은 조건으로 처리하지 않는다.

## 5. 제어권 전환

| 요청 | 서비스·내용 | 성공 판정 |
|---|---|---|
| Offboard | /mavros/set_mode, base_mode=0, custom_mode=OFFBOARD | 최신 state.mode=OFFBOARD |
| 시동 | /mavros/cmd/arming, CommandBool.value=true | 응답 결과와 최신 state.armed=true |
| 현재 위치 착륙 | /mavros/set_mode, custom_mode=AUTO.LAND | 실제 모드 전환, 이후 landed·disarmed |
| 지상 시동 해제 | CommandBool.value=false | 지상 확인 후 요청, 최신 armed=false |

SetMode.mode_sent는 메시지를 보냈다는 뜻이다.
PX4 수락 ACK나 모드 전환 완료로 쓰지 않는다.
arm의 success/result도 실제 상태와 대조한다.
공중에서 disarm을 실패 처리 수단으로 사용하지 않는다.
근거: [SetMode 구현](https://raw.githubusercontent.com/mavlink/mavros/2.14.0/mavros/src/plugins/sys_status.cpp), [arming 구현](https://raw.githubusercontent.com/mavlink/mavros/2.14.0/mavros/src/plugins/command.cpp).

### 기본 자동 이륙

1. 지상·시동 해제·입력 준비·공간·출발점·변환본을 확인한다.
2. 현재 위치·방향 유지 목표를 연속 송신한다.
3. 선행 송신 조건을 충족한 뒤 OFFBOARD를 요청하고 실제 모드를 확인한다.
4. arm 요청과 실제 armed를 확인한다. 두 요청은 동시에 내지 않는다.
5. 상승 목표를 생성해 별도 takeoff z까지 이동한다. 초기 지상 좌표를 무기한 유지하지 않는다.
6. 상승 안정과 위치 유지를 확인한 뒤 시작 yaw·임무를 진행한다.

이 순서는 Offboard 위치 목표로 PX4가 이륙하도록 하는 안이다.
AUTO.TAKEOFF나 전역 좌표 기반 CommandTOL 이륙과 섞지 않는다.
시동 거부·자동 disarm·순서별 실제 반응은 SITL/지상에서 검증한다.
상승 전 취소는 지상 상태를 확인해 종료하고, 상승 중 취소는 AUTO.LAND를 요청한다.
취소 뒤 arm 응답이 늦게 오면 상승 목표를 차단한 채 실제 지상·armed를 재확인한다.
실제 지상인 경우에만 disarm을 요청하고, 상태 불명은 취소 완료로 처리하지 않는다.

### 예외 RC 인계

조종자가 실제 이륙점 위 제자리 호버를 유지한다.
Jetson은 웹 시작·현재 방향·기록된 출발점·현 고도 경로를 검증한다.
현재 위치·방향 유지 목표를 선행 송신한 뒤 OFFBOARD 전환을 확인한다.
실제 인계 전 목표 고도나 XY로 이동시키지 않는다.
실패하면 RC 제어권을 유지하고 인계 실패를 보고한다.

### 반환·착륙

ReturnHome은 Jetson이 검증된 실내 경로로 실제 이륙점에 복귀하는 행동이다.
PX4 AUTO.RTL의 경로·홈 좌표를 대신 사용하지 않는다.
착륙 요청 후 실제 AUTO.LAND 전환을 확인하면 Offboard 목표 송신을 끝낸다.
모드 확인 전에는 위치 유지가 가능한 경우에만 짧게 정지 목표를 보낸다.
T_mode 내 전환이 확인되지 않으면 승인된 재시도 한도 이후 송신을 중단해 PX4 Offboard 상실 대응에 맡긴다.
연결이 끊겼으면 서비스 응답을 기다리며 이전 이동 목표를 반복하지 않는다.

RC 인계·PX4 자체 비상 모드는 Jetson보다 우선한다.
Jetson이 요청한 AUTO.LAND와 PX4가 독립적으로 선택한 비상 모드는 전환 문맥으로 구분한다.
예상치 못한 Offboard 이탈을 자동 Offboard 재진입으로 복구하지 않는다.
원인 불명도 제어권 회수 금지로 처리하고 실제 모드를 보고한다.
RC 인계가 확인되면 착륙 후 새 임무까지 잠근다.

## 6. 송신·watchdog·실패

PX4는 Offboard 진입 전 1초를 넘는 선행 신호와 2Hz를 넘는 지속 입력을 요구한다.
이는 PX4 최소 조건이며 제품 송신 주기·허용 지연은 별도 실측으로 정한다.
MAVROS/MAVLink 경로에서는 위치 유지 때도 목표를 지속 송신한다.
근거: [PX4 v1.17 Offboard](https://docs.px4.io/v1.17/en/flight_modes/offboard).

| 실패 | 출력 처리 | 확인할 결과 |
|---|---|---|
| BT/planner 목표 lease 만료 | 이전 이동 목표 폐기, 가능한 정지·Land 요청 | 송신기가 살아도 낡은 목표 재사용 없음 |
| 위치 상실 | 유지 가능할 때 최대 8초 복구 대기 | 유지 불가/기한 만료 시 Land |
| yaw reset/품질 상실 | 이동 중단·Land | 같은 비행에서 재정렬해 임무 재개 없음 |
| MAVROS/Jetson 송신 상실 | PX4 독립 Offboard failsafe | COM_OF_LOSS_T=1초 시험값과 추가 지연 실측 |
| RC 인계/PX4 failsafe 활성 | 자율 권한 반환·명령 경쟁 차단 | 자율 재진입·재arm 없음 |
| 서비스 timeout | 실제 상태 재조회, 같은 전환의 제한 재시도 | 늦게 온 응답이 새 행동을 덮어쓰지 않음 |

COM_OBL_RC_ACT=4(Land) 등 설정 후보·원본값은 [DESIGN.md](DESIGN.md) 6절을 따른다.
2026-10-04에 제공된 현 수동비행 파라미터와 커스텀 펌웨어 기준은
[PX4_PARAMETER_PROFILE.md](PX4_PARAMETER_PROFILE.md)를 함께 따른다.
설정 적용기는 지상 전용으로 구현할 예정이며 임무 목표 송신·RC 인계 경로에서 파라미터를 바꾸지 않는다.
PX4 설정은 아직 변경하지 않았다.
T_mode·T_lease·재시도 한도·송신 주기는 [MEASUREMENT_REGISTER.md](MEASUREMENT_REGISTER.md)에서 관리한다.

## 7. 구현된 구독 전용 접점 — 2026-10-05

| 토픽 기본값 | ROS 타입 | 기록 필드 |
|---|---|---|
| `/mavros/state` | `mavros_msgs/msg/State` | connected/armed/guided/manual_input/mode/system_status |
| `/mavros/extended_state` | `mavros_msgs/msg/ExtendedState` | landed_state/vtol_state |
| `/mavros/battery` | `sensor_msgs/msg/BatteryState` | present/voltage/current/percentage/health; 미측정 current/percentage=null |
| `/mavros/rc/in` | `mavros_msgs/msg/RCIn` | 채널 수/RSSI/미확인 여부; 원시 스틱/스위치 값 저장 없음 |

C++ `sangwon_px4_observer`는 `MAVROS_TOPICS_UNVERIFIED_AIRCRAFT` 출처다. 발행자가 하나이고 현재 시각 범위 안에서 증가한 header만 LIVE로 표시한다. 반복/미래/오래된 header·불량값·발행자0/복수는 이전 필드를 폐기하고 새 메시지로 복구한다. NaN 잔량을 0%/정상으로 대체하지 않는다.
State.connected와 header는 MAVROS bridge 보고다. 동일 FC identity/boot/source time/prearm를 증명하지 않으므로 `source_sample_time_verified=false`, `aircraft_identity_verified=false`를 유지한다. RCIn은 실제 PX4 override 확인을 대신하지 않는다.
`sangwon-px4-health/1` private JSON의 boot/session/순번·monotonic 나이를 Python helper가 검증하고 optional host diagnostics만 생성한다. 합성 출처/domain은 실제 host 점검에서 제외한다. 격리 ROS 시험에서 관측기의 publisher/service/client endpoint가 없음을 확인했다. READY/비행 권한/물리 출력은 false다.
실제 MAVROS transport 자동 기동, FC identity/firmware/boot 결합, prearm/failsafe/위치/yaw 품질 연결은 후속이다. UWB는 전체 사양 수령 전 보류한다.
