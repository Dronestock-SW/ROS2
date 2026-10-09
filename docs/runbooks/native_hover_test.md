# 시동·이륙·호버·착륙 시험
기본 비행 순서를 한 번 실행하는 절차다.
전체 미션 전에 명령·상태 확인을 시험할 때 읽는다.

시험은 PX4 기본 이륙 모드를 사용한다.
companion은 setpoint를 만들지 않는다.
PX4 이륙 높이는 1.3m다.
안정 0.5초 뒤 연속 2초를 호버한다.
착륙과 시동 해제를 모두 확인한다.

```text
현재 상태 조회 -> 새 세션 START
 -> AUTO.TAKEOFF 실제 모드 확인
 -> 일반 ARM -> 실제 armed 확인
 -> AUTO.LOITER + 상승·정지 확인
 -> 안정 0.5초 + 호버 2초
 -> AUTO.LAND -> ON_GROUND + disarmed
 -> PASS / FAIL / CANCELLED / UNCONFIRMED
```

## 1. 빌드와 모사 시험

Jetson의 검토 폴더에서만 실행한다.
기존 현장 폴더와 main을 교체하지 않는다.
이유: 현장 미커밋 소스와 서비스 설정을 보존한다.

```bash
cd /home/arialhanho/ROS2-review-20261008-codex
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --symlink-install --packages-select sangwon_ai_replay
source install/setup.bash
ctest --test-dir build/sangwon_ai_replay \
  -R 'native_hover_sequence|native_hover_ros' --output-on-failure
```

ROS 시험은 domain 173·localhost 전용이다.
실제 C++·HTTP·IPC 경로를 실행한다.
MAVROS 응답과 위치는 모사 입력이다.
실제 PX4 펌웨어 시험과 구분한다.

## 2. 실제 PX4 가상 시험

PX4 v1.17.0의 SIH-as-SITL을 사용한다.
Jetson의 프로세스로 가상 기체를 실행한다.
실물 FC의 SIH 모드를 켜는 절차가 아니다.
직렬 장치를 열거나 실물 params를 바꾸지 않는다.

```bash
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/arialhanho/ROS2-integration-20261007/.test-deps:${PYTHONPATH:-}
python3 src/sangwon_AI/tests/native_hover_px4.py \
  --position-source ideal-ev \
  --px4-root .review/hover-20261008/PX4-Autopilot \
  --binary install/sangwon_ai_replay/bin/sangwon_native_hover \
  --output .review/hover-20261008/px4-trial
```

RC 입력은 명시적으로 모사한다.
`ideal-ev`는 SIH의 실제 가상 기체 위치를 사용한다.
MAVLink ODOMETRY로 PX4 EKF에 전달한다.
실행기는 그 값을 직접 받지 않는다.
EKF·명령 수락·비행 동역학은 실제 SITL이다.
이 입력은 실물 UWB·flow의 성능을 입증하지 않는다.
가상 센서 설정은 `summary.json`에 남는다.
기본 `gnss` 시험의 표류 실패 기록도 보존한다.

`--scenario operator_cancel`은 호버 중 LAND를 시험한다.
`--scenario rc_stick`은 PX4의 스틱 인계를 시험한다.
인계 뒤 시험 조종자가 가상 기체를 착륙시킨다.
companion은 추가 명령을 보내지 않는다.

## 3. 실물 관측 화면

### GPS 없는 실내 기준점의 가상 시험

GNSS 융합을 끈 별도 SITL은 다음 옵션을 사용한다.
새 출력 폴더마다 별도 PX4 rootfs를 만든다.
기존 SITL 파라미터와 실물 설정은 건드리지 않는다.

```bash
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/arialhanho/ROS2-integration-20261007/.test-deps:${PYTHONPATH:-}
python3 src/sangwon_AI/tests/native_hover_px4.py \
  --local-origin --position-source ideal-ev \
  --px4-root .review/hover-20261008/PX4-Autopilot \
  --binary install/sangwon_ai_replay/bin/sangwon_native_hover \
  --output /dev/shm/new-native-local-reference
```

`EKF2_GPS_CTRL=0`은 EKF 시작 전부터 적용한다.
SIH의 첫 실제 가상 위치로 지역 기준점을 정한다.
이 값은 GPS 센서 관측이나 실물 지리 측량값이 아니다.
출발점 XY는 START 때 PX4 위치에서 저장한다.
기준점을 넣어도 유효한 위치 관측은 따로 필요하다.
`--local-origin`과 `gnss` 입력의 조합은 거절한다.
10월9일 가상 결과는 [현장 기록](../report/field_readiness_20261009.md)을 따른다.
이 시험의 ideal EV는 실물 UWB·flow 검증을 대신하지 않는다.

### 실물 관측 실행

기본 실행은 명령 출력을 잠근다.
포트 소유자를 먼저 확인한다.
이미 MAVROS가 실행 중이면 재사용한다.
`--start-mavros`를 붙여 중복 실행하지 않는다.
이유: 직렬 포트와 상태 발행자가 충돌한다.

```bash
export ROS_DOMAIN_ID=2
unset ROS_LOCALHOST_ONLY
fuser /dev/pixhawk
python3 src/sangwon_AI/ops/native_hover_launch.py \
  --start-mavros --state-dir /tmp/dronestock-native-hover
```

로컬 PC의 별도 터미널에서 화면을 연결한다.

```powershell
ssh -N -L 8350:127.0.0.1:8350 arialhanho@100.110.163.94
```

브라우저에서 `http://127.0.0.1:8350`을 연다.
높이·PX4 모드·RC·배터리·차단 사유를 확인한다.
이 프로필은 Tag B/domain 2의 RC 매핑을 사용한다.
다른 기체에 같은 매핑을 가정하지 않는다.

## 4. 실물 비행 조건

확인하지 않은 항목은 확인 완료로 표시하지 않는다.

| 항목 | 통과 조건 |
|---|---|
| 현장 | 배터리·기체·시험 공간·조종자 준비 |
| PX4 추정 | 최신 위치·속도·고도·예측 수평 위치 플래그 |
| 기본 상태 | disarmed·ON_GROUND·POSCTL·정지 |
| 정지 유지 | 최신 입력으로 3초 유지. XY 변화 0.1m 이하 |
| RC | 최신 유효 8채널·스틱 중앙·스위치 매핑 일치 |
| RC 인계 | 실제 모드 스위치·스틱 인계 시험 기록 |
| FC 자동 인계 | `COM_RC_OVERRIDE`의 bit 0 활성 |
| 배터리 | 최신 실측 상태·잔량 20% 이상 |
| 이륙 | `MIS_TAKEOFF_ALT=1.3`, `COM_TAKEOFF_ACT=0` |
| 로그·명령 | 로그 쓰기 가능·MAVROS 서비스 사용 가능 |
| 주체 | 다른 writer·외부 GCS 명령과 충돌 없음 |
| 자체 대응 | RC·companion·USB 상실의 FC 정책 확인 |

2026-10-08 직접 조회의 `COM_RC_OVERRIDE=2`는
자동 모드 스틱 인계 bit 0을 끈 값이다.
기존 offboard bit도 유지하려면 후보 값은 3이다.
FC 설정과 물리 인계 시험을 함께 확인한다.
원본 params를 보존한 뒤 변경·조회 근거를 남긴다.
[PX4 v1.17 파라미터 정의](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#COM_RC_OVERRIDE)를 따른다.

증거를 확인한 현장 조종자가 출력을 활성화한다.
인계 확인 플래그는 RC 수신만으로 켜지 않는다.
이유: 유효 채널과 실제 조종권 인계는 다르다.

```bash
python3 src/sangwon_AI/ops/native_hover_launch.py \
  --start-mavros --state-dir /tmp/dronestock-native-hover \
  --enable-output --rc-handoff-verified
```

실행 자체로 시동·이륙하지 않는다.
새 세션의 화면에서 START를 별도로 누른다.
한 세션에서 한 번만 실행한다.
버튼 응답은 FC의 동작 완료가 아니다.

## 5. 중단과 결과 해석

`중단 / LAND`는 시점에 따라 처리한다.
시동 응답 대기 중이면 응답을 먼저 확인한다.
지상에서 시동이 켜졌으면 일반 disarm을 요청한다.
공중이면 PX4 착륙을 요청한다.
인계받은 조종자에게 자동 조종권을 다시 요구하지 않는다.

| 결과 | 의미 |
|---|---|
| PASS | 정상 호버를 마치고 착륙·시동 해제 확인 |
| FAIL | 명령·추정·배터리 등 실패로 중단. 착륙 확인 별도 |
| CANCELLED | 조종자가 중단. 정상 미션 성공과 구분 |
| UNCONFIRMED | 상태·명령 결과·인계 불명. 완료를 보장하지 못함 |

시동 응답 timeout은 시동 거부의 증거가 아니다.
늦은 FC 응답으로 시동이 켜질 수 있다.
이때 재시동·강제 disarm을 자동 반복하지 않는다.
현장 조종자가 RC와 실제 기체 상태를 확인한다.

로그는 `--state-dir`의 `events.jsonl`에 남는다.
MAVROS·실행기·웹 로그도 같은 폴더에 남는다.
중복·역순 메시지는 상태를 덮어쓰지 않는다.
만료 시각은 마지막 유효 입력을 기준으로 한다.

실행기·프로세스·전원 전체 소실은 FC와 RC가 담당한다.
이번 단계는 현장 조종자가 지켜보는 기본 순서 시험이다.
이 시험 프로필에는 유효 RC를 요구한다.
전체 임무 설계의 RC 미연결 정책과 구분한다.
무인 운용·UWB 이동·스캔 미션 완료로 확대 해석하지 않는다.
이유: 각 단계의 실제 융합·목표·장애 대응 검증이 남았다.
[공식 SIH 설명](https://docs.px4.io/v1.17/en/sim_sih/)과
[전체 미션 인계](local_jetson_handoff_20261008.md)를 함께 읽는다.

실물 수평 센서는 [융합 점검 절차](position_sensor_check.md)를 따른다.
