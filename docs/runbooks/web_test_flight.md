# 로컬 웹 이륙·이동·착륙 시험

Jetson의 웹 비행 시험 절차다. 실물 연결과 지상 확인을 준비할 때 읽는다.

로컬 웹과 전체 순서 실행기를 구현했다.
실물 융합·비행은 아직 미실시다.
먼저 지상 관측 전달을 확인한다.
그 뒤 같은 설정으로 짧은 경유지를 시험한다.

먼저 [로컬·Jetson 인계](local_jetson_handoff_20261008.md)를 읽는다.
어제 Tag B·앵커 0.15m·이륙 1.3m 기록과 차이가 있다.
이 문서의 Tag A 기본값은 실제 적용값이 아니다.
현재 [통합·검증 기록](../report/local_jetson_integration_20261008.md)을 함께 읽는다.
태그·배치·domain을 launch에서 함께 검사한다.
실물 확인과 현장 배포는 별도 단계다.

## 1. 코드와 환경 준비

현재 원격 코드는 접속 후 확인한다.
사용자가 지정한 계정은 `arialhanho`다.
주소는 기존 장비 기록의 `100.110.163.94`다.

```bash
ssh arialhanho@100.110.163.94
```

원격 `AGENTS.md`와 Git 변경을 먼저 확인한다.
이번 코드가 든 저장소 루트로 이동한다.
확인 없이 기존 파일을 덮거나 reset하지 않는다.
이유: Jetson의 미반영 실물 설정을 보존해야 한다.

저장소에 읽기 전용 수집기를 포함했다.
`src/drone_mission/tools/observe_ground.py`를 Jetson에서 사용한다.
기존 ROS 환경에서 아래처럼 현재 상태를 기록한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
python3 /path/to/review/src/drone_mission/tools/observe_ground.py \
  --domain 1 --seconds 10 > ground-observation.json
```

실제 기체가 domain 2면 두 값 모두 2로 바꾼다.
수집기는 기존 상태·XYZ·ToF·EV 전달·mirror를 읽는다.
시리얼을 열거나 FC 명령·설정 변경을 하지 않는다.
발행자 목록과 실제 수신 횟수를 함께 확인한다.
수신 로그만으로 PX4 융합·비행 준비를 판정하지 않는다.

전달 브랜치는 `codex/web-flight-handoff-20261008`이다.
기준 commit은 `deb940f612d7ee5198762aa73c7d7f22140beb61`다.
Jetson 적용은 미실시다.
로컬의 별도 검토 폴더에서 브랜치를 읽는다.
어제 통합 코드·실물 설정을 보존하며 통합한다.
Git 준비와 비교 명령은 인계 절차에 있다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
git status --short
python3 -c 'import numpy, serial, websockets; from websockets.asyncio.server import serve'
ros2 pkg prefix mavros
colcon build --symlink-install --packages-up-to drone_mission
source install/setup.bash
ros2 launch drone_mission test_flight.launch.py --show-args
```

ROS2 Humble, MAVROS, 지오이드가 필요하다.
웹 의존성은 `websockets==13.1`이다.
기존 전용 경로가 있으면 해당 PYTHONPATH를 사용한다.
예: `~/.local/share/dronestock-companion/deps`.
설치는 [companion 환경 절차](companion_setup.md)와
[플랫폼 서비스 절차](companion_platform_service.md)를 따른다.

같은 기체의 MAVROS·UWB 실행기를 중복 시작하지 않는다.
이유: 포트 소유와 관측·명령 발행자가 충돌한다.
시험 중 기존 플랫폼 자동 시작 서비스도 확인한다.
통합 launch가 플랫폼 연결을 함께 실행한다.

## 2. 장착값과 FC 설정 확인

기본 설정 파일을 사용자 작업 경로로 복사한다.
확인한 값만 이 파일에 넣는다.

```bash
mkdir -p "$HOME/.config/dronestock-flight"
cp -n src/drone_mission/config/flight.json "$HOME/.config/dronestock-flight/flight.json"
```

기존 파일이 있으면 보존하고 차이를 대조한다.
표의 수치는 이 브랜치의 Tag A 시험 기본값이다.
현재 기체·앵커 실측과 일치하는지 먼저 확인한다.

| 항목 | 확인·기록할 값 |
|---|---|
| A1~A4 | (0,0), (6.3,0), (0,4.6), (6.3,4.6)m, 높이 2.2m |
| 태그 계약 | tag/drone ID 5, RAW 거리 4개, 배치 ID 일치 |
| ToF | `/mavros/downward_0`, IMU·TIMESYNC 수신 |
| 렌즈→태그 | B_TF 현재 FLU [0,0,0.12]m. 실제 장착 대조 |
| FC→태그 | `EKF2_EV_POS_X/Y/Z`, PX4 FRD 기준 실측 |
| 좌표 정렬 | `enu_yaw_deg`, `enu_offset_x_m`, `enu_offset_y_m` |
| 수평 EV | FC `EKF2_EV_CTRL=1` |
| 관측 공분산 | FC `EKF2_EV_NOISE_MD=0` |
| 관측 지연 | FC `EKF2_EV_DELAY`와 `expected_ev_delay_ms` 일치 |
| 이륙 | FC `MIS_TAKEOFF_ALT`와 기대값 0.6m 일치 |
| 착륙 | FC의 자동 disarm 설정·ON_GROUND 확인 |
| FC 원점 | `/mavros/global_position/gp_origin` 수신 |
| 프로세스 상실 | 실제 PX4 datalink failsafe와 다른 GCS 연결 영향 |

EV_POS는 렌즈→태그 값이 아니다.
FRD의 +Z는 아래다. [축 설명](../architecture/web_test_flight.md)을 읽는다.
보존된 9월 params의 EV_CTRL은 0이었다.
MIS_TAKEOFF_ALT도 2.5m였다.
그 파일을 이번 시험에 그대로 적용하지 않는다.
이유: 수평 융합과 0.6m 시험 기대값이 다르기 때문이다.

측정·대조 후 해당 확인값을 `true`로 바꾼다.

| 확인값 | 바꾸는 시점 |
|---|---|
| `layout_confirmed` | 실제 앵커 위치·높이와 선택 배치 대조 뒤 |
| `alignment_confirmed` | +X·+Y 이동과 PX4 ENU 정렬 확인 뒤 |
| `timing_confirmed` | TIMESYNC·RAW 시각과 전달 지연 확인 뒤 |
| `sensor_mount_confirmed` | ToF·태그·FC 기준점 대조 뒤 |
| `fusion_confirmed` | 실제 PX4 EV 융합·위치·속도 확인 뒤 |
| `takeoff_settings_confirmed` | 실제 FC 고도·모드·착륙 설정 확인 뒤 |

브리지부터 확인할 때 fusion 확인값은 false로 둔다.
FC 파라미터를 바꿨다면 MAVROS mirror를 갱신한다.
브리지 조회는 MAVROS의 mirror다.
FC에서 실제 설정·적용 상태도 대조한다.

## 3. 로컬 웹 실행

Jetson에 웹 서버를 시작한다. 비행 노드와 별도 터미널이다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 run drone_mission local_flight_web
```

노트북 브라우저는 SSH 포워딩으로 연다.

```bash
ssh -N -L 8001:127.0.0.1:8001 -L 8002:127.0.0.1:8002 arialhanho@100.110.163.94
```

브라우저 주소는 `http://127.0.0.1:8001`이다.
서버 기본 바인딩은 localhost다.
노트북에서 서버를 실행할 경우에는 `ssh -R`로
Jetson의 같은 두 localhost 포트를 노트북에 연결한다.
HTTP와 WebSocket 두 포트를 함께 연결한다.

## 4. 지상 관측 전달

`tag:=A`가 기본값이다. B는 `tag:=B`를 명시한다.
기본 `start_mavros=false`는 기존 MAVROS를 사용한다.
포트와 기존 프로세스 확인 뒤에만 true로 시작한다.
이유: 같은 FC에 MAVROS를 중복 연결하지 않기 위해서다.

Tag B의 비활성 설정은 `flight_tag_b.json`이다.
ID 6·domain 2·임시 0.15m 배치·이륙 기대값 1.3m다.
확인값은 모두 false다. 현재 배치를 실측한 증거가 아니다.
웹에는 같은 파일을 `--config`로 전달한다.
예: `ros2 run drone_mission local_flight_web --config /path/to/flight_tag_b.json`.
launch도 `tag:=B config:=/path/to/flight_tag_b.json`을 사용한다.
2.2m 복원 때 mission·B_TF·anchor 파일을 함께 바꾼다.
하나라도 다르면 프로세스를 시작하기 전에 거부한다.

프로펠러를 제거한 지상 시험에서 관측부터 확인한다.
`execute=false`는 arm·takeoff 요청을 보내지 않는다.

```bash
export ROS_DOMAIN_ID=1
ros2 launch drone_mission test_flight.launch.py \
  config:="$HOME/.config/dronestock-flight/flight.json" \
  bridge_enabled:=true execute:=false \
  record_directory:="$HOME/flight-records/ground-$(date +%Y%m%d-%H%M%S)"
```

새 터미널에서 동일 ROS 환경을 소싱한다.

```bash
ros2 topic hz /uwb/btf_pose
ros2 topic echo /uwb/btf_xyz --once
ros2 topic echo /uwb/bridge_status --once
ros2 topic hz /mavros/vision_pose/pose_cov
ros2 topic echo /mavros/estimator_status --once
ros2 topic echo /mavros/local_position/odom --once
ros2 topic echo /mavros/global_position/gp_origin --once
ros2 topic echo /flight_state --once
```

`gate=ready`와 `published` 증가를 확인한다.
`last_observation_stamp_ns`도 최신이어야 한다.
이는 companion 발행 증거다.
PX4 수신·융합 성공은 FC 로그에서 추가로 확인한다.
UWB 방향 이동과 PX4 XY·속도를 대조한다.
EKF innovation·EV 융합 상태도 확인한다.

웹 XYZ는 태그 기준 측정값이다.
PX4 ENU는 FC 추정값이다.
고도 자리값 z=0을 실제 높이로 읽지 않는다.
입력 누락 때 웹은 null을 표시해야 한다.
텔레메트리가 1초 끊기면 현재 상태 표시를 비운다.

원점이 없으면 `px4_global_origin_required`로 막힌다.
실내 FC가 전역 목표를 지원하지 않아도 실물 시험이 막힌다.
원점을 임의 생성해 통과시키지 않는다.
먼저 실제 FC의 위치 모드와 원점 지원을 확인한다.

## 5. 짧은 전체 시퀀스

지상 정렬·융합을 확인한 설정으로 시작한다.
기존 tether·조종자·수동 전환 시험 조건을 따른다.
지상 시험 launch를 끝낸 뒤 새 시행을 실행한다.

```bash
ros2 launch drone_mission test_flight.launch.py \
  config:="$HOME/.config/dronestock-flight/flight.json" \
  bridge_enabled:=true execute:=true \
  record_directory:="$HOME/flight-records/flight-$(date +%Y%m%d-%H%M%S)"
```

웹에 실제 현재 XY에서 0.3m 떨어진 경유지를 넣는다.
화면의 기본 (2.3,2.0)은 현재 위치 (2.0,2.0)일 때의 예다.
첫 경유지와 각 구간은 1m 이내여야 한다.
허용 사각형은 x=0.5~5.3m, y=0.5~3.9m다.
현재 위치도 이 사각형 안에 있어야 한다.
선반·장애물 keepout 모델은 이번 경로에 없다.
경유지 입력 후 `이륙 · 경유지 이동 · 착륙`을 누른다.

| 확인 | 기대 동작 |
|---|---|
| 요청 검사 | 장치·배치·축·단위·revision 승인 |
| ARMING | PX4의 실제 armed 확인 |
| TAKING_OFF | PX4 고도로 이륙. AUTO.LOITER 진입 |
| MOVING | PX4가 같은 목표를 보고. 속도 0.3m/s 요청 |
| 도착 | 반경 0.15m·속도 0.1m/s 이하를 1초 유지 |
| LANDING | PX4 착륙 요청 |
| 완료 | ON_GROUND + disarmed 뒤 `landing_verified=true` |
| 전체 성공 | 모든 경유지 완료 + 착륙 뒤 `mission_complete=true` |

웹 ACK만으로 이륙·도착 성공을 판정하지 않는다.
입력 단절·목표 미적용은 착륙 요청으로 전환한다.
센서 복구만으로 다시 이륙하지 않는다.
수동 모드 전환 때 FSM은 추가 명령을 중지한다.
arm 중 착륙 버튼은 이륙을 취소하고 disarm을 요청한다.
수락·새 지상 상태가 없으면 취소 완료를 확정하지 않는다.
공중에서 `execute=true` 노드를 재시작하면 착륙을 요청한다.
수동 모드와 관측 전용 모드에서는 조종권을 가져오지 않는다.
출발점 복귀 버튼은 직선 구간 1m 한도 내에서만 이동한다.
불가능하면 현재 위치 착륙을 요청한다.

PX4가 착륙 명령을 거부하거나 응답이 없을 수 있다.
이 구현은 요청을 보냈다는 이유로 착륙을 확정하지 않는다.
조종자의 수동 전환과 FC failsafe를 실물로 확인한다.
실행 중 프로세스 종료 시험은 별도 시행으로 기록한다.
이유: 살아 있는 FSM의 웹 단절 처리와 다른 고장이다.

## 6. 기록과 후속 분석

시행 디렉터리와 PX4 ULog를 함께 보관한다.

| 경로 | 내용 |
|---|---|
| `uwb/` | 원본 UART·수신·판정 기록 |
| `btf/` | 높이·시간·후보·거부 사유 |
| `mission/events.jsonl` | 명령 요청·응답·상태 전이 |
| `mission/settings.json` | 이번 시행의 실제 설정 |
| `platform/` | GET·ACK·단계·텔레메트리 연결 상태 |
| PX4 ULog | FC 수신·융합·고도·실제 제어 확인 |

웹 재시험은 새 요청 ID를 생성한다.
ledger를 삭제해 과거 요청을 재사용하지 않는다.
이유: 재시작 시 의도하지 않은 재이륙을 막는 기록이다.
검증 방법과 현재 결과는 [확인 기록](../report/web_test_flight_20261008.md)을 따른다.
