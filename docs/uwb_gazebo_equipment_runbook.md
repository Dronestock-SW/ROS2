# UWB 앵커와 탑재 센서를 Gazebo에 넣기

사용자 기체 배치를 반영한 가상 모델 실행 절차다. 기존 x500을 센서 시험 모델로 바꿀 때 읽는다.

파일을 만든 원리와 연결 구조는 [구현 설명](uwb_gazebo_architecture.md)에 있다.

## 현재 범위

앵커 배치·센서 생성·가상 RAW 수신부터 확인한다.
모델 파일 준비와 WSL 실행 성공은 구분한다.
센서 비교 뒤 UWB 관측을 PX4에 연결한다.

```text
앵커 4기 + 기체 + 센서 + 시험 벽
  ├─ 기체 위치·자세 → 안테나 위치 → 가상 UWB RAW
  ├─ 하방 거리계 → Gazebo 거리 토픽 → PX4 거리 센서
  ├─ IMU·광학흐름 → PX4 가상 센서
  └─ 수평 LiDAR·카메라 → Gazebo 거리·영상 토픽

같은 비행의 위치 기록 → A/B/C/D/WLS 파일 비교
  → ToF 높이 입력 연결 → 선정한 관측의 PX4 융합 시험
```

현재 파일 비교는 시뮬레이터의 안테나 높이를 사용한다.
생성된 ToF 토픽을 파일 비교에 넣는 연결은 남았다.
실시간 UWB RAW 발행은 Gazebo 내부 통신이다.
비행제어 또는 MAVLink 송신 기능은 없다.

## 0. Python 연결 확인

WSL Ubuntu 창에서 시스템 Python으로 확인한다.

```bash
/usr/bin/python3 -c "from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V; import numpy; print('Gazebo Python OK')"
```

`No module named 'numpy'`만 나오면 앞의 두 Gazebo 모듈은 불러온 상태다.
Ubuntu의 [python3-numpy 패키지](https://packages.ubuntu.com/jammy/python3-numpy)를 설치하고 위 명령을 다시 실행한다.

```bash
sudo apt update
sudo apt install -y python3-numpy
```

성공하면 `Gazebo Python OK`가 출력된다.
이 확인은 모듈 로딩 검사이며 센서 데이터 수신 검사는 아니다.
2026-09-27 첫 사용자 출력에서는 NumPy 누락을 확인했다.
후속 Ubuntu 출력에서 `python3-numpy` 버전 `1:1.21.5-1ubuntu22.04.1`의
`Setting up` 완료와 셸 복귀를 확인했다. 패키지 설치는 완료됐다.
후속 재검사에서 `Gazebo Python OK` 출력을 받았다.
사용자 WSL의 세 모듈 로딩 확인은 완료됐다.
센서 메시지 수신·새 모델 실행 검증은 별도다.

`gz topic -l`에서 `/world/default/`와 `x500_lidar_down_0`이 보이면
목록에 나타난 것은 기존 월드·기체다.
아래 새 모델을 적용한 결과와 구분한다.
토픽 이름만으로 발행 중인 센서 데이터까지 확인했다고 판단하지 않는다.

## 1. 소스 준비

작업공간의 ZIP을 사용자 WSL로 복사한다.
2026-09-27 작업공간 주소는 `100.110.163.94`다.
VS Code가 연결한 컴퓨터와 사용자 WSL은 별개다.
WSL에서 아래 명령을 한 줄씩 실행한다.
입력줄의 사용자가 `dronestock@DESKTOP-0C8GRSK`인지 먼저 확인한다.
`pgyxn@user-desktop`에서 실행하면 원격 작업공간 안에 복사된다.
그 결과로는 사용자 WSL에 파일이 전달되지 않는다.

```bash
mkdir -p ~/uwb_sim
scp pgyxn@100.110.163.94:/home/pgyxn/uwb_gazebo_equipment_20260927.zip ~/uwb_sim/
```

`mkdir -p`는 자료를 받을 폴더를 만든다.
`scp`는 SSH 연결로 파일을 복사한다.
`~`는 이 셸의 사용자 홈인 `/home/dronestock`이다.
계정 비밀번호를 물으면 원격 컴퓨터의 `pgyxn` 계정 인증이다.
후속 사용자 출력에서 WSL의 접속·인증·ZIP 전송 완료를 확인했다.

첫 연결의 ED25519 지문은 아래 서버 공개키 지문과 대조한다.
일치하면 연결 확인 질문에 `yes`를 입력한다.
다른 지문이나 인증 오류가 나오면 출력부터 확인한다.

```text
SHA256:xnGX6BSg2IecyH9jPOyZSv2QOnaU2T0YOe6tIrBLoQ0
```

현재 ZIP은 250,177바이트이며 파일 99개를 포함한다.
최상위 폴더는 `uwb-gazebo-equipment`다.
작업공간에서 ZIP 무결성 검사를 통과했다.
사용자 WSL로의 복사는 `scp` 100% 출력으로 확인했다.
후속 출력에서 풀린 소스 폴더와 설정 파일 세 개를 확인했다.
사용자 WSL에서 전체 파일 해시 비교는 미실시다.
ZIP의 SHA256은 다음과 같다.

```text
faf4fdc44a659f6733d9d3c314f76638c43097860d8f7b23adbfb312ed0de8ba
```

이전 `uwb_gazebo_shadow` ZIP에는 새 도구가 없다.
이번에 받은 장비 묶음을 풀고 설정 파일을 확인한다.
WSL Ubuntu 창에서 아래를 한 줄씩 실행한다.

```bash
cd ~/uwb_sim
/usr/bin/python3 -m zipfile -e uwb_gazebo_equipment_20260927.zip .
cd uwb-gazebo-equipment
ls src/drone_uwb/config
```

`cd`는 현재 작업 폴더를 바꾼다.
`-m zipfile`은 Python의 ZIP 처리 도구를 실행한다.
`-e`는 압축 해제이며 마지막 `.`은 현재 폴더다.
[Python 명령 설명](https://docs.python.org/3.10/library/zipfile.html#command-line-interface)을 따른다.
`ls`는 지정한 폴더의 파일 이름을 보여준다.
예상 목록은 다음 세 파일이다.

```text
anchors_20260906.json
gazebo_equipment.json
gazebo_shadow.json
```

초기 ZIP에는 시험 벽의 이름 중복 오류가 있다.
첫 생성 전에 아래 명령으로 생성 코드를 갱신한다.
이미 `equipment_01`을 설치했다면 2.1절을 따른다.

```bash
scp pgyxn@100.110.163.94:/home/pgyxn/github/ROS2/src/drone_uwb/drone_uwb/integration/gazebo_rig.py src/drone_uwb/drone_uwb/integration/gazebo_rig.py
```

파일 이름 확인 후 같은 소스 폴더에서 도구 로딩을 확인한다.

```bash
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m drone_uwb.integration.gazebo_rig --help
```

설정은 `src/drone_uwb/config/gazebo_equipment.json`이다.
실측 수평 위치와 높이 시험값을 구분해 적었다.
생성기는 설치된 PX4의 원본 센서를 읽는다.
기존 x500·기체 파라미터 파일은 수정하지 않는다.

## 2. 새 모델 생성·설치

새 모델 이름 두 개만 추가한다.
이미 같은 이름이 있으면 덮어쓰지 않고 중단한다.
입력줄은 `dronestock`이며 소스 폴더 안에 있어야 한다.
앞 절의 `PYTHONPATH` 설정은 같은 셸에서 유지된다.
이 값은 Python이 `src/drone_uwb`의 코드를 찾게 한다.

```bash
/usr/bin/python3 -m drone_uwb.integration.gazebo_rig \
  --px4-gz "$HOME/github/PX4-Autopilot/Tools/simulation/gz" \
  --output runs/equipment_01 --install
```

| 명령·옵션 | 역할 |
|---|---|
| `-m drone_uwb.integration.gazebo_rig` | 이번에 전달한 모델 생성 도구 실행 |
| `--px4-gz` | 원본 기체·센서 파일을 읽을 PX4 Gazebo 폴더 |
| `--output` | 생성한 모델·설정·기록을 저장할 새 폴더 |
| `--install` | 생성 후 PX4의 모델·월드 폴더에 새 이름으로 복사 |

`--install`은 이 도구의 파일 복사 옵션이다.
Gazebo를 시작하는 명령은 다음 3절에 있다.
성공하면 생성 기록 JSON을 출력한다.
`gazebo_runtime_verified: false`와 `flight_verified: false`는
아직 Gazebo 실행·비행 검증 전이라는 상태값이다.
모델 생성 실패를 뜻하는 값은 아니다.
2026-09-27 사용자 WSL에서 `--install` 실행 후
이 JSON 출력과 셸 복귀를 확인했다.
생성·설치는 완료됐으며 다음은 파일 구조 검사다.

| 생성물 | 역할 |
|---|---|
| `models/dronestock_x500/model.sdf` | IMU·ToF·광학흐름·LiDAR·카메라·태그 표시 |
| `worlds/dronestock_uwb.sdf` | 저장된 앵커 4기·시험 벽·무늬 바닥 |
| `trial.json` | 화면과 같은 앵커·안테나 장착 위치 |
| `equipment.json`, `anchors.json` | 적용한 설정 복사 |
| `manifest.json` | 원본 모델 해시·적용 질량·미검증 상태 |

설치 후 Gazebo의 SDF 검사도 수행한다.
SDF는 기체·센서·시험장 구성을 적는 파일 형식이다.
초기 생성본 검사에서 드론은 `Valid.`로 통과했다.
시험장은 이름 중복 오류로 실패했다. 수정 절차는 2.1절에 있다.

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/github/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
gz sdf -k runs/equipment_01/models/dronestock_x500/model.sdf
gz sdf -k runs/equipment_01/worlds/dronestock_uwb.sdf
```

첫 줄은 Gazebo가 기본 모델·외형 파일을 찾을 경로를 추가한다.
다음 두 줄의 `-k`는 각 SDF의 문법·구조를 검사한다.
정상 종료 시 `Valid.` 출력과 경고 유무를 확인한다.
플러그인 로드·센서 수신·비행 성공은 이 검사로 확인하지 않는다.
파일 오류나 경고가 나오면 출력부터 보관한다.
새 설치 전체를 다시 수행하기 전에 실패 항목을 구분한다.

### 2.1. 초기 시험장의 이름 중복 수정

수정한 생성기로 새 파일을 만들고 검사한 뒤 시험장 파일을 교체한다.
초기 벽의 외형과 충돌 영역 이름이 모두 `wall`이었다.
같은 링크의 이름 중복으로 SDF 검사가 실패했다.
수정본은 외형 `wall`, 충돌 영역 `wall_collision`이다.

WSL의 `~/uwb_sim/uwb-gazebo-equipment`에서 실행한다.

```bash
scp pgyxn@100.110.163.94:/home/pgyxn/github/ROS2/src/drone_uwb/drone_uwb/integration/gazebo_rig.py src/drone_uwb/drone_uwb/integration/gazebo_rig.py
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m drone_uwb.integration.gazebo_rig \
  --px4-gz "$HOME/github/PX4-Autopilot/Tools/simulation/gz" \
  --output runs/equipment_02
gz sdf -k runs/equipment_02/worlds/dronestock_uwb.sdf
```

기존 `equipment_01`은 검토 기록으로 보존한다.
같은 모델·월드 이름이 이미 설치돼 있어 `--install`은 생략한다.
`Valid.` 확인 뒤 아래 명령으로 설치된 시험장만 갱신한다.
사용자 WSL에서 `equipment_02` 재생성과 `Valid.` 출력을 확인했다.
후속 사용자 출력에서 백업·교체 명령과 `cmp` 무출력 종료를 확인했다.
설치 경로에 수정 시험장이 반영됐다.

```bash
cp -n "$HOME/github/PX4-Autopilot/Tools/simulation/gz/worlds/dronestock_uwb.sdf" "$HOME/github/PX4-Autopilot/Tools/simulation/gz/worlds/dronestock_uwb.sdf.before_wall_fix"
cp runs/equipment_02/worlds/dronestock_uwb.sdf "$HOME/github/PX4-Autopilot/Tools/simulation/gz/worlds/dronestock_uwb.sdf"
cmp runs/equipment_02/worlds/dronestock_uwb.sdf "$HOME/github/PX4-Autopilot/Tools/simulation/gz/worlds/dronestock_uwb.sdf"
```

`cp -n`은 이전 시험장 파일을 보관한다.
같은 백업 파일이 이미 있으면 덮어쓰지 않는다.
다음 `cp`가 수정본을 설치 경로에 반영한다.
`cmp`는 생성본과 설치본의 내용을 비교한다.
차이가 없으면 출력 없이 종료한다.

드론의 `gz_frame_id` 경고는 이 이름 중복과 다르다.
PX4 원본 센서 SDF에도 있는 항목이다.
[Gazebo Sensors 소스](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/Sensor.cc)는
이 값을 센서의 frame ID로 읽는다. 해당 항목은 유지한다.
드론 검사에서 `Valid.`가 나왔어도 센서 동작은 실행 후 확인한다.

## 3. 새 월드·기체 실행

먼저 WSL Ubuntu에서 기존 실행 상태를 확인한다.

```bash
pgrep -a -x px4
gz topic -l
```

첫 명령은 이름이 정확히 `px4`인 프로세스를 조회한다.
결과가 없으면 해당 이름의 프로세스는 찾지 못한 것이다.
두 번째는 Gazebo 통신 목록을 조회한다.
`/world/default/clock` 등이 보이면 기존 월드가 발견된 상태다.
토픽 목록만으로 센서가 정상 동작한다고 판단하지 않는다.

기존 Gazebo·PX4 창은 정상 종료한다.
실행 중인 월드가 있으면 PX4가 그 월드에 붙을 수 있다.
그 상태에서는 새 앵커 배치를 확인할 수 없다.

사용자 조회에서 `pgrep`와 `gz topic -l`은 모두 무출력이었다.
해당 조회에서 기존 PX4 프로세스·Gazebo 토픽을 찾지 못했다.
이미 빌드한 SITL 실행 파일로 새 월드·기체를 시작한다.
WSL Ubuntu 창에서 다음 두 단계를 실행한다.

```bash
cd ~/github/PX4-Autopilot
PX4_SYS_AUTOSTART=4016 \
PX4_SIM_MODEL=gz_dronestock_x500 \
PX4_GZ_WORLD=dronestock_uwb \
PX4_GZ_MODEL_POSE="2.09,1.68,0.24,0,0,0" \
./build/px4_sitl_default/bin/px4
```

| 실행값 | 의미 |
|---|---|
| `PX4_SYS_AUTOSTART=4016` | 하방 거리계를 사용하는 x500용 PX4 시작 설정 |
| `PX4_SIM_MODEL=gz_dronestock_x500` | 이번에 생성한 드론 모델 선택 |
| `PX4_GZ_WORLD=dronestock_uwb` | 수정한 앵커 시험장 선택 |
| `PX4_GZ_MODEL_POSE` | 시작 위치 x/y/z(m)와 roll/pitch/yaw(rad) |
| `./build/px4_sitl_default/bin/px4` | 이미 빌드한 가상 비행제어기 실행 |

이 값들은 마지막 실행 명령에 전달하는 환경변수다.
위치는 `(2.09, 1.68, 0.24)m`, 세 시작 각도는 0이다.
[PX4 공식 실행 예시](https://docs.px4.io/main/en/sim_gazebo_gz/#examples)를 참고한다.
정상 시작 시 Gazebo 화면과 PX4 콘솔 `pxh>`를 확인한다.
실행 후 이 창은 유지한다. 후속 Ubuntu 명령은 별도 WSL 창에서 실행한다.
후속 사용자 로그에서 새 시험장 준비와 `gz_bridge` 초기화를 확인했다.
PX4 시작 스크립트도 정상 종료했다.
센서별 메시지 값과 Gazebo 화면은 아직 별도 확인이 필요하다.

`make ... gz_x500_lidar_down`은 기본 모델을 지정한다.
위 명령은 이미 빌드한 PX4로 새 모델을 선택한다.
이 실행 경로는 사용자 PX4 커밋의 시작 스크립트에 맞췄다.
모델에 사용하는 광학흐름 플러그인은 PX4 빌드 산출물이다.
`libOpticalFlowSystem.so` 로드 오류도 확인한다.

이번 단계는 배치와 센서 수신 확인이다.
이륙 검증 전에 IMU 장착 위치와 EKF 기준점을 맞춘다.
PX4 장착 파라미터의 기준은 질량중심이다.
사용자가 알려준 기체 중심과 같은지 아직 확인하지 않았다.
센서 높이·질량 분포도 실측과 구분한다.

## 4. 배치·센서 수신 확인

새 Ubuntu 창에서 실행한다.

```bash
gz topic -l
```

월드 이름은 `dronestock_uwb`다.
기체 이름은 첫 인스턴스에서 `dronestock_x500_0`다.

| 위치·센서 | 토픽 끝부분 또는 PX4 조회 |
|---|---|
| 기체 위치 | `/world/dronestock_uwb/dynamic_pose/info` |
| IMU | `/link/base_link/sensor/imu_sensor/imu` |
| 하방 거리 | `/link/lidar_sensor_link/sensor/lidar/scan` |
| 광학흐름 | `/link/flow_link/sensor/optical_flow/optical_flow` |
| 수평 LiDAR | `/link/link/sensor/lidar_2d_v2/scan` |
| 카메라 | `/link/camera_link/sensor/camera/image` |

센서 토픽에는 앞에 월드·기체 경로가 붙는다.
실제 목록에서 확인한 이름을 사용한다.
PX4 `pxh>`에서는 다음을 조회한다.

```text
listener sensor_combined
listener sensor_baro
listener distance_sensor
listener sensor_optical_flow
param show EKF2_IMU_POS_X
param show EKF2_RNG_POS_X
```

토픽 존재와 연속 데이터 수신은 별개로 확인한다.
초기 로그에 기압·전원 경고가 있었으므로 지속 여부를 확인한다.
반복된 `No connection to the GCS`는 QGroundControl 연결 단계에서 확인한다.
시작 스크립트 성공만으로 비행 준비 완료를 판정하지 않는다.
광학흐름 시험을 위해 바닥에 반복 무늬를 넣었다.
카메라·LiDAR는 제조사 장비의 완전한 복제 모델이 아니다.

## 5. 가상 UWB 수신 시작

새 도구 폴더의 별도 WSL 창에서 실행한다.
Gazebo Python 모듈·NumPy는 [연결 준비](uwb_gazebo_shadow_runbook.md) 1절을 따른다.

```bash
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m drone_uwb.integration.gazebo_ranges \
  --config runs/equipment_01/trial.json \
  --output runs/equipment_capture_01
```

이 프로세스는 앵커별 가상 RAW 사거리를 발행한다.
관측 토픽은 `/dronestock/sim/uwb/ranges`다.
`gz.msgs.StringMsg` 안에 JSON을 넣는다.
다른 창에서 다음으로 내용을 확인한다.

```bash
gz topic -e -t /dronestock/sim/uwb/ranges
```

종료는 가상 UWB 기록 창에서 `Ctrl+C`다.

| 파일 | 내용 |
|---|---|
| `raw_ranges.jsonl` | `sim_uwb_cycle`, A1~A4 RAW, 시뮬레이션 시각 us |
| `truth.jsonl` | 별도 평가용 기체·안테나 위치, 기하 거리 |
| `poses.jsonl` | 원래 기체 위치·자세 |
| `capture.json` | 실제 기록률·누락·종료 사유 |

RAW에는 위치 정답을 넣지 않는다.
시뮬레이션 시계를 ESP32 시계로 표기하지 않는다.
네 거리는 동시에 생성하며 RF 전파를 해석하지 않는다.
현재 실시간 생성에는 정규 잡음과 설정 편향만 적용한다.
가림·단절 시나리오는 다음 파일 비교에서 적용한다.

## 6. 같은 경로에서 계산 후보 비교

기록과 동일한 배치·장착 설정을 사용한다.

```bash
/usr/bin/python3 -m drone_uwb.processing.experiments.gazebo_scenarios \
  --input runs/equipment_capture_01/poses.jsonl \
  --config runs/equipment_01/trial.json \
  --output runs/equipment_compare_01
```

여섯 조건의 잡음·가중치 값은 [비교 절차](uwb_gazebo_shadow_runbook.md)를 따른다.
실시간에 기록된 RAW를 그대로 소비하는 명령은 아니다.
같은 위치 기록에서 지정한 시나리오의 거리를 재생성한다.
단일 실행기는 같은 seed·잡음이면 실시간 거리와 일치한다.
동일 입력 일치는 단위 검사로 확인했다.

남은 작업과 실제 로그 검토는 [구성 결과](report/uwb_gazebo_equipment_20260927.md)에 있다.
