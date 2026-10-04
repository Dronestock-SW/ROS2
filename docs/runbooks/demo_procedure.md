# 공용 데모 실행 절차
이 문서는 `drone_demo` 실행 순서다.
XY 시험과 실측 ToF 표시를 연결할 때 읽는다.

## 1. 빌드

작업 공간에서 한 번 빌드한다.

```bash
cd /home/user/drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 노드 기동 없이 파일 생성

먼저 12초짜리 데모 파일을 만든다.
명령은 실제 12초를 기다리지 않고 계산한다.
하드웨어와 서버에 접속하지 않는다.

```bash
ros2 run drone_demo demo_export \
  --scenario gap \
  --output /tmp/drone_demo_gap
```

기존 출력 폴더가 있으면 새 이름을 쓴다.
이유: 이전 시험 기록을 덮어쓰지 않는다.

| 생성 파일 | 다음 작업에서의 사용 |
|---|---|
| `poses.csv` | 위치·목표·공백을 표로 보기 |
| `samples.jsonl` | 데모 위치와 처리 결과 읽기 |
| `summary.json` | 설정·출처·처리 수·데모 오차 확인 |

```bash
head -8 /tmp/drone_demo_gap/poses.csv
cat /tmp/drone_demo_gap/summary.json
```

`demo_xy`는 XY 시험 관측을 생성한 상태다.
가상 고도·사선거리·`received.jsonl`은 생성하지 않는다.
`demo_gap`은 설정한 수신 공백이다.
관측이 없으면 CSV의 UWB x·y 칸이 비어 있다.
그 행의 데모 위치는 정답이며 관측이 아니다.

## 3. ROS2 노드에 연결

데모 전용 터미널에서 12초 동안 실행한다.
노드는 시간이 끝나면 자동 종료한다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 launch drone_demo demo.launch.py scenario:=gap
```

관찰할 터미널도 같은 환경을 사용한다.
첫 터미널을 실행한 직후 아래 명령을 쓴다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 topic echo /demo_status
```

목표점을 늦게 구독하면 보관 QoS를 지정한다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 topic echo /target_pose --qos-durability transient_local --once
```

UWB 출력은 sensor data QoS로 구독한다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 topic echo /uwb_pose --qos-reliability best_effort
```

시험할 노드도 같은 환경에서 기동한다.
현재 위치 입력을 `/demo_pose`에 연결한다.
단, 그 노드가 `PoseStamped`를 받을 때의 예다.
메시지 형식이 다르면 연결 변환을 추가한다.
데모 위치를 EKF2 관측 입력에 연결하지 않는다.
이유: 생성기의 정답을 센서 측정으로 오인하게 한다.

## 4. 실측 ToF 거리 표시

실제 `sensor_msgs/Range` 토픽을 `real_tof_topic`으로 지정한다.
센서 메시지에는 측정 시각·frame·거리 범위가 필요하다.
측정 시계는 ROS 노드와 대응해야 한다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 launch drone_demo demo.launch.py \
  real_tof_topic:=/tof/range tof_timeout_s:=0.2
```

`/demo_status`의 `tof.range_m`에서 하방거리를 확인한다.
미수신·오류·0.2초 초과 표본은 null이다.
`tof.reason`에서 사유를 확인한다.
실측 입력이 들어와도 XY 데모는 계속 실행된다.
DOMAIN_ID 99 밖의 센서는 별도 연결이 필요하다.
이 저장소 변경만으로 장치 토픽이 연결되지는 않는다.

ToF 거리를 `/demo_pose.z`에 대입하지 않는다.
이유: 광축 거리와 지도 높이는 서로 다른 값이다.
실제 비행 고도는 PX4 추정 출력을 사용한다.

## 5. 값 바꾸기

설정 파일 하나에서 임시값을 관리한다.
[기본 설정](../../src/drone_demo/config/demo.json)을 수정한다.
노드를 다시 실행하면 새 설정을 읽는다.

파일 생성만 바꿀 때는 인자를 쓸 수 있다.

```bash
ros2 run drone_demo demo_export \
  --scenario stationary --duration-s 60 \
  --output /tmp/drone_demo_stationary_60s
```

개별 시험용 설정은 파일을 복사해 보관한다.

```bash
cp src/drone_demo/config/demo.json /tmp/my_drone_demo.json
```

복사본의 XY·잡음·기간을 고친 뒤 실행한다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 launch drone_demo demo.launch.py \
  config_file:=/tmp/my_drone_demo.json
```

가상 고도 설정과 `--z-*` 인자는 더 이상 제공하지 않는다.
구 설정은 현재 `demo.json`을 기준으로 다시 작성한다.
실제 UWB RAW 재생은 [자료 실행 절차](uwb_data_runbook.md)를 따른다.
XY 데모 파일은 RAW 재생 입력이 아니다.

## 6. 실측 입력으로 전환

필요한 기능만 [공용 데모 기준](../architecture/demo_design.md)대로 교체한다.

| 이번에 필요한 기능 | 사용할 것 |
|---|---|
| 거리·도착 조건 | `/demo_pose`, `/target_pose` |
| 수신 공백 처리 | `gap`, `/demo_status`, `/uwb_pose` |
| 실제 UWB | 실기 DOMAIN_ID의 기존 `uwb_node` |
| 실제 하방거리 표시 | `real_tof_topic`의 실측 Range |
| 실제 현재 위치·고도 | 검증한 PX4 추정 출력 |
| 실제 LiDAR | T-mini Pro의 `/scan` |
| 비행·EKF2·보상 학습 | 별도 PX4 SITL 단계 |

데모 출력만 실기 DOMAIN_ID로 옮기지 않는다.
이유: 실기 입력으로 전환할 때 출처도 바꿔야 한다.

## 7. 임무 판단 한 사이클 실행

생성기와 상태 판단 노드를 함께 실행한다.
12초 뒤 두 노드가 함께 종료한다.
첫 실행 결과는 [개발 1사이클 기록](../report/demo_cycle_20260913.md)에 있다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 launch drone_demo mission_demo.launch.py \
  scenario:=gap record_directory:=/tmp/drone_mission_gap
```

다른 터미널에서 판단 결과를 읽는다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 topic echo /demo_mission_state
```

`DEGRADED` 뒤 `RECOVERING`을 확인한다.
이후 접근·정착·도착 상태를 확인한다.
상태의 뜻은 [임무 판단 기준](../architecture/demo_mission_design.md)을 본다.

종료 후 JSONL 기록을 확인한다.

```bash
cat /tmp/drone_mission_gap/mission_state.jsonl
```

상태 판단만 따로 계속 실행할 수도 있다.
이 경우 입력을 끊고 시간 초과를 볼 수 있다.
종료할 때는 Ctrl+C를 누른다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 run drone_demo demo_mission_node
```

세 시나리오는 파일로 한꺼번에 재현한다.
이 명령은 ROS2 노드를 기동하지 않는다.

```bash
ros2 run drone_demo demo_cycle --output /tmp/drone_mission_cycle
```

각 폴더에 `summary.json`과 전체 기록이 생긴다.
기록의 `demo_time_s`는 가상 시간이다.
ROS2 기록의 `monitor_elapsed_s`는 실제 경과 시간이다.

자동 검증은 다음 명령으로 실행한다.

```bash
python3 -m pytest -q src/drone_demo/test
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  python3 src/drone_demo/test/check_ros_topics.py --mission
```

ROS 연결 시험은 `/demo/test_tof_range`에 시험 표본을 발행한다.
실물 ToF 검증과 구분한다.
