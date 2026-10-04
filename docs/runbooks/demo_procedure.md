# 공용 데모 실행 절차
이 문서는 `drone_demo` 실행 순서다.
장비 없이 새 기능에 시험 입력을 넣을 때 읽는다.

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
| `received.jsonl` | 기존 `uwb_replay` 입력으로 쓰기 |
| `summary.json` | 설정·출처·처리 수·데모 오차 확인 |

```bash
head -8 /tmp/drone_demo_gap/poses.csv
cat /tmp/drone_demo_gap/summary.json
```

`warming_up`은 초기 시각 정합 준비 상태다.
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

## 4. 실제 ToF 연결 시 데모 차단

실제 `sensor_msgs/Range` 측정을 데모의 `/tof/range`에 연결한다.
유효한 측정값을 받으면 데모 발행이 종료된다.
토픽 이름이 다르면 `real_tof_topic:=<토픽>`으로 지정한다.
연결 전 임시 차단은 `synthetic_z_enabled:=false`를 쓴다.
DOMAIN_ID 99 밖의 측정은 데모에서 자동 감지할 수 없다.

## 5. 값 바꾸기

설정 파일 하나에서 임시값을 관리한다.
[기본 설정](../../src/drone_demo/config/demo.json)을 수정한다.
노드를 다시 실행하면 새 설정을 읽는다.

파일 생성만 바꿀 때는 인자를 쓸 수 있다.

```bash
ros2 run drone_demo demo_export \
  --scenario stationary --z-min-m 0.2 --z-max-m 2.2 --z-period-s 8 --duration-s 60 \
  --output /tmp/drone_demo_stationary_60s
```

개별 시험용 설정은 파일을 복사해 보관한다.

```bash
cp src/drone_demo/config/demo.json /tmp/my_drone_demo.json
```

복사본의 z·목표·기간을 고친 뒤 실행한다.

```bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  ros2 launch drone_demo demo.launch.py \
  config_file:=/tmp/my_drone_demo.json
```

## 5. 기존 UWB 재생기에 사용

데모 RAW도 실제 수집 파일과 같은 입력 형식을 쓴다.

```bash
ros2 run drone_uwb uwb_replay \
  /tmp/drone_demo_gap/received.jsonl \
  --layout src/drone_uwb/config/anchors_20260906.json \
  --settings src/drone_uwb/config/uwb.yaml \
  --output /tmp/drone_demo_gap_replay
```

움직이는 경로에 단일 `--reference`를 넣지 않는다.
이유: 정지 기준점 하나로 이동 정확도를 잴 수 없다.
데모 오차는 `summary.json`의 정답 비교를 본다.
이는 실물의 정확도 검증 수치가 아니다.

## 6. 실측 입력으로 전환

필요한 기능만 [공용 데모 기준](../architecture/demo_design.md)대로 교체한다.

| 이번에 필요한 기능 | 사용할 것 |
|---|---|
| 거리·도착 조건 | `/demo_pose`, `/target_pose` |
| 수신 공백 처리 | `gap`, `/demo_status`, `/uwb_pose` |
| 실제 UWB | 실기 DOMAIN_ID의 기존 `uwb_node` |
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
