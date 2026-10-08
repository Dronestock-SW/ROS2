# UWB·PX4·AI 지상 관측 절차
통합 브랜치를 빌드하고 지상 수신을 확인하는 절차다.
Tag A/B를 각각 독립 companion에서 시험할 때 읽는다.

이 절차는 관측만 실행한다.
ARM·이륙·FC 파라미터 변경을 실행하지 않는다.
이유: 실측·융합·실제 SITL 검증이 남아 있다.

## 작업 보존

기존 실행 경로와 통합 checkout을 분리한다.
기존 서비스나 직렬 포트 소유자를 먼저 확인한다.

```bash
git status --short --branch
git branch -avv
systemctl --user list-units 'sangwon-*' --no-pager
ps -eo pid,ppid,args | grep -E 'px4|gazebo|mavros|sangwon'
ls -l /dev/uwb /dev/pixhawk /dev/lidar
fuser /dev/uwb /dev/pixhawk /dev/lidar
```

포트 소유자가 있으면 중복 실행하지 않는다.
기존 MAVROS와 같은 도메인을 쓸 때는 재사용한다.
Tag A는 ID 5/domain 1이다.
Tag B는 ID 6/domain 2다.

## 빌드와 소프트웨어 시험

통합 checkout의 루트에서 실행한다.
AI의 외부 BT 의존성은 고정 커밋을 사용한다.

```bash
source /opt/ros/humble/setup.bash
git submodule update --init src/ydlidar_ros2_driver
colcon build --symlink-install --executor sequential
source install/setup.bash
ros2 pkg prefix drone_bringup
```

AI 웹 시험에는 `websockets`가 필요하다.
실행할 Python에 의존성을 설치한다.
별도 가상환경을 쓰면 CMake에 지정한다.

```bash
colcon build --symlink-install --packages-select sangwon_ai_replay \
  --cmake-args -DSANGWON_WEB_PYTHON=/absolute/path/to/venv/bin/python
ctest --test-dir build/sangwon_ai_replay --output-on-failure -j1
python3 -m pytest src/drone_uwb/test -q
python3 -m pytest src/drone_demo/test -q
python3 -m pytest src/drone_platform_link/test -q
(cd src/drone_bringup && python3 -m pytest test -q)
```

MAVLink wire 시험에는 `pymavlink`가 필요하다.
서로 다른 패키지의 pytest는 따로 실행한다.
이유: 같은 이름의 기존 시험 모듈이 있다.

## 통합 지상 관측

아래 명령은 615초 뒤 자체 종료한다.
빌드가 끝난 뒤 실행한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch drone_bringup companion_observe.launch.py \
  tag:=B ai_root:="$PWD/src/sangwon_AI" \
  start_mavros:=true \
  record_directory:="$PWD/.integration-evidence/tag-b-ground-new" \
  stop_after_s:=615.0
```

`start_mavros` 기본값은 `false`다.
위 예시는 포트가 비어 있을 때만 쓴다.
Tag A는 `tag:=A`, `ROS_DOMAIN_ID=1`로 바꾼다.
통합 실행기는 bridge 출력을 강제로 비활성화한다.
AI는 C++ PX4 관측기만 실행한다.
기존 mission core와 부팅 서비스를 바꾸지 않는다.

별도 SSH 셸에서 수신 증거를 기록한다.
출력 디렉터리는 매번 새 경로를 쓴다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
python3 -m drone_uwb.integration.ros.bench_probe \
  --output .integration-evidence/tag-b-probe-new --seconds 600
```

확인할 출력은 다음과 같다.

| 출력 | 확인할 내용 |
|---|---|
| `/uwb/status` | tag_id, TDMA, 원본 수신·거절 사유 |
| `/uwb_pose` | 기존 배치 기준 진단 XY. 정확도 보장 없음 |
| `/uwb/btf_status` | ToF·IMU·장착값의 유효성 |
| `/uwb/btf_pose` | 조건을 만족한 보정 안테나 XY |
| `/uwb/bridge_status` | `disabled`, `published=0` |
| `/mavros/local_position/pose` | PX4가 추정한 같은 메시지의 XYZ |
| `src/sangwon_AI/.runtime/px4/health.json` | domain 2, local_position LIVE, 비행 권한 false |

B_TF는 원래 네 거리 계산이 일치하면 그대로 통과시킨다.
불일치 후보의 ToF 보정은 실측 장착값이 필요하다.
B_TF 발행만으로 장착·융합 완료를 판단하지 않는다.

MAVROS의 `command` plugin은 버전 조회에 필요하다.
이 실행기에 ARM·모드·setpoint 클라이언트는 없다.
`vision_pose`·setpoint plugin도 로드하지 않는다.
관측기의 배터리 WARN도 지우지 않는다.
이유: 알 수 없는 배터리 값을 정상으로 만들 수 없다.

## 10분 전송 판정

평균 Hz만으로 통과시키지 않는다.
10초 준비 뒤 600개의 1초 창을 검사한다.

```bash
python3 -m drone_uwb.integration.ground_audit \
  --received .integration-evidence/tag-b-ground-new/uwb/SESSION/received.jsonl \
  --tag-id 6 --output .integration-evidence/tag-b-audit-new.json
```

각 창에서 유효한 4거리+TDMA가 35회 이상이어야 한다.
최대 연속 누락은 1개 이하여야 한다.
세션 변경과 불완전한 기록은 통과하지 못한다.
이 시험은 B_TF 좌표 정확도·FC 융합 시험이 아니다.
UART JSON 파싱 오류는 `node_status.jsonl`도 확인한다.

## 대략 배치로 볼 수 있는 것

A1을 원점, A2를 +X, A3를 +Y로 둔다.
먼저 장치 ID와 거리 변화를 확인한다.
실제 앵커가 설정과 다르면 XY가 거절될 수 있다.
Tag B는 현재 6.3×4.6m, 임시 높이 0.15m다.
Tag A와 복원 프로파일은 기존 2.2m를 보존한다.
[임시 높이 절차](anchor_low015_bench.md)를 따른다.
실제 높이가 무작위면 z=0 설정도 맞지 않는다.
이유: 기울어진 거리를 XY로 투영할 때 높이가 필요하다.
좌표를 보려고 유효성 검사를 완화하지 않는다.

임시 배치는 실제 측량과 구분해 기록한다.
앵커 위치·높이와 태그 위치를 실측한 뒤 설정한다.
`layout_confirmed`를 포함한 확인값은 그대로 false다.

## 다음 검증 순서

| 순서 | 완료 증거 |
|---|---|
| 1. 센서 실측 | 앵커 XYZ·ID, ToF→안테나 FLU, FC→안테나 FRD |
| 2. 정렬 | 창고→ENU 회전/이동, 원본 시각·지연 측정 |
| 3. 실제 SITL | PX4·Gazebo 환경, 좌표 변환·거절·도착 기록 |
| 4. 지상 FC 융합 | 검토한 FC 설정, 실제 EKF 입력·추정 상태 로그 |
| 5. 비전·AI | 스캔 창·라벨·원본 시각, 단일 명령 권한, RC 반환 |
| 6. 별도 비행 시험 | 승인된 장소·조건에서 이륙·호버·수평 이동 |

LiDAR의 독립 수신은 다음 명령으로 확인한다.
포트 소유자가 없을 때 실행한다.
별도 셸의 `/scan` 구독으로 수신 수를 센다.
시험 종료 시 해당 실행만 Ctrl-C로 종료한다.

```bash
ROS_DOMAIN_ID=2 ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
  --ros-args --params-file src/drone_bringup/params/lidar_tmini.yaml
```

이 실행은 장착 TF나 스캔매칭을 승인하지 않는다.
이유: 거리 수신과 기체 기준 장애물 위치는 다르다.

## 추후 부팅판 전환

현재 실행 중인 원본 서비스를 유지한다.
기존 checkout의 AI는 원래 미추적 파일이었다.
따라서 그 경로에서 즉시 `git pull`하면 충돌할 수 있다.
검토 후 정비 시간에 원본 백업과 소스 해시를 대조한다.
통합 branch를 별도 checkout에서 먼저 빌드한다.
기체별 관측 설정·실제 local 설정을 확인한다.
그 뒤 서비스 경로를 전환하고 cold boot를 시험한다.
키·실제 임무·runtime은 Git으로 옮기지 않는다.
이유: 배포 소스와 개인 실행 상태의 수명이 다르다.
새 배포 helper는 workspace 설치본을 먼저 찾는다.
기존 nested 설치본 경로도 계속 지원한다.
