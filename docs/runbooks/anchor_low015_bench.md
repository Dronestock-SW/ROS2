# 임시 0.15m 앵커와 웹 시험 준비
Tag B의 임시 설정을 실행·복원하는 절차다.
지상 관측이나 네 시험 경로를 재생할 때 읽는다.

현재 웹은 계획 확인용이다.
실제 ARM·이륙을 보내는 기능은 아직 없다.
적용 결과는 [당일 기록](../report/anchor_low015_bench_20261007.md)을 본다.

## 앵커와 기체 배치

안테나 중심 기준으로 네 위치를 맞춘다.
A1을 원점, A2를 +X, A3를 +Y로 둔다.

```text
       +Y
        ^
A3 (0,4.6) -------- A4 (6.3,4.6)
   |                     |
 4.6m                  4.6m
   |                     |
A1 (0,0) --- 6.3m --- A2 (6.3,0) --> +X

네 안테나 중심 높이 = 공통 바닥 +0.15m
기체 기본 이륙 높이 = PX4 설정 1.3m (별도 값)
```

대각선의 계산 길이는 약 7.8006m다.
실제 높이가 다르면 개별 실측 좌표를 써야 한다.
높이를 임의로 0으로 두어 일치시킨 것으로 보지 않는다.
기체 출발점은 A1과 달라도 된다.
1m 시험 경로는 출발점에서 창고 축 방향으로 계산한다.

## Tag B 지상 수신

Tag B의 `z015` 펌웨어를 먼저 확인한다.
Jetson 통합 checkout에서 빌드 후 실행한다.
기존 포트 소유자 확인은 [지상 절차](flight_uwb_ai_ground.md)를 따른다.

```bash
cd /home/arialhanho/ROS2-integration-20261007
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb drone_bringup sangwon_ai_replay
source install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch drone_bringup companion_observe.launch.py \
  tag:=B ai_root:="$PWD/src/sangwon_AI" start_mavros:=true \
  record_directory:="$PWD/.integration-evidence/low015-new" stop_after_s:=615
```

기존 MAVROS를 재사용하면 `start_mavros:=false`로 둔다.
실행기는 Tag B의 0.15m B_TF와 수신 설정을 선택한다.
UWB bridge는 비활성이며 기체 명령을 보내지 않는다.

## 웹 계획과 C++ 시험

통합 checkout에서 loopback 웹을 연다.
상태 파일은 마지막 읽기 결과이며 실시간 연결이 아니다.

```bash
python3 src/sangwon_AI/ops/bench_preview.py --port 8347 \
  --audit-record .integration-evidence/post-param-probe/probe/summary.json
ctest --test-dir build/sangwon_ai_replay -R 'bench_plan|sangwon_mode_tests' --output-on-failure
```

같은 컴퓨터에서 `http://127.0.0.1:8347`을 연다.
Windows에서도 Python으로 같은 스크립트를 실행할 수 있다.
앵커 높이와 별개로 모의 목표 Z를 입력한다.
경로를 생성하면 REPLAY JSON을 저장할 수 있다.
이 페이지에는 실제 기체 START 버튼이 없다.

## 2.2m로 복원

펌웨어와 ROS 설정을 함께 복원한다.
기존 펌웨어 저장소 `experiments/lora-sync1hz`의
Tag B 빌드를 사용한다. ID 6을 유지한다.
지상 수신 실행은 복원 B_TF를 명시한다.

```bash
share="$(ros2 pkg prefix --share drone_uwb)"
ros2 launch drone_bringup companion_observe.launch.py \
  tag:=B ai_root:="$PWD/src/sangwon_AI" start_mavros:=true \
  btf_config:="$share/config/runtime/uwb_btf_tag_b_z2p2.json" \
  record_directory:="$PWD/.integration-evidence/restored-z2p2-new" stop_after_s:=615
```

launch가 같은 2.2m 수신 배치를 자동 선택한다.
수신 노드만 실행할 때는 `uwb_tag_b_z2p2.yaml`을 쓴다.
복원 후 heartbeat 배치 ID와 실제 앵커 높이를 대조한다.
FC 이륙 높이 1.3m는 앵커 복원과 별개다.
장착·시각·정확도 확인 플래그를 자동으로 켜지 않는다.
