# UWB 지상 검증 절차
이 문서는 UWB 관측 노드 실행 절차다.
태그 연결과 지상 재현 시험 때 읽는다.

## 먼저 확인

기본 설정은 관측 수신만 수행한다.
브리지는 비활성 상태로 시작한다.
시동·비행 명령과 FC 설정 쓰기는 포함하지 않는다.
이유: 좌표·지연·장착 위치를 먼저 검증해야 한다.

1. `/dev/uwb`가 태그 포트를 가리키는지 확인한다.
2. 해당 포트를 읽는 다른 프로그램을 종료한다.
3. 아래 명령은 workspace 루트에서 실행한다.

```bash
cd /home/user/drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1
export PYTHONIOENCODING=utf-8
```

위 ROS_LOCALHOST_ONLY는 한 companion 지상 시험용이다.
기체 구분은 ROS_DOMAIN_ID를 사용한다.

## 60초 UWB 수신

새 기록 폴더를 지정하고 실행한다.
기존 기록 폴더를 덮어쓰지 않는다.
이유: 서로 다른 시험 결과가 섞일 수 있다.

```bash
ros2 launch drone_uwb uwb.launch.py \
  record_directory:=/tmp/uwb_bench_run_01 \
  stop_after_s:=60.0
```

60초가 지나면 수신 노드와 브리지가 종료된다.
연속 수신은 `stop_after_s:=0.0`을 사용한다.
설정 파일은 `src/drone_uwb/config/uwb.yaml`이다.
변경한 설정은 노드 재시작 때 적용한다.

| 확인 대상 | 정상 해석 |
|---|---|
| `/uwb/raw` | 현재 태그의 원본 JSON |
| `/uwb/status` | 상태 대기·통과·거부 횟수 |
| `/uwb_pose` | `uwb_map`의 새 x·y 관측 |
| `/uwb/bridge_status` | 기본 `gate=disabled`, 발행 0개 |
| `raw/serial.raw` | 읽은 시리얼 바이트 |
| `raw/received.jsonl` | 메시지와 companion 수신 시각 |
| `processed/decisions.jsonl` | 주기별 처리 사유와 관측 |

상태 메시지 대기와 초기 30개 준비 구간이 있다.
이 시간의 미발행을 통신 실패와 구분한다.

## MAVROS 연결 점검

MAVROS가 이미 실행 중이면 중복 기동하지 않는다.
이유: 같은 포트를 두 프로그램이 읽으면 데이터가 나뉜다.
첫 터미널에서 다음을 실행한다.

```bash
ros2 run mavros mavros_node --ros-args -r __ns:=/mavros \
  --params-file src/drone_uwb/config/mavros_uwb.yaml \
  -p fcu_url:=/dev/pixhawk:921600 \
  -p tgt_system:=1 -p tgt_component:=1 -p fcu_protocol:=v2.0
```

별도 터미널에서도 같은 ROS 환경을 적용한다.
UWB 노드를 실행한 뒤 다음을 수행한다.

```bash
ros2 run drone_uwb uwb_bench_probe \
  --output /tmp/uwb_ros_probe_01 --seconds 30
```

이 도구는 topic과 FC 파라미터 사본을 읽는다.
센서·노드 설정은 바꾸지 않는다.
시험 종료 후 MAVROS 터미널에서 Ctrl+C를 누른다.

## 기록 재생과 태그 갱신 검산

같은 입력으로 전처리 결과를 재현한다.
기준점 좌표는 해당 기록에서 확인한 경우에만 넣는다.

```bash
ros2 run drone_uwb uwb_replay /tmp/uwb_bench_run_01/raw/received.jsonl \
  --layout src/drone_uwb/config/anchors_20260906.json \
  --settings src/drone_uwb/config/uwb.yaml \
  --output /tmp/uwb_replay_01
```

검산할 때 `--source-mode tag_xy`를 추가한다.
태그 x·y와 새 배치의 재계산 차이를 확인한다.
기존 배치가 남으면 `tag_layout_mismatch`가 나온다.
실제 태그 갱신 전에는 기본 `raw_ranges`를 유지한다.

태그 갱신값은 [설정 사전](uwb_tag_layout_update.md)을 따른다.
수신 성공만으로 갱신 완료를 판정하지 않는다.
이유: 배치 ID와 좌표 계산의 일치를 검증해야 한다.

## PX4 융합 전 현장 확인

확인값은 [검증 계획](report/uwb_integration_20260906.md)을 따른다.
설계의 브리지 조건을 하나씩 검증한다.

| 확인 | 방법 |
|---|---|
| 태그 배치 | 새 앵커 좌표로 RAW와 태그 x·y 대조 |
| 방향·원점 | 같은 기체로 +x·+y 직선 이동 |
| 장착 위치 | 태그 안테나와 FC 기준점 차이 측정 |
| 지연 | 이동·정지 전환을 공통 시각 기준으로 비교 |
| PX4 수신·융합 | v1.16의 외부 관측·EKF2 상태 확인 |

확인 플래그만 켜서 검증을 대신하지 않는다.
이유: 숫자 입력만으로 현장 정렬이 완료되지는 않는다.
자세·고도 제어와 센서 융합은 PX4가 담당한다.

## 2026-09-27 기록 경로 변경

새 세션은 `raw/`와 `processed/`를 나눈다.
이전 기록의 파일 위치는 그대로 읽을 수 있다.
새 세션 경로는 미리 만들지 않는다.
[자료 실행 절차](uwb_data_runbook.md)에 전체 구조를 적었다.
