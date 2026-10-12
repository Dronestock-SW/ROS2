# 회전 LiDAR 격리 재현
2026-10-12의 광선 시각·부분 가림 시험 절차다.
실기 연결 없이 같은 모델을 검사할 때 읽는다.

## 실행 범위

실기 서비스·센서·FC에는 명령을 보내지 않는다.
출력 폴더는 새 이름을 사용한다.
다른 PX4·Gazebo 프로세스를 중지하지 않는다.
private network namespace와 loopback만 사용한다.
동시에 실행하는 matrix는 instance 번호를 분리한다.
이미 사용 중인 instance를 강제로 해제하지 않는다.

아래 명령은 WSL Bash 기준이다.
`EXPERIMENT_ROOT`는 보존된 시험 폴더다.
원시 실기 기록과 기존 runtime을 덮어쓰지 않는다.

```bash
EXPERIMENT_ROOT=/home/dronestock/uwb_sim/uwb-lidar-integration-20261012
EXPERIMENT_REPO=/mnt/c/Users/A/Documents/Codex/2026-10-12/new-chat/work/ros2-uwb-lidar-integration
EXPERIMENT_PYTHON=/home/dronestock/.venvs/px4/bin/python
export PYTHONPATH="$EXPERIMENT_ROOT/hover10-python:$EXPERIMENT_REPO/src/drone_mission:$EXPERIMENT_REPO/src/drone_uwb:$EXPERIMENT_REPO/src/drone_demo"
cd "$EXPERIMENT_REPO"
```

의존성은 numpy·SciPy·pymavlink·pyulog다.
기존 venv는 변경하지 않았다.
SciPy 1.15.3은 `hover10-python`에 별도 설치했다.
회귀 검사는 Windows Python에서 수행했다.

## 가상 센서 비교

동일 seed의 세 방식에 같은 광선·IMU를 준다.
폐루프 비행과 분리한 센서 시험이다.

```bash
"$EXPERIMENT_PYTHON" src/drone_uwb/tools/benchmark_spinning_lidar.py \
  --output ../../outputs/spinning_frontend_repeat.json \
  --seconds 12 --seeds 901 902 903
```

결과에는 거절한 스캔도 남는다.
속도 오차의 분모는 수용 스캔이다.
보류율의 분모는 준비 스캔을 포함한 전체다.
기존 held-out 값을 보고 조정했다면 개발 시험으로 분류한다.
그 뒤 성능 검증에는 새로운 seed가 필요하다.

## 전용 SIH runtime

기존 2D 입력 runtime에 truth 전송만 추가했다.
추정기에 정답을 입력하는 기능이 아니다.
가상 센서의 광선별 위치 계산에 사용한다.
SIH 동역학·센서 잡음 코드는 변경하지 않았다.

아래 소스는 이 시험이 소유한 격리 checkout이다.
PX4 base는 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`다.
기존 준비는 [이전 절차](uwb_lidar_reproduce_20261012.md)를 따른다.
`--output`은 존재하지 않는 새 경로여야 한다.

```bash
"$EXPERIMENT_PYTHON" src/drone_uwb/tools/prepare_spinning_sitl.py \
  --source "$EXPERIMENT_ROOT/PX4-lidar2d"
cmake --build "$EXPERIMENT_ROOT/PX4-lidar2d/build/px4_sitl_lidar2d" --parallel 2
"$EXPERIMENT_PYTHON" src/drone_uwb/tools/prepare_spinning_sitl.py \
  --source "$EXPERIMENT_ROOT/PX4-lidar2d" \
  --output "$EXPERIMENT_ROOT/runtime-spinning-repeat"
```

runner는 바이너리·패치·board hash를 검사한다.
기존 `runtime-v2`와 `runtime-spinning-v1`을 보존한다.
후자는 이번 실제 실행에 사용한 artifact다.

## 비행·고장 matrix

정상·벽은 각각 세 번 시험한다.
나머지 고장·가림은 첫 seed로 한 번씩 시험한다.
프로파일은 [보존 JSON](../report/evidence/spinning_lidar_20261012/profiles/captured_sensors.json)이다.

```bash
"$EXPERIMENT_PYTHON" src/drone_mission/tools/run_obstacle_matrix.py \
  --runtime "$EXPERIMENT_ROOT/runtime-spinning-v1" \
  --output "$EXPERIMENT_ROOT/spinning-validation-repeat" \
  --instances 52 53 --seeds 501 502 503 --aiding-mode enabled \
  --lidar-source spinning --sensor-capture-time --speed-factor 1 \
  --hover-duration-s 10 --fusion-horizon-ms 200 \
  --zero-unused-airspeed-delay --velocity-output-tau-s .05 \
  --sitl-parameters docs/report/evidence/spinning_lidar_20261012/profiles/captured_sensors.json \
  --scenarios clear wall person partial_occlusion imu_gap clock_gap \
    scan_gap lidar_gap uwb_gap uwb_jump
```

LiDAR 융합 비활성 대조는 새 출력 폴더에 실행한다.
동일 명령에서 `--aiding-mode disabled --scenarios clear`를 쓴다.
거리 생성·deskew·guard·관측 송신은 유지된다.
정합 보정 비활성 대조는 `--no-deskew`를 추가한다.
이 대조의 aiding mode는 enabled다.
동시 실행 시 `--instances 54 55`를 사용한다.
같은 seed라도 폐루프 물리 표본은 완전히 같지 않다.

시나리오 이름 오타는 비행 전에 거절한다.
원래 시행에서는 `partial` 오타 1회가 있었다.
출력·ARM 전에 종료했고 카탈로그에 보존했다.
수정한 `partial_occlusion`은 별도 폴더에서 실행했다.

## 검증과 판정

`run_status.json`의 종료 사유부터 확인한다.
ARM·AIRBORNE·OFFBOARD·명령 feedback이 필요하다.
단순 프로세스 종료를 호버 합격으로 보지 않는다.

```bash
"$EXPERIMENT_PYTHON" src/drone_mission/tools/audit_spinning_sitl.py \
  --runs "$EXPERIMENT_ROOT/spinning-validation-repeat/clear-s501-a1" \
  --output ../../outputs/spinning_repeat_audit.json
```

| 기준 | 판정 방법 |
|---|---|
| 목표점 XY RMS | 첫 도착 후 10초, 평균값 차감 없음 |
| 두 위치 자료 | 고율 FC ULog와 SIH 원시 위치 각각 3cm 이하 |
| 안정 유지 | 기존 위치·속도·고도 조건 연속 10초 |
| 벽 여유 | 실제 SIH 위치에서 반경 20cm를 뺀 외곽 50cm 이상 |
| 시각 | 첫 광선·중점·PX4 수신 시각을 별도 확인 |
| 실제 융합 | `estimator_aid_src_aux_vel.fused` 확인 |
| reset | 이동 중 XY·Z·VXY·VZ reset 횟수 확인 |
| 보류 | 전체 분모·사유·회복을 기록 |

미션 기록의 약 60ms 검사와 고율 ULog를 구분한다.
RMS가 작아도 속도 조건이 끊기면 불합격이다.
사람 가림의 위치 관측과 이동 허가도 구분한다.
최종 결과·한계는 [수행 기록](../report/spinning_lidar_20261012.md)을 따른다.

## Windows 회귀 검사

아래 다섯 파일은 ROS·웹 의존성으로 제외했다.
전체 ROS 노드 통합 검증은 미실시다.

```powershell
$env:PYTHONPATH='src/drone_mission;src/drone_uwb;src/drone_demo'
python -m pytest -q src/drone_mission/test src/drone_uwb/test `
  --ignore=src/drone_mission/test/test_local_flight_ros.py `
  --ignore=src/drone_mission/test/test_local_web.py `
  --ignore=src/drone_mission/test/test_mission_store.py `
  --ignore=src/drone_mission/test/test_operator_arm.py `
  --ignore=src/drone_mission/test/test_site.py
```
