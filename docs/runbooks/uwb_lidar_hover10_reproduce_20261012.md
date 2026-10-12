# 10초 호버 격리 재현
2026-10-12의 스캔 정합 폐루프 시험 절차다.
실기 연결 없이 같은 입력·판정으로 재검증할 때 읽는다.

## 준비

Ubuntu-22.04의 보존 SIH runtime을 사용한다.
다른 PX4·Gazebo 프로세스를 중지하지 않는다.
출력 폴더는 매번 새 이름으로 만든다.
실기 주소와 장치를 인자로 받지 않는다.

아래는 WSL Bash 명령이다.

```bash
EXPERIMENT_ROOT=/home/dronestock/uwb_sim/uwb-lidar-integration-20261012
EXPERIMENT_REPO=/mnt/c/Users/A/Documents/Codex/2026-10-12/new-chat/work/ros2-uwb-lidar-integration
EXPERIMENT_PYTHON=/home/dronestock/.venvs/px4/bin/python
export PYTHONPATH="$EXPERIMENT_ROOT/hover10-python:$EXPERIMENT_REPO/src/drone_mission:$EXPERIMENT_REPO/src/drone_uwb:$EXPERIMENT_REPO/src/drone_demo"
```

기존 venv의 numpy·pymavlink·pyulog를 사용했다.
SciPy는 별도 폴더에 설치했다.
기존 venv의 패키지는 변경하지 않았다.

```bash
"$EXPERIMENT_PYTHON" -m pip install --no-deps \
  --target "$EXPERIMENT_ROOT/hover10-python" scipy==1.15.3
```

이미 설치된 폴더에서는 설치를 반복할 필요가 없다.
runtime의 바이너리·패치·board 해시는 실행기가 검사한다.
기준 commit은 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`다.

## 고정 프로파일

선정값은 보고서의 불확실성 모델과 함께 사용한다.
실기 권장 파라미터 파일이 아니다.

```bash
EXPERIMENT_PROFILE="$EXPERIMENT_REPO/docs/report/evidence/uwb_lidar_hover10_20261012/profiles/armed_imu_model.json"
```

다른 후보는 증빙의 `profiles/`에 보존했다.
아래 새 seed를 본 뒤 선정값을 바꾸지 않았다.
기존 개발 시행도 삭제하지 않았다.

## 스캔 정합·벽·공백 10회

clear·wall은 seed마다 한 번씩 실행한다.
네 공백·점프 조건은 첫 seed에서 실행한다.

```bash
"$EXPERIMENT_PYTHON" "$EXPERIMENT_REPO/src/drone_mission/tools/run_obstacle_matrix.py" \
  --runtime "$EXPERIMENT_ROOT/runtime-lidar2d-v2" \
  --output "$EXPERIMENT_ROOT/hover10-reproduce-scan" \
  --seeds 301 302 303 \
  --scenarios clear wall scan_gap lidar_gap uwb_gap uwb_jump \
  --aiding-mode enabled --lidar-source scan_matcher \
  --lidar-velocity-intervals 1 --speed-factor 1 --hover-duration-s 10 \
  --fusion-horizon-ms 100 --zero-unused-airspeed-delay \
  --velocity-output-tau-s .05 --sitl-parameters "$EXPERIMENT_PROFILE"
```

실행기는 52·53번 인스턴스를 순서대로 빌린다.
각 자식은 별도 user/network namespace를 만든다.
외부 네트워크가 있으면 시작을 거부한다.
위 행렬이 끝나기 전에 다음 행렬을 시작하지 않는다.
같은 인스턴스 잠금의 충돌을 막기 위해서다.

## LiDAR 보조 입력 비활성 비교 3회

같은 파라미터·센서·정합 계산을 유지한다.
전용 PX4 LiDAR 융합만 비활성으로 둔다.

```bash
"$EXPERIMENT_PYTHON" "$EXPERIMENT_REPO/src/drone_mission/tools/run_obstacle_matrix.py" \
  --runtime "$EXPERIMENT_ROOT/runtime-lidar2d-v2" \
  --output "$EXPERIMENT_ROOT/hover10-reproduce-noaid" \
  --seeds 301 302 303 --scenarios clear \
  --aiding-mode disabled --lidar-source scan_matcher \
  --lidar-velocity-intervals 1 --speed-factor 1 --hover-duration-s 10 \
  --fusion-horizon-ms 100 --zero-unused-airspeed-delay \
  --velocity-output-tau-s .05 --sitl-parameters "$EXPERIMENT_PROFILE"
```

## 분석

return code 0은 실행기의 정상 종료만 뜻한다.
호버·벽·융합의 판정은 JSON으로 별도 확인한다.

```bash
"$EXPERIMENT_PYTHON" "$EXPERIMENT_REPO/src/drone_mission/tools/audit_hover10_sitl.py" \
  --matrix "$EXPERIMENT_ROOT/hover10-reproduce-scan" \
  --output "$EXPERIMENT_ROOT/hover10-reproduce-metrics.json"

"$EXPERIMENT_PYTHON" "$EXPERIMENT_REPO/src/drone_mission/tools/analyze_obstacle_sitl.py" \
  --matrix "$EXPERIMENT_ROOT/hover10-reproduce-scan" \
  --output "$EXPERIMENT_ROOT/hover10-reproduce-audit.json"
```

`mission_10s_complete`와 `precision_hover_pass`를 구분한다.
`first_10s_ulog`는 도착 직후 미리 정한 창이다.
`whole_dwell`은 약 35초의 모든 평가 표본이다.
가장 좋은 10초를 찾아 합격으로 바꾸지 않는다.

원본 `mission.jsonl`, `scan_observations.jsonl`, ULog를 보존한다.
`source_snapshot/`와 `manifest.json`으로 실행 코드를 대조한다.
물리 잡음의 SIH RNG는 upstream의 seed 1234를 유지한다.
301·302·303은 UWB·LiDAR 합성 잡음 seed다.
실행 스케줄은 조금 달라질 수 있다.
표본별 바이트 일치까지 보장하는 replay가 아니다.
