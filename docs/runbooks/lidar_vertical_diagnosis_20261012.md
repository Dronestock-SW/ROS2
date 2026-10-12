# LiDAR 수직 위험 후속 진단 재실행
보존 ULog 분석과 계측 라이브러리 비교 절차다.
2026-10-12 후속 결과를 재확인할 때 읽는다.

## 1. 범위를 확인한다

[후속 보고서](../report/lidar_vertical_followup_20261012.md)를 먼저 읽는다.
현재 입력 후보와 S14는 보류다.
이 절차는 운영 PX4를 패치하거나 실행하지 않는다.
이유: 원인 분리 조작은 비행용 수정안이 아니다.
원시 파일·기존 출력 디렉터리를 덮어쓰지 않는다.
모든 `--output`에는 새 디렉터리를 지정한다.

기존 [스캔·SIH 절차](lidar_keyframe_validation_20261012.md)는 유지한다.
원본 −267° 재현은 그 절차의 해시 확인부터 시작한다.
이번 수직 probe로 원본 replay를 대체하지 않는다.

## 2. 환경과 경로를 정한다

아래 명령은 WSL Ubuntu-22.04의 Bash용이다.
실기 SSH 셸에서 실행하는 명령이 아니다.
경로가 바뀌면 존재·커밋·해시부터 확인한다.

```bash
TASK_REPO=/mnt/c/Users/A/Documents/ChatGPT/DroneStock/ROS2
TASK_PX4=/home/dronestock/uwb_sim/lidar-uwb-20261012/PX4-Autopilot
TASK_OLD=/home/dronestock/uwb_sim/lidar-uwb-20261012/matrix-01
TASK_NEW=/home/dronestock/uwb_sim/lidar-vertical-followup-repeat
TASK_TOOLS="$TASK_REPO/src/drone_uwb/tools"
TASK_PY=/home/dronestock/.venvs/px4/bin/python
git -C "$TASK_PX4" rev-parse HEAD
git -C "$TASK_PX4" status --short
```

고정 PX4 커밋은 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`다.
빌더는 다른 커밋이나 추적 소스 변경 시 중단한다.
기존 미추적 보드 설정은 보존한다.
기존 `build/px4_sitl_lidar`의 라이브러리가 필요하다.
그 빌드가 없으면 이전 절차로 별도 복제본에서 준비한다.

분석 의존성은 NumPy·pyulog·Matplotlib이다.
단위 테스트에는 SciPy·pytest도 필요하다.
없으면 새 전용 venv에만 설치한다.
기존 PX4/ROS venv를 임의로 갱신하지 않는다.
이유: 이미 사용 중인 환경을 비교 기준으로 보존한다.

## 3. 기존 원시 해시와 내부 상태를 분석한다

```bash
"$TASK_PY" "$TASK_TOOLS/analyze_lidar_state_output.py" \
  --matrix "$TASK_OLD" \
  --prior-runs "$TASK_REPO/docs/report/evidence/lidar_keyframe_sitl_20261012/sitl_runs.json" \
  --archive /mnt/c/Users/A/Documents/Codex/2026-10-12/dronestock-sw-ros2-feature-uwb-agents/outputs/lidar-sitl-raw-20261012.tar.gz \
  --output "$TASK_NEW/ulog-review"
```

기대 결과는 ULog 140개·기록 840개 일치다.
높이 제거의 속도 포함·위치 전용 각 5회를 분석한다.
원시 불일치가 있으면 재분석 전에 중단한다.
기존 정상 해시를 새 파일의 해시로 덮어쓰지 않는다.
이유: 원본 교체를 정상으로 숨길 수 있기 때문이다.

## 4. 별도 라이브러리 실험을 실행한다

```bash
"$TASK_PY" "$TASK_TOOLS/build_lidar_vertical_probe.py" \
  --px4-root "$TASK_PX4" --output "$TASK_NEW/probe"
"$TASK_PY" "$TASK_TOOLS/run_lidar_vertical_probe.py" \
  --probe "$TASK_NEW/probe/vertical_probe" \
  --output "$TASK_NEW/matrix"
```

빌더는 원본 파일을 읽고 새 폴더에 계측 복사본을 만든다.
기존 라이브러리는 다시 빌드하거나 수정하지 않는다.
기본 행렬은 7방식·5 seed·2기울기다.
실행기는 별도 user/network namespace로 다시 진입한다.
loopback 외 인터페이스가 있으면 실행을 중단한다.
실물 장치나 ROS 송신 경로는 코드에 없다.

다음 대조는 계측이 stock 결과를 바꾸는지 확인한다.

```bash
"$TASK_PY" "$TASK_TOOLS/build_lidar_vertical_probe.py" \
  --px4-root "$TASK_PX4" --output "$TASK_NEW/stock-library" \
  --stock-library
"$TASK_PY" "$TASK_TOOLS/run_lidar_vertical_probe.py" \
  --probe "$TASK_NEW/stock-library/vertical_probe" \
  --output "$TASK_NEW/stock-check" \
  --modes stock --seeds 11 --tilts 0
```

## 5. 기록의 일치와 그림을 확인한다

```bash
"$TASK_PY" "$TASK_TOOLS/summarize_lidar_vertical_followup.py" \
  --matrix "$TASK_NEW/matrix" \
  --stock-check "$TASK_NEW/stock-check" \
  --state-output "$TASK_NEW/ulog-review/state_output_comparison.json" \
  --output "$TASK_NEW/review"
```

70개 원시 해시와 840,000표본의 분모를 확인한다.
stock과 출력 리셋 제외의 내부 기록을 10쌍 대조한다.
원본 라이브러리와 stock 계측의 모든 기록 필드도 대조한다.
실행기는 빌드 manifest의 바이너리 해시도 확인한다.
계측 없는 바이너리에 리셋 생략 모드를 요청하면 거부한다.
합격 오차를 새로 정하는 검사가 아니다.
9자리 유효숫자로 출력된 기록값의 동일성 검사다.
생성한 PNG 두 장을 열어 축·범례·잘림을 확인한다.

기존 17개 테스트는 Windows PowerShell에서 확인했다.

```powershell
$env:PYTHONPATH = (Resolve-Path src/drone_uwb).Path
$env:PYTHONIOENCODING = 'utf-8'
python -m pytest -q -p no:cacheprovider `
  src/drone_uwb/test/experiments/test_lidar_scanmatch.py `
  src/drone_uwb/test/experiments/test_lidar_replay.py
```

## 6. 다음 검증으로 인계한다

현재 [시나리오 상태](../report/evidence/lidar_vertical_followup_20261012/scenario_status.json)를 따른다.
S04/S05의 이동·회복은 미실시다.
S12 광류 비교와 S14 폐루프도 미실시다.
aux2d의 낮은 정지 RMS를 비행 승인으로 쓰지 않는다.
이유: fake height·합성 NE·수평 보조 조건이 남아 있다.
원본·장착 회전·FC 기준 레버암을 확보한 뒤,
body 2D 관측 계약과 동적 비교를 따로 설계한다.
