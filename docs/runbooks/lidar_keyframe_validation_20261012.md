# LiDAR 오프라인·격리 진단 재실행
10월 12일 개선 후보를 재실행하는 절차다.
원본을 회수하거나 같은 커밋의 진단을 반복할 때 읽는다.

## 먼저 확인한다

전체 S01–S14는 아직 통과하지 않았다.
[결과와 남은 게이트](../report/lidar_keyframe_sitl_20261012.md)를 읽는다.
실기 서비스·파라미터·센서 포트는 이 절차에 없다.
현재 `vz=0` 후보는 리셋 위험으로 채택 보류다.

모든 명령의 시작 위치는 ROS2 저장소 루트다.
`work/` 출력 경로는 새 이름을 사용한다.
원본을 출력 경로로 지정하지 않는다.
이유: 비교 기준과 새 계산을 분리해야 하기 때문이다.

## 1. 오프라인 Python 환경

실기 ROS 환경과 별도의 Python 환경을 사용한다.
NumPy·SciPy·pytest가 필요하다.
실험 코드는 `processing/experiments`에 있다.
운영 ROS 노드의 실행 경로는 바꾸지 않았다.

Windows PowerShell 예시:

```powershell
python -m venv work/lidar-python
& work/lidar-python/Scripts/python.exe -m pip install numpy scipy pytest
$env:PYTHONPATH = (Resolve-Path src/drone_uwb).Path
& work/lidar-python/Scripts/python.exe -m pytest -q `
  src/drone_uwb/test/experiments/test_lidar_scanmatch.py `
  src/drone_uwb/test/experiments/test_lidar_replay.py
& work/lidar-python/Scripts/python.exe -m drone_uwb.processing.experiments.lidar_synthetic `
  --seeds 11 22 33 44 55 66 77 88 99 110 `
  --output work/lidar-synthetic.json
```

합성 평가와 실기 원본 재현은 별도다.
`original_baseline_reproduced=false`를 유지한다.
순간 스캔 모델은 드라이버 시간 왜곡을 재현하지 않는다.
`--strict-numerical-baseline`은 첫 수치 수렴 조건이다.
새 comparator의 1e-5m·1e-5rad 설정을 재실행한다.
기존 `analyze.py`를 재현하는 옵션은 아니다.

## 2. 원본 회수 후 replay

SHA-256이 일치하는 원본만 지정한다.
새 도구는 기존 분석기의 대체 재현이라고 주장하지 않는다.
기존 `analyze.py`도 회수해서 별도로 실행해야 한다.

```powershell
$env:PYTHONPATH = (Resolve-Path src/drone_uwb).Path
& work/lidar-python/Scripts/python.exe -m drone_uwb.processing.experiments.lidar_replay `
  --events work/raw/capture-1791703092109574820-28f571f8/events.jsonl `
  --expected-sha256 80f1c6c94ee0aa1d7ec392c8922a1676e5366c29d7b40642213c8f4ea620ac79 `
  --output work/lidar-replay-01
```

위 실행은 LiDAR 단독 진단이다.
IMU 검사에는 실측한 장착 회전이 필요하다.

장착 회전을 확인하면 UTF-8 JSON으로 저장한다.
형식은 `[[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]]`다.
IMU 메시지 좌표의 벡터를 scan 좌표로 바꾸는 회전이다.
행렬식 +1의 직교 회전이어야 한다.
장착을 모르면 단위행렬을 임의로 넣지 않는다.
이유: MAVROS IMU와 scan 축이 같다는 증거가 없기 때문이다.

위 명령에 다음 인자를 추가한다.

```text
--imu-to-scan-rotation work/measured_imu_to_scan.json
```

출력은 스캔별 결과와 summary JSON이다.
원본 행 번호·시각·거절 이유를 보존한다.
IMU 공백은 적분 구간을 분리한다.
역행하는 IMU 시계를 조용히 정렬하지 않는다.
FC 발행은 없다. 광선 시각 보상도 하지 않는다.

## 3. 같은 PX4 커밋으로 격리 빌드

기존 PX4·Gazebo 작업을 멈추지 않는다.
새 Linux 체크아웃에서만 아래를 실행한다.
WSL 전체 종료 명령을 사용하지 않는다.
이유: 다른 작업의 실행 상태를 보존해야 하기 때문이다.

Linux Bash 예시:

```bash
git clone --no-checkout https://github.com/PX4/PX4-Autopilot.git work/PX4-lidar
git -C work/PX4-lidar checkout --detach d6f12ad1c4f70ad3230afd7d86e971421e02fef4
python3 -m venv work/sitl-python
work/sitl-python/bin/python -m pip install numpy pymavlink pyulog kconfiglib jinja2 empy==3.3.4 pyyaml jsonschema packaging pyserial
bash src/drone_uwb/tools/build_lidar_sitl.sh \
  "$PWD/work/PX4-lidar" "$PWD/work/sitl-python/bin/python"
bash src/drone_uwb/tools/build_lidar_sitl.sh \
  "$PWD/work/PX4-lidar" "$PWD/work/sitl-python/bin/python" lockstep
```

PX4의 일반 C++ 빌드 도구도 설치되어 있어야 한다.
이번 환경은 기존 GCC 11.4·CMake·Ninja를 사용했다.
빌드가 성공해야 다음 단계로 간다.
`lidar`는 실시간, `evbench`는 lockstep board다.
기존 board 파일과 내용이 다르면 도구가 덮어쓰지 않는다.

## 4. 120초 진단 한 개를 실행한다

runner가 새 network namespace를 만든다.
Linux의 unprivileged user namespace 지원이 필요하다.
loopback 외 인터페이스가 있으면 실행을 거절한다.
실제 host 네트워크로 우회 실행하지 않는다.
이유: FC·QGC·실기 MAVLink와 접촉하지 않기 위해서다.

```bash
work/sitl-python/bin/python src/drone_uwb/tools/lidar_ev_sitl.py \
  --px4-root "$PWD/work/PX4-lidar" \
  --output "$PWD/work/sitl-nominal-01" \
  --scenario nominal --seed 11 --duration 120 --speed-factor 4 --instance 20
work/sitl-python/bin/python src/drone_uwb/tools/analyze_lidar_sitl.py \
  "$PWD/work/sitl-nominal-01"
```

동시에 실행할 때는 서로 다른 instance를 쓴다.
UDP 포트는 `18570 + instance`다.
network namespace만으로 UNIX socket은 분리되지 않는다.
따라서 instance와 rootfs도 각각 다르게 둔다.

TIMESYNC 50Hz와 PX4 시계를 사용한다.
실제 companion의 시계 drift 시험은 아니다.
ULog의 수신−관측 시각과 RTT를 확인한다.
송신 성공만으로 지연·중복 주입 성공을 판정하지 않는다.
초기 디버깅에서는 시계 동기화 전 수신 시각 대체를 확인했다.
해당 smoke는 정식 matrix 근거로 사용하지 않는다.

## 5. 정지 진단 matrix

28개 조건 × 5 EV 잡음 seed를 실행한다.
SIH 센서 내부 seed는 1234로 고정되어 있다.
이는 140개의 독립 현물 실험이 아니다.

```bash
work/sitl-python/bin/python src/drone_uwb/tools/run_lidar_sitl_matrix.py \
  --px4-root "$PWD/work/PX4-lidar" \
  --output "$PWD/work/lidar-matrix-01" --workers 4 --speed-factor 4
work/sitl-python/bin/python src/drone_uwb/tools/collect_lidar_sitl.py \
  --matrix "$PWD/work/lidar-matrix-01" --output "$PWD/work/lidar-evidence-01"
work/sitl-python/bin/python src/drone_uwb/tools/analyze_lidar_vertical.py \
  --matrix "$PWD/work/lidar-matrix-01" --seed 11 \
  --output "$PWD/work/lidar-evidence-01/vertical_trace_seed11.json"
```

모든 실행은 정지·DISARM 상태다.
실제 LiDAR·광류 장면과 비행 궤적은 포함하지 않는다.
표준 EV 수신·분산·결측·시각 처리의 진단이다.

| 출력 | 의미 |
|---|---|
| `manifest.json` | 커밋·바이너리·board 해시·조건 |
| `inputs.jsonl` | 합성 입력의 값·시각·sequence |
| `telemetry.jsonl` | 수신 MAVLink 원문 |
| `px4.log` | 콘솔·리스너·버전 확인 |
| `rootfs/log/.../*.ulg` | 수정하지 않은 PX4 ULog |
| `analysis.json` | 정답 오차·관측 갱신·분산 집계 |
| `run_status.json` | 실행 종료·송신 수·오류 |
| `sitl_summary.json`, `sitl_runs.json` | 전체 집계·reset counter 증가·유효 플래그 |

관측 갱신에는 융합과 관측 기반 리셋이 포함된다.
리셋은 uint8 counter의 순환까지 반영해 별도로 센다.
단순 갱신 빈도를 정상적인 융합 성능으로 해석하지 않는다.
`vertical_aiding`은 40초 이후 높이 관측을 끈 채 유지한다.
해당 실행의 `recovery` 집계는 실제 복구가 아니다.
50–120초의 제거 후 관찰 구간이라는 뜻이다.

`nan_vz` 등의 원시 JSONL에는 `NaN`이 남는다.
이는 주입한 비유한값을 보존하기 위한 Python JSON 형식이다.
엄격한 JSON parser에는 그대로 입력하지 않는다.
요약 JSON은 `allow_nan=false`로 저장한다.

## 6. 실제 EKF 리셋 경로 probe

같은 빌드의 EKF 정적 라이브러리에 연결한다.
실제 펌웨어 소스나 운영 파라미터는 바꾸지 않는다.

```bash
work/sitl-python/bin/python src/drone_uwb/tools/build_lidar_reset_probe.py \
  --px4-root "$PWD/work/PX4-lidar" --output "$PWD/work/lidar-reset-probe"
work/lidar-reset-probe/reset_probe > work/lidar-reset-probe/result.log
```

`RESULT` 행은 기울기·입력 분산·리셋 전후 값이다.
실제 함수 호출 결과와 소스 수식을 비교한다.
동적 비행에서 리셋이 자동 발동한 결과는 아니다.

## 7. 다음 단계의 조건

원래 [S01–S14 절차](lidar_uwb_sitl_handoff_20261012.md)를 유지한다.
이번 정지 진단으로 전체 시나리오를 대체하지 않는다.
원본 재현·동적 시작·광류 비교·수직 영향이 남았다.
실기 이전에는 장착 회전·거리 실측도 필요하다.
리셋 위험이 남은 설정으로 S14를 진행하지 않는다.
이유: 선행 추정기 시험에서 상태 훼손 경로를 확인했기 때문이다.
