# UWB 자료 기록·재생 절차

새 폴더 구조에서 수신과 보정을 실행하는 절차다.
자료를 기록하거나 기존 정지 시험을 재생할 때 읽는다.

## 빌드

저장소 루트에서 패키지를 빌드한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb
source install/setup.bash
```

기존 `uwb_node`, `uwb_pipeline` 실행 이름은 유지한다.
새 정지 실행 이름은 `uwb_static_a`다.
이전 `drone_uwb.preimu.*`도 같은 구현으로 연결한다.
새 코드에서는 `drone_uwb.processing.*`를 사용한다.

## A/B/C/D 합성 비교

UWB 알고리즘 시뮬레이터를 소스에서 실행한다.
Gazebo나 PX4를 띄우는 명령은 아니다.
기존 파일과 섞이지 않게 새 출력 경로를 준다.

```bash
PYTHONPATH=src/drone_uwb python src/drone_uwb/tools/run_subset_simulation.py \
  --config src/drone_uwb/config/subsets_cd_static_20260906.json \
  --anchors src/drone_uwb/config/anchors_20260906.json \
  --output /tmp/uwb_cd_simulation_new
```

설정에서 A/B/C/D 계수만 읽는다.
그 안의 실측 파일 경로는 이 도구가 사용하지 않는다.
8초·40Hz·seed 7의 여섯 상황을 생성한다.
시나리오 자체는 기존 `benchmark_subsets.simulate`에 있다.
현재 CLI에는 길이·seed 변경 인수가 없다.

| 출력 | 내용 |
|---|---|
| `synthetic_results.jsonl` | 주기별 기준 XY·각 모델 위치·실패 사유 |
| `synthetic_summary.json` | 같은 시각 오차·전체 가용률 |
| `diagnostics.json` | 평균 편차·표준편차·D 슬롯 사유 집계 |
| `simulation_conditions.json` | 경로·잡음·거리 이상·단절 조건 |
| `model_settings.json`, `anchors.json` | 실제 사용한 계산 설정·지도 |
| `simulation_errors.png/pdf` | 여섯 상황의 오차·출력률 |
| `simulation_paths.png/pdf` | 직선·반전의 경로와 X 시간 변화 |
| `manifest.json` | 도구·계산 소스·설정·출력 해시 |

[2026-09-27 실행 기록](report/uwb_cd_simulation_20260927.md)에 확인 결과가 있다.

## 새 수신 기록

`record_directory`에는 아직 없는 세션 경로를 준다.
다음 예시 경로가 있으면 다른 세션 이름을 사용한다.

```bash
ros2 run drone_uwb uwb_node --ros-args \
  --params-file src/drone_uwb/config/uwb.yaml \
  -p record_directory:="$PWD/data/captures/session_001" \
  -p stop_after_s:=60.0
```

```text
data/captures/session_001/
  raw/serial.raw            UART 원본 bytes
  raw/received.jsonl        수신 시각 + 파싱된 message
  raw/node_metadata.json   장치·설정·배치
  processed/decisions.jsonl 판정·계산 관측
  processed/node_status.jsonl 상태 집계
```

기존 단일 폴더 기록과 경로가 달라졌다.
새 기록을 분석할 때 `raw/received.jsonl`을 지정한다.
원본 bytes는 파싱 전에 기록한다.
파싱 실패 행도 `serial.raw`에 남는다.
계산·상태 기록은 `processed/`에만 쓴다.
기존 세션 경로를 재사용하면 시작을 거부한다.
이는 다른 수신 세션이 섞이는 것을 막기 위함이다.
장치 수신 시험은 실제 장치를 연결한 뒤 실행한다.

## 기존 정지 UWB 계산

고정된 교정·평가 입력을 새 결과 경로로 계산한다.

```bash
ros2 run drone_uwb uwb_static_a \
  --config src/drone_uwb/config/baseline_a_static_20260906.json \
  --root "$PWD" \
  --output data/processed/static_a_new_run
```

ROS 없이 실행하려면 다음 명령을 쓴다.

```bash
PYTHONPATH=src/drone_uwb python3 -m drone_uwb.processing.experiments.static_a \
  --config src/drone_uwb/config/baseline_a_static_20260906.json \
  --root "$PWD" \
  --output data/processed/static_a_new_run
```

두 명령은 같은 작업이다. 둘 중 하나만 실행한다.
실행기는 입력 해시와 교정·평가 분리를 검사한다.
결과는 CSV·JSONL·요약·manifest로 기록한다.
기존 출력 폴더가 있으면 새 이름을 지정한다.

## 자료·경계 확인

다음 검사는 장치 없이 수행할 수 있다.

```bash
PYTHONPATH=src/drone_uwb python3 -m pytest -q src/drone_uwb/test
```

검사는 catalog의 바이트·해시·호환 경로를 대조한다.
수신부의 계산기 의존과 하위 계층의 ROS 의존도 검사한다.
새 RAW는 `data/raw/`에 등록하고 해시를 남긴다.
새 보정 결과는 `data/processed/`에 별도 실행으로 등록한다.
과거 catalog 해시를 새 파일에 맞춰 덮어쓰지 않는다.
입력 변경은 별도 데이터로 추적하기 위함이다.

26일 ULog는 자세·거리 분석에 사용한다.
동시 UWB·시계 대응·장착 확인 전에는 결합하지 않는다.
[모듈 계약](uwb_module_api.md)의 시각·좌표 조건을 따른다.
