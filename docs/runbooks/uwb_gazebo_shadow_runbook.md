# Gazebo 경로로 UWB 후보 비교하기

기체 경로를 기록하고 계산 후보를 비교하는 절차다. WSL에서 가상 UWB 시험을 시작할 때 읽는다.

## 시험 범위

현재 도구는 비행 경로를 기록한 뒤 파일로 비교한다.
계산 결과를 PX4에 보내는 단계는 남아 있다.

```text
Gazebo 기체 위치 + 시뮬레이션 시각
  -> poses.jsonl
  -> 가상 앵커 거리 + 잡음 + 편향 + 수신 단절
  -> 편향 차감
  -> A / B / C / D / WLS
  -> 같은 시각의 오차 + 출력률 + 실패 사유
```

기본 비행이 성공해도 UWB 호버링 검증은 아니다.
선정한 위치를 PX4에 넣는 후속 시험이 필요하다.
당장은 ROS 없이 Gazebo의 Python 연결을 사용한다.
[공식 Python 연결](https://gazebosim.org/api/transport/13/python.html)을 따른다.

## 1. 기체 실행과 통신 확인

[WSL 실행 절차](gazebo_wsl_runbook.md) 3절로 기체를 켠다.
현재 확인 모델은 `gz_x500_lidar_down`이다.
별도 PowerShell에서 WSL 창을 하나 더 연다.

```powershell
wsl -d Ubuntu-22.04
```

새 Ubuntu 창에서 다음을 실행한다.

```bash
gz topic -l
/usr/bin/python3 -c "from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V; import numpy; print('Gazebo Python OK')"
```

모듈 누락 오류가 있을 때만 해당 패키지를 설치한다.
기존 Gazebo apt 저장소가 등록된 환경을 전제한다.

```bash
sudo apt update
sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-numpy
```

위 검사를 다시 실행한다.
PX4 venv와 Gazebo 기록용 Python을 구분한다.
기록은 apt 모듈을 읽는 `/usr/bin/python3`로 실행한다.

## 2. 프로그램과 기체 이름 확인

이 도구를 포함한 소스 폴더에서 작업한다.
별도 PC의 기존 Git 저장소에 새 코드가 있다고 가정하지 않는다.
배포 ZIP을 쓸 때도 `src/drone_uwb`가 있는 폴더로 이동한다.

```bash
test -f src/drone_uwb/drone_uwb/integration/gazebo_capture.py
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m drone_uwb.integration.gazebo_capture --help
```

통신 목록에서 `/world/.../dynamic_pose/info`를 찾는다.
아래는 월드 이름이 `default`인 예시다.

```bash
gz topic -e -t /world/default/dynamic_pose/info
```

내용을 확인한 뒤 `Ctrl+C`로 조회만 종료한다.
Gazebo 화면의 최상위 기체 이름과 대조한다.
링크나 중첩 모델 이름을 사용하지 않는다.
그 좌표는 월드 기준 위치가 아닐 수 있기 때문이다.
기체 이름은 정확히 일치해야 한다.

## 3. 경로 기록

확인한 통신 경로와 기체 이름을 넣는다.
아래의 이름은 예시이며 아직 사용자 출력으로 확인하지 않았다.

```bash
/usr/bin/python3 -m drone_uwb.integration.gazebo_capture \
  --topic /world/default/dynamic_pose/info \
  --model x500_lidar_down_0 \
  --duration-s 60 --rate-hz 40 \
  --output runs/gazebo_capture_01
```

기존 가상 비행 절차로 이륙·이동·착륙을 기록한다.
조기 종료는 기록 창에서 `Ctrl+C`를 누른다.
출력 폴더는 매번 새 이름을 사용한다.

| 출력 | 내용 |
|---|---|
| `poses.jsonl` | 시뮬레이션 시각 us, 위치 m, 자세, 기체 이름 |
| `capture.json` | 개수, 실제 평균 기록률, 최대 간격, 종료 사유 |

`40Hz`는 기록률 상한이며 보장 주기가 아니다.
입력 주기에 따라 실제 기록률이 더 낮을 수 있다.
시간과 위치를 보간하거나 중복 발행하지 않는다.
시각 역행은 중단한다. 진행 없는 120초도 중단한다.
수신 큐 초과와 기체 이름 불일치를 별도로 기록한다.

## 4. 같은 경로에 여섯 조건 적용

잡음과 풀이 가중치를 별도로 바꿔 효과를 비교한다.

```bash
/usr/bin/python3 -m drone_uwb.processing.experiments.gazebo_scenarios \
  --input runs/gazebo_capture_01/poses.jsonl \
  --config src/drone_uwb/config/gazebo_shadow.json \
  --output runs/gazebo_comparison_01
```

| 조건 | 센서에 넣는 변화 | WLS가 가정하는 표준편차 |
|---|---|---|
| `equal_quality` | 네 거리 잡음 0.03m | 모두 0.03m |
| `noisy_A2_equal` | A2 잡음만 0.30m | 모두 0.03m |
| `noisy_A2_weighted` | 위와 같은 RAW | A2만 0.30m |
| `noisy_A2_wrong_weight` | 위와 같은 RAW | A1만 0.30m |
| `A2_bias_unmodelled` | 2~6초 A2에 +0.35m | 모두 0.03m |
| `dropout` | 3~3.25초 전체 수신 누락 | 모두 0.03m |

잡음은 정규분포이며 기본 seed는 7이다.
고장 구간은 기록 시작 시각을 기준으로 한다.
6초보다 짧은 기록은 일부 구간을 시험하지 못한다.
60초의 긴 기록에서는 구간별 결과도 확인한다.
전체 평균이 짧은 고장 구간의 영향을 희석할 수 있다.

`profiles/`에 실제 적용한 설정을 저장한다.
이 실행기는 표의 잡음·가중치·고장 조건을 덮어쓴다.
사용자 정의 설정은 단일 실행기로 적용한다.

```bash
/usr/bin/python3 -m drone_uwb.processing.experiments.gazebo_trial \
  --input runs/gazebo_capture_01/poses.jsonl \
  --config my_trial.json --output runs/my_trial_01
```

`my_trial.json`은 기본 설정을 복사해 작성한다.
앵커 배치·잡음·편향·단절·가중치·seed를 변경할 수 있다.
B 비교에는 같은 높이의 네 앵커 배치를 사용한다.
이는 현재 B 구현의 입력 조건이다.

## 5. 결과 판정

오차와 출력률을 함께 읽는다.

| 파일·키 | 해석 |
|---|---|
| `comparison.json` | 여섯 조건 전체 요약 |
| 각 조건의 `summary.json` | `evaluation.models`의 개별 오차·출력률 |
| `evaluation.pairwise.A_WLS` 등 | 두 모델이 모두 출력한 같은 시각의 비교 |
| `results.jsonl` | RAW·보정 거리·개별 후보·실패 사유·기준 위치 |
| `manifest.json` | 입력·설정·코드·출력 해시 |

실패한 표본을 출력률 분모에서 빼지 않는다.
이전 위치로 실패 구간을 채우지 않는다.
위치 정답은 거리 생성과 사후 평가에만 사용한다.
풀이기는 정답 XY를 받지 않는다.

Z는 같은 시각의 시뮬레이터 위치에서 제공한다.
ToF 오차·기울기·장착 위치 오차는 아직 모사하지 않는다.
태그 기준점은 기체 모델 원점으로 가정한다.
네 거리도 같은 시각에 생성한다.
이 전제들은 실측 비행 모델과 구분한다.

실행·검증 범위는 [준비 결과](../report/uwb_gazebo_shadow_20260927.md)에 있다.
