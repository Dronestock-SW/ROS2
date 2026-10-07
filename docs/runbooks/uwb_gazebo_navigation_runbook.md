# Gazebo 목표 이동 실행 절차
이 문서는 WSL의 관측·목표 이동 통합 실행 절차다.
현재 PX4 기동 확인 뒤 새 실행기를 반영할 때 읽는다.

## 현재 상태와 이번 순서

목표 실행기의 로컬 모의 통신 시험 142개가 통과했다.
후속 독립 평가·이상 주입 누적본의 시험 68개도 통과했다.
시험 묶음은 서로 겹치므로 합산하지 않는다.
빌드한 5개 패키지의 설치 경로 import도 통과했다.
사용자 WSL의 배포·실제 연결·비행은 미검증이다.
기존 UWB 정지 기록의 성과는 그대로 보존한다.
이번 구현이 7cm 목표 달성을 뜻하지는 않는다.
Windows에서 작업을 이어받을 때는 [인계 절차](uwb_windows_handoff.md)를 먼저 읽는다.

```text
PX4·Gazebo 기동 확인
 → 새 실행기 import 확인
 → Shadow 센서 계산 기록
 → 같은 연결에서 FC 상태·시계 기록
 → 정렬·불확실도·관측 시간·융합 확인
 → 공중 Hold에서 한 목표 이동
 → 출발점 복귀·착륙
 → 실제 궤적·ULog로 독립 평가
```

## 갱신본 반영

아래는 WSL Ubuntu 셸에서 실행한다.
`pxh>`와 원격 `pgyxn` 셸의 명령이 아니다.
기존 실행 폴더와 분리해 원본 설정을 보존한다.

```bash
cd ~/uwb_sim
scp pgyxn@100.110.163.94:/home/pgyxn/uwb_gazebo_fault_evaluation_v2_20261002.zip .
scp pgyxn@100.110.163.94:/home/pgyxn/uwb_gazebo_fault_evaluation_v2_20261002.zip.sha256 .
sha256sum -c uwb_gazebo_fault_evaluation_v2_20261002.zip.sha256
mkdir fault_eval_v2_20261002
unzip uwb_gazebo_equipment_20260927.zip -d fault_eval_v2_20261002
unzip -o uwb_gazebo_fault_evaluation_v2_20261002.zip -d fault_eval_v2_20261002/uwb-gazebo-equipment
cd fault_eval_v2_20261002/uwb-gazebo-equipment
mkdir -p runs
cp -a "$HOME/uwb_sim/uwb-gazebo-equipment/runs/equipment_02" runs/equipment_02
export PYTHONPATH="$PWD/src/drone_uwb:$PWD/src/drone_demo${PYTHONPATH:+:$PYTHONPATH}"
```

`mkdir`에서 이미 있는 폴더라고 나오면 새 시행 이름을 쓴다.
기존 시행을 덮어써 재사용하지 않는다.
이 갱신본은 PX4 모델이나 월드를 다시 설치하지 않는다.
실행기는 기본값으로 목표를 송신하지 않는다.
이 갱신본은 관측·목표·독립 평가의 누적본이다.
이전 갱신 ZIP을 차례로 덮어쓸 필요가 없다.
v2는 Windows 인계와 설치 안내를 정리한 문서 개정이다.
코드·설정·시험 파일은 직전 누적본과 같다.
이상 주입은 별도 옵션으로 선택한다.
[이상 시험 절차](uwb_gazebo_fault_trials.md)를 따른다.

## 한 Python에서 필요한 모듈 확인

관측과 목표 연결은 한 프로세스다.
같은 Python에서 Gazebo·NumPy·pymavlink가 필요하다.
서로 다른 Python의 import 성공을 합산하지 않는다.

```bash
~/.venvs/px4/bin/python -u -c "from gz.transport13 import Node; from gz.msgs10.clock_pb2 import Clock; import numpy; from pymavlink import mavutil; from drone_demo.sitl_navigation import SITLNavigationSession; print('Navigation runtime imports OK')"
~/.venvs/px4/bin/python -m drone_demo.sitl_navigation --help
```

현재 확인한 것은 가상환경의 pymavlink import다.
시스템 Python의 Gazebo·NumPy import도 따로 확인했다.
위 결합 확인은 사용자 WSL에서 아직 미실시다.
실패하면 누락된 모듈을 확인한 뒤 환경만 보완한다.

## 비행 명령 없이 상태 확인

PX4·Gazebo와 가상 RAW 발행기가 먼저 실행돼야 한다.
별도 link probe나 관측 CLI는 동시에 실행하지 않는다.
이유: 같은 UDP 14540 수신을 서로 가로챌 수 있다.

```bash
~/.venvs/px4/bin/python -m drone_demo.sitl_navigation \
  --navigation-settings src/drone_uwb/config/sitl/gazebo_sitl_target.json \
  --navigation-plan src/drone_demo/config/gazebo_navigation_plan.json \
  --mission-config src/drone_demo/config/mission.json \
  --navigation-mode monitor \
  --config runs/equipment_02/trial.json \
  --height-profile src/drone_uwb/config/gazebo/gazebo_sensor_height_profile.json \
  --sitl-odometry src/drone_uwb/config/sitl/gazebo_sitl_odometry.json \
  --sitl-observer src/drone_uwb/config/sitl/gazebo_sitl_observer.json \
  --sitl-mode timesync \
  --duration-s 30 \
  --output runs/navigation_monitor_01
```

이 명령은 비행 명령과 관측 ODOMETRY를 보내지 않는다.
시계 조회·응답, 파라미터 조회는 보낸다.
FC 상태 메시지의 송신 간격도 요청한다.
따라서 완전한 수동 수신만 하는 명령은 아니다.
그 결과와 원본 설정은 출력 폴더에 기록된다.

| 요청 메시지 | 요청 빈도 |
|---|---:|
| ODOMETRY | 30Hz |
| ESTIMATOR_STATUS | 10Hz |
| POSITION_TARGET_GLOBAL_INT | 10Hz |
| EXTENDED_SYS_STATE | 2Hz |
| PX4 시각 조회 | 20Hz |

상태 요청이 수락됐는지는 실제 수신으로 확인한다.
요청을 보낸 사실만으로 해당 자료가 있다고 처리하지 않는다.
관측을 송신하지 않은 이 단계의 임무 상태는 대기 상태다.

## 목표 시험으로 전환하는 조건

관측 송신과 융합을 먼저 검증한다.
[관측 실행 절차](uwb_gazebo_sitl_observer.md)를 따른다.
실시간 ToF·자세·장착 위치와 좌표 정렬을 확인한다.
EKF 사용 상태·innovation·기각·reset도 기록한다.
UWB 외의 위치원 사용 여부를 함께 적는다.

| 항목 | 시험 전에 확인할 내용 |
|---|---|
| 현재 상태 | 시동·공중·Hold, 유효한 FC 수평 위치·속도 |
| 관측 조건 | 실제 관측 송신과 융합 근거 |
| 지도 정렬 | 관측과 목표에 같은 변환 사용 |
| 목표 | PX4 기준점의 지도 좌표. 기체 중심과 구분 |
| 실행 설정 | 기본 설정을 시행 폴더에 복사하고 확인값 반영 |
| GCS 단절 | 요구 파라미터와 다른 GCS 존재 여부 |
| 기록 | 새 출력 폴더, PX4 ULog 경로, 별도 정답 기록 |

목표·관측 확인 플래그를 일괄 true로 바꾸지 않는다.
이유: 미확인 정렬이나 시각을 유효 관측으로 보낼 수 있다.
각 플래그의 근거를 시행 기록에 연결한다.
실제 기체 파라미터는 이 절차에서 변경하지 않는다.

초기 목표는 지도 `(2.59, 1.68)m`이다.
이는 설정 예시이며 현재 FC 위치와 대조해야 한다.
실행 시작 때 FC 위치를 출발점으로 기록한다.
기본 경로는 목표 한 곳 → 출발점 → 착륙이다.
목표마다 최대 1m, 0.3m/s, 20초 제한이다.
고도 유지·착륙은 PX4가 수행한다.
시동·이륙은 이 실행기가 수행하지 않는다.

준비 완료 후 같은 실행기의 두 모드를 바꾼다.
`--sitl-mode send`는 관측 송신을 요청한다.
`--navigation-mode send`는 목표·중단·착륙 요청을 허용한다.
관련 설정 파일 경로도 검증한 시행 복사본으로 바꾼다.
새 출력 폴더를 사용하고 ULog를 함께 보존한다.
송신 모드의 GCS heartbeat는 1Hz다.
상세 조건·중단 정책은 [연결 구조](../architecture/uwb_gazebo_target_adapter.md)를 따른다.

## 결과 확인

`navigation_events.jsonl`에서 세 가지를 구분한다.

1. COMMAND_ACK가 명령을 수락했는가.
2. FC 목표 메시지가 새 목표와 일치했는가.
3. FC 위치·속도·유지시간이 도착 조건을 충족했는가.

`navigation_summary.json`은 위 판정의 요약이다.
착륙은 접지·disarmed까지 확인해야 기록된다.
실제 궤적·정답 오차·융합의 최종 증명은 별도다.
후속 [독립 평가기](../architecture/uwb_gazebo_flight_evaluation.md)를 구현했다.
새 기록의 시각·기준점을 맞춰 네 오차를 계산한다.
기본 runtime ZIP 이후의 기록 필드 확장이 필요하다.
목표·관측·Gazebo 정답·ULog를 같은 시행으로 묶는다.
이번 로컬 시험에는 실제 비행 궤적이 없다.

## 제작 시 확인 결과

| 확인 | 2026-10-02 결과 |
|---|---|
| 관련 시험 | 142개 통과 |
| 시험 범위 | 시계·관측·명령·도착·순회·복귀·중단·기록 종료 |
| 통신 | 실제 pymavlink 형식, 모의 FC, 기존 UDP loopback 포함 |
| 빌드 | 5개 패키지 성공. 9.77초 |
| 설치 환경 import | 성공 |
| 사용자 WSL 실행 | 미실시 |
| 비행·실제 오차·EKF 융합 | 미검증 |

테스트는 실제 PX4나 사용자 WSL로 명령을 보내지 않았다.
현재 ZIP은 검증된 실기 배포본이 아니다.

초기 `uwb_gazebo_navigation_runtime_20261002.zip`은 보존한다.
이 초기 ZIP 크기는 86,218바이트다.
SHA-256은 아래와 같다.
`1e9e66ab1f67f8b25ecde3008e169821c55492780bbdb83828d8bce9d44fb806`
27일 기본 ZIP에 겹쳐 31개 파일 해시를 대조했다.
임시 폴더에서 모듈 import·CLI 도움말이 통과했다.
관측·목표 설정의 enabled=false도 확인했다.
이 제작 결과는 ZIP 작성 뒤 문서에 추가했다.
ZIP 제작 당시 문서는 이 단락 추가 전이었다.
당시 코드 파일의 일치는 확인했다.

이후 독립 평가용 기록 필드를 추가했다.
PX4 위치 표본 시각과 목표 변경의 Gazebo 시각을 남긴다.
위 ZIP은 그 추가 전 버전으로 보존했다.
후속 갱신본은 아래 별도 파일이다.
`uwb_gazebo_navigation_evaluation_v2_20261002.zip`
독립 평가 코드·설정과 새 기록 필드를 함께 포함한다.
이 파일도 당시 배포 이력으로 보존한다.

이후 `uwb_gazebo_fault_trials_20261002.zip`을 만들었다.
위 기능에 선택적 편향·단절 주입을 추가한 누적본이다.
같은 기본 ZIP 위에 현재 갱신본 하나를 반영하면 된다.
SHA-256은 함께 배포하는 `.zip.sha256`으로 확인한다.
실제 WSL 반영과 비행 검증은 별도다.
기본 ZIP과 분리 폴더에서 조합한 시험 54개가 통과했다.
CLI 도움말 3개와 모듈 로딩도 확인했다.

현재 적용 명령은 `uwb_gazebo_fault_evaluation_v2_20261002.zip`을 쓴다.
구간별 CSV·관측 복구 시간·주입 기록 대조를 추가한 누적본이다.
초기 주입 갱신본도 당시 기록으로 보존한다.
구간 평가 관련 시험 44개와 5개 패키지 빌드를 통과했다.
별도 폴더에 적용한 관련 시험 68개도 통과했다.
기본 ZIP 위에 현재 갱신본 하나만 반영한다.
