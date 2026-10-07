# Gazebo UWB 관측의 SITL 실행 절차

이 문서는 실시간 UWB 계산과 MAVLink 연결 절차다.
Shadow 기록 뒤 시계 응답·관측 송신을 시험할 때 읽는다.

## 현재 상태

**실시간 계산과 관측 송신 경로를 연결했다.**
기본 실행은 Shadow다. 확인 설정은 모두 닫혀 있다.
실제 WSL의 UDP 수신·관측 융합·비행은 미실시다.

2026-10-02 사용자 출력에서 다음을 확인했다.

| 항목 | 확인 결과 |
|---|---|
| `pgrep -a -x px4` | 무출력. 해당 조회에서 PX4 프로세스 미발견 |
| `timeout 10s gz topic -l` | 무출력. 조회 가능한 토픽 미발견. 종료 코드는 미수집 |
| 가상환경의 `pymavlink` import | 정상. 종료 코드 0 |
| 모듈 경로 | `/home/dronestock/.venvs/px4/lib/python3.10/site-packages/pymavlink/mavutil.py` |
| 동일 가상환경의 Gazebo·NumPy import | 미확인. 이전 성공은 `/usr/bin/python3` 기준 |

이번 사용자 확인으로 재설치 단계는 필요하지 않다.
기존 UWB 모델의 [재실행 명령](uwb_gazebo_equipment_runbook.md)을 먼저 실행한다.

## 실행 모드

기존 `gazebo_live_shadow` CLI에 SITL 연결 옵션을 추가했다.
계산기는 모든 모델을 계속 비교한다.
송신 모델은 시행 시작 시 하나만 선택한다.
계산 실패 시 다른 모델로 자동 전환하지 않는다.

| `--sitl-mode` | PX4 연결 | 관측 송신 |
|---|---|---|
| `shadow` 또는 옵션 생략 | 없음 | 없음. 변환 후보·판정만 기록 |
| `timesync` | 파라미터 읽기·시각 조회 응답 | 없음. PX4 시계 필터는 갱신됨 |
| `send` | 위 기능과 기준점 초기화 감시 | 확인 조건을 통과한 새 표본만 송신 |

`timesync`는 읽기 전용 점검과 다르다.
PX4가 시작한 시각 조회에 응답하기 때문이다.
이 관측 CLI를 단독 실행하면 비행 명령은 없다.
파라미터 쓰기도 없다.
별도 [목표 이동 실행기](uwb_gazebo_navigation_runbook.md)는
같은 연결에 목표·중단·착륙 요청을 추가한다.
그 실행기는 별도의 명시적 송신 모드를 사용한다.

```text
가상 UWB RAW ─→ 편향 차감 ─→ A/B/C/D/WLS 비교
ToF + ToF 시각의 자세 ────→ 안테나 높이
RAW 시각의 자세 ──────────→ 안테나↔PX4 기준점 변환
                                       │
                          선택 모델 + 선언한 XY 불확실성
                                       ↓
                           NED 좌표·공분산 변환
                                       ↓
Gazebo /clock ─→ 시각 응답 ─→ 신선도·설정·초기화 검사
PX4 heartbeat·설정·ODOMETRY ────────────┘
                                       ↓
                          새 관측 한 번 송신 + 시행 기록
```

## 설정과 실행 준비

설정 파일 네 종류를 구분한다.

| 파일 | 역할 |
|---|---|
| `runs/equipment_02/trial.json` | 현재 월드의 앵커·거리 편향·계산 설정 |
| `gazebo_sensor_height_profile.json` | ToF·자세·장착 위치와 바닥면 조건 |
| `gazebo_sitl_odometry.json` | 지도↔PX4 정렬·기준점·시각 확인 |
| `gazebo_sitl_observer.json` | 시행별 선택 모델·불확실성·대기 조건 |

높이 계산과 관측 변환의 안테나 장착값은 같아야 한다.
CLI는 두 설정의 불일치를 실행 전에 거부한다.
기본 선택 모델은 A다. B 등은 별도 시행에서 선택한다.
기존 최선 후보의 시간창도 해당 시행의 `trial.json`에 반영한다.
이 변경은 B의 기본 시간창을 바꾸지 않았다.

기본 XY 표준편차는 각 축 0.10m다.
이는 교정되지 않은 SITL 시험 설정이다.
A/B/C/D 잔차에서 계산한 공분산이 아니다.
`uncertainty_source`에 출처를 남긴다.
`uncertainty_reviewed`는 이 선언값의 시험 적용 검토다.
그 값이 `true`여도 실제 오차 교정을 뜻하지 않는다.

## WSL에서 Shadow 실행

[현재 누적본 반영 절차](uwb_gazebo_navigation_runbook.md#갱신본-반영)를 먼저 따른다.
초기 관측 ZIP은 당시 배포 기록으로 보존한다.
현재 누적본 위에 과거 파일을 다시 덮어쓰지 않는다.
이유: 관측·목표·평가 모듈의 버전이 섞일 수 있다.
아래 명령은 누적본의 프로젝트 루트에서 사용한다.
기존 PX4·Gazebo와 가상 RAW 발행기를 먼저 실행한다.
출력 폴더 이름은 시행마다 새로 정한다.

```bash
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m drone_uwb.integration.gazebo.gazebo_live_shadow \
  --config runs/equipment_02/trial.json \
  --height-profile src/drone_uwb/config/gazebo/gazebo_sensor_height_profile.json \
  --sitl-odometry src/drone_uwb/config/sitl/gazebo_sitl_odometry.json \
  --sitl-observer src/drone_uwb/config/sitl/gazebo_sitl_observer.json \
  --sitl-mode shadow \
  --output runs/observer_shadow_20261002_01 --duration-s 60
```

확인 설정이 닫혀 있으면 후보가 없을 수 있다.
`results.jsonl`의 높이·자세 판정 사유부터 읽는다.
Shadow 후보가 계산돼도 설정 확인 완료로 판정하지 않는다.
`sitl_events.jsonl`에 확인 플래그를 함께 남긴다.

`timesync`와 `send`는 같은 Python에 두 라이브러리가 필요하다.
Gazebo·NumPy와 `pymavlink`를 함께 가져올 수 있어야 한다.
확인 후 위 명령의 Python과 모드만 바꾼다.
현재 사용자 출력으로는 이 조합까지 확인되지 않았다.

연결 주소는 `udpin:127.0.0.1:14540`으로 제한한다.
다른 점검기와 같은 포트를 동시에 열지 않는다.
이는 같은 PX4에 중복 수신·송신 경로가 생기는 것을 막는다.

## 송신 검사

[좌표·시각 계약](../reference/uwb_gazebo_sitl_odometry.md)을 그대로 사용한다.
런타임은 다음 조건도 확인한다.

| 조건 | 현재 처리 |
|---|---|
| ToF·자세 | 유효한 센서 높이와 RAW 시각의 자세 필요 |
| RAW 시각의 자세 | 미래 표본 제외. 프로파일의 허용 시차 이내 |
| B 출력 | 현재 주기에 새 거리 관측이 있어야 함 |
| WSL 대기 시간 | RAW 콜백 시작부터 최대 150ms |
| Gazebo 표본 나이 | 현재 Gazebo 시각과 0~150ms 차이 |
| PX4 heartbeat | 최근 3초 이내 PX4 `1/1` |
| 파라미터 | 11개 모두 최근 5초 이내. 2초마다 읽기 요청 |
| PX4 기준점 감시 | 최근 0.5초 이내 전진한 AUTOPILOT ODOMETRY |
| 시각 응답 | 기본 10회 이상, 마지막 응답은 2초 이내 |
| MAVLink 수신 대기열 | 기존 메시지를 읽은 뒤 송신. 초기화 메시지 추월 방지 |
| 재전송 | 같은 UWB 시각·순번은 재전송하지 않음 |

시각 응답 10회는 초기 대기 조건이다.
PX4 시계 동기화 수렴의 증거가 아니다.
`time_mapping_confirmed`에는 실제 시각 대조가 필요하다.
`vehicle_visual_odometry.timestamp_sample`도 별도로 확인한다.

PX4 원점·방향 초기화는 수신 ODOMETRY의
`reset_counter` 변화로 감시한다.
[대상 버전 송신 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/mavlink/streams/ODOMETRY.hpp)는
PX4 기준 시각·초기화 카운터·AUTOPILOT 출처를 보낸다.
[온보드 기본 스트림](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/mavlink/mavlink_main.cpp)은
ODOMETRY 30Hz와 TIMESYNC 10Hz를 설정한다.
실제 수신률은 사용자 WSL에서 별도 확인한다.

시각 역행·기준점 초기화·송신 실패는 해당 시행을 중단한다.
센서 순서 오류·콜백 오류·큐 넘침도 중단한다.
정렬·시각을 다시 확인하고 새 시행으로 시작한다.
정지한 Gazebo 시계만으로는 예전 위치를 재발행하지 않는다.

## 산출물과 검증 범위

입력·설정·소스 해시를 시행 폴더에 남긴다.

| 산출물 | 내용 |
|---|---|
| `manifest.json` | 실행 인자·Python·설정/소스 해시·토픽 |
| `raw_ranges.jsonl`, `tof.jsonl`, `attitude.jsonl` | 원본 센서와 WSL 콜백 시각 |
| `clock.jsonl` | 같은 월드의 시뮬레이션 시계 |
| `results.jsonl` | 편향 차감·높이·자세 선택·모델별 계산 |
| `sitl_events.jsonl` | PX4 상태 원문·설정 조회·시각 응답·관측 후보·차단 사유·송신 필드 |
| `failures.jsonl`, `terminal_error.json` | 표본 거부와 실행 오류. 후자는 오류 시 생성 |
| `capture.json` | 실제 송신 수·종료 사유 |
| `input_index.json` | 종료 후 산출물 크기와 SHA-256 |

MAVLink의 미관측 방향·속도는 NaN이다.
JSON 로그에서는 이를 `null`로 기록한다.
로그의 `sent_to_transport`는 전송 함수 호출 성공이다.
PX4 수신·EKF 융합 성공은 아니다.
오차 지표·비행 결과는 독립 평가 시행에서 추가한다.

### PX4 상태 기록 보완

2026-10-02 후속 변경은 PX4 상태 메시지의 필드를 보존한다.
이전 연결기는 ODOMETRY의 시각·초기화 카운터만 남겼다.
그 기록만으로는 실제 위치·속도·목표를 비교할 수 없었다.
현재는 아래 메시지의 `payload`를 함께 보관한다.

| 수신 메시지 | 평가에 필요한 필드 |
|---|---|
| HEARTBEAT | 시동 비트·모드·시스템 상태 |
| ODOMETRY | 위치·속도·자세·공분산·두 좌표계·초기화 카운터 |
| LOCAL_POSITION_NED | 위치·속도·부트 시각 |
| POSITION_TARGET_LOCAL_NED | FC 목표 위치·속도·축 무시 비트·좌표계 |
| ESTIMATOR_STATUS | 추정 상태 플래그·innovation 비율 |
| ESTIMATOR_SENSOR_FUSION_STATUS | 해당 dialect에서 해석 가능한 융합 상태 필드 |
| EXTENDED_SYS_STATE | 착륙·기체 전환 상태 |
| GPS_GLOBAL_ORIGIN | 목표 변환에 사용할 PX4 원점·기준 시각 |
| POSITION_TARGET_GLOBAL_INT | PX4가 반영한 전역 목표·좌표계·시각 |
| COMMAND_ACK | 명령 ID·결과·응답 대상 |

지원되지 않거나 수신되지 않은 메시지를 생성하지 않는다.
일반 ESTIMATOR_STATUS만으로 UWB 융합을 단정하지 않는다.
세부 관측 사용·기각 여부는 ULog/uORB와 대조한다.
이 보완은 스트림 설정·기체 설정을 변경하지 않는다.

원래 좌표계와 PX4 시각을 그대로 남긴다.
NED·body 속도를 UWB 지도 속도로 바꿨다고 표시하지 않는다.
이를 UWB 계산기로 되먹이지 않는다.
목표 이동 전 필요한 추가 연결은
[구조 설명](../architecture/uwb_gazebo_architecture.md)의 목표 명령 검토를 따른다.

2026-10-02 최초 관측 연결의 로컬 확인 결과다.
아래 전체 시험 수는 상태 원문 보존 변경 전 기록이다.

| 확인 | 결과와 범위 |
|---|---|
| `drone_uwb` 전체 시험 | 208개 통과. 선택 의존성 모듈 1개 건너뜀 |
| 실제 `pymavlink 2.4.50` 시험 | 3개 통과. 패킷 왕복·시각 응답·UDP loopback |
| UDP 시험의 PX4 역할 | 가짜 응답기. 실제 PX4·EKF 시험이 아님 |
| CLI 연결 시험 | 가짜 Gazebo 메시지로 입력→계산→후보·해시 기록 확인 |
| 빌드 | `colcon build --symlink-install`, 5개 패키지 성공 |
| 설치 환경 | `source install/setup.bash` 후 새 모듈 import 성공 |

원본 거리에서 실제 A/B 계산을 거쳐 패킷을 만드는 경로를 시험했다.
중복·지연·센서 실패·기준점 초기화·시각 역행도 확인했다.
UDP 시험은 OS가 배정한 loopback 포트를 사용했다.
실행 중인 SITL의 `14540` 포트에는 접근하지 않았다.
메시지에는 PARAM_REQUEST_READ·TIMESYNC·ODOMETRY만 있었다.

상태 원문 보존 변경 후에는 관련 시험을 다시 실행했다.

| 확인 | 결과와 범위 |
|---|---|
| 관측 연결기·CLI 시험 | 27개 통과. 출처·좌표계·필드 보존 확인 |
| 실제 `pymavlink 2.4.50` 시험 | 3개 통과. UDP 수신 원문의 필드 보존 포함 |
| 빌드 | 5개 패키지 성공. 8.34초 |
| 설치 환경 | 설치 설정 적용 후 `px4_telemetry_record` import 성공 |
| 전체 시험 재실행 | 미실시. 이번 변경 관련 시험만 재실행 |

처음 시험 호출은 모듈 검색 경로 누락으로 수집에 실패했다.
`PYTHONPATH=src/drone_uwb`를 적용한 재실행에서 통과했다.
상태 원문 보존 변경은 현재 원격 소스에만 반영했다.
앞서 만든 관측 연결 ZIP은 덮어쓰지 않았다.
따라서 해당 ZIP에는 이 후속 변경이 포함되지 않는다.

사용자 WSL의 라이브 연결·EKF 융합·시험비행은 미실시다.
7cm 목표와 목표 좌표 이동의 완료 판정도 남아 있다.

같은 날 목표 이동 실행기를 위한 세션 연결을 추가했다.
수신 메시지는 관측 처리 뒤 세션에 한 번 전달한다.
목표 실행기는 두 번째 UDP 소켓을 열지 않는다.
종료 처리 실패도 기록하고 구독·연결 해제를 계속한다.
통합 관련 시험 142개가 통과했다.
원격 코드·로컬 모의 통신 시험의 결과다.
