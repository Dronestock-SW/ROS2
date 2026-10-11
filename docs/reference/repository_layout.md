# 단계별 코드·자료 위치표

저장소의 실제 구현과 관련 자료를 찾는 사전이다. 수정할 단계와 검증 범위를 정할 때 읽는다.

## 저장소 구조

실제 구현은 역할별 폴더에 둔다.
옛 이름의 파일은 호환 경로로 남는다.

```text
ROS2/
|-- src/
|   |-- drone_uwb/          UWB 수신·후보정·관측 연결
|   |-- drone_demo/         데모·임무·독립 평가
|   |-- drone_bringup/      센서 실행·인쇄 마커
|   |-- drone_platform_link/ 플랫폼 수신·상태 전달
|   |-- drone_mission/      로컬 웹·PX4 전체 비행 순서
|   `-- ydlidar_ros2_driver/ 제조사 서브모듈
|-- config/udev/            장치 이름 규칙
|-- docs/
|   |-- roadmap.md, altitude_policy.md
|   |-- glossary.md, equipment_inventory.md
|   |-- architecture/      설계·책임 경계
|   |-- reference/         API·배치·경로 사전
|   |-- runbooks/          실행·재생·점검 절차
|   |-- specs/uwb/         기능별 명세·비교 후보
|   |-- hw_handoff/        HW·외부 API 인계
|   `-- report/            날짜별 결과·증빙
|-- data/
|   |-- raw/               원본·수신 조건
|   |-- processed/         계산·평가 결과
|   |-- catalog.json       원본 해시·자료 경로
|   `-- module_paths.json  이전 import -> 실제 구현
`-- build/, install/, log/ 자동 생성물
```

자동 생성물은 빌드 도구가 관리한다.
직접 편집하면 다음 빌드에서 덮어쓰므로 수정하지 않는다.
제조사 서브모듈도 이번 정리에서 변경하지 않았다.

## HW 송신부터 출력까지

A~H는 책임 위치를 가리킨다.
모든 단계가 한 실행기에 연결됐다는 뜻은 아니다.

2026-10-08 실물 웹 시험은 `drone_mission`에서 실행한다.
`/uwb/btf_pose`·FC 브리지·HTTP 미션을 함께 연결한다.
[비행 절차](../runbooks/web_test_flight.md)를 따른다.
다음 세션은 [로컬·Jetson 인계](../runbooks/local_jetson_handoff_20261008.md)부터 읽는다.
읽기 전용 수집기는 `src/drone_mission/tools/observe_ground.py`다.
어제의 C++ AI·Tag B 통합 코드는 별도 원격 브랜치에 있다.

```text
HW 앵커 A1~A4 <-> ESP32 RAW JSONL
                          |
                    [A] acquisition/
                          | 검사·원본 보존
                    [B] processing/timing/
                          | 시각·센서 선택
       [C] config/ + processing/settings.py + geometry/
                          |
          +---------------+----------------+
          |                                |
 [D] geometry/height.py            [E] ranges.py
     ToF·자세·레버암                    bias·거리 게이트
          |                                |
          +----------> solvers/rawxy.py <---+
                          |
             [F] solvers/h80.py
                          |
             [G] solvers/qs10.py (조건부 복귀 명세)
                          |
 [H] 호출자의 품질 판정 + geometry/transform.py
                          |
           integration/ros/ 또는 integration/sitl/
                          |
               기록·재생·독립 평가
```

실시간·파일·SITL 경로의 활성 상태는 아래 표를 따른다.
폴더 정리로 계산·전송 게이트를 열지 않았다.

## 단계별 수정 위치

코드 열의 경로는 `src/drone_uwb/drone_uwb/` 기준이다.
설정과 시험은 `src/drone_uwb/` 기준이다.
현재 앵커는 [6.3×4.6m 직사각형](uwb_anchor_layout.md)이다.
이전 날짜 좌표는 해당 실측 기록의 재생용이다.

| 단계 | 실제 코드 | 설정·시험 | 기준·현재 범위 |
|---|---|---|---|
| A 입력 | `contracts/protocol.py`, `acquisition/`, `integration/recording.py` | `config/runtime/`, `test/processing/test_observations.py`, `test/test_module_layout.py` | [입력 명세](../specs/uwb/01_input_time_gate.md). RAW와 판정 기록 분리 |
| B 시간 | `processing/timing/clock.py`, `sensors.py` | `processing/settings.py`, `test/processing/test_preimu_pipeline.py` | [공통 계약](../specs/uwb/00_interfaces.md). source와 수신 시각 구분 |
| C 설정·좌표 | `processing/settings.py`, `geometry/transform.py`, `geometry/gazebo_geometry.py` | `config/anchors/`, `test/processing/test_preimu_settings.py` | [API](uwb_module_api.md). 좌표·시간·단위 검사 |
| D Z 준비 | `processing/geometry/height.py`, `processing/experiments/gazebo_height.py` | `config/gazebo/gazebo_sensor_height_profile.json`, `test/integration/test_gazebo_sensor_height.py` | [Z 명세](../specs/uwb/02_tof_z_adapter.md). 독립 수식·시험 입력, 고도 제어 없음 |
| E 거리·raw XY | `processing/ranges.py`, `solvers/rawxy.py`, `solvers/observations.py` | `config/pipeline/`, `test/processing/test_preimu_pipeline.py` | [거리 게이트](../specs/uwb/03_range_correction_gate.md). 현 실시간 Processor는 별도 경로 |
| F H80 | `processing/solvers/h80.py`, `processing/experiments/h80_b.py` | `config/replay/h80_b_static_20260906.json`, `test/experiments/test_h80_b.py` | [H80](../specs/uwb/04_h80.md). 독립 풀이·A/B 파일 비교 구현 |
| F 위치 게이트 | `processing/settings.py`의 후보 설정, `pipeline.py`의 blocked 상태 | `test/processing/test_preimu_pipeline.py` | [위치 게이트](../specs/uwb/14_position_gate.md). 공용 파이프라인은 계산 차단 상태 |
| G Q_S10 | `processing/solvers/qs10.py` | `processing/settings.py`, `test/processing/test_preimu_settings.py` | [복귀 명세](../specs/uwb/05_q_s10.md). 독립 수식 존재, 복귀 상태기계 연결 완료로 표시하지 않음 |
| H 관측 출력 | `processing/solvers/observations.py`, `integration/ros/`, `integration/sitl/` | `config/runtime/`, `config/sitl/`, `test/integration/` | [출력 경계](../specs/uwb/17_output_boundary.md). 진단 유효성과 비행 전달 조건 구분 |
| 모델 비교 | `processing/solvers/`, `processing/experiments/` | `config/replay/`, `config/gazebo/`, `test/experiments/` | [모델 선택](../specs/uwb/stage_selection.md). 비교 모델을 기본 모델로 자동 채택하지 않음 |

작은 거리·설정·파이프라인 모듈은 단일 파일로 유지했다.
별도 품질 폴더를 만들기 위해 미구현 게이트를 추가하지 않았다.
함수별 입력·출력은 [API 사전](uwb_module_api.md)에 있다.

## 실행 경로

각 실행기의 기존 진입점과 출력 조건을 유지한다.

| 용도 | 진입점·호출 흐름 | 출력·활성 조건 |
|---|---|---|
| 실제 UART 관측 | `uwb_node` → `integration/ros/node.py` → `processing/solvers/observations.py` | `/uwb_pose` 수평 관측. 수신 설정·기존 유효성 검사 적용 |
| 기존 PX4 브리지 | `uwb_px4_bridge` → `integration/ros/bridge.py` | `mavros_uwb.yaml`의 기존 기본 전달 비활성 유지 |
| 파일 파이프라인 | `uwb_pipeline` → `processing/runner.py` → `pipeline.py` | 원본·진단 기록. 계산·외부 출력 게이트 닫힘 |
| 파일 모델 비교 | `uwb_static_a`, `uwb_h80_b`, `uwb_subset_compare`, `uwb_baseline_a` | 별도 파일 결과. 명령 이름 유지 |
| 공용 XY 데모 | `drone_demo/core.py`, `node.py`, `tof_readout.py` | XY 경로·공백과 실측 ToF 하방거리 표시. 가상 고도·RAW 생성 없음 |
| Gazebo/SITL | `integration/gazebo/gazebo_live_shadow.py` + `integration/sitl/` | 기존 명시적 CLI 옵션·연결 조건 유지. 실제 실행은 이번 검증에서 미실시 |
| 독립 평가 | `drone_demo/flight_evaluation.py` | 기록 종료 후 정답 대조. 정답을 관측 계산에 넣지 않음 |

[파일 재생](../runbooks/uwb_data_runbook.md),
[SITL 관측](../runbooks/uwb_gazebo_sitl_observer.md),
[목표 연결](../architecture/uwb_gazebo_target_adapter.md)을 읽는다.

## 상태와 의존성

상태는 기존 객체가 계속 소유한다.

| 소유 모듈 | 입력·출력과 상태 | 초기화·실패 |
|---|---|---|
| `acquisition/validation.py` | schema·seq·mask 검사, `Cycle` | boot·disconnect 및 기존 입력 오류 계약 |
| `timing/clock.py` | ESP32 us → host 단조시계 s, 대응 창 | `reset_temporal()`에서 새 ClockMap 생성 |
| `timing/sensors.py` | host 시각의 ToF·자세 버퍼와 age | 미래·오래된 표본 구분, 시계 불일치 거부 |
| `ranges.py` | m 단위 거리·pending·승인 이력 | 3표본 재획득, 공백·재부팅의 기존 이력 처리 |
| `geometry/height.py` | 거리 m·R_WB·레버암 → FC 높이 | 독립 수식. AB 상태는 호출자가 전달 |
| `solvers/h80.py`, `qs10.py` | 창의 거리·시각 → 위치 계수·잔차·성공 여부 | 독립 풀이. 각 모델 실행기가 이력 소유 |
| `pipeline.py` | event → 진단·reason·차단 상태 | boot·출처 변경 시 시간 상태 초기화 |
| `integration/` | ROS·MAVLink·Gazebo 형식 연결 | 기존 계약·연결·시각 게이트 유지 |

`contracts`는 상위 계층을 import하지 않는다.
`acquisition`은 계산부를 import하지 않는다.
`processing`은 ROS와 `integration`을 import하지 않는다.
장치·계산을 독립 검증하기 위한 경계다.
[경계 시험](../../src/drone_uwb/test/test_module_layout.py)이 이를 확인한다.

## 인계서와 현재 기준의 차이

9월 20일 인계서는 알고리즘 참조다.
기술 책임과 활성 상태는 현재 로드맵·코드를 따른다.

| 항목 | 인계서 v1.2 | 현재 저장소에서 유지한 범위 |
|---|---|---|
| 최종 위치 | warehouse의 FC 원점 XYZ 목표 | 실시간 `/uwb_pose`는 수평 관측. 파일 계산 좌표·FC 변환은 API별로 구분 |
| Z | ToF·자세·레버암과 Z 필터 연결 | 독립 수식·가상 센서 시험. 공용 파일 파이프라인의 Z 계산은 차단 |
| IMU/EKF | 기준선 후 별도 Shadow 추가 구상 | 비행용 추정기는 PX4 EKF2 단일. companion 추정기 추가 없음 |
| H80/Q_S10 | 정상·복귀 경로 통합 목표 | H80 비교 실행·Q 독립 수식과 통합 상태를 구분 |
| 위치 게이트·hold | 전 구간 valid 보호 목표 | 명세·후보 설정과 현재 실행 경로를 구분. 미구현 연결을 정리 작업에 추가하지 않음 |
| 성능 | 정지 실측 P95 7cm 목표 달성 | 원본 기록 보존. 구조 변경 검증은 수치 동일성 확인 |

## 설정·자료·호환 경로

설정은 용도별로 찾고 원본 바이트를 유지한다.

| 위치 | 용도 |
|---|---|
| `config/anchors/` | 앵커 좌표 |
| `config/runtime/` | 실제 UART·ROS·브리지 |
| `config/pipeline/` | 닫힌 파일 파이프라인 |
| `config/replay/` | 정지 교정·비교·비행 입력 분석 |
| `config/gazebo/` 및 `scenarios/` | 가상 장비·센서·경로·이상 주입 |
| `config/sitl/` | PX4 관측·목표·시계 연결 |

이 표의 `config/`는 `src/drone_uwb/config/`다.
옛 설정 파일명은 상대 링크다.
설치 때 새 경로와 옛 파일명을 모두 포함한다.
옛 모듈명은 [경로표](../../data/module_paths.json)에서 찾는다.
호환 모듈은 실제 구현을 직접 참조한다.
같은 객체·예외·monkeypatch 대상을 유지한다.

원본 ULog·JSONL과 기존 가공 결과는 기존 분류를 유지했다.
`perfect_holdv2.params`는 날짜별 비행 원본 폴더로 옮겼다.
루트의 기존 이름은 상대 링크로 남겼다.
인쇄 마커는 `drone_bringup/assets/markers/`에 있다.
마커 1의 사용 여부는 미확인 상태로 표시했다.

저장소 밖의 PDF·ZIP·실행 폴더는 이동하지 않았다.

| 외부 경로 | 역할 |
|---|---|
| `/home/pgyxn/0920_2_uwb_jetson_codex_handoff_v1_2.pdf` | 이번 구조와 대조한 HW 인계서 |
| `/home/pgyxn/korean-thesis-audit-1.2.0.zip` | 사용자 지정 한국어 문체·근거 보존 지침 |
| `/home/pgyxn/uwb_pipeline_runs/` | 기존 전체 실행 결과. 정리 범위 밖 |

그 밖의 참고 ZIP·원본은 [기존 참조 기록](uwb_h80_qs10_reference.md)을 따른다.
전체 이동표·검증·남은 사항은 [작업 기록](../report/repository_modularization_20261004.md)에 있다.
