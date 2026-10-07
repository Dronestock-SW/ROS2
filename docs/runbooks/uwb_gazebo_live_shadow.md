# Gazebo UWB 실시간 Shadow 기록

이 문서는 Gazebo 가상 UWB·ToF·IMU를 한 세션에서 기록하는 절차다.
PX4 융합 전에 A/B/C/D의 실시간 계산 경로를 확인할 때 읽는다.

## 현재 범위

Gazebo 토픽의 가상 RAW와 별도 센서 표본을 받는다.
수신한 RAW에서 거리 편향을 차감하고 A/B/C/D/WLS를 계산한다.
결과에는 높이 출처·입력 시각·모델별 실패 사유를 남긴다.
정답 위치를 계산 입력으로 읽지 않는다.
기본 실행에는 PX4·MAVLink 송신이 없다.
2026-10-02에 선택형 [SITL 연결 옵션](uwb_gazebo_sitl_observer.md)을 추가했다.
이 CLI를 단독으로 쓰면 목표 이동 명령은 없다.
별도 [목표 실행기](uwb_gazebo_navigation_runbook.md)는 구현했다.

```text
Gazebo 가상 UWB 토픽 ─┐
Gazebo 하방 거리 토픽 ├─ 시각·유효성 검사 → RAW-편향 → A/B/C/D/WLS
Gazebo IMU 토픽 ──────┘                    │
                                        └→ 결과·거부 사유 파일
Gazebo 정답 위치 ─────────────────────────→ 별도 평가기만 사용
```

높이 프로파일의 방향 정렬과 바닥면 확인 게이트는 기본값 `false`다.
게이트가 닫혀 있으면 높이가 필요한 A/C/D/WLS는 이유와 함께 출력을 거부한다.
B H80의 수평 진단은 별도로 계산하되 PX4 관측으로 송신하지 않는다.
센서 장착·방향·높이 계산을 검증한 뒤 프로파일을 명시적으로 바꾼다.

## WSL 실행 순서

[현재 누적본 반영 절차](uwb_gazebo_navigation_runbook.md#갱신본-반영)를 먼저 따른다.
기존 `runs/equipment_02/trial.json`을 새 폴더에 보존한다.
과거 진단 ZIP은 이력으로 유지한다.
새 실행 폴더에 과거 갱신본을 다시 덮어쓰지 않는다.
이유: 같은 모듈이 과거 버전으로 바뀔 수 있다.
아래 명령은 누적본의 프로젝트 루트에서 실행한다.

첫 터미널에서 PX4·Gazebo가 이미 실행 중인지 확인한다.
실행 중이면 두 번째 WSL Ubuntu 창에서 가상 RAW 발행기를 띄운다.

```bash
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m drone_uwb.integration.gazebo.gazebo_ranges \
  --config runs/equipment_02/trial.json \
  --output runs/raw_shadow_20260928_01
```

세 번째 WSL Ubuntu 창에서 수신·계산기를 실행한다.
`--output` 폴더는 매번 새 이름을 쓴다.

```bash
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m drone_uwb.integration.gazebo.gazebo_live_shadow \
  --config runs/equipment_02/trial.json \
  --height-profile src/drone_uwb/config/gazebo/gazebo_sensor_height_profile.json \
  --output runs/live_shadow_20260928_01 \
  --duration-s 60
```

이 명령은 `/usr/bin/python3`의 Gazebo 바인딩을 사용한다.
가상 RAW 발행기는 실험이 끝나면 `Ctrl+C`로 저장·종료한다.
`live_shadow`는 60초의 시뮬레이터 시각이 흐르면 자동 종료한다.
도중 `Ctrl+C`를 눌러도 받은 표본과 요약을 저장한다.

`capture.json`의 `raw_recorded`, `tof_recorded`, `imu_recorded`,
`processed`와 `stop_reason`을 먼저 확인한다.
`results.jsonl`은 매 UWB 주기의 계산 결과다.
`failures.jsonl`은 거부된 주기와 센서 순서 오류다.
`raw_ranges.jsonl`, `tof.jsonl`, `attitude.jsonl`은 입력 보관본이다.
`queue_delay_wall_ms`는 수신 후 계산 대기 시간이다.
센서 측정부터 PX4 수신까지의 지연으로 해석하지 않는다.

세 입력 파일의 각 표본과 `results.jsonl`에는
`host_callback_start_monotonic_ns`와
`host_callback_end_monotonic_ns`를 함께 기록한다.
두 값은 WSL 콜백의 파싱 시작과 종료 시각이다.
센서의 `time_us`는 Gazebo 시뮬레이션 시각 그대로 보존한다.
콜백 시각은 센서가 실제 측정된 시각이나 PX4 부트 시각이 아니다.
둘을 비교해 큐·파싱 지연과 시뮬레이션 일시정지를 진단한다.
PX4 부트 시계와의 변환은 별도 실측 전까지 보류한다.

## 현재 확인과 남은 검증

순수 계산 시험 5개가 통과했다.
콜백 시각 보존 시험 1개도 통과했다.
2026-10-02 저장소의 `drone_uwb` 시험 전체 176개가 통과했다.
`colcon build --symlink-install`로 5개 패키지가 빌드됐고,
`install/setup.bash`를 적용한 모듈 가져오기도 성공했다.
이전 Gazebo 정지 RAW 696주기를 재생했을 때 B의 판정·좌표는
기존 파일 계산과 모두 일치했다. 최대 위치 차이는 0m였다.
기존 27일 배포 ZIP에 새 네 모듈을 덮어쓴 임시 추출본에서
Python 모듈 가져오기는 성공했다.

사용자 WSL에서 이 새 모듈의 Gazebo 토픽 구독은 미실시다.
ToF·IMU의 실제 메시지 형식과 방향 정렬, 높이 계산 결과도 미검증이다.
PX4 연결·EKF 융합·비행·7cm 목표는 이 기록기로 입증되지 않는다.
다음 단계는 실제 토픽 기록을 받아 센서 게이트 조건을 검증하는 것이다.
