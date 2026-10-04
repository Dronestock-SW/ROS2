# PX4 SITL 부트 시계 조회

이 문서는 WSL 시계와 PX4 부트 시각을 비교하는 진단 절차다.
Gazebo 관측의 시각을 PX4로 옮기기 전에 읽는다.

## 현재 결론

`sitl_clock_probe`는 PX4에 `TIMESYNC` 조회만 보낸다.
PX4 응답의 부트 시각과 WSL 단조 시각의 왕복 구간을 기록한다.
관측 송신·파라미터 변경·시동·비행 명령은 없다.
진단 결과는 Gazebo→PX4 시계 변환의 완료 증거가 아니다.

[MAVLink TIMESYNC 규약](https://mavlink.io/en/services/timesync.html)은
요청 시 `tc1=0`, `ts1=요청자의 ns 시각`을 사용한다.
[사용자 PX4 커밋의 응답 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/mavlink/mavlink_timesync.cpp)는
`tc1`에 PX4 부트 시각을 ns로 담고 `ts1`을 되돌려준다.
현재 WSL의 `pymavlink` 버전은 확인되지 않았다.
구형 Python dialect는 TIMESYNC 대상 ID 필드를 노출하지 않을 수 있다.
그 경우 `reply_target_verified=false`로 기록한다.
결과 파일의 `clock_mapping_verified`는 항상 `false`다.

## WSL 실행

PX4·Gazebo가 실행 중인 WSL Ubuntu의 새 창에서 입력한다.
`14540` 포트는 다른 Python 프로그램과 동시에 열지 않는다.
결과 파일마다 새 이름을 사용한다.
[현재 누적본 반영 절차](uwb_gazebo_navigation_runbook.md#갱신본-반영)를 따른다.
과거 진단 ZIP은 이력으로 보존한다.
새 누적본 위에 과거 파일을 다시 덮어쓰지 않는다.
이유: 현재 연결 모듈을 과거 버전으로 바꿀 수 있다.
아래 명령은 누적본의 프로젝트 루트에서 실행한다.

```bash
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
~/.venvs/px4/bin/python -m drone_uwb.integration.sitl.sitl_clock_probe \
  --output runs/sitl_clock_probe_20261002_01.json \
  --duration-s 20 --samples 8
```

출력에서 `pymavlink import completed`를 먼저 확인한다.
그 뒤 `status`, `samples` 개수, `message_counts`를 확인한다.
`no_heartbeat`나 `partial`이면 UDP 수신과 응답을 먼저 살핀다.
JSON 파일을 원격 작업 경로로 보내 분석할 때는 다음을 쓴다.

```bash
scp runs/sitl_clock_probe_20261002_01.json pgyxn@100.110.163.94:/home/pgyxn/
```

## 기록 해석

| 필드 | 의미 |
|---|---|
| `sent_host_monotonic_ns` | WSL에서 요청 직전 읽은 시각 |
| `received_host_monotonic_ns` | WSL에서 응답 처리 직후 읽은 시각 |
| `px4_boot_ns` | PX4 응답의 부트 시각 |
| `round_trip_ns` | 요청부터 응답까지 걸린 WSL 시간 |
| `px4_minus_host_midpoint_ns` | 왕복 중간점을 기준으로 한 임시 시각 차이 |
| `half_round_trip_ns` | 왕복 비대칭을 모를 때의 지연 범위 지표 |
| `adjacent_clock_rate_ratios` | 이웃한 표본에서 본 PX4/WSL 시계 진행률 |

왕복 경로가 비대칭이면 중간점은 정확한 측정시각이 아니다.
Gazebo가 일시정지하거나 느리게 진행할 수도 있다.
따라서 이 결과와
[Gazebo Shadow의 콜백 시각](uwb_gazebo_live_shadow.md)을 함께 검토한다.
측정시각 변환과 송신 게이트는 그 뒤 별도 검증한다.

## 확인과 남은 작업

가짜 MAVLink 응답의 단위 시험 5개에서
출처·대상·요청 시각 불일치 거부를 확인했다.
2026-10-02 저장소의 `drone_uwb` 시험 176개가 통과했다.
`colcon build --symlink-install`로 5개 패키지가 빌드됐다.
설치 환경에서 모듈 가져오기도 확인했다.
2026-10-02 사용자 WSL의 `pymavlink` 가져오기는 성공했다.
PX4 시각 조회 응답은 아직 미실시다.
Gazebo↔WSL 진행률과 PX4 EKF 입력 시각 검증도 미실시다.
