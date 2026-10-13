# 13일 저녁 웹 비행 세션
준비된 Jetson 실행본을 여는 절차다.
조종기·앵커·현장 조종자가 준비된 뒤 읽는다.

통합 브랜치는 `이번에-상원-한거-통합-브랜치`다.
[검증 결과와 한계](../report/jetson_field_preparation_20261013.md)를 먼저 읽는다.
지상 검증 성공과 실제 비행 성공은 구분한다.

## 1. 노트북에서 연결

PowerShell에서 SSH와 웹 터널을 연다.

```powershell
ssh -L 8001:127.0.0.1:8001 -L 8002:127.0.0.1:8002 arialhanho@100.110.163.94
```

이후 명령은 열린 Jetson 셸에서 실행한다.
기존 MAVROS·UWB·B_TF 수집 서비스는 유지한다.
다른 서비스가 FC 직렬 포트를 다시 열면 안 된다.
동시 소유로 입력과 명령이 끊길 수 있기 때문이다.

## 2. 준비된 실행본 선택

```bash
FLIGHT_SLOT=/home/arialhanho/ROS2-review-20261008-codex/.review/fc-owned-runtime-20261013
FLIGHT_SOURCE="$FLIGHT_SLOT/source"
source /opt/ros/humble/setup.bash
source "$FLIGHT_SOURCE/install/setup.bash"
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
export PYTHONPATH="$FLIGHT_SLOT/field-deps:$FLIGHT_SOURCE/src/drone_mission:$FLIGHT_SOURCE/src/drone_uwb:$FLIGHT_SOURCE/src/drone_platform_link${PYTHONPATH:+:$PYTHONPATH}"
FLIGHT_RUN="$FLIGHT_SLOT/sessions/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$FLIGHT_RUN"
```

소스·실행 파일 신원은 slot의 배포 기록과 대조한다.
센서 서비스는 별도 체크아웃에서 계속 실행 중이다.
해당 체크아웃의 변경을 이 실행본으로 덮어쓰지 않는다.
오늘은 LiDAR 보완을 활성화하지 않는다.

## 3. FC와 기록 확인

DISARM·지상에서 읽기만 수행한다.
낮 동안 디스크 재소모를 막으려고 원본 수집기를 멈췄다.
아래 명령으로 저녁 원본 수집을 먼저 재개한다.
MAVROS·UWB·B_TF 관측 서비스는 계속 실행 중이다.

```bash
systemctl --user start dronestock-manual-capture.service
systemctl --user --no-pager status dronestock-manual-capture.service
python3 "$FLIGHT_SOURCE/src/sangwon_AI/ops/px4_sensor_readback.py" \
  --profile timing --ros-domain 2 --output "$FLIGHT_RUN/fc-timing.json"
python3 "$FLIGHT_SOURCE/src/sangwon_AI/ops/px4_sensor_readback.py" \
  --profile manual-flight --ros-domain 2 --output "$FLIGHT_RUN/fc-recording.json"
```

RC가 실제 채널값을 내는지 확인한다.
UWB가 heartbeat 외에 거리·XY를 내는지 확인한다.
ToF와 배터리도 실제 값이어야 한다.
FC의 `cs_ev_pos`와 `cs_opt_flow`를 각각 확인한다.
설정값 1이나 `xy_valid`만으로 융합을 확정하지 않는다.
`cs_valid_fake_pos`만 켜진 상태는 위치 유지 증거가 아니다.
logger 상태와 원본 수집 기록의 파일 증가를 확인한다.
원본 수집 상태 파일은 다음 경로다.
`~/.local/share/dronestock/manual-captures-20261013/status.json`.
`reason=collecting`과 새 `capture` 경로를 확인한다.
`storage_reserve_waiting_no_flight_deletion`이면 기록 대기다.
그 상태로 비행을 시작하면 원본이 남지 않는다.
수집기 전용 systemd override로 이 새 경로를 사용한다.
기존 manual-captures의 어제 기록은 자동 만료 대상이 아니다.
센서 관측 서비스의 실행 경로는 바꾸지 않았다.

FC는 SDLOG_MODE=1, SDLOG_PROFILE=129로 기록한다.
기본 항목에 외부 위치 상세 기록을 추가한 값이다.
13일 DISARM 상태에서 logger 재시작·파일 증가를 확인했다.
FC ULog 파일명 시계가 PC와 다르므로 날짜만 믿지 않는다.
새 실행의 UTC·부팅 신원·ULog 경로를 같이 기록한다.

`EKF2_EV_DELAY`는 현재 0ms다.
관측 stamp와 실제 측정 시각의 잔여 차이를 구한다.
단순 전달 나이 0.48초를 그대로 넣지 않는다.
값을 바꾸면 FC·미션·관측 송신기 예상값을 함께 맞춘다.
파라미터 변경 뒤 재부팅·readback·재정렬을 수행한다.

## 4. 현재 부팅의 지상 정렬

기체 코가 앵커 좌표의 -Y를 향하는지 현장에서 확인한다.
+Y를 향하면 아래 인자를 `--confirm-nose-plus-y`로 바꾼다.
방향을 확인하지 않고 과거 yaw를 재사용하지 않는다.
잘못된 회전 변환은 제어 보정 방향을 바꾸기 때문이다.

```bash
python3 "$FLIGHT_SOURCE/src/sangwon_AI/ops/final_ground_alignment.py" \
  --confirm-nose-minus-y \
  --alignment-template "$FLIGHT_SLOT/templates/alignment.json" \
  --mission-template "$FLIGHT_SLOT/templates/mission.json" \
  --output-dir "$FLIGHT_RUN/aligned"
```

장착값은 실측 FC→태그 FRD (-0.10, 0, -0.10)m다.
일반 예제의 다른 장착값을 복사하지 않는다.
정렬은 8초 정지 자료를 기록한다.
이 과정은 동적 정확도나 비행 안정성을 인증하지 않는다.

## 5. 웹·C++ 실행기 시작

```bash
FLIGHT_PRIVATE=/home/arialhanho/ROS2-review-20261008-codex/.review/field-20261009
FLIGHT_ARGS=(
  --config "$FLIGHT_RUN/aligned/mission.json"
  --alignment "$FLIGHT_RUN/aligned/alignment.json"
  --site-config "$FLIGHT_PRIVATE/site.json"
  --mission-store "$FLIGHT_PRIVATE/mission-library.sqlite3"
  --ledger /home/arialhanho/.local/state/dronestock-flight/operator-requests.json
  --state-dir "$FLIGHT_RUN/runtime"
  --cpp-ai-binary "$FLIGHT_SLOT/native-build/sangwon_field_mission_ai"
  --cpp-executor-binary "$FLIGHT_SLOT/native-build/sangwon_field_executor"
)
python3 "$FLIGHT_SOURCE/src/sangwon_AI/ops/final_field_session.py" "${FLIGHT_ARGS[@]}" --check-only
```

위 검사가 성공했을 때 시작한다.

```bash
python3 "$FLIGHT_SOURCE/src/sangwon_AI/ops/final_field_session.py" "${FLIGHT_ARGS[@]}"
```

실행 도구는 START·ARM을 제출하지 않는다.
관측 송신·웹·실행기의 입력 수신을 시작한다.
이미 켜진 세션이나 포트가 있으면 로그부터 확인한다.
기존 실행기나 원장을 지워 강제로 시작하지 않는다.
비행 중 실행기 재시작은 현재 미션의 소유권을 잃게 한다.

## 6. 웹에서 시험 순서 선택

노트북에서 http://127.0.0.1:8001 을 연다.
실행기 신원은 `sangwon_cpp_executor`여야 한다.
입력 차단 이유를 확인한 뒤 현장 조종자가 START한다.

| 순서 | 웹 미션 | 성공 확인 |
|---|---|---|
| 1 | 제자리 호버 | 출발 XY 유지 → 1.7m → 연속 10초 → 착륙 |
| 2 | +X 1m 왕복 | 목표 도착 → 원래 출발점 → 착륙 |
| 3 | +Y 1m 왕복 | 목표 도착 → 원래 출발점 → 착륙 |
| 4 | X·Y 경유 | 두 경유점 → 역순 복귀 → 착륙 |

상승 제한은 0.5m/s, 수평 속도는 0.25m/s다.
밀려간 위치를 새 출발점으로 바꾸지 않는다.
관측 공백에는 승인된 목표를 유지한다.
공백 시간을 도착·호버 완료 시간으로 세지 않는다.
실제 FC 이상·조종자 전환은 기존 중단 정책을 따른다.
END는 명령 접수가 아니라 착륙·DISARM 확인 뒤다.

각 실행의 원본 UWB/B_TF·C++ 기록·ULog·영상을 묶는다.
시뮬레이션에서 통과한 결과로 실제 밀림 해결을 단정하지 않는다.
