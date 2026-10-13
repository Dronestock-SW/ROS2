# 13일 저녁 웹 비행 세션
준비된 Jetson 실행본을 여는 절차다.
조종기·앵커·현장 조종자가 준비된 뒤 읽는다.

통합 브랜치는 `이번에-상원-한거-통합-브랜치`다.
[웹·기록 지연 수정](../report/web_record_latency_20261013.md)을 먼저 읽는다.
[C++ 분리 이륙 검증](../report/cpp_position_handoff_20261013.md)을 먼저 읽는다.
[초기 FC·기록 준비](../report/jetson_field_preparation_20261013.md)도 보존했다.
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
`latency-release.json`의 소스 커밋은 `82e3bf1`이다.
실행기 SHA256도 [배포 대조 결과](../report/evidence/web_record_latency_20261013/deployment.json)와 확인한다.
이후 문서·JS 시험 전용 수정은 실행기 변경이 없다.
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
미션 설정은 `start_mode=takeoff_hold`다.
`takeoff_backend=local_position`을 함께 사용한다.
이륙 전 조종기로 POSITION을 선택한다.
웹의 실제 FC 모드가 `POSCTL`인지 확인한다.
입력 차단 이유를 확인한 뒤 웹 이륙을 누른다.
고정 출발 XY에서 1.7m까지 OFFBOARD로 오른다.
높이·XY·속도·UWB가 3초간 안정돼야 전환한다.
실제 FC `POSCTL` 확인 뒤 목표 송신을 멈춘다.
POSITION에서 다시 3초간 안정 상태를 확인한다.
조종기 스틱은 중립을 유지한다.
이때부터 웹의 별도 미션 START를 누를 수 있다.
START는 원래 출발 목표를 1.2초 이상 송신한다.
OFFBOARD 재진입을 확인하고 선택 미션을 수행한다.
공중에서 ARM을 다시 요청하지 않는다.
요청 ACK만으로 FC 모드가 바뀌었다고 보지 않는다.
웹 응답이 끊기면 같은 버튼으로 접수 여부를 확인한다.
웹은 저장한 동일 요청 ID를 재사용한다.
새로고침 뒤에도 미확인 요청 ID를 보존한다.
응답을 기다린 시간만으로 비행 실패를 판정하지 않는다.

| 순서 | 웹 미션 | 성공 확인 |
|---|---|---|
| 1 | 제자리 호버 | 이륙 → POSITION → 별도 START → 연속 10초 → 착륙 |
| 2 | +X 1m 왕복 | 목표 도착 → 원래 출발점 → 착륙 |
| 3 | +Y 1m 왕복 | 목표 도착 → 원래 출발점 → 착륙 |
| 4 | X·Y 경유 | 두 경유점 → 역순 복귀 → 착륙 |

상승 제한은 0.5m/s, 수평 속도는 0.25m/s다.
밀려간 위치를 새 출발점으로 바꾸지 않는다.
OFFBOARD 관측 공백에는 승인된 목표를 유지한다.
POSITION 대기에서는 FC의 위치 유지를 계속한다.
관측 공백이 있으면 START를 허용하지 않는다.
공백 복구 뒤 안정 확인 시간을 다시 계산한다.
상태 보고 지연만으로 LAND하지 않는다.
재진입 전 공백에는 새 모드 요청을 보류한다.
공백 시간을 도착·호버 완료 시간으로 세지 않는다.
실제 FC 이상·조종자 전환은 기존 중단 정책을 따른다.
END는 명령 접수가 아니라 착륙·DISARM 확인 뒤다.

각 실행의 원본 UWB/B_TF·C++ 기록·ULog·영상을 묶는다.
시뮬레이션에서 통과한 결과로 실제 밀림 해결을 단정하지 않는다.

기수 왼쪽 밀림은 [추적 기록](../report/cpp_position_handoff_20261013.md)으로 비교한다.
POSITION으로 바꾸면 밀림이 해결된다고 가정하지 않는다.
OFFBOARD도 XYZ 위치 제어를 활성화하고 있다.

## 저장 대기를 구분하기

웹 상세의 `웹저장`과 `실행기저장`을 함께 확인한다.
조회와 LAND는 START 저장 대기 중에도 가능하다.
START 응답이 늦으면 같은 요청 ID로 접수를 확인한다.
새 ID를 연속 생성하면 미확인 요청 구분이 어려워진다.
명령 기록 대기는 FC ACK 시간 초과와 구분한다.
입력 나이와 실제 송신 간격은 별도로 기록된다.
`record_pending`이면 착륙 후 완료 기록을 기다린다.
이때 END를 임의로 만들거나 원장을 삭제하지 않는다.
미션 DB가 WAL 모드이므로 DB 한 파일만 복사하지 않는다.
정지 후 SQLite 백업 절차를 사용해야 일관성을 유지한다.
