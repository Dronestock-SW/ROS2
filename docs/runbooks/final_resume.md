# FINAL 재개 절차

다음 현장 시험에서 FINAL을 다시 실행하는 절차다.
재부팅·작업 인계 시 읽는다.

다음 연구는 [최신 실행 8 인계](../report/hover_research_handoff_20261011.md)부터 읽는다.
저장 후 수정본으로 재비행했지만 이륙 밀림이 남았다.

## 저장 범위와 한계

FINAL에 웹·미션 실행기·UWB 브리지·테스트를 모았다.
다른 브랜치나 `/tmp` 소스 없이 시작할 수 있다.
비밀번호·요청 원장·실제 미션 DB는 Git에 넣지 않는다.
보정 템플릿과 네 미션 생성 규칙은 소스에 포함된다.
전체 미션 성공·밀림 원인 해결은 아직 미검증이다.
[현장 기록](../report/local_xy_takeoff_20261011.md)을 먼저 읽는다.

## 현장 자료

Jetson 계정: arialhanho. ROS_DOMAIN_ID: 2.
런타임 루트: `~/.local/share/dronestock/web-anchor-trial-20261011`.

| 파일 | Jetson 위치 |
|---|---|
| 미션 설정 | 런타임 루트/mission.json |
| 보정 설정 | 런타임 루트/bridge-runtime/alignment.json |
| 현장 설정 | ~/ROS2-review-20261008-codex/.review/field-20261009/site.json |
| 미션 DB | 같은 디렉터리/mission-library.sqlite3 |
| 요청 원장 | ~/.local/state/dronestock-flight/operator-requests.json |

PC 복구본: `C:/Users/ASWPC_NTBOOK/Documents/Drone5-evidence/20261011/final-resume/`.
DB는 SQLite backup API로 복사했다.
요청 원장을 삭제하면 이전 START를 재수락할 수 있다.
원장을 보존하고 비행 중 프로세스는 재시작하지 않는다.

## 설치

FINAL 체크아웃에서 실행한다.

```bash
git switch FINAL
git pull --ff-only origin FINAL
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb drone_mission drone_platform_link
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
```

Python 환경에 pymavlink와 websockets를 준비한다.
현재 현장에는 기존 `.test-deps` 환경이 있다.
MAVROS·B_TF 센서 서비스를 먼저 확인한다.
현장 서비스의 DRONESTOCK_LOCAL_SETPOINTS=1을 보존한다.

## 새 부팅 보정

기수를 실제 앵커 -Y로 놓고 착륙·DISARM·정지를 확인한다.
NEW_DIR는 매번 새로운 비공개 경로다.
앵커·장착이 달라졌으면 예제를 먼저 수정한다.

```bash
python3 src/sangwon_AI/ops/final_ground_alignment.py \
  --confirm-nose-minus-y \
  --alignment-template src/drone_mission/config/final_alignment.example.json \
  --mission-template src/drone_mission/config/final_mission.example.json \
  --output-dir "$NEW_DIR"
```

8초 동안 FC 상태·속도·방위를 관측한다.
파라미터·비행 명령은 보내지 않는다.
부팅마다 달라질 수 있는 방위 기준을 새로 기록한다.
동적 정확도를 검증하는 도구는 아니다.

## 스택 시작

SITE, STORE, LEDGER는 위 표의 기존 파일 경로다.
STATE_DIR는 새로운 로그·프로세스 기록 경로다.
센서와 같은 OS 계정에서 실행한다.

```bash
python3 src/sangwon_AI/ops/final_field_session.py \
  --config "$NEW_DIR/mission.json" --alignment "$NEW_DIR/alignment.json" \
  --site-config "$SITE" --mission-store "$STORE" \
  --ledger "$LEDGER" --state-dir "$STATE_DIR"
```

기존 프로세스·사용 중 포트·이전 부팅 보정이면 중단한다.
이 도구는 START·ARM을 제출하지 않는다.
PC에서 다음 터널을 열고 웹을 새로고침한다.

```bash
ssh -N -L 8001:127.0.0.1:8001 -L 8002:127.0.0.1:8002 arialhanho@100.110.163.94
```

`http://127.0.0.1:8001/local/status`에서 최신 상태를 확인한다.
DISARM·착륙·IDLE·blockers=[]·can_start=true가 필요하다.
실제 위치·방위 대조 후 사람이 새 START를 누른다.
START 활성화는 안정적인 비행의 증거가 아니다.

현재 요청 상승 속도는 0.5m/s다. XY는 0.25m/s다.
상승 속도 확인 도구는 기본적으로 조회만 한다.
변경이 필요한 지상 DISARM 상태에서만 --apply를 추가한다.

```bash
python3 src/sangwon_AI/ops/px4_takeoff_speed.py \
  --speed 0.5 --output "$STATE_DIR/takeoff-speed-check.json"
```

이 도구는 두 상승 파라미터만 다룬다.
2026-10-11 실제 FC 재조회로 0.5m/s를 확인했다.

## RC 인계·관측 중단

RC 인계 후 비행 중 자동 제어를 되찾지 않는다.
착륙·DISARM 확인 후 웹에서 종료된 기록을 닫는다.
새 실행기 세션과 새로운 START를 사용한다.
자동 지상 세션 초기화는 아직 미구현이다.

지상 이동으로 이전 관측 이력이 남았는지 먼저 진단한다.
현재 위치·거리 측정이 정상일 때 지상 서비스를 사용한다.

```bash
ros2 service call /uwb/reset_observation_history std_srvs/srv/Trigger '{}'
```

서비스 성공 뒤 좌표·PX4 추정·발행 나이를 재확인한다.
차단 임계값을 완화해서 통과시키지 않는다.

## 검증

2026-10-11: 미션·웹·ROS 어댑터·UWB 계약 116개 통과.
테스트 ROS_DOMAIN_ID는 99였다.
새 시작·보정 도구는 구문·도움말만 검증했다.
두 도구를 이용한 새 부팅 실기 재현은 미실시다.
저장 작업 중 기존 실기 스택은 재시작하지 않았다.
