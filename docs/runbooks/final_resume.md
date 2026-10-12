# FINAL 재개 절차

다음 현장 시험에서 FINAL을 다시 실행하는 절차다.
재부팅·작업 인계 시 읽는다.

최신 코드는 [브랜치 대조·FINAL 정리](../report/final_cleanup_20261012.md)를 따른다.
12일 후속 작업은 `FINAL-TEST1-SANGWON`이다.
[LiDAR 관측 보완](../report/lidar_gap_support_20261012.md)을 함께 읽는다.
[C++ 미션 AI 통합](../report/cpp_field_mission_integration_20261012.md)을 필수로 읽는다.
[LiDAR 지상 시각 확인](../report/lidar_ground_audit_20261012.md)도 읽는다.
[실측·활성화 절차](lidar_ground_qualification.md)를 따른다.
[OFFBOARD 자세 추종 분석](../report/run8_offboard_attitude_20261012.md)을 읽는다.
12일 [UWB 공백·현재 관측](../report/uwb_gap_processing_20261012.md)도 읽는다.
실기 원인 조사는 [실행 8 인계](../report/hover_research_handoff_20261011.md)를 따른다.
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

아래는 후속 브랜치의 설치 절차다.

```bash
git fetch origin
git switch FINAL-TEST1-SANGWON
git pull --ff-only origin FINAL-TEST1-SANGWON
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to drone_mission drone_platform_link
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
cmake -S src/sangwon_AI -B .review/final-ai-build
cmake --build .review/final-ai-build --target sangwon_field_mission_ai sangwon_lidar_relative_replay -j2
CPP_AI_BINARY="$PWD/.review/final-ai-build/sangwon_field_mission_ai"
ctest --test-dir .review/final-ai-build -R 'field_mission_ai_contracts|lidar_relative_contracts' --output-on-failure
```

Python 환경에 pymavlink와 websockets를 준비한다.
고정 버전은 src/drone_mission/requirements-field.txt에 있다.
다른 체크아웃의 임시 라이브러리 경로에 의존하지 않는다.
pip가 준비된 Python에서 새 비공개 폴더에 설치한다.

```bash
python3 -m pip install --target .review/final-deps -r src/drone_mission/requirements-field.txt
export PYTHONPATH="$PWD/.review/final-deps${PYTHONPATH:+:$PYTHONPATH}"
```

ROS Humble·MAVROS·시스템 NumPy는 별도로 필요하다.
상단 LiDAR의 lidar_gap.enabled 기본값은 false다.
장착·시각·환경을 확인한 뒤에만 활성화한다.
YDLIDAR와 MAVROS 관측은 같은 ROS domain이어야 한다.
ROS_LOCALHOST_ONLY도 센서 서비스와 같아야 한다.
현재 센서 서비스는 domain 2·localhost 1이다.
상대 이동 입력은 /point_cloud의 SDK stamps다.
/scan 배열 번호로 광선 시각을 만들지 않는다.
의존 패키지 drone_demo도 같은 체크아웃에서 빌드한다.
이유: 이전 install을 쓰면 새 체크아웃의 누락이 숨겨진다.
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
새 예제의 송신기 점프 보호는 true다.
보호 ON/OFF 모두 새 송신기는 착륙·DISARM에서 시작한다.
시작 확인 뒤 ARM·RC 인계 중 관측 융합은 유지한다.
실행 8의 false 설정을 새 보정에 자동 재사용하지 않는다.
이유: false는 당시 별도 송신기 우회 시험의 값이다.
B4 높이 연구 검사는 명시적 true일 때만 적용한다.
기본 현장 경로·비행 준비를 연구 재생과 구분한다.

## 스택 시작

SITE, STORE, LEDGER는 위 표의 기존 파일 경로다.
STATE_DIR는 새로운 로그·프로세스 기록 경로다.
센서와 같은 OS 계정에서 실행한다.

먼저 같은 인자로 --check-only를 실행한다.
의존성·부팅·태그·좌표·레버암·지연 계약을 검사한다.
기존 프로세스와 포트 점유도 확인한다.
새 예제의 mission_ai=cpp_bt를 사용한다.
이전 python_test 설정은 현장 local_xy 실행에서 거부한다.
CPP_AI_BINARY는 이 체크아웃에서 빌드한 C++ 실행 파일이다.
check-only는 파일·권한·해시를 검사한다.
실제 프로세스의 hello는 스택 시작 때 확인한다.
검사는 START·ARM·관측 송신을 시작하지 않는다.
FC 상태·관측의 현장 대조는 별도로 수행한다.

```bash
python3 src/sangwon_AI/ops/final_field_session.py \
  --cpp-ai-binary "$CPP_AI_BINARY" \
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

2026-10-12: 새 격리 소스로 전체를 다시 검증했다.
Python 1,105개 통과. 실패·skip은 0개다.
ROS 패키지 4개와 C++ 전체 빌드가 성공했다.
CTest 마지막 묶음은 31/32개 통과했다.
실패한 native_hover_ros는 단독 재실행에서 통과했다.
32개 개별 통과와 묶음 반복 안정성을 구분한다.
간헐 입력 만료·IPC 지연은 미해결로 기록했다.
정리본의 실비행·새 SITL은 수행하지 않았다.
[검증 기록](../report/evidence/final_cleanup_20261012/README.md)을 확인한다.
