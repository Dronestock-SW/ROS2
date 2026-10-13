# 상원 통합 브랜치 재개
통합 코드를 받아 검증하고 FC의 현재 값을 읽는 절차다.
다른 노트북·Jetson 세션에서 작업을 이어갈 때 읽는다.

브랜치는 `이번에-상원-한거-통합-브랜치`다.
[통합 범위와 제한](../report/sangwon_integration_20261013.md)을 먼저 읽는다.
main·FINAL은 작업 대상으로 사용하지 않는다.

## 코드 받기

기존 변경을 보존한 별도 체크아웃에서 받는다.

```bash
git fetch origin
git switch 이번에-상원-한거-통합-브랜치
git pull --ff-only origin 이번에-상원-한거-통합-브랜치
```

새 로컬 추적 브랜치가 없으면 다음을 사용한다.

```bash
git switch --track origin/이번에-상원-한거-통합-브랜치
```

## 검증과 실행 경로

빌드는 [기존 FINAL 재개](final_resume.md)를 따른다.
C++ 실행기는 [지상 시험](cpp_executor_ground.md)을 먼저 따른다.
웹·관측 Python 라이브러리는 그대로 필요하다.

| 선택 | launcher 인자·설정 |
|---|---|
| C++ 판단+Python 실행 | cpp-ai-binary. 분리 TAKEOFF/HOLD/START도 지원 |
| C++ 판단+C++ 실행 | cpp-executor-binary 추가. start_mode=ground_takeoff |
| 기존 공통 관측 송신기 재사용 | external-observation-bridge 추가 |

실행 도구 자체는 START·ARM을 제출하지 않는다.
사용 중인 비공개 설정·부팅 보정·원장은 보존한다.
두 실행기를 중복으로 시작하지 않는다.

## 전달 관측 제한

실제 사용할 mission.json에 다음 값을 명시한다.
새 예제에는 이미 들어 있다.

```json
"bridge_observation_timeout_s": 0.7
```

기존 설정에 없으면 0.35초를 유지한다.
비행 중 설정을 변경하거나 실행기를 재시작하지 않는다.
전달 관측 나이와 원시 UWB 나이를 따로 확인한다.
UWB 0.25초와 FC 위치 0.2초 검사는 그대로다.
낡은 stamp를 현재 시각으로 바꿔 통과시키지 않는다.

## FC 현재 값 읽기

Windows에서 SSH 연결한다.

```powershell
ssh arialhanho@100.110.163.94
```

Jetson의 기존 MAVROS와 같은 계정·domain에서 조회한다.
OUTPUT은 매번 새 비공개 파일의 절대 경로다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
python3 src/sangwon_AI/ops/px4_sensor_readback.py \
  --profile timing --ros-domain 2 --output "$OUTPUT"
```

FC가 최신 DISARM 상태인지 도구가 확인한다.
기존 MAVROS 라우터로 고정 read-only 명령만 보낸다.
직렬 포트를 중복으로 열지 않는다.
COM_OF_LOSS_T·EKF2_EV_DELAY·EKF2_NOAID_TOUT를 구분한다.
FC 버전·높이·상승 속도·융합 상태도 함께 기록한다.
이 조회는 파라미터를 쓰거나 ARM·이륙하지 않는다.

XY 유지 관련 값도 기존 offboard profile로 읽는다.
다른 새 출력 경로를 사용한다.

```bash
python3 src/sangwon_AI/ops/px4_sensor_readback.py \
  --profile offboard --ros-domain 2 --output "$OFFBOARD_OUTPUT"
```

위 프로파일은 MPC_XY_P·속도 이득·호버 추력·기체 축을 읽는다.
이 값을 임의로 늘리거나 바꾸지는 않는다.

0.35초 LAND의 직접 제한은 mission.json이다.
FC 값을 0.7로 바꿨다고 이 검사가 자동 변경되지 않는다.
실물 readback과 새 지상 검증 후 다음 비행을 판단한다.
