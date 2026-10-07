# C++ 모의 실행 절차
이 문서는 Jetson의 C++ 빌드·시험 절차다.
핵심 코드를 수정하거나 결과를 재현할 때 읽는다.

## 1. 실행 범위

현재 실행은 가짜 PX4를 이용한 REPLAY다.
ROS·MAVROS·시리얼·모터 출력 연결은 없다.
추가 `sangwon_companiond`는 로컬 Unix socket으로 Python 웹 어댑터와 연결한다.
서비스 운영·시험은 [WEB_JETSON_RUNBOOK.md](WEB_JETSON_RUNBOOK.md)를 따른다.
SITL·FLIGHT 프로파일은 실행 파일이 거부한다.
PX4 SITL 검증은 다음 단계다.

2026-10-05: `sangwon_replay --scenario nominal --native-takeoff`로 합성 native TAKEOFF→OFFBOARD 인계→복귀·LAND를 실행한다. `sangwon_mode_tests`는 모드/ARM 실제 상태 확인·응답 문맥·재시도·이륙 완료·RC 인계·기한과 공중 disarm 금지를 검사한다. `web_mode_integration`은 HTTP/WS mock→C++→FakePx4→영속 결과 receipt를 검사한다. 모두 REPLAY이며 실제 MAVROS 명령 전송이나 파라미터 적용 시험이 아니다.

## 2. 빌드

ROS2 루트에서 우리 패키지만 빌드한다.
생성물도 `src/sangwon_AI/.build`에 둔다.

```bash
cd /home/arialhanho/Desktop/ROS2
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon \
  --log-base src/sangwon_AI/.build/colcon-log build \
  --symlink-install --paths src/sangwon_AI \
  --build-base src/sangwon_AI/.build/colcon-build \
  --install-base src/sangwon_AI/.build/colcon-install \
  --event-handlers compile_commands- \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
source src/sangwon_AI/.build/colcon-install/setup.bash
```

첫 빌드는 GitHub 접근이 필요하다.
의존 소스는 작업 폴더로 받는다.
시스템 패키지·권한·서비스는 바꾸지 않는다.

| 의존성 | 기준 |
|---|---|
| C++ | C++17, GCC 11.4로 확인 |
| CMake | 3.22 이상 |
| BehaviorTree.CPP | 4.6.2 |
| 고정 commit | `2a8a226fbbd99f524f0796e0d8a3145773c61c06` |
| Python | 기존 guard/host 시험은 표준 라이브러리, 웹 통합은 `.venv`의 websockets 15.0.1 |
| SQLite3/OpenSSL | 새 C++ 서비스 원장·SHA256, Jetson 개발 헤더 확인 |

`colcon`이 CMake 프로젝트를 직접 발견한다.
ROS 메시지 생성 패키지는 아직 없다.

## 3. 시험

CTest는 핵심 사례와 독립 감시기를 검증한다.

```bash
ctest --test-dir src/sangwon_AI/.build/colcon-build/sangwon_ai_replay \
  --output-on-failure
src/sangwon_AI/.build/colcon-install/sangwon_ai_replay/bin/sangwon_replay \
  --scenario nominal
src/sangwon_AI/.build/colcon-install/sangwon_ai_replay/bin/sangwon_replay \
  --scenario overshoot
```

정상 시나리오는 이륙→2점 방문→복귀→착륙이다.
오버슛 시나리오는 첫 목표 근처에 외란을 넣는다.
CSV는 `--csv /절대/경로.csv`로 저장한다.
출력 파일도 이 디렉터리 안에 둔다.

## 4. 수정 위치

| 대상 | 파일 |
|---|---|
| 타입·모의 설정 | [types.hpp](include/sangwon_ai/types.hpp) |
| 목표 속도·감속 | [trajectory.cpp](src/trajectory.cpp) |
| 판단·임무 진행 | [runtime.cpp](src/runtime.cpp) |
| 우선순위 트리 | [mission.xml](trees/mission.xml) |
| 목표 검사·lease | [guard.cpp](src/guard.cpp) |
| 별도 감시기 프로세스 | [guard_replay.cpp](src/guard_replay.cpp) |
| 합성 기체 모델 | [fake_px4.hpp](include/sangwon_ai/fake_px4.hpp) |
| 사례 시험 | [core_tests.cpp](tests/core_tests.cpp) |

설정 이름만 FLIGHT로 바꾸어 실행하지 않는다.
실기체 연결과 승인값 검증이 없기 때문이다.

근거: [고정한 BehaviorTree.CPP 소스](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/2a8a226fbbd99f524f0796e0d8a3145773c61c06).

## 5. 구독 전용 PX4 관측기

Linux에 ROS Humble setup이 있으면 CMake는 `rclcpp`, `mavros_msgs`, `sensor_msgs`를 찾아 `sangwon_px4_observer`를 함께 빌드한다. 먼저 `/opt/ros/humble/setup.bash`를 source한다. ROS가 없는 Linux에서는 순수 관측 reducer/단위 시험만 빌드한다. 기존 BT는 ament와 분리한다.
관련 시험은 `px4_health_reducer`, `px4_health_boundaries`, `ros_px4_observation`이다. 마지막 시험은 domain183·격리 synthetic 토픽·private 임시 runtime에서 실제 C++ 구독을 확인한다. FC/운영 namespace를 건드리지 않으며 FLIGHT 승인 결과가 아니다.
설치는 `cmake --install .build/colcon-build/sangwon_ai_replay` 후 `deployment/install_user_stack.sh`를 따른다. 설치기는 관측 바이너리를 확인하고 기존 네 서비스와 함께 기동한다. 장치별 `config/px4.local.json`은 Git/소스 전달 ZIP에서 제외한다.
