# C++ 관측 연결 준비 결과
수동 호버 대기 중 준비한 코드·시험 기록이다.
다음 비행 로그를 회수해 C++에서 분석할 때 읽는다.

C++ 공통 입력과 파일 재생 경로를 구현했다.
기존 수집기·FC 설정·명령 서비스는 변경하지 않았다.
실물 C++ BT 실행·동적 보정·UWB 융합 완료와 구분한다.

## 구현

| 파일 | 역할 |
|---|---|
| `include/sangwon_ai/capture_observations.hpp` | 공통 `ingest`·`snapshot` 인터페이스 |
| `src/capture_observations.cpp` | 10개 관측 스트림·시각·frame·공분산 검사 |
| `src/capture_replay.cpp` | 원본 수집 폴더의 읽기·SHA256·보고서 |
| `tests/capture_observation_tests.cpp` | 유효성·누락·중복·만료·권한 경계 |
| `tests/capture_replay_files.py` | 파일 손상·태그 혼합·순서 역행 등 5개 시험 |

경로는 모두 `src/sangwon_AI` 아래다.
별도 Python 변환이나 새 관측 파일 형식을 요구하지 않는다.
메시지의 원본 정수 ns 정밀도를 유지한다.
설정 내용의 SHA256을 대조해 보고서에 참조만 남긴다.
UWB z·yaw는 null이다. PX4 ENU 위치와 분리한다.
ToF 범위 미달에 0m·0.15m를 대입하지 않는다.
`sample_valid`는 메시지 형식·시각 판정이다.
EKF 준비·실측 보정·비행 승인을 뜻하지 않는다.
이유: 수신 성공만으로 측정 정확도를 알 수 없다.

## 재생 방법

Jetson에서 새 대상 두 개만 빌드한다.
동작 중인 수집·MAVROS 서비스는 재시작하지 않는다.

```bash
cd ~/ROS2-review-20261008-codex
source /opt/ros/humble/setup.bash
cmake -S src/sangwon_AI -B src/sangwon_AI/.build/capture-observe \
  -DCMAKE_BUILD_TYPE=Release
cmake --build src/sangwon_AI/.build/capture-observe \
  --target sangwon_capture_replay sangwon_capture_observation_tests -j1
ctest --test-dir src/sangwon_AI/.build/capture-observe \
  -R 'capture_observation_contract|capture_replay_files' --output-on-failure
src/sangwon_AI/.build/capture-observe/sangwon_capture_replay \
  --capture /absolute/path/to/closed-capture > /private/path/report.json
```

이번 시험은 별도 `/dev/shm` 디렉터리에서 수행했다.
기존 BehaviorTree.CPP 소스를 읽기 전용으로 재사용했다.
Git SHA는 CMake에 고정한 `2a8a226f…`와 같았다.
빌드·재생은 실비행 중 수행하지 않는다.
이유: 분석 CPU 사용이 관측 타이밍에 영향을 줄 수 있다.
보고서는 원본 manifest·events의 SHA256을 포함한다.
원본 설정·실제 좌표·부팅 식별자는 Git에 넣지 않는다.

시험한 실행 파일은 다음 위치에 설치했다.
설치 전후 SHA256이 일치했다.
기존 서비스에는 연결하지 않았다.

```bash
~/.local/libexec/dronestock/sangwon_capture_replay \
  --capture /absolute/path/to/closed-capture > /private/path/report.json
```

설치본은 RAM이 아닌 홈 디렉터리에 보존한다.
ROS·명령 runtime 라이브러리를 링크하지 않는다.

## 확인 결과

| 구분 | 결과 |
|---|---|
| Jetson C++17 Release 빌드 | 두 실행기 성공, 경고를 오류로 처리 |
| 신규 CTest | 2/2 통과, 파일 시험 내부 5개 통과 |
| 기존 수동 비행 파일 재생 | 356,130행·330,762,327byte 처리 |
| 새 부팅의 종료된 수집 조각 | 113,304행·69,641,656byte 처리 |
| 기존 수집·관측 서비스 | active 유지, 재시작 명령 없음 |
| FC 파라미터·ARM·모드 명령 | 이번 변경에서 없음 |
| SITL·복원 후 수동 호버·자동 비행 | 이번 작업에서 미실시 |

이전 비행 events SHA256은 다음과 같다.
`a4926f2480cecb9657b613bec9118a8d81a314911280b8a48d97a411dfb387df`
기존 PC 보관본과 일치한다.

이전 비행의 UWB pose는 13,539개다.
진단 기준상 13,526개를 수락하고 13개는 오래돼 거부했다.
발행 표본 사이 최대 공백은 10.231초다.
이는 전체 수집 구간 값이다. 공중 구간만의 값이 아니다.
PX4 odom은 12,984개 중 39개가 수신 시각보다 미래였다.
IMU·RC·ToF에도 일부 미래 시각이 있었다.
이는 동적 보정 완료 근거가 아니다.
센서별 원본 시각·ULog를 함께 조사해야 한다.

새 부팅 수집 조각에는 B_TF XY가 없었다.
ToF는 현재 지상 최소 범위 미달을 기록했다.
동일 구간 여러 FC 토픽에 약 6.2초 수신 공백이 있었다.
이후 조각에서도 약 2.1초 공백을 확인했다.
관측 서비스의 NRestarts=0이었다.
공통 수신 공백 원인은 이번 결과로 확정하지 않는다.
단순 active 상태를 연속 수신 완료로 처리하지 않는다.

새 boot_id에서 수집기의 fresh checkpoint를 확인했다.
이전 부팅과 달라 자동 시작 후 기록 지속을 확인했다.
이는 전원 버튼 조작 자체를 관찰한 시험은 아니다.
부팅 직후 FC 읽기 기록은 MAG_TYPE=0이다.
FLOW_ROT=4·EV_CTRL=1도 유지됐다.
그 시점의 FC logger는 새 파일에 1.20MiB를 기록했다.
이후 계속 증가하는지와 비행 ULog 회수는 별도로 확인한다.

## 다음 순서

1. 수동 호버 복구를 확인하고 같은 시행 ULog를 확보한다.
2. 새 로그의 수신 공백·미래 시각을 먼저 대조한다.
3. 충분한 움직임 자료로 좌표·시간 보정을 평가한다.
4. 확정한 관측을 PX4에 연결해 융합 상태를 확인한다.
5. C++ BT의 실제 상태·명령 adapter를 이식한다.
6. 이후 사용자 요청에 맞춰 웹 UI를 정리한다.

이번 입력은 `State`의 위치 품질을 자동 승격하지 않는다.
창고↔PX4 변환·FC 기준점 적용은 후속 adapter 책임이다.
Native 고도 정책과 기존 Offboard 설계도 먼저 조정한다.
[입력 계약](../architecture/cpp_observation_input.md)을 따른다.
