# 웹·Jetson 개발·운영 절차
이 문서는 자동 기동 서비스와 격리 연동 시험을 실행하는 방법이다.
Jetson에서 상태 확인·재현·개발 재개 시 사용한다.

## 1. 적용 경로

로컬: `C:\Users\ASWPC_NTBOOK\Desktop\Drone5\src\sangwon_AI`
Jetson: `/home/arialhanho/Desktop/ROS2/src/sangwon_AI`
SSH: `100.110.163.94`. 수정·빌드·상태 파일은 이 패키지 안에 둔다.

읽기 전용 센서 관측은 `sangwon-perception-monitor.service`가 자동 시작한다. `python ops/stack_status.py`에서 perception과 C++ 호스트 체크를 함께 확인한다.
실제 노드의 ROS domain/topic을 확인한 뒤 `config/perception.local.json`으로 관측 설정을 맞춘다. 공개 기본 설정은 domain=1이다. 토픽 존재·관측기 실행을 센서 정상/비행 READY로 대체하지 않는다.
격리 ROS 수신·만료·복구 시험은 `bash tests/run_ros_perception_test.sh`다. 합성 전용 토픽을 사용하며 실제 비행 점검 근거가 되지 않는다.

## 2. 환경·빌드

ROS2 Humble/기존 `.venv`를 유지한다. C++17, CMake 3.22+, SQLite3/OpenSSL 개발 라이브러리가 필요하다.
Python 보조 의존성은 `deployment/requirements-jetson.txt`의 websockets 15.0.1 고정본이다.
최초 환경 생성은 [JETSON_ENV.md](JETSON_ENV.md), 패키지 빌드는 [BUILD_REPLAY.md](BUILD_REPLAY.md)를 따른다.

```bash
cd /home/arialhanho/Desktop/ROS2/src/sangwon_AI
# 기존 CMake 빌드 트리가 있을 때
cmake --build .build/colcon-build/sangwon_ai_replay -j2
ctest --test-dir .build/colcon-build/sangwon_ai_replay --output-on-failure
# 운영 중 바이너리 교체 전 서비스 정지
systemctl --user stop sangwon-autonomy.target
cmake --install .build/colcon-build/sangwon_ai_replay
bash deployment/install_user_stack.sh
```

설치 스크립트는 다른 경로의 동명 unit을 덮어쓰지 않는다.
현재 계정 Linger=yes이므로 사용자 로그인 없이 user systemd가 기동 가능하다.
linger가 꺼진 다른 장치는 관리자와 활성화를 확인한다.
서비스 재시작 시험은 했고, 실제 전원 재인가 후 부팅 시험은 아직 하지 않았다.

## 3. 상태/로그 조회

```bash
systemctl --user status sangwon-autonomy.target sangwon-core.service sangwon-web-adapter.service
.venv/bin/python ops/stack_status.py
python3 ops/health_monitor.py --status
journalctl --user -u sangwon-core.service -u sangwon-web-adapter.service -n 80 --no-pager
```

| 표시 | 의미 |
|---|---|
| active / RUNNING | 서비스 프로세스가 살아 있음 |
| CONNECTED_READ_ONLY | 현재 웹 GET 조회 성공 |
| WEB_CONTRACT_MISMATCH | 서버 1.1, 요청 draft.4. 새 계약 실행 보류 |
| HOST_OBSERVE / NONE | 기체 명령 출력이 없는 현재 운영 모드 |
| BLOCKED / NOT_READY | 하드웨어·계약·전체 preflight 미완료 |
| REPLAY_ONLY + flight_authority=false | 가짜 기체 시험만 가능 |

실제 위치·armed가 null인 것을 0 또는 false 센서값으로 변환하지 않는다.
운영 웹에 상태를 업로드하려면 웹팀이 새 API/WS와 인증을 준비해야 한다.
현재 실제 서버에는 GET만 수행한다.

## 4. 격리 mock을 수동 실행

세 터미널에서 각각 실행한다. mock은 loopback만 열며 boot target에 포함되지 않는다.

```bash
# A
cd /home/arialhanho/Desktop/ROS2/src/sangwon_AI
PYTHONPATH=python .venv/bin/python -m sangwon_web.mock_platform \
  --snapshot contracts/runtime_replay/waypoint_snapshot.json --state-dir .runtime/mock
# B
.build/colcon-install/sangwon_ai_replay/bin/sangwon_companiond --config config/companion.replay.json
# C
PYTHONPATH=python .venv/bin/python -m sangwon_web.adapter --config config/web.replay.json
```

상태 확인 후 **별도 운영자 명령으로만** 합성 임무를 시작한다.

```bash
curl --fail http://127.0.0.1:8878/debug/state
curl --fail -H 'Content-Type: application/json' \
  -d '{"action":"START","client_request_key":"operator-test-001"}' \
  http://127.0.0.1:8878/debug/command
```

같은 client_request_key는 기존 명령을 반환한다.
PAUSE/RESUME/CANCEL/LAND_NOW도 같은 debug 경로를 사용하되 새 key를 준다.
이 debug API는 운영 웹 API가 아니다. 원격 서버/운영 기체 ID로 연결하지 않는다.
REPLAY는 임의 새 assignment를 자동 생성하지 않는다. 반복 시험은 CTest의 임시 상태를 권장한다.
RECOVERY_LOCK은 DB 파일을 삭제해 우회하지 않는다. 현재 자동 해제 API는 없고, 시험 상태를 조사한 뒤 별도 격리 시험 인스턴스를 만든다.

## 5. 검증 범위

CTest 7개 묶음: C++ 핵심, nominal, overshoot, 독립 guard 시험,
host monitor, 웹 전송 경계, HTTP/WS/C++ 통합.
통합 시험은 합성 waypoint, fresh START, TTL/문맥/순서 거부,
scan 미지원 거부, 중복 실행 방지, 단절 중 완료, 재접속 결과 전송,
SIGKILL 후 복구 잠금, PAUSE/RESUME/CANCEL 및 복귀 중 pause 거부를 검증한다.

재현 보고서는 `reports/WEB_JETSON_CTEST_2026-10-04.txt`와 boot/status JSON이다.
센서·실제 웹의 새 API·SITL·실비행·스캔 성공의 증거는 아니다.

## 6. 장치 등록

```bash
.venv/bin/python ops/setup_device_identity.py --drone-id 5
```

기존 ID/키를 재사용하며 자동 회전하지 않는다. 출력에는 비밀이 없다.
서버 등록용 공개 자료는 `.runtime/device_enrollment_request.json`이다.
키 파일은 Jetson에만 보존한다. `cat device.env`나 문서/소스 압축에 포함하지 않는다.
등록 요청·후속 단계는 [WEB_JETSON_INTEGRATION_REQUEST.md](WEB_JETSON_INTEGRATION_REQUEST.md)를 따른다.

## 7. 다음 개발 작업

1. 웹팀 회신과 실제 샘플로 계약 차이를 고정하고 TEST 기체 연동.
2. 지도/경로 검증·센서 상태 어댑터·PX4 SITL 및 출력 프로세스 분리.
3. ArUco/QR subtree·기록/ULog 파이프라인.
4. UWB 전체 사양 수령 후 어댑터와 융합 검증.
5. 실측 프로파일·RC/PX4 failsafe·전체 preflight 확인 후 실비행 승인.

현 설정의 `physical_output_enabled`를 true로 바꾸어도 비행 기능은 생기지 않는다.
C++가 해당 설정을 거부한다.

## 8. 실제 웹 W01/W02 수신 전용 시험 준비

현재는 준비만 완료했으며 자동 서비스에는 적용하지 않았다.
필수 선행 조건: 웹팀의 TEST 장치/키 등록 확인. 이후 전체 snapshot·capability 계약도 맞춰야 임무 파일 배정을 시험할 수 있다.
현재 축약 mock fixture는 웹 W02 전체 지도 스키마와 호환되지 않으므로 그대로 업로드하지 않는다.

Jetson에 시험 전용 ID/비밀을 생성했다. 아래 명령은 같은 파일을 재사용하며 기존 기체 5 키를 바꾸지 않는다.

```bash
.venv/bin/python ops/setup_device_identity.py --drone-id TEST-DRONE-01 --identity-name replay-device --profile REPLAY
```

웹 등록·공동 시험 준비 후 격리 터미널에서 실행한다. 부팅 서비스와 별도 상태/소켓 경로를 쓴다.

```bash
# 터미널 A: FakePx4만 있는 시험 core
.build/colcon-install/sangwon_ai_replay/bin/sangwon_companiond --config config/companion.web-receive.example.json
# 터미널 B: 생성한 로컬 비밀을 환경으로 전달. 출력하거나 문서에 복사하지 않는다.
set -a
. .runtime/private/replay-device.env
set +a
PYTHONPATH=python .venv/bin/python -m sangwon_web.adapter --config config/web.receive-only.example.json
```

이 시험은 W01 POST와 W02 GET만 수행한다. 명령 실행·WS·outbox POST는 하지 않는다.
`CONNECTED_NO_ASSIGNMENT`는 세션/조회 성공, `SNAPSHOT_RECEIVED`는 원본 수신 성공이며 준비 결과는 별도 `preparation_error`다.
타임아웃·인증 거부·세션 교체 시 과거 명령을 재실행하지 않는다.

## 9. 웹 소스 업데이트 조회

Windows 로컬에서 `python ops/check_web_git.py`를 실행한다.
사용자 Git Credential Manager 인증으로 origin/main을 fetch하고 커밋/변경 경로만 `.runtime/web_monitor/git_latest.json`에 보존한다.
참조 checkout은 변경하지 않으며 코드를 실행·배포하거나 push하지 않는다.
소스 열람은 기록된 SHA의 `git show <sha>:<path>`를 사용한다. 최신 Git 커밋과 실제 8876 서버 배포본은 별개다.
