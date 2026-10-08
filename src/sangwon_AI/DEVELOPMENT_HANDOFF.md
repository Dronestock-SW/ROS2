# 웹·Jetson 개발 인계

2026-10-07 현재 상태는 [ROS2 통합 기록](../../docs/report/flight_uwb_ai_integration_20261007.md)을 따른다.
UWB 사양을 수령했고 Tag B와 PX4 관측을 연결했다.
기존 서비스는 원본 checkout에서 계속 실행한다.
통합판의 실측·실제 writer·융합·SITL·cold boot는 남았다.
아래의 2026-10-04/05 기록은 당시 상태다.
이 문서는 개발 상태와 다음 작업을 보존한다.
컨텍스트 압축·작업 재개 시 먼저 읽고 현재 파일/서비스를 확인한다.

최신 2026-10-05 드론별 미션 흐름 수정: 정상 작업은 드론 목록→선택 기체의 planner→경유점·라벨 편집→경로 적용이다. 이 POST가 기존 DroneMission과 불변 PLANNING_ONLY 저장본을 만들며 signed companion-mission의 mission_plan_ref→원본 해시·길이 확인→C++ plan.receive→내구성 receipt→같은 드론 화면으로 이어진다. JSON 파일 게시 UI는 검증된 REPLAY용 개발 도구로 접었다. 경유점별 Z·도착 yaw·호버 시간과 별도 이륙 높이·시작 yaw를 보존하며, 라벨 기존 높이를 QR 중심으로 임의 변경하지 않는다. 재연결·새 경로·삭제 후 지연 receipt는 원래 plan/session으로만 보존한다. 새 경로가 있으면 이전 시험 배포본 START/게시를 거부하고, 명령 처리 중 경로 변경도 거부한다. 웹88/88, UI8/8, 관련 Jetson CTest6/6 및 비정상 숫자 추가 사례를 검증했다. 로컬 격리 서버와 개발 Jetson의 FIELD-OBSERVE-01 HOST peer에서 저장→C++ 수신→화면 확인 및 입력값 보존을 확인했다. tested ELF 설치·own5 active·HOST_OBSERVE/can_start=false/flight_authority=false/physical_output_enabled=false 유지. planner_only는 HOST에서 receipt만 보내는 개발 단계다. 정상 planner 저장본→실행용 snapshot 생성·교정/지도·준비 검증 연결은 아직 구현 대상이며 현재 EXECUTION_SNAPSHOT_REQUIRED로 보류한다. 운영 서버 배포·실비행·FC/UWB 연결·cold boot·PC 시계 동기화 후 공동 스캔 인수는 완료로 간주하지 않는다.

## 현재 작업 — 2026-10-04

최신 2026-10-05 모드 전환: `ModeManager`/`FlightGuard`에 요청별 문맥·실제 mode/armed 확인·제한 재시도·deadline·RC 반환·Land 실패 처리를 연결했다. 승인 REPLAY에서 native TAKEOFF→이륙 완료/안정→고정 pose prestream→OFFBOARD→복귀·LAND를 선택하며, 기본 OFFBOARD 이륙 및 HOST_OBSERVE 부팅을 유지한다. ACK만으로 mode 성공 처리하지 않는다. Mode 전환 사건은 다음 요청에 덮이지 않고 SQLite/outbox에 보존하며, 읽기 전용 `after_key`로32개 초과 결과를 조회한다. 웹 START→모의 완료→전환/실행 receipt와 기존 스캔·우회·watchdog를 최종 관련 CTest14/14(112.75초), 모드19개 사례로 검증했다. 설치 ELF .text/.rodata 일치·own5 active·target enabled·비행 권한 false를 확인했다. 전체24개/실비행/cold boot 통과 주장이 아니다. 실제 MAVROS writer/native 고도·완료/독립 failsafe, UWB 전체 사양·PC 시계 동기화 후 공동 스캔은 남는다. [PX4_INTERFACE.md](PX4_INTERFACE.md)와 선택 fixture `config/companion.replay.native-takeoff.json`를 따른다.

후속 2026-10-05 PX4 관측: C++ `sangwon_px4_observer`의 MAVROS State/ExtendedState/BatteryState/RCIn→private report→Python host monitor→C++ `host_diagnostics`를 구현했다. header 시각은 FC 원천 시각을 증명하지 않는다. 중복/만료/복수 발행자/불량값을 배제하며 실제 identity/firmware/boot/prearm/위치/yaw·RC 설정은 UNKNOWN이다. 관련 CTest8/8(32.89초), 실제 관측기 중단→UNKNOWN→새 세대 복구와 tested ELF 설치를 확인했다. own5 active·target enabled·Linger=yes. 실제 domain1의 MAVROS 발행자0/네 스트림 UNKNOWN이다. HOST_OBSERVE/can_start=false/flight_authority=false/physical_output_enabled=false. FC 연결·cold boot·PC 시계 동기화 후 공동 스캔 인수는 남고 운영 웹은 WEB_CONTRACT_MISMATCH다.

후속 2026-10-05: `replay_static_detour_v1`·제공된 웹 경계/금지 구역·승인 시험 설정으로 정적 지도 XY A*를 연결했다. 중간점 방문은 필수 목표에 포함하지 않으며 원래 목표/yaw/hover와 복귀 이력을 유지한다. 동일 차단 10초 예산·복귀 실패 Land·RC 잠금과 탐색 수/시간/길이 상한을 검사한다. 도착 회전의 거리 잔차 정체 오판과 복귀 수직 단계 조기 전환을 고쳤다. 관련 Jetson CTest9/9(93.75초), 웹52+26 통과. 전체18개 통과/실비행 완료로 표현하지 않는다. tested binary 설치·own4 active·target enabled, HOST_OBSERVE/can_start=false/flight_authority=false/physical_output_enabled=false 유지. 실제 장애물 갱신/합집합/안전 고도 변경점 탐색·센서 producer/교정과 PC 시계 동기화 후 공동 스캔 인수가 후속이다.

### 최신: 웹 서버 직접 구현 승인·공동 연동

사용자가 웹 Git clone 수정·commit·push를 승인했다. 최신 구현은
[WEB_BACKEND_INTEGRATION_2026-10-04.md](WEB_BACKEND_INTEGRATION_2026-10-04.md)를 따른다.
개발 clone은 `.runtime/web-development`이며 참조 clone은 `.runtime/web-source`다. main을 fetch한 뒤 검증된 변경만 병합·push한다.
기존 서버 세션·불변 파일·점검 구현에 명령 제출/ACK 원장/모든 outbox receipt/기체별 분리를 추가했다.
실제 Django↔개발 Jetson 두 기체의 합성 임무 실행·재연결 결과 저장을 확인했다.
웹 repo의 `docs/UI_UX_COMPANION_HANDOFF.md`가 화면 담당자 인계 기준이다.
운영 8876 서버 배포/실키 등록/물리 비행 승인은 별도이며 관리자 페이지 작업 금지는 유지한다.
아래는 이전 MD 수신 단계의 이력이며 미구현 항목의 최신 상태는 위 문서를 우선한다.

최신 개발: [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md)의 합성 스캔 단계와 시계 동기화 항목을 먼저 확인한다.
C++ 스캔/지도 변환·영속 결과·웹 개정 검증과 다각형 경로 검사를 구현했다. canonical은 이 디렉터리이며 Git source mirror는 `companion/sangwon_AI`다. 최신 Jetson CTest 13/13; 다각형의 연속 선분·기체 여유·스캔 미세 조정/복귀를 검사한다. 실제 센서 제어·물리 비행·자동 우회는 후속이다.
읽기 전용 ROS 관측기 `sangwon-perception-monitor.service`를 추가해 core/web/host/perception 4개 active를 확인했다. 설정 domain=1, 센서는 미수신 UNKNOWN. 기존 String QR는 소스 시각/창 문맥이 없어 스캔 결과로 연결하지 않는다. 합성 ROS 전송과 실제 관측기 중단/만료/재시작을 검증했다. `ops/stack_status.py`에서 관측 경로를 확인한다. 소스 전달 ZIP에 장치별 local 설정·런타임·보고서가 섞이지 않도록 보완했다.
후속: 원천 `CaptureGate`→격리 ROS→Python relay→C++ `scan.observe`→ScanAction/SQLite 연결을 추가했다. C++ task/window lease·같은 host 원천 시각·producer·순번·활성화 ACK를 검증하며, 내부 fixture 판독을 동시에 넣지 않는다. 관련 7개 시험 통과(신규/스캔 3 + 기존 core/웹 4), 이전 전체 13/13과 구분한다. 실제 producer stamping/교정/map pose 변환과 PC–Jetson 공동 스캔은 미완료다. 기본 HOST 관측·4서비스·비행 권한 false를 유지한다.
최신 수신 보완: 요청/관측 내부 v2, 입력 receipt v1·원천 최대 32개 메모리 큐·C++ 최대 64개 ACK 메타데이터 cache. 관측/ACK 각각 1회 유실→원본 재전송→중복 ACK를 확인했고 나이/건강/안정 샘플을 재갱신하지 않는다. 관련 5개 시험 통과(센서 3/3 + 기존 웹 2/2), CTest 등록16 전체 결과가 아니다. 단위 ACK는 INPUT_VALIDATION_ONLY이며 최종 작업 결과·FLIGHT 승인과 구분한다.
PC 시간 동기화 후 `ops/check_peer_clock.py --ssh-host 100.110.163.94`, 웹 clone의 `scripts/test_jetson_companion_e2e.py --scan`을 실행한다. 공동 인수 미완료를 통과로 표현하지 않는다.
최신 후속: [WEB_FOLLOWUP_2026-10-04.md](WEB_FOLLOWUP_2026-10-04.md).
웹팀 검수 2~10차와 W01/W02 명세를 수신했다. W01 5줄/이후 7줄 인증·필수 필드·receive_only를 반영했다.
execution_result receipt는 웹팀 구현 보고가 있으나 C++ 일반 사건과 본문 차이는 남는다.
TEST 장치 ID와 키는 Jetson에서 별도 생성했으며 서버 등록은 대기다.
사용자는 관리자 페이지에서 작업하지 말라고 명시했다. 서버 변경은 담당자 경로로 협의한다.
GitHub 사용자 인증 완료. 읽기 전용 참조는 `.runtime/web-source`, 9a3ce22까지 확인했다.
`ops/check_web_git.py`와 기존 30분 heartbeat로 후속 커밋을 확인한다. checkout 자동 변경·push·웹 관리자 작업 없음.
실제 소스 확인: ACK는 type/accepted만 있어 문맥 필드가 필요하며 telemetry ACK도 반드시 소비해야 한다.
기존 runtime 축약 fixture는 웹의 전체 snapshot/map_volumes 계약에 맞지 않는다. 등록만 끝나면 곧바로 임무 수신 완료라고 말하지 않는다.
9차 대응표에서 draft.4 명령 생성·ACK 원장·준비/스캔 API 미구현을 다시 확인했다. 구형 control-action으로 대체하지 않는다.
최종 Python 변경 후 전송 단위 10개와 통합 묶음 2/2 통과. 운영 web adapter 재시작 후 HOST_OBSERVE·3서비스 active를 확인했다.
후속 C++ 관측 시각 보완까지 최종 CTest 7/7(23.70초) 후 바이너리 설치·3서비스 active 확인.
새 전달본 `WEB_JETSON_SOURCE_HANDOFF_2026-10-04_r2.zip`에는 누락됐던 서비스/빌드/설치 파일과 배포 기록을 포함한다. 실제 콜드 부팅은 미검증이다.

사용자가 웹↔Jetson 구현·자동 부팅·확장 가능한 구조를 승인했다.
수정 범위는 src/sangwon_AI다. SSH 반영도 승인됐다.
기억 보존을 명시 요청했다. 인증 비밀은 메모리/문서에 넣지 않는다.

기준은 WEB_REDESIGN_REQUEST_2026-10-04.md r2와 draft.4다.
2026-10-04 16:43 KST GET companion-mission은 contract_version=1.1이다.
autonomy-state는 과거 404에서 로그인 redirect 302로 변경됐다.
웹팀 자동 점검 구현 보고·현재 코드 차이·다음 공동 확인은
[WEB_TEAM_PROGRESS_2026-10-04.md](WEB_TEAM_PROGRESS_2026-10-04.md)를 따른다.
17:03 KST 후속: 7줄 HMAC/33개 보고/문맥·순서 결합 readiness ACK 후보를 구현하고 CTest 7/7을 통과했다.
실제 서버 ACK와 장치 등록은 확인 대기다. 초기 세션 규격은 이후 수신한 W01 명세로 반영했다. 이전 서명 임무 GET은 401 device_auth_failed였다.
LoRa는 생존 신호만 사용하며 좌표는 Jetson→Wi-Fi→웹이다. [TELEMETRY_CHANNEL_SPEC.md](TELEMETRY_CHANNEL_SPEC.md)를 따른다.

## 구현 경계

- C++: 기존 BehaviorTree.CPP Runtime, 준비/명령 중재, 영속 원장, 출력 권한.
- Python: 웹 HTTP/WS 전송, 세션 협상, 자료 다운로드, 재전송, 운영 도구.
- 로컬 Unix socket IPC로 두 프로세스를 분리한다.
- 부팅은 HOST_OBSERVE + 서버 읽기 전용으로 자동 기동한다.
- 격리 REPLAY에서만 새 START로 FakePx4를 실행한다.
- 미지원 scan/지도 계획/FLIGHT는 명확히 거부한다. 작업을 조용히 제거하지 않는다.
- 기존 RC 설정 유지, 비상 인계 후 동일 비행 자율 회수 금지.
- UWB 전체 사양 수령 전 장치 어댑터/융합 구현은 보류다.

## 이 작업의 완료 확인

2026-10-04 확인 결과:

- C++ daemon + Engine + SQLite Ledger/outbox, Python HTTP/WS 어댑터·mock 구현.
- private AF_UNIX IPC v1. C++가 준비/명령/실행을 결정하고 Python은 운반한다.
- 격리 REPLAY는 exact SHA 허용 목록의 waypoint만 실행. 전체 scan fixture는 준비 거부.
- 10초 TTL, boot/runtime/control 문맥, 준비 ID/개정, sequence, 중복 실행 방지.
- 네트워크/어댑터 중단 중 모의 임무 완료, 재연결 후 저장 보장 결과 전송.
- SIGKILL 후 원장 복원·RECOVERY_LOCK. 자동 재개/재이륙 없음.
- PAUSE/RESUME/CANCEL와 복귀 중 pause 거부 시험 포함 CTest 7/7.
- remote install 경로 `.build/colcon-install/sangwon_ai_replay/bin/sangwon_companiond`.
- `sangwon-autonomy.target` enabled, core/web/host services active, Linger=yes.
- 감독된 서비스 restart 후 새 runtime ID·서버 조회 복구 확인. 실제 콜드 부팅은 미시험.
- 당시 실제 웹 GET는 1.1. `.runtime/web/adapter_status.json`은 CONNECTED_READ_ONLY + WEB_CONTRACT_MISMATCH였다. 최신 웹 변경은 위 진행 기록을 따른다.
- core HOST_OBSERVE, can_start=false, flight_authority=false. 실제 기체 출력 없음.
- 기체 ID/키를 Jetson `.runtime/private/device.env`에 0600으로 생성. 서버 등록은 미완료.
- 비밀은 로컬로 복사하거나 문서/메모리에 기록하지 않는다.

## 재개 순서

### PX4 기준 자료 추가 — 2026-10-04

- 현재 장치는 FC가 연결되지 않은 개발 Jetson이다. 연결 완료 상태로 표현하지 않는다.
- 사용자 제공 커스텀 PX4 v1.17.0/FMUv6C와 현재 수동비행 `FC_praameters.params` 1,095개를 검사했다.
- 최신 기준·보존 사본·해시·15개 과거 대비 차이는 [PX4_PARAMETER_PROFILE.md](PX4_PARAMETER_PROFILE.md)에서 연결한다.
- 검증된 프로파일의 지상 대조·적용 기능은 구현 예정이다. 현재 자동 부팅은 읽기 전용이며 파라미터 writer는 없다.
- 임무 속도/좌표는 목표 메시지로 처리한다. RC 인계 중 설정 복원이나 PID 자동 조정은 하지 않는다.
- UWB는 완성 후 전체 사양을 받기로 했으므로 장치 어댑터 구현 보류를 유지한다.

1. SERVICE_ARCHITECTURE.md / WEB_JETSON_RUNBOOK.md를 읽는다.
2. SSH로 `ops/stack_status.py`와 systemd 상태를 재확인한다. 과거 보고서를 현재 상태로 단정하지 않는다.
3. 웹팀에는 WEB_JETSON_INTEGRATION_REQUEST.md와 기존 r2 전체 요청서를 전달한다.
4. 회신 후 계약/기체 인증 등록을 검증하고 TEST 기체부터 실제 테스트 서버와 연동한다.
5. PX4 SITL·출력 분리·지도 검증·Scan subtree·기록/ULog를 구현한다.
6. UWB는 전체 사양 수령 후 어댑터/융합 작업을 시작한다.

현재 BP-01~33 전체 preflight, 지도 경로계획, 실제 센서/RC/기체 출력은 미완료다.
REPLAY/host 통과를 실제 비행 준비 완료로 표현하지 않는다.
기존 RC 설정 보존과 비상 인계 후 같은 비행 자율 회수 금지는 계속 적용한다.
