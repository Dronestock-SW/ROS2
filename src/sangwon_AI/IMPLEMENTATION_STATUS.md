# 1차 C++ 구현 현황

2026-10-07 후속: 네 지상 시험 계획을 만드는 loopback 웹을 추가했다.
호버·X·Y·XY 왕복은 기존 C++ FakePx4 경로에서 통과했다.
실제 PX4 writer·SITL·비행은 완료되지 않았다.
FC 세 파라미터의 사용자 요청 적용·저장·reboot는 별도로 확인했다.
[후속 기록](../../docs/report/anchor_low015_bench_20261007.md)을 따른다.

2026-10-07 ROS2 통합: 기존 소스 191개를 보존해 편입했다.
Tag별 domain과 선택 PX4 local_position 구독을 추가했다.
같은 메시지의 XYZ·원본 stamp·frame·유효기간을 검사한다.
수신은 진단이며 비행 readiness나 창고 정렬을 증명하지 않는다.
Jetson의 등록 CTest 25개가 최초·수정 후 재시험을 합쳐 통과했다.
실제 Tag B/domain 2에서 PX4 XYZ의 AI 구독을 확인했다.
실제 writer·SITL·융합·cold boot는 미검증이다.
자세한 결과는 [ROS2 통합 기록](../../docs/report/flight_uwb_ai_integration_20261007.md)에 있다.
아래 날짜별 기록은 당시 상태로 보존한다.

2026-10-05 모드 전환 후속: C++ `ModeManager`의 요청 문맥·ACK와 실제 state 구분·6초 전환 기한·1초 간격 최대3회 재시도·영속 전환 사건을 구현했다. 명시 승인 REPLAY 설정에서 native TAKEOFF→ARM 확인→이륙 완료/높이 안정→고정 pose prestream→OFFBOARD 호버/이동/스캔/복귀→LAND를 선택한다. 기존 OFFBOARD 이륙은 기본값이다. RC 인계 후 동일 비행 재획득 금지, 목표 lease 만료·mode 거부/시간 초과·Land 확인 실패와 공중 disarm 금지를 검사한다. Land 대기 중 위치/yaw/frame 무효이면 위치 stream도 중단한다. outbox는 기존32개 묶음을 유지하고 읽기 전용 `after_key` 조회를 추가했다. 단순 조회가 receipt ACK나 삭제를 만들지 않는다.

최종 관련 Jetson CTest **14/14 통과(112.75초)**: C++ 모드19개 사례를 포함한 core/scan/map/planner, 기존·native replay, 독립 watchdog, 소스 전달, 웹 service/native mode/scan/planner 통합. 등록된24개 전체 결과가 아니다. HTTP/WS mock→C++ native 모드 확인→완료 결과/전환 사건 receipt, 오프라인32개 초과 결과 조회를 확인했다. tested companiond의 ELF .text/.rodata와 설치본 일치·own5서비스 active·target enabled를 확인했다. 기본 HOST_OBSERVE/can_start=false/flight_authority=false/physical_output_enabled=false를 유지한다.

실제 MAVROS command/parameter writer, native 고도 기준 변환·완료/후속 mode·Jetson 전체 상실 대응의 SITL/FC 검수는 남는다. 현재 FC/UWB 연결·cold boot·PC 시계 동기화 후 공동 스캔 인수가 미완료인 점은 유지한다. 상세 계약은 [PX4_INTERFACE.md](PX4_INTERFACE.md), 선택 fixture는 `config/companion.replay.native-takeoff.json`이다.

2026-10-05 PX4 관측 후속: C++ 구독 전용 관측기로 mode/armed/connected·landed_state·배터리·RC 메타데이터 접점을 추가했다. 출력 publisher/command client/parameter write/직렬 포트 접근은 없다. private report는 host monitor→C++ readiness.host_diagnostics의 선택 OBS_PX4_* 항목으로만 전달한다. BP-C10~18 실제 기체/prearm/센서/RC 검사는 계속 UNKNOWN이다.
관련 CTest8/8(32.89초): 신규 reducer/보고서/격리 ROS 3 + 기존 host/perception/소스 전달/web transport/web service 5. 등록된 21개 전체 통과 주장이 아니다. 실제 구독, NaN 잔량/RC RSSI 미확인, 중복 시각·불량값·만료·복수 발행자 거부와 복구, 합성 보고의 실제 host 점검 배제를 검증했다. tested ELF 설치·5서비스 active·target enabled·Linger=yes, 관측기 중단→C++ UNKNOWN→새 session PASS를 확인했다. PASS는 구독 프로세스 상태이며 현재 domain1 네 스트림은 발행자0/UNKNOWN이다. HOST_OBSERVE/can_start=false/flight_authority=false/physical_output_enabled=false를 유지한다.

2026-10-05 최신: 제공된 웹 비행 경계·금지 구역과 capability/승인 설정이 있는 REPLAY에 C++ 정적 지도 XY A*를 추가했다. 선분 전체의 충돌·기체/오차/제동 여유·고도/yaw/경계를 검사한다. 중간점은 필수 방문으로 세지 않으며 순서·도착 yaw·호버를 유지하고 복귀 이력에 저장한다. 준비 중 경로 실패는 START 전 거부, 실행 중 실패는 원래 10초 안 재평가→복귀, 복귀 불가 시 Land다. 원본 임무/지도 개정은 바꾸지 않는다.
최종 Jetson 관련 CTest 9/9 통과(93.75초): 기존 core/scan/geometry·정상/오버슛 6 + 웹 서비스/스캔/우회 3. 등록 18개 전체 통과 주장이 아니다. 신규 planner 7개 사례는 오목 경계·장애물/대각선·고도 유지·탐색 한계·필수 방문/복귀·10초 예산·RC 인계를 검사한다. 웹 Django 계약/세션 52 + 기존 bridge/alignment 26개 통과. 정확한 snapshot SHA로 우회→2개 필수 목표→복귀·착륙·결과/개정 사건을 별도 C++ 프로세스에서 확인했다.
도착 후 긴 회전에서 작은 위치 잔차를 거리 정체로 오판하던 문제를 수정했다. yaw 목표는 각도 진행을 사용하며 허용 오차·속도는 유지했다. 복귀 수직 구간도 고도 허용 오차 진입으로 넘어가지 않고 정지·안정 판정을 끝낸 뒤 수평 이동한다.
검증된 바이너리를 Jetson에 설치하고 ELF .text/.rodata 일치, 4서비스 active·autonomy target enabled, HOST_OBSERVE/can_start=false/flight_authority=false/physical_output_enabled=false를 확인했다. 부팅 시 우회 실행은 꺼져 있다. 실제 LiDAR 장애물 갱신/단절 연동, 허용 영역 합집합·안전 고도 변경점 탐색, 실제 source/calibration, PC–Jetson 공동 스캔과 PX4/UWB 검증은 남는다.

2026-10-04 최신: C++ ScanAction/Runtime, draft.4 합성 지도·스캔 변환, SQLite 스캔 결과·복귀 개정과 웹 수신/API를 추가했다.
이전 전체 Jetson CTest 13/13 이후 정규화 센서 접점을 추가했고, 이번 변경에 해당하는 7개 검증을 통과했다(센서/스캔 3/3, 기존 core·웹 회귀 4/4). CMake 등록 항목은 15개이며 전체 15개 재실행 결과로 표현하지 않는다. 웹 core 268개 단위 검증은 이전 단계의 결과다. 스캐너 3회→카메라, 판독 실패→대기점→다음 목표, 재시작 후 결과 보존을 확인했다.
지도 검사는 단순 다각형의 수직 볼륨을 지원한다. 자기 교차·퇴화 형상은 거부하고 기체 반경·위치 오차·제동 여유를 반영한다. 오목한 구역의 두 끝점만으로 통과를 승인하지 않으며, 각 선분 전체가 하나의 고도/yaw/경계 볼륨 안에 있어야 한다.
스캔은 실제 작업 다각형과 저장된 지도에서 수직 선행 접근·미세 조정·대기점 복귀를 매번 검사한다. Runtime도 목표 이동·복귀를 재검사하고 복귀 불가 시 Land로 전환한다. 정적 지도 우회는 위 최신 항목을 따른다. 실시간 장애물 갱신과 여러 구역의 합집합 경로 분할은 미구현이다.
지도 변환은 exact SHA REPLAY만 지원하며 수직 라벨·항등 지도 변환·MOCK 카메라 외부파라미터로 제한한다. 실제 센서·비행 승인이나 우회 계획의 완성을 뜻하지 않는다.
PC–Jetson 공동 스캔 인수는 Windows UTC 미동기화로 미완료다. 최근 SSH 측정에서 Jetson이 약 0.32초 앞서 웹의 250ms 미래 허용 범위를 벗어났다. 관리자 `w32tm /resync /rediscover` 후 `ops/check_peer_clock.py`와 공동 인수를 다시 실행한다. 허용 범위·비밀·운영 서버는 변경하지 않았다.

ROS 읽기 전용 관측기·자동 기동·C++ 호스트 점검 연결을 추가했다. 격리 ROS 전송의 실제 콜백·소스 만료·복구, 기존 QR 문맥 미비 거부, 합성 관측의 실센서 점검 혼입 차단을 확인했다. 실센서 제어 어댑터는 아직 미완료다.
현재 개발 장치의 기본 ROS domain=1에서 카메라/마커/스캐너는 미수신 UNKNOWN이다. 관측기 중단→보고 만료→C++ 진단 UNKNOWN과 재시작 복구를 확인했다. HOST_OBSERVE와 비행 권한 false를 유지한다.
소스 전달 ZIP은 장치별 `*.local.json`, 런타임·로그·보고서를 제외한다. 새 장치에서 키와 연결 설정을 별도로 생성한다.

정규화 센서 입력 `scan.request`/`scan.observe`, 원천 `CaptureGate`와 격리 ROS relay를 구현했다. C++가 task/window 토큰과 200ms lease를 발급하고, 기체·boot·runtime·execution·frame epoch·producer·순번·원천 monotonic/UTC 시각을 검증한다. 카메라 QR는 해당 창의 활성화 ACK 후에만 수락하며 원천 판독 UTC를 영속 결과에 보존한다.
실제 ROS 전송→Python relay→C++ ScanAction→SQLite에서 스캐너 첫 시도 성공, 3회 실패 후 카메라 성공, 최종 실패 기록·대기점 복귀·다음 목표를 확인했다. 내부 합성 판독을 동시에 공급하지 않고 외부 프로세스가 보내는 관측으로 실행했다. 같은 소스 프레임의 재전송·잘못된 시각/producer/기체·ACK 미비·종료 후 응답·HOST 입력 거부도 검증했다.
모든 입력은 `EXTERNAL_SENSOR_REPLAY`의 합성 자료다. 실제 HID/카메라 producer의 시각/창 stamping, 카메라→기체/map 변환과 승인된 외부파라미터는 후속이다. 기존 String을 임의로 새 창에 묶지 않는다. 센서 relay는 자동 부팅 서비스에 포함하지 않는다.
후속 수신 보완: 내부 요청/관측 v2에 observation ID를 추가하고 입력 receipt v1·제한된 원천 메모리 큐를 연결했다. C++는 동일 본문 재전송에 기존 ACK만 반환하며 TTL·건강 상태·안정 샘플을 갱신하지 않는다. ID 본문 충돌·순번 재사용은 거부한다. receipt는 입력 검증만 확인하며 최종 판독/복귀 결과와 구분한다.
이번 변경 검증은 관련 5개 통과다: 원천 창/전달 단위와 실제 ROS 4개 실행 사례를 포함한 센서 묶음 3/3(77.68초), 기존 웹 서비스/스캔 회귀 2/2(79.70초). 등록된 CTest 16개 전체 통과로 표현하지 않는다. 관측 1회 유실+ACK 1회 유실 후 원본 bytes 재전송·중복 ACK, 반복 정상 상태의 실제 만료→CAMERA_UNAVAILABLE 기록·다음 목표, 구형 v1 거부를 확인했다. 시험 소스 발행은 상태 조회와 별도 실행해 조회 timeout이 센서를 멈추지 않도록 했다. 기존 장치 카메라 프로세스는 변경하지 않았다.

최신 변경: [웹 backend 공동 연동](WEB_BACKEND_INTEGRATION_2026-10-04.md).
명령·준비·실행 결과와 재연결 outbox를 실제 Django와 개발 Jetson의 두 REPLAY 실행으로 확인했다.
C++는 모의 종료 관측으로 execution_result를 생성한다. 물리 비행·실센서 스캔·UWB 장치 연동은 미완료다.
이하 이전 구현 단계의 기록은 최신 문서와 구분한다.
이 문서는 구현 기능과 검증 범위를 기록한다.
개발 진행 상태와 다음 작업을 확인할 때 읽는다.

상태: C++ v0.1.0 REPLAY + v0.2 웹 실행 서비스 단계 / 2026-10-04.
설계 v1.0의 전체 구현 완료를 뜻하지 않는다.
최신: W01/W02 서명·세션 필드와 receive_only 수신 단계를 구현했다.
[검수 2~5차 후속](WEB_FOLLOWUP_2026-10-04.md)에 실제 서버와 남은 본문/ACK/등록 차이를 기록한다.
2026-10-04 17:03 보완: 7줄 HMAC/WS ACK 후보, 33개 점검 보고 형식 및
Jetson Wi-Fi UWB 변환 좌표의 미수신 표현을 추가하고 Jetson에 적용했다.
33개 실제 검사·UWB 변환기는 미완료다. 승인된 REPLAY 환경 가정과 실제 UNKNOWN을 구분한다.
실제 장치 인증 임무 GET은 401 device_auth_failed. [진행 기록](WEB_TEAM_PROGRESS_2026-10-04.md)을 따른다.
추가 구현: C++ daemon/준비·명령 중재/SQLite 원장·outbox, Python HTTP·WS 어댑터,
loopback mock, 장치 ID 생성, systemd 자동 기동 target.
Jetson에 적용하고 CTest 7/7 및 서비스 재시작을 검증했다.
실제 웹의 비인증 조회는 1.1이다. 인증 draft.4 REPLAY는 등록 후 별도 공동 인수하며 현재 자동 서비스는 HOST_OBSERVE 조회만 한다.
격리 REPLAY에서만 승인된 waypoint snapshot을 새 START로 실행한다.
[구조](SERVICE_ARCHITECTURE.md)·[실행 절차](WEB_JETSON_RUNBOOK.md)·[웹팀 추가 요청](WEB_JETSON_INTEGRATION_REQUEST.md)을 따른다.
2026-10-04 확인: 기존 `drone_bringup`에 QR 판독·파서와 ArUco 설정이 있다.
새 BT의 ArUco 미세 정렬·Scan·결과 저장/업로드 연결은 미구현이다.
[QR_SCAN_SPEC.md](QR_SCAN_SPEC.md)에 사용자 정책·재사용 범위·추가 계약을 기록했다.
실행은 [BUILD_REPLAY.md](BUILD_REPLAY.md)를 따른다.

## 1. 구현된 흐름

C++ 행동 선택과 목표 생성을 가짜 기체로 실행한다.

```text
합성 State + Mission
        |
BehaviorTree.CPP -> Runtime -> ControlIntent
                                  |
                             FlightGuard -> FakePx4

독립 시험: Python 입력 생성기 -> 별도 C++ guard
          의도 갱신 중단          lease 만료 -> Land 출력
```

`sangwon_replay`는 결정적 시험용 단일 프로세스다.
`sangwon_guard_replay`는 별도 프로세스 시험이다.
웹/서비스 간 Unix socket IPC·재기동 잠금은 구현했다.
운영용 ROS/PX4 출력 프로세스 통신과 실제 비행 중 재기동 처리는 후속 작업이다.

| 구현 항목 | 현재 범위 |
|---|---|
| BT | RC·Land·복구·차단·일시정지 우선순위 |
| 임무 | 자동 이륙, 순서별 XYZ·yaw·체류, 역순 복귀·착륙 |
| 목표 생성 | 삼각형/사다리꼴 속도, 가속·감속 제한 |
| yaw | 최단 각도 차이, 회전 속도·가속도 제한 |
| 도착 | 위치·yaw·실제 속도·연속 안정 시간 |
| 오버슛 | 목표 너머 거리 기록, 안정 전 방문 금지, 정체/시간 초과 실패 |
| 위치 복구 | 8초 예산, 정상 관측·경로 확인 뒤 자동 재개 |
| 중단 | RC 잠금, 30%/기록 실패 복귀, yaw/프레임 상실 Land |
| 출력 감시 | ID·세대·순서·프레임·시간·NaN, lease 만료 Land |
| 좌표 | 측량된 map→ROS ENU 강체 변환 함수 |
| 프로파일 | REPLAY만 허용, SITL·FLIGHT 거부 |

복구 대기는 PX4가 유효 위치를 유지할 때만 가능하다.
PX4 위치 자체가 무효면 즉시 Land 의도를 낸다.
실제 PX4 서비스·물리적 착륙 검증은 아직 없다.

## 2. 로그와 목표 속도

호버 추정값은 참고 근거로 보존한다.
추력 명령 재생·모터 출력 계산은 하지 않는다.
[LOG_REVIEW.md](LOG_REVIEW.md)의 0.32089는 PX4 정규화 추정치다.
목표 속도별 제동거리·수평 응답 모델은 미확정이다.

| 모의 설정 | 값 |
|---|---|
| 수평 목표 속도 상한 | 0.5m/s |
| 수직 목표 속도 상한 | 0.3m/s |
| 위치 목표 가속·감속 | 0.4m/s² |
| yaw 목표 속도 | 0.4rad/s |
| yaw 목표 가속도 | 0.5rad/s² |

위 수치는 시험 설정이며 로그로 측정한 값이 아니다.
목표 생성기는 끝점의 속도를 0으로 만든다.
실제 기체의 같은 움직임을 보장하지 않는다.
실제 추종·제동은 PX4 SITL과 실측으로 검증한다.
참고 제동거리 함수는 `v·지연 + v²/(2a) + 여유`다.
아직 지도 경로 안전 승인에는 연결하지 않았다.

가짜 기체는 합성 위치 추종 모델이다.
PX4 코드나 ULog로 식별한 기체 모델이 아니다.
오버슛 주입은 복구 정책 시험에 쓴다.
물리적인 최대 오버슛의 증거로 쓰지 않는다.

## 3. 시험 범위

Jetson GCC 빌드와 CTest로 검증한다.

| 시험 | 검사 내용 |
|---|---|
| 핵심 32개 | 속도·가속도, 입력 거부, 좌표, 임무·중단·복구 |
| nominal | 두 필수 점 방문, 출발점 복귀·착륙 |
| overshoot | 목표 너머 0.45m 외란 뒤 도착 재판정 |
| independent_guard_process | 상태는 살아 있고 BT 의도만 끊기면 별도 guard가 Land 출력 |

2026-10-02 Jetson에서 `colcon` 빌드·설치 후 재현했다.
[CTest 원시 결과](reports/ctest.txt)는 4개 묶음 전체 통과다.

| 결과 | 모의 관측값 |
|---|---|
| [정상 임무](reports/nominal.txt) | 2점 방문, 59.10초 후 완료 |
| [오버슛 주입](reports/overshoot.txt) | 최대 이탈 0.4825m, 2점 방문, 59.95초 후 완료 |
| 독립 감시기 | lease 0.5초, Land 의도 관측 0.503초 |

[정상 CSV](reports/nominal.csv)와 [외란 CSV](reports/overshoot.csv)를 보관했다.
정상 모의 기체의 최대 수평 속도는 0.5184m/s였다.
외란 시험에서는 0.5882m/s였다.
목표 속도 상한과 실제 추종 속도가 다름을 보여준다.
이 결과로 실제 기체의 속도 상한을 보장하지 않는다.

주요 실패 시험은 다음과 같다.

- 오래된 세대·프레임·부팅 세션·중복 순서·NaN 거부
- FLIGHT 프로파일과 준비되지 않은 시작 거부
- RC 인계 후 같은 비행 재개 거부
- 위치 복구 중 배터리 복귀와 일시정지 문맥 유지
- 관측이 흔들려도 8초 예산을 초기화하지 않음
- PX4 위치가 무효면 복구 시간을 기다리지 않음
- Land 모드 미확인 시 송신 해제, 공중 disarm 금지
- 일시정지 뒤 연속 호버 시간을 다시 계산

## 4. 남은 구현

아래 항목은 완료로 처리하지 않는다.

| 순서 | 작업 | 준비 조건 |
|---|---|---|
| 1 | 장애물·경계·고도 제한, A*·수직/회전 공간 검사 | 합성 지도부터 가능 |
| 2 | ROS IDL·C++ 노드 분리·운영용 guard 통신 | 내부 계약 |
| 3 | MAVROS·품질 입력·모드/arm 비동기 처리 | 펌웨어·토픽·SITL |
| 4 | 명령 ID·10초 기한·영속 원장·재기동 | 내부 모의 시험부터 가능 |
| 5 | 예외 RC 이륙 인계·회전 공간 제한 | 제어권·지도 검사 |
| 6 | 웹·로그 어댑터·자동 수집·72시간 정리 | 웹 회신·PX4 수집 연결 |
| 7 | UWB 어댑터·PX4 융합 | 전체 사양 수령 |
| 8 | FLIGHT 승인·실측 제동/추종 | 실장·제한 비행 검증 |

`approved_fixture`는 합성 경로 표식이다.
실제 경로 승인이나 웹 입력 필드가 아니다.
이 값으로 실기체에 연결할 수 있는 경로는 없다.
현재 모의 경로는 장애물 없는 직선·수직 구간이다.
지도 검사·안전 지점 탐색은 아직 구현하지 않았다.

## 5. 부팅 점검 추가 구현

호스트 진단 보조 프로세스를 Python으로 추가했다.
실제 비행 준비 판단은 향후 C++ readiness_manager가 맡는다.

| 추가 | 확인 결과 |
|---|---|
| 호스트 진단 | OS·시계·용량·장치·권한·미구현 항목 보고 |
| 신선도 | boot_id·monitor_session_id·monotonic, 호스트 TTL 10초 |
| 서비스 | systemd user enabled/active, Linger=yes |
| 재시작 | 모니터 장애 주입 후 자동 복구·새 세션 확인 |
| 단위 시험 | 비행 권한 미발급·오래된 부팅/시각·원자적 저장 6개 |
| 준비 권한 | 항상 BLOCKED, 임무 준비 ID·비행 명령 없음 |

웹·RC·부팅 상세 요구를 함께 정리했다.
읽는 순서는 [웹 전달](WEB_HANDOFF.md) → [부팅 명세](BOOT_PREFLIGHT_SPEC.md)
→ [RC 명세](RC_EMERGENCY_SPEC.md) → [배포 기록](JETSON_DEPLOYMENT.md)다.
현재 구현된 RC 반환 정책과 실제 RC 하드웨어 시험은 구분한다.
