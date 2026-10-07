# 웹·Jetson 실행 서비스 구조
이 문서는 v0.2 서비스의 책임·데이터 흐름·확장 경계를 설명한다.
새 모듈을 구현하거나 웹 연동 범위를 검토할 때 읽는다.

작성: 2026-10-04. 기반 비행 로직은 기존 v0.1 REPLAY다.
서비스 구현 단계는 **HOST_OBSERVE + 제한된 REPLAY**다.
`1.1-draft.4` 전체 기능이나 실기체 연결이 완료된 상태가 아니다.
최신 합성 REPLAY는 다각형 지도·스캔 행동·영속 결과까지 지원한다. 아래의 초기 waypoint 제한 기록은 현재 범위와 구분한다.
정적 지도 우회는 C++ `planner`가 담당한다. 제공된 웹 경계·금지 구역과 `replay_static_detour_v1`·승인 설정을 확인한다. Runtime은 중간점을 필수 방문으로 세지 않고 순서·도착 회전·체류를 유지하며 이력을 복귀에 사용한다. 현재 고도 XY 탐색만 지원하고 실시간 장애물/허용 구역 합집합·안전 고도 변경점 탐색은 후속이다. 원천 데이터를 Python에서 경로 권한으로 바꾸지 않는다.
읽기 전용 ROS 관측기는 `python/sangwon_sensors`와 `ops/perception_monitor.py`다. 실제 소스 시각·중복·만료·프레임/해상도를 검사하고 호스트 점검에 요약만 전달한다. 실제 QR/마커 관측을 비행 제어 입력으로 변환하는 경로는 후속이다.
PX4 관측은 C++ `px4_observer.cpp`/`px4_health.cpp`다. MAVROS header 나이·단일 발행자·수신값을 확인해 private JSON→Python host monitor→C++ host_diagnostics로 전달한다. 실제 FC identity/boot/source time·prearm·제어권/파라미터 검증은 후속이다. 관측기에 command/output 경로가 없다.

## 1. 디렉터리와 책임

```text
src/sangwon_AI/
  include/sangwon_ai/       C++ 타입·Runtime·Guard 인터페이스
    service/               서비스 Engine·Ledger·JSON
  src/                     기존 BT·목표 생성·가짜 기체 시험
    service/               C++ 상주 서비스·명령/준비 중재·영속 원장
  trees/mission.xml        결정적 행동 우선순위
  python/sangwon_web/      HTTP/WS 전송·로컬 IPC 클라이언트·mock 서버
  python/sangwon_sensors/  원천 관측 신선도·실센서/합성 출처 분리, 비행 권한 없음
  config/                  host/observe 및 격리 replay 설정
  contracts/
    web_v1_1_draft4/       웹팀 전체 목표 계약 예제
    runtime_replay/        지금 실행 가능한 합성 waypoint snapshot
  deployment/             Python 환경·자동 기동·설치 스크립트
  ops/                    환경 점검·상태 조회·기체 등록 자료 생성
  tests/                  핵심 및 프로세스 간 통합 시험
  reports/                검증 결과·실행 시점의 상태 스냅샷
  .runtime/               비공개 실행 상태·SQLite·socket·장치 키, Git 제외
  .build/                 이 패키지 빌드/설치, Git 제외
  .venv/                  Python 보조 프로세스 환경, Git 제외
```

| 구성 요소 | 책임 | 결정하지 않는 항목 |
|---|---|---|
| C++ Engine | 준비 검증, 세션, 명령 수락, 상태·결과 생성 | 웹 UI·HTTP 재시도 |
| C++ Runtime/BT | 이륙·좌표·yaw·체류·복귀·착륙 순서와 비상 우선순위 | 인증·서버 저장 |
| C++ Planner | 고정 고도 A*·연속 선분/여유 검사·유한 예산 | 필수 경유점 생략·고도 우회·실측 승인 |
| C++ FlightGuard | 출력 ID·세대·시간·유효기간 검사 | 웹 명령 해석 |
| C++ Ledger | 명령 중복 검사, SQLite 실행 기록·발신 대기열 | 임무 변경 |
| C++ PX4 Observer | MAVROS 구독·중복/만료/복수 발행자 검사·요약 | 비행 승인·RC 인계 판정·setpoint/모드/arm/파라미터 쓰기 |
| Python Adapter | GET/POST/WS, 다운로드 해시 검사, 재연결, IPC | READY·비행 권한·행동 선택 |
| Python Host Monitor | OS·환경·장치 존재 진단 | PX4 실제 preflight 승인 |
| Python mock | loopback HTTP/WS·SQLite 저장·운영자 시험 명령 | 실제 플랫폼·기체 제어 |

## 2. 현재 동작 흐름

```text
Jetson 전원 ON
  -> 사용자 systemd + linger
  -> host/perception monitor + C++ PX4 observer + C++ companiond + Python web adapter
  -> 현재 웹 GET 조회 / 장치 상태 점검
  -> HOST_OBSERVE: 비행 출력 없음, 준비 미완료 이유 표시

격리 REPLAY 시험
  mock HTTP assignment + immutable snapshot
    -> Python 전체 다운로드·SHA256 확인
    -> Unix socket -> C++ approved fixture 검증 -> preparation + READY
  운영자의 새로운 START (10초 이내)
    -> C++ 문맥/유효시간/순서/중복 검사
    -> SQLite 수락 기록 후 Runtime 시작
    -> BT -> ControlIntent -> Guard -> FakePx4
    -> C++ phase/result -> SQLite outbox -> HTTP durable receipt
    -> C++ readiness/telemetry -> Python -> WebSocket -> mock
```

**자동 시작은 서비스에만 적용된다.** 임무 실행은 새 START가 필요하다.
REPLAY의 `can_start=true`는 `allowed_execution=REPLAY_ONLY`와 함께 해석한다.
모든 현재 프로파일의 `flight_authority=false`다. HOST의 위치·armed는 미확인/null이다.
READY 알림을 위해 모터를 돌리는 기능은 없다.

## 3. 실행 프로파일

| 프로파일 | 서버/기체 | 허용 |
|---|---|---|
| HOST_OBSERVE | 실제 서버 GET만, 실제 기체 연결 없음 | 자동 부팅·진단·조회 |
| REPLAY | loopback mock + TEST-* 식별자 + FakePx4 | 승인된 합성 snapshot의 명시적 START |
| SITL / FLIGHT | 아직 미구현 | C++ 서비스가 설정을 거부 |

REPLAY snapshot은 **파일 전체 SHA256 허용 목록**에 있어야 한다.
현 구현은 `waypoint` 작업과 `AUTO_TAKEOFF`만 지원한다.
scan 작업·RC 이륙 인계·일반 지도 경로계획을 지원한다고 광고하지 않는다.
전체 scan fixture를 받으면 작업을 생략하지 않고 준비를 거부한다.
형상·지도 검증기를 구현하기 전 임의 좌표를 허용 목록에 자동 추가하지 않는다.

## 4. 명령과 영속성

- 동일 `control_request_id` + 같은 JSON 의미: 기존 결과 반환, 다시 실행하지 않는다.
- 같은 ID의 내용 변경: `REQUEST_ID_CONFLICT`, 감사 이벤트. 기존 명령의 결과는 보존한다.
- 신규 수락: 생성 후 10초, 서버 세션·boot/runtime·assignment/snapshot·준비 개정 확인.
- `command_seq`는 해당 세션의 마지막 수락값보다 커야 한다.
- START: preparation/readiness 일치, 실행·flight ID가 null이어야 한다.
- 비행 제어 명령: 현재 execution/flight 문맥 일치. 복귀·착륙 단계의 PAUSE/RESUME 거부.
- SQLite WAL + synchronous FULL. 수락 기록 완료 후에만 실행이 tick 된다.
- 실행 중 Python/웹 단절: C++가 저장된 임무를 계속한다. 새 명령은 링크가 유효할 때만 수락한다.
- 재연결: 새 control session. 전 세션의 미전달 명령을 자동으로 새 요청으로 바꾸지 않는다.
- C++ 재시작: 새 runtime ID. DB에 미종료 실행이 있으면 `RECOVERY_LOCK`, 자동 재개 금지.
- 재시작 후 기존 ACCEPTED 결과는 조회할 수 있다. 최종 기체 상태를 확인한 것으로 꾸미지 않는다.
- 완료 assignment 재수행 거부. 다음 실행에는 새 assignment와 준비·START가 필요하다.
- 수신 결과 HTTP 200만으로 삭제하지 않는다. 서버의 `stored/key/sha256` 확인 후 outbox ACK한다.
- 사건·결과는 outbox에 보존한다. telemetry는 최신 값 전송이며 전체 센서 기록 대체물이 아니다.

SQLite 원장·outbox는 72시간 ULog 정리 대상이 아니다.
현재 자동 ULog 수집/보관 삭제와 운영 기록 용량 제한은 후속 구현이다.

## 5. 로컬 IPC v1

AF_UNIX stream, 연결당 한 요청, UTF-8 JSON + LF, 최대 2 MiB.
socket·상태는 `.runtime/<instance>` 안에만 생성한다. 단일 writer 파일 잠금을 사용한다.

```json
{"ipc_version":1,"request_id":"unique-id","method":"status","payload":{}}
```

정상: `ipc_version/request_id/ok=true/payload`.
오류: `ipc_version/ok=false/code`; 원 요청 ID는 파싱 불가 시 없을 수 있다.

| method | 입출력 의미 |
|---|---|
| status | core context/readiness/telemetry/capabilities |
| scan.request | C++ 발급 작업/판독 창·200ms lease·원천 문맥. 외부 센서 합성 REPLAY 전용 |
| scan.observe | 원천 시각/순번/작업/창·활성화 ACK 검증 후 정규화 관측 수락. HOST는 거부 |
| link.update | 통신 fresh 상태. 기체 센서 상태를 바꾸지 않음 |
| session.set | 서버 세션·기체·런타임·UTC 확인 |
| prepare | assignment, snapshot_ref, 원문 snapshot_text -> 준비 결과 |
| prepare.clear | 미실행 할당 취소. 실행 중 임무에는 영향 없음 |
| command | 구조화 control_request -> 저장된 수락/거부 결과 |
| command.get | control_request_id -> 최신 저장 결과 또는 null |
| outbox.list | 미저장 사건·결과 최대 32개, 정확한 raw_body와 SHA |
| outbox.ack | key+SHA로 해당 전송의 저장 확인 |

IPC는 같은 Jetson 사용자 계정의 신뢰된 프로세스 경계다. 네트워크에 노출하지 않는다.
서비스 IPC는 최대 0.5초 요청 읽기 제한을 가진 단일 루프다.
`EXTERNAL_SENSOR_REPLAY`는 한 tick만 실행하며 내부 가짜 판독 생성을 끈다. ROS source/relay가 원천에서 찍은 monotonic/UTC 시각과 C++ 문맥을 그대로 전달한다.
어댑터 relay는 재연결이나 새 창에 이전 QR을 다시 묶지 않는다. 실제 HID/카메라 producer 변경·장착 교정·PX4 상태 연결은 후속이며, 이 연결 시험은 센서 실측 또는 FLIGHT 승인이 아니다.
센서 요청/관측은 내부 v2, 입력 receipt는 v1이다. observation ID의 정확한 재전송은 기존 입력 ACK만 반환하고 나이·샘플·행동 상태를 갱신하지 않는다. relay는 원본 bytes의 SHA와 문맥을 receipt에 묶고, 원천은 제한된 메모리 큐에서 source TTL 안에만 재전송한다. 입력 ACK의 의미는 `INPUT_VALIDATION_ONLY`다.
현재 Runtime과 Guard는 같은 REPLAY 프로세스에 있다.
따라서 이 구조를 실시간 독립 PX4 출력 watchdog 검증 결과로 해석하지 않는다.

## 6. 확장 순서와 접점

1. 웹팀: 계약/등록/receipt를 맞춰 loopback 시험을 팀 테스트 서버로 확장한다.
2. Mission Validator: map·장애물·측량·승인 프로파일 검증. 합성 SHA 허용 목록과 분리한다.
3. C++ PX4 Port/별도 출력 프로세스: 실제 State 공급·출력 lease·RC 인계·SITL 검증 후 추가한다.
4. UWB: **전체 사양 수령 후** Python/C++ 수집 어댑터와 공통 관측 계약을 확정한다.
5. Scan subtree: ArUco 정렬·정지 확인·스캐너 3회·카메라 QR·대기점 복귀를 연결한다.
6. 운영 기록·ULog 자동 수집·72시간 보관 정책·저장공간 검사를 연결한다.
7. 센서 freshness·시간/좌표 변환·실측 프로파일·전체 preflight를 통합한 뒤 FLIGHT gate를 만든다.

Python 웹 전송을 바꾸어도 C++의 임무/비행 권한을 우회할 수 없도록 이 경계를 유지한다.
새 capability는 구현·검증 후 광고한다. 모듈 이름만 추가해서 지원 완료로 처리하지 않는다.
