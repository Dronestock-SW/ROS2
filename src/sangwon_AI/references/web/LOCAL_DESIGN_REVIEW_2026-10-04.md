# 로컬 수정 4차 — DESIGN.md 위치 표시

원본: `C:/Users/Lee/Desktop/WEB_TEAM_HANDOFF_2026-10-04/DESIGN.md`

**상태: 위치·관측·목표 표시 구현 완료 / DESIGN 전체 구현 완료는 아님**

검수 화면: http://203.247.41.82:8876/platform/drones/5/behavior/

진입: 드론 목록 → Drone-5 실시간 조회 → 상단 **비행 행동 상태** → **위치 · 관측 · 목표**.

## 이번 변경과 원문 대응

| 원문 | 반영 |
|---|---|
| §2 실제 위치는 PX4 융합, 원시 UWB는 감시·대조 | PX4 융합 위치 / UWB 원시 관측 / Jetson 목표 위치를 세 카드로 분리 |
| §2 미지원 z·yaw를 유효한 0으로 취급 금지 | 누락·null·불리언·비유한 좌표는 미수신. 실제 숫자 0은 0으로 표시 |
| §2 신규 관측과 유지·재전송 구분 | 장치의 is_new_observation=false는 유지값으로 표시. 같은 좌표 숫자라는 이유로 신규 관측을 거부하지 않음 |
| §5 값별 시각·유효성·출처 | 원천 관측 시각·관측 경과·출처·좌표계·장치 유효성 표시 |
| §5 관측 UWB와 제어용 위치 구분 | 원시 UWB를 PX4 추정 위치에 대입하지 않음. 좌표계 미수신은 미확인으로 표시 |
| §6 기체 목표와 scan 라벨 XYZ 분리 | active_target만 목표 카드에 표시. 목표 종류·작업 ID·계획 ID 포함. 라벨 위치로 대체하거나 접근 pose를 웹에서 계산하지 않음 |
| 통신 단절 시 마지막 상태 구분 | 보고/관측 만료 또는 조회 실패 시 마지막 보고로 전환. 숫자를 남겨도 현재 값으로 사용 불가를 표시 |

보고된 좌표계와 m 단위를 그대로 표시한다. 승인된 변환 없이 축·단위·yaw를 추정해 바꾸지 않는다.
관측 경과는 원천 관측 시각과 `source_age_ms + 보고 경과` 중 더 오래된 쪽을 사용한다.
원천 시각·경과가 없거나 모순되면 신선한 관측으로 표시하지 않는다.
서버 응답 이후에도 브라우저의 단조 시간으로 표시가 만료된다.
3초는 기존 BT 화면의 표시 정책이며, 실기체 센서 승인·융합·비행 임계값을 정한 것이 아니다.

## 구현 경계

- 기존 세션별 draft.4 telemetry를 읽는 조회 기능 확장. 새로운 DB 테이블·마이그레이션 없음.
- 기존 수신 인증, 현재 세션·기체·실행 문맥 검증, REPLAY 격리 유지.
- `UWB_INTEGRATION_REQUEST.md`의 장치 어댑터·시리얼·융합 구현을 추가한 것이 아님.
- 새 계약의 위치가 기존 기체 위치, 경로, 재고, 지도 마커를 자동으로 덮어쓰지 않음.
- 실제 기체 DB에 합성 보고를 넣지 않음. 현재 새 계약 보고가 없어 화면은 미수신 상태.
- 현재 지원은 REPLAY 보고이며 실제 PX4/Jetson 비행 연동이나 FLIGHT 승인을 의미하지 않음.

## 문서의 나머지 요구사항

DESIGN은 웹·Jetson·PX4 전체 설계 문서다. §5는 웹 개발 범위와 JSON/API를 WEB_REQUIREMENTS, WEB_CONTRACT_DRAFT, WEB_JSON_DRAFT에 연결한다. 다음 항목은 이번 위치 표시 수정으로 완료 처리하지 않는다.

| 항목 | 현재 상태 / 후속 범위 |
|---|---|
| 임무·지도 개정, 불변 배포본 | 기존 W02의 REPLAY 스냅샷 저장·배정 경계 구현. 실제 비행 중 버전 고정은 Jetson 연동 필요 |
| 이륙 높이, 선택 시작/도착 yaw, 이동점·scan 라벨 분리 | 스냅샷 계약 구조 존재. 운영자 편집 화면은 후속 웹 요구사항 작업에 남음 |
| 실제 장애물·천장·z 구역·비행 경계·승인 변환본 | 스냅샷 구조와 검증 존재. 편집 화면·실측 승인·Jetson 경로 검증 남음 |
| 시작·일시정지·재개·취소·착륙 명령 | 새 명령 계약·수락과 실제 결과 연결은 후속 작업. 이번에 실행 버튼을 추가하지 않음 |
| 출발 위치 동기화·융합/원시 위치 차이 판정 | 비교용 숫자 표시만 구현. 실제 지도 동기화와 허용 오차 판정은 승인 변환·측정값 필요 |
| RC/PX4 우선권, BT·경로·목표 출력·통신 단절 시 실행 지속 | BOOT/BT 상태 조회는 기존 구현. 실제 동작은 Jetson/PX4 소스·장치 시험 필요 |
| 로그 전송·보관·재시도 | 기존 BT 종료 결과의 ULog 상태 조회만 구현. 파일 수집·보관 정책은 별도 로그 연동 작업 |

사용자 검수 전 다음 문서를 구현하지 않는다. 목록상 다음 문서는 **DESIGN_REVIEW.md**.

## 수정 파일과 확인

- `dashboard/DroneStock-main/apps/core/position_status.py`: 세 출처의 표시용 정규화·독립 관측 만료.
- `dashboard/DroneStock-main/apps/core/behavior.py`: 기존 조회 API에 positions 추가.
- `dashboard/DroneStock-main/templates/drones/behavior.html`: 위치·관측·목표 영역.
- `dashboard/DroneStock-main/static/js/behavior.js`: 표시 및 브라우저 만료 처리.
- `dashboard/DroneStock-main/static/css/behavior.css`: 기존 화면 스타일에 맞춘 세 카드.
- `dashboard/DroneStock-main/apps/core/test_behavior.py`: 위치 구분·만료·미수신·유지/신규 관측 검사.

핵심 검사 10개 통과(실행 0.124초, 테스트 DB 준비 별도). JS 문법 검사 통과.
기존 로컬 8876 서버 재시작 후 로그인된 브라우저에서 새 영역을 확인했다. Vercel 배포 없음.

![로컬 DESIGN 위치 표시](design-local-2026-10-04.jpg)

## 추가 참고 — WEB_JETSON_INTEGRATION_HANDOFF_2026-10-04

사용자가 이번 검수 중 새 통합 패키지를 참고자료로 제공했다. 기존 문서 순서를 유지하며 다음 연동 작업에서 함께 확인한다.

읽은 자료: WEB_JETSON_INTEGRATION_REQUEST.md, WEB_JETSON_RUNBOOK.md, SERVICE_ARCHITECTURE.md, 구현 검수 보고서·소스 manifest, draft.4 telemetry 예제.

- 새 자료도 전체 목표 계약은 `1.1-draft.4`, 전체 웹 요청은 WEB_REDESIGN r2를 기준으로 명시한다. 위치 telemetry의 세 필드는 이번 표시 구조와 일치한다.
- 제공 보고서의 실제 Jetson 상태는 **HOST_OBSERVE / GET 전용 / flight_authority=false / 등록 요청 중**이다. 현재 서버를 실시간 재확인한 결과나 등록·쓰기 연동 완료 증거로 해석하지 않는다.
- **인증 차이:** 새 요청서는 METHOD/PATH/SHA/timestamp/nonce의 5줄 HMAC 후보를 제시한다. 현재 웹 draft.4 후속 요청·WS는 contract_version/control_session을 포함한 7줄 서명이다. 양쪽 서명 계약을 맞추기 전 연결 완료로 처리할 수 없다.
- **저장 확인 응답 차이:** Jetson outbox는 `stored/key/sha256` receipt를 요구한다. 현재 웹 execution_result 응답은 `ok/type/execution_id/result_revision`이다. Idempotency-Key와 원본 바이트 SHA, 저장 commit 이후 응답, 동일 key의 내용 충돌 처리까지 후속 조율·구현이 필요하다.
- **준비 검사 차이:** 서비스의 SVC_* 5개 점검은 웹의 BP/QS 전체 점검을 대신하지 않는다. 현재 웹 readiness 수신과의 구조 차이를 맞추고, HOST_OBSERVE와 REPLAY 준비 상태를 따로 표시해야 한다.
- 현재 Jetson REPLAY 지원은 허용 목록의 합성 waypoint + AUTO_TAKEOFF이며 scan·일반 지도 경로계획·RC_HANDOVER·SITL·FLIGHT 지원 완료가 아니다. 전체 계약 fixture의 예제 값만으로 기능 지원을 판단하지 않는다.
- 새 폴더에 runtime/guard 등 C++ 자료가 추가로 있다. 다만 파일 목록에는 manifest가 언급한 `python/sangwon_web/adapter.py`, `src/service/engine.cpp`, CMakeLists.txt 등 서비스·빌드 파일이 보이지 않는다. 전달본을 전체 실행 가능한 서비스 소스로 간주하지 않는다.
- 장치 등록, 키 생성·전달, Jetson 접속·배포, 비행 모드 변경은 이번 참고 확인에서 수행하지 않았다.

기존 BUILD_REPLAY 보고서의 “소스 없음”은 당시 받은 첫 패키지·웹 저장소를 조사한 결과다. 새 자료 수령 이후 C++ 참고 소스 존재 여부와 실행 서비스 전체 소스 제공 여부를 나누어 판단한다.
