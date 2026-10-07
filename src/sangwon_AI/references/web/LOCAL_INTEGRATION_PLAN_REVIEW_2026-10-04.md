# 로컬 검토 9차 — INTEGRATION_PLAN.md

원본: `C:/Users/Lee/Desktop/WEB_TEAM_HANDOFF_2026-10-04/INTEGRATION_PLAN.md`

**상태: 웹 측 연동 대응표·샘플 목록·시험 서버 정보 정리 완료. 웹↔Jetson 공동 연동 완료는 아님.**

검토 기준: 웹 `main`의 `0ca0eb0`, 2026-10-04. 아래 상태는 이 시점의 코드·기존 검수 기록·제공 자료를 대조한 결과다.

## MD 내용 간단 요약

- 웹 계약·JSON·시험 서버, UWB 사양, Jetson 환경, PX4 시험, 측량 자료를 누가 준비할지와 선행 관계를 정한다.
- 내부 구현과 합성 시험은 진행할 수 있지만, 실제 연동 완료에는 양측의 같은 계약·샘플·검수 증거가 필요하다.
- 실기체 비행은 실측 설정과 시험 승인 뒤 별도로 허용하며, 미확정 수치를 운영 기본값으로 채우지 않는다.

## 이번 산출물과 화면

이번 문서는 새 화면 명세가 아닌 연동 준비 계획이다. 웹 기능을 추가하지 않고 아래 W01~W18 대응표, D01~D10 자료 목록, 연동 선행 조건을 정리했다. 이전 요청서와 인증 문서에도 최신 현황을 찾을 수 있는 링크를 추가했다.

- 기존 자동 점검: http://203.247.41.82:8876/platform/drones/5/preflight/
- 기존 비행 행동 상태: http://203.247.41.82:8876/platform/drones/5/behavior/
- 이번 단계의 새 화면·제어 버튼·장치 등록·배포 없음. 검수 대상은 이 대응표다.

## W01~W18 대응표

‘웹 구현’은 해당 웹 부분의 구현 상태다. 요청자 수락, Jetson 연결 및 실제 비행 시험 완료를 뜻하지 않는다. 역할은 작업 구분이며 실제 담당자·일정의 확정이 아니다.

| ID | 현재 웹 대응 | 남은 작업·역할 / 근거 |
|---|---|---|
| W01 | REPLAY용 draft.4 버전·capability 협상, 기체 인증 범위, 세션 교체 구현 | 웹·Jetson: 실제 등록 및 서명 합의·공동 시험. [W01 보고서](WEB_W01_REVIEW_2026-10-04.md) |
| W02 | 전체 snapshot 검증·원본 저장·SHA256/길이 보존·서명 다운로드 구현 | 웹·Jetson: 최소 REPLAY snapshot과 전체 웹 snapshot 차이 해결, 실제 수신 시험. [W02 보고서](WEB_W02_REVIEW_2026-10-04.md) |
| W03 | snapshot에서 waypoint 기체 XYZ와 scan 라벨 참조·순서·이륙 높이·체류 값 검증 | 웹: 해당 계약의 편집·발행 UI와 임무 배정 수명주기는 남음. 기존 구형 편집기를 새 계약 완료로 간주하지 않음 |
| W04 | snapshot의 선택 yaw와 0 구분, 행동 화면의 목표 yaw·현재 heading 구분 | 웹·측량·Jetson: 편집 UI·실측 지도 각도 기준 공동 확인. [용어 보고서](LOCAL_GLOSSARY_REVIEW_2026-10-04.md) |
| W05 | snapshot의 장애물·천장·고도 구역 외곽/z 구조·참조 검증과 합성 샘플 있음 | 웹: 새 지도 편집 UI. 측량·Jetson: 실제 형상과 경로·충돌 검사. 구조 검증은 물리 경로 승인과 별개 |
| W06 | draft.4 제어 요청 생성·ACK 원장·요청 ID별 결과 조회 미구현. 임무 조회의 control_requests는 빈 배열 | 웹: 명령/결과 원장과 최신 준비에 묶인 제어 UI. Jetson: 같은 계약으로 10초·세션·단계·중복 공동 시험 |
| W07 | telemetry 단계·착륙 관측과 execution_result의 비행/작업 결과를 구분해 저장·표시 | 웹: 명령 ACK/개정 이력은 남음. [BT 보고서](LOCAL_BT_SPEC_REVIEW_2026-10-04.md) |
| W08 | PX4 위치·UWB 원시 관측·현재 목표, 출처·시각·만료·미수신 분리 표시 | Jetson·측위: 실제 품질 보고와 공동 확인. UWB 장치 어댑터는 이번 웹 작업 범위 밖. [DESIGN 보고서](LOCAL_DESIGN_REVIEW_2026-10-04.md) |
| W09 | 원본 배포본 불변, 현재 시험 배정 교체 거부 | 웹·Jetson: QUEUED 이후 실행/PAUSED/완료 배정 수명주기 공동 시험은 남음 |
| W10 | 합성 승인·지도/기체 프로파일·변환 개정 참조 검증 및 저장 | 웹·측량·운영: 실제 승인 작성·확인 UI, 실측 자료와 승인 유효성 확인은 남음 |
| W11 | 세션 교체·기존 보고 문맥/순서 거부, 종료 결과 중복·재전송 저장 구현 | 웹: 명령 영속 원장·오프라인 명령/재시작 정책 완성. Jetson: 실제 재연결 시험. 종료 결과 receipt를 명령 중복 방지 완료로 간주하지 않음 |
| W12 | 점검 항목의 보고된 기록/공간 상태와 종료 결과의 ULog 수집 상태 표시 | Jetson·웹: 실제 기록 건강 정보·자동 수집/보존 연동. [결과 저장 보고서](LOCAL_DESIGN_REVIEW_ITEMS_2026-10-04.md) |
| W13 | snapshot에서 경계·금지 구역 미제공/제공 구분, global_detour_enabled=false | 웹: 지도 편집/제공 UI. Jetson: 경계·기하 계획 시험. 자동 우회 구현 완료 아님 |
| W14 | 합성 JSON 4개, 로컬 서버, 인증/등록 설정 절차와 격리 테스트 코드 있음 | 웹·Jetson: 실제 TEST 기체 등록, HTTPS/WSS 주소, 같은 fixture의 양방향 인수 시험은 남음 |
| W15 | 자동 점검 33항목, 부팅/원천 상태·원인·조치 표시 및 서명 WS 수신 | Jetson: 실제 전체 점검 보고 연결. [BOOT 보고서](LOCAL_BOOT_PREFLIGHT_REVIEW_2026-10-04.md) |
| W16 | 시스템 준비와 임무 준비 분리, 현재 웹 시작 버튼 비활성 | 웹·Jetson: 준비 검증/명령 경로 연결 뒤 시작 허용 조건 공동 검수 |
| W17 | readiness의 현재 boot/runtime/control·배정 문맥과 관측/보고 만료 검사 | 웹·Jetson: 준비 POST 및 START 최종 재검사 연결은 남음. [관측 품질 보완](LOCAL_IMPLEMENTATION_BASELINE_REVIEW_2026-10-04.md) |
| W18 | 현재 화면의 상태 전환 안내·중복 억제·오래된 배지 해제 | 웹: 영속 사건 이력과 브라우저 재접속을 포함한 알림 중복 방지 완성. Jetson: 준비 해제 사건 공동 시험 |

## D01~D10 자료 목록

| ID | 제공 가능한 자료 | 남은 범위 |
|---|---|---|
| D01 | 위 대응표와 아래 기존/확장 연결점 표, [인증·세션 명세](WEB_SERVER_W01_W02_AUTH_SESSION.md) | 미구현 명령·사건·준비 API의 전체 필드 대응과 양측 합의 |
| D02 | [mission_full.json](../dashboard/DroneStock-main/apps/core/testdata/web_draft4/mission_full.json), [snapshot_content.json](../dashboard/DroneStock-main/apps/core/testdata/web_draft4/snapshot_content.json) | 현재 파일은 합성 QUEUED 예제. 실제 준비·단일 좌표 등 전체 시나리오별 서버 캡처는 남음 |
| D03 | snapshot_content.json의 map: 장애물·천장·고도 구역·경계 상태 | 실제 창고 측량값·운영 승인 지도 |
| D04 | W01/W02 인증·충돌 오류 및 관련 테스트 코드 | 새 제어 요청/ACK·중복·만료·선점 API 샘플은 미구현 |
| D05 | [telemetry.json](../dashboard/DroneStock-main/apps/core/testdata/web_draft4/telemetry.json), [execution_result.json](../dashboard/DroneStock-main/apps/core/testdata/web_draft4/execution_result.json) | 복귀/RC 인계/단절 등 전체 상태·사건 시나리오와 사건 저장 API |
| D06 | snapshot의 approvals·transforms·yaw_validation_zones 및 개정 참조 | SYNTHETIC_ONLY/REPLAY 예제다. 실제 운영자 확인과 승인 자료는 미확정 |
| D07 | 아래 서버·인증 접속 정보 및 W01/W02 문서 | 실제 시험 장치 등록·비밀 전달·외부 접속 인수 확인 |
| D08 | 이 보고서의 대응표·담당 역할·선행 조건 | 실제 담당자와 확인 예정일은 미확정. 사용자의 MD별 검수 순서에 맞춰 진행 |
| D09 | [test_readiness.py](../dashboard/DroneStock-main/apps/core/test_readiness.py)의 합성 준비/차단/만료·문맥 오류 입력 | 독립 파일 형태의 전체 상태별 JSON 묶음과 실제 Jetson 보고 캡처 |
| D10 | BOOT·BASELINE 보고서의 화면 및 회귀 검증 기록 | 명령 API 연결 뒤 최신 준비에 묶인 START와 브라우저 재접속 공동 인수 |

위 JSON은 테스트용 합성 원문이다. `control-demo-01` 같은 설명용 ID·과거 시각·가짜 센서 값을 실제 장치 상태로 제출하지 않는다. 실제 협상에서 발급받은 문맥을 사용해야 하며, 배포본을 바꾸면 원본 길이·해시도 다시 계산해야 한다.

## 기존 API와 확장 연결점

| 연결점 | 현재 draft.4 처리 |
|---|---|
| POST control-sessions | 신규 REPLAY 세션 협상, HMAC 5줄 |
| GET companion-mission | 계약/현재 세션 헤더로 확장 분기, HMAC 7줄. assignment·snapshot_ref 제공, 제어 요청/확인/재점검 배열은 비어 있음 |
| GET mission-snapshots/{id}/content | 신규 불변 원문 다운로드, HMAC 7줄 |
| WS drones/{id} | 서명된 handshake와 현재 문맥으로 readiness/telemetry 수신. 구형 UWB 수신에 새 계약 보고를 섞지 않음 |
| GET autonomy-state / behavior-state | 로그인·기체 조회 권한을 사용하는 브라우저용 상태 API. 장치 HMAC만으로 호출하는 API가 아님 |
| POST companion-phase | draft.4 execution_result 저장 및 Idempotency-Key receipt 구현. 모든 event/preparation/scan 자료를 처리하는 공통 수신기는 아님 |
| POST control-action | 로그인 운영자용 구형 요청 경로 유지. draft.4 세션·준비·10초 기한 검증을 갖춘 새 명령 API는 미구현이며, 이 경로를 새 계약 START에 사용하지 않음 |
| control-action/ack·launch-snapshot | 구형 장치 경로 존재. 새 계약을 구형 처리로 넘기는 요청은 차단하며 draft.4 구현 완료로 안내하지 않음 |
| preparation-reports / scan-task-results | 새 계약 API 미구현 |

세부 필드는 [control_sessions.py](../dashboard/DroneStock-main/apps/core/control_sessions.py), [mission_snapshots.py](../dashboard/DroneStock-main/apps/core/mission_snapshots.py), [readiness.py](../dashboard/DroneStock-main/apps/core/readiness.py), [behavior.py](../dashboard/DroneStock-main/apps/core/behavior.py), [api_urls.py](../dashboard/DroneStock-main/apps/core/api_urls.py)를 기준으로 확인했다.

## 시험 서버·인증 정보

- HTTP 기본 주소: `http://203.247.41.82:8876`, 화면은 `/platform/`, API는 `/api/`.
- 장치 상태 WS 경로: `ws://203.247.41.82:8876/ws/drones/{drone_id}/`. 주소와 경로 안내이며 이번에 Jetson WS 연결을 수행한 것은 아니다.
- 계약 `1.1-draft.4`, 웹 수신 프로파일 `REPLAY`, `flight_authority=false`.
- 합성 샘플 ID는 `TEST-DRONE-01`. 실제 기체 5에 합성 자료를 넣지 않는다. 이번 단계에서 시험 기체·키·세션을 생성하지 않았다.
- 관리자가 `DEVICE_AUTH_KEYS`와 `AUTONOMY_REPLAY_DEVICES`의 인증 장치→시험 기체 범위를 등록해야 한다. 키 교체/폐기도 관리자 설정 절차이며 별도 키 관리 UI 완료로 표시하지 않는다.
- W01은 5줄 HMAC, 이후 확장 HTTP/WS는 계약 버전·현재 제어 세션까지 포함한 7줄 HMAC. nonce 재사용 차단. 상세는 인증·세션 명세 참조.
- 관리자 로그인 비밀번호는 장치 인증에 사용하지 않는다. 비밀값은 보고서/샘플에 싣지 않는다.
- HTTPS/WSS 배포 주소·인증서 신뢰 경로는 아직 이 로컬 인수 자료에서 확정하지 않았다.

## Jetson과 맞춰야 할 선행 조건

추가 자료 `WEB_JETSON_INTEGRATION_HANDOFF_2026-10-04`와 현재 웹 코드를 대조했다. 아래는 이번에 해결됐다고 표시할 수 없는 차이다.

| 항목 | 차이 및 다음 산출물 | 역할 |
|---|---|---|
| 인증 | Jetson 제안은 5줄 서명, 웹의 세션 이후 경로는 7줄. HTTP/WS 동일 서명 fixture 합의 필요 | 웹·Jetson |
| snapshot | Jetson 최소 예제는 replay_waypoints_v1·합성 바닥 기준, 웹은 전체 draft.4 지도/승인/변환 구조. 버전 문자열만 같다고 호환되지 않음 | 웹·Jetson |
| 준비 | Jetson SVC_* 5개와 웹 BP/QS 33항목의 보고 범위가 다름. 전용 준비 POST도 남음 | Jetson·웹 |
| 명령 | 웹의 control_requests는 빈 배열. 준비→START→ACK→결과 조회 경로를 구현해야 전체 모의 임무 공동 인수 가능 | 웹·Jetson |
| receipt | 웹은 execution_result에 stored/key/sha256/duplicate 제공. 사건·준비·명령 결과 전체 저장 확인으로 확대했다고 간주하지 않음 | 웹·Jetson |
| 등록·접속 | 새 인수 문서는 등록 요청·HOST_OBSERVE GET 상태를 기록. 현재 등록 완료·쓰기 연동 성공의 증거로 사용하지 않음 | 웹 관리자·Jetson |
| 실제 비행 | 측량·PX4·전체 preflight·실측 승인 및 실패 시험은 별도. UWB 어댑터는 사용자 지정 웹 작업 범위에서 제외 | 비행·측위·운영 |

공동 검수 순서는 **계약·서명·snapshot 합의 → 격리 TEST 기체 등록 → 세션/전체 파일/해시 → 준비 보고 → 새 명령/ACK/모의 완료 → 단절·재시작·중복 시험**이다. 각 단계의 입력·기대 결과·원문 증거를 확보한 뒤 완료로 바꾼다. 새 capability를 이름만 추가하거나 누락된 지도/승인을 임의 기본값으로 채워 통과시키지 않는다.

## 이번 확인 결과

- 저장소의 합성 JSON 4개 구문 및 `1.1-draft.4` 버전 확인.
- snapshot 원본 **10,699바이트**, SHA256 `5419d8a9a9cd20e54f79d4bfc3551bf0e7894b647d94685763172ef741cf1909`. mission_full의 길이·해시와 일치.
- 로컬 자동 점검 URL을 비로그인으로 조회하면 로그인 화면으로 이동하고 최종 HTTP 200 확인. 장치 인증·Jetson 네트워크·공동 인수 성공을 검증한 것은 아니다.
- 앱 코드·DB 변경이 없어 서버 재시작·전체 테스트 재실행 없음. 과거 테스트 통과 기록은 각 원래 보고서에 연결했다.
- 외부 담당자에게 메시지·자료를 전송하지 않았고, 담당자 수락이나 예정일을 임의 확정하지 않음.
- 사용자 지시에 따라 이 문서 단위 보고서와 최신 현황 링크를 커밋·푸시한다.

다음 문서는 **JETSON_DEPLOYMENT.md**. 사용자 검수 후 진행한다.
