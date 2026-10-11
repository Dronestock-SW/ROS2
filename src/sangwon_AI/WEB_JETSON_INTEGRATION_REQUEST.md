# 웹팀 요청: Jetson 등록과 실행 서비스 연동
이 문서는 Jetson에서 구현한 연동 경계와 웹팀의 다음 작업을 정리한다.
기존 전체 UI/API 개편 요청서와 함께 담당자에게 전달한다.

작성: 2026-10-04. 기준 계약 후보 `1.1-draft.4`.
전체 요구는 [WEB_REDESIGN_REQUEST_2026-10-04.md](WEB_REDESIGN_REQUEST_2026-10-04.md) r2가 기준이다.
이 문서는 그 요구를 축소하거나 서버 적용 완료를 선언하지 않는다.

2026-10-04 후속 수신: 웹팀 자동 점검/REPLAY 수신 구현 보고와 16:43 KST 실제 조회 결과는
[WEB_TEAM_PROGRESS_2026-10-04.md](WEB_TEAM_PROGRESS_2026-10-04.md)에 기록했다.
아래 최초 상태 표의 autonomy-state 404는 과거 값이며 현재 비인증 조회는 로그인 redirect 302다.
최신 W01/W02 명세를 반영했다. W01은 5줄, 이후 세션 요청은 7줄이다.
장치 등록 및 전체 명령 연동 완료를 뜻하지 않는다.
후속 Jetson 구현은 W01 필수 필드·7줄 서명·ACK 후보·receive_only를 추가했다. ACK 실제 규격과 장치키 등록은 확인해야 한다.
최신 웹 회신과 우선 등록할 별도 TEST 장치 정보는 [WEB_FOLLOWUP_2026-10-04.md](WEB_FOLLOWUP_2026-10-04.md)를 따른다.
검수 5차의 execution_result receipt 지원은 확인했으나 다른 사건/결과 경로 호환까지 완료된 것은 아니다.
사용자 정정: LoRa는 생존 확인만 담당하며, 변환 좌표는 Jetson이 Wi-Fi로 웹에 제공한다.
웹 시각화 추가 필드는 [TELEMETRY_CHANNEL_SPEC.md](TELEMETRY_CHANNEL_SPEC.md)를 따른다.

## 1. 지금 확인한 연결 상태

| 항목 | 확인/요청 |
|---|---|
| 실제 서버 | `http://203.247.41.82:8876` |
| Jetson 네트워크 | Wi-Fi 연결 확인, Jetson이 서버로 outbound 연결 |
| 현재 GET | `/api/drones/5/companion-mission/` 200, contract_version=1.1 |
| 현재 상태 조회 | `/api/drones/5/autonomy-state/` 404 |
| 자동 기동 | C++ core·Python adapter·host 진단 동작 |
| 현재 운영 모드 | HOST_OBSERVE, GET만 사용, 새 API POST/WS 미연결 |
| 개발 시험 | loopback mock HTTP/WS와 C++ REPLAY 연결 확인 |
| 웹팀 작업 | draft.4 계약·기체 등록·UI/API 구현 요청, 아직 회신/등록 전 |

`1.1` 데이터를 draft.4로 묵시 변환하지 않는다.
앵커 cm 값을 단순히 m로 바꾸는 것으로 측량/좌표 승인을 대신할 수 없다.
현재 응답의 `anchor_layout_valid=false`도 자동 비행 준비를 뜻하지 않는다.

## 2. Jetson이 생성한 등록 정보

| 필드 | 값 |
|---|---|
| drone_id | `5` |
| device_auth_id | `jetson-6b10e33185e44f9e8674d94c16857d48` |
| 등록 상태 | `REQUESTED_NOT_REGISTERED` |
| 요청 인증 | 기존 기체 HMAC v1 확장, 서버 확인 필요 |
| 요청 계약 | `1.1-draft.4` |
| 현재 실행 허용 | `NONE`, 실제 비행 권한 false |
| 시험 전용 ID | `TEST-DRONE-01`, 실제 drone 5와 격리 |

Jetson이 인증 비밀을 생성해 권한 0600의 `.runtime/private/device.env`에 저장했다.
비밀은 이 요청서·압축본·메모리·상태 API에 포함하지 않는다.
웹 관리자와 별도 안전한 전달/등록 경로를 정하고, **서버 등록 완료 여부를 회신**한다.
현재 observer는 이 비밀을 서버로 사용/전송하지 않는다.
등록 전 `drone_id=5`에 임의 인증 계정·권한이 생긴 것으로 간주하지 않는다.

요청: 장치 ID→기체 5 바인딩, 허용 API scope, 키 폐기/교체 방법, 테스트 기체 분리.
관리자 웹 비밀번호를 Jetson 서비스 인증으로 사용하지 않는다.
배포 시 HTTPS/WSS 주소·인증서 신뢰 경로도 제공한다.

## 3. 실제 연동 순서

1. Jetson C++가 boot/runtime ID·실행 프로파일·지원 capability를 제공한다.
2. Python이 새 control session을 협상한다. 서버는 현재 세션에만 신규 명령을 묶는다.
3. GET assignment와 snapshot_ref를 받는다. 전체 snapshot 다운로드와 길이/SHA 검사가 완료되어야 준비한다.
4. C++ 준비 결과를 preparation report와 readiness로 전송한다.
5. 웹은 현재 준비 ID·개정에 묶인 START를 생성한다. 생성 후 신규 수락 기한은 10초다.
6. Jetson은 명령 수락과 실제 실행 완료를 구분해 보고한다.
7. telemetry/readiness는 WS로, 사건·최종 결과는 저장 보장 HTTP로 전달한다.
8. 연결 복구 시 현재 상태부터 재조회한다. 전 세션의 미전달 START/RESUME/LAND를 다시 만들지 않는다.

이미 Jetson이 수락한 명령은 기존 ID로 결과를 조회한다.
응답 유실만으로 START ID를 새로 만들어 재전송하지 않는다.

## 4. API 구현/확인 요청

| API 후보 | 방향·용도 | 웹팀 요청 |
|---|---|---|
| POST `/api/drones/{id}/control-sessions/` | Jetson→서버 세션 협상 | 기체/boot/runtime/profile/계약 확인, 새 세션·서버 UTC 반환 |
| GET `/api/drones/{id}/companion-mission/` | 서버→Jetson 할당·명령 | assignment + snapshot_ref + 현재 세션 control_requests |
| GET `/api/mission-snapshots/{id}/content/` | 불변 자료 | 원본 UTF-8 바이트, 길이·SHA와 정확히 일치 |
| POST `/api/drones/{id}/preparation-reports/` | 준비 결과 | 성공/실패와 준비 ID·개정 저장 |
| POST `/api/drones/{id}/control-action/ack/` | 명령 결과 | ACCEPTED/REJECTED와 후속 revision 완료 결과 모두 보존 |
| POST `/api/drones/{id}/companion-phase/` | 사건 | 준비 무효·단계·이륙 관측·실행 종료·충돌/거부 사건 |
| WS `/ws/drones/{id}/` | Jetson 상태→서버→웹 | readiness/telemetry 구분, profile/source/시간/문맥 보존 |
| GET `/api/drones/{id}/autonomy-state/` | 웹 최신 상태 조회 | Jetson offline/오래된 READY 만료 표현 |
| POST launch-snapshot / scan-task-results | 후속 비행·스캔 결과 | 전체 계약대로 준비. 현재 서비스 생성은 미구현 |

**현 서비스가 시험하는 저장 receipt 후보:**

```json
{"stored":true,"key":"request-id:2","sha256":"<SHA256 of exact received body>","duplicate":false}
```

- Jetson은 `Idempotency-Key` 헤더와 원본 JSON 바이트를 반복 전송한다.
- 서버는 DB 저장 commit 후에만 `stored=true`를 반환한다.
- 같은 key+같은 body는 같은 저장 결과, 다른 body는 conflict다.
- 서버가 위 receipt를 다른 구조로 만들려면 계약을 함께 수정하고 양쪽 시험을 통과시킨다.
- 상태 WS 실패가 HTTP 결과 저장 완료를 취소하지 않는다. 미확인 결과는 Jetson outbox에 남는다.

HMAC 후보는 기존 v1 형식이다:
`METHOD + LF + PATH_WITH_QUERY + LF + SHA256(raw_body) + LF + unix_timestamp + LF + nonce`.
HMAC-SHA256 hex를 `X-DS-Signature`로 보내고 Device-ID/Timestamp/Nonce 헤더를 함께 사용한다.
서버의 허용 시계 오차·nonce 재사용 차단·WS handshake 인증 지원을 회신한다.
서명 시험과 실제 서버 등록/권한 시험은 별도다.

## 5. UI/UX 우선 변경

| 화면 | 변경 |
|---|---|
| 기체 상세/관제 | 연결됨, 계약 호환, 시스템 준비, 임무 준비, 실행 프로파일을 별도 표시 |
| 현재 HOST_OBSERVE | “Jetson 연결·진단 중 / 비행 준비 미완료”. START 비활성. 모의 READY와 실비행 READY 혼합 금지 |
| 임무 준비 | 검증된 plan/preparation/readiness에 종속. 변경/만료 시 기존 시작 버튼 승인 해제 |
| 시작 버튼 | fresh START 생성, 10초 진행 표시, ACCEPTED는 이륙 완료와 구분 |
| 실행 제어 | 서버 추측 대신 최신 allowed_commands 사용. 복귀/착륙에서는 PAUSE/RESUME 비활성 |
| 재연결 | 현재 세션·실행/결과 조회 후 안내. 과거 클릭을 자동 실행하지 않음 |
| 재시작 복구 | RECOVERY_LOCK와 미확정 기존 결과 표시, 임의 재개 버튼 제공하지 않음 |
| 오류 | code + 이유 + 조치. `WEB_CONTRACT_MISMATCH`, 미지원 capability, 준비/좌표/센서 실패 구분 |
| 스캔 작업 편집 | QR 라벨 좌표와 드론 좌표 구분, ArUco 고정 관계·라벨 정면·허용 작업 구역 제공 |
| 결과 | 비행 성공과 라벨 판독 성공 구분, 실패 후 다음 목표 정책 보존 |

## 6. 지금 웹에서 제공해야 하는 것과 Jetson 생성값

| 웹/지도 담당 제공 | Jetson 생성/판단 |
|---|---|
| 기체 등록·인증 권한·URL·계약 지원 | device ID/비밀 생성, boot/runtime 식별자 |
| assignment·불변 mission/map snapshot·측량 참조 | 수신 검증·준비 결과·실행/flight 식별자 |
| 구조화 waypoint 및 scan 작업, 목표 yaw/이륙 높이 | 유효 경로·정지/회전/스캔 행동 선택 |
| 라벨 정면·ArUco↔QR 배치·외곽/높이·접근 허용 구역 | ArUco 관측 기반 미세 조정과 스캔 판독 결과 |
| 지도 경계·금지 구역·장애물·yaw 검증 구역 | 센서/위치/상태 freshness 및 preflight 판정 |
| 운영자 확인과 새로운 유효 명령 | PX4/RC 우선권 보존, 수락·실행·실패 기록 |

UWB 전체 사양은 별도 수령 대기다. 웹 필드만으로 UWB 융합 완료를 가정하지 않는다.

## 7. 웹팀 회신 양식

```text
담당자/적용 예정일:
테스트 서버 HTTP(S)/WS(S) URL:
지원 contract_version / capability:
device_auth_id 등록 여부 / drone_id 바인딩:
비밀 전달 경로 / 인증 scope / 키 폐기 방법:
control-session 요청·응답 예시:
mission GET 및 원본 snapshot 예시:
명령 10초/세션/중복/재연결 처리:
durable receipt 구현 또는 변경 제안:
autonomy-state / readiness UI 반영:
지도 경계·라벨 정면·작업 허용 구역 제공 일정:
미구현/협의 항목:
```

## 8. 첫 공동 인수 범위

물리 출력 없는 TEST 기체로 세션→snapshot→준비→새 START→모의 완료를 검수한다.
추가로 중복 START, 만료 START, 이전 세션 명령, Wi-Fi 단절/복구, 결과 응답 유실,
Jetson core 재시작, 미지원 scan 전체 거부를 시험한다.
이 인수는 실기체 비행·UWB 정확도·QR 작업 성공 검증을 대신하지 않는다.

서비스 구조: [SERVICE_ARCHITECTURE.md](SERVICE_ARCHITECTURE.md).
실행 절차: [WEB_JETSON_RUNBOOK.md](WEB_JETSON_RUNBOOK.md).
