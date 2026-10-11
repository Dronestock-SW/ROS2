# 로컬 검수 5차 — DESIGN_REVIEW.md

원본: `C:/Users/Lee/Desktop/WEB_TEAM_HANDOFF_2026-10-04/DESIGN_REVIEW.md`

추가 근거: `C:/Users/Lee/Desktop/WEB_JETSON_INTEGRATION_HANDOFF_2026-10-04/WEB_JETSON_INTEGRATION_REQUEST.md` §4 저장 receipt, SERVICE_ARCHITECTURE.md §4 영속성.

**상태: RV01~RV13 대조 완료. RV12 중 실행 종료 결과의 서버 저장 확인·재전송 처리 보완 완료. 스캔 결과·재고 업무 연동 등 미결 항목은 남음.**

검수 화면: http://203.247.41.82:8876/platform/drones/5/behavior/#execution-results

## 사용자가 보는 변경

기존 **비행 행동 상태 → 실행 종료 결과** 아래에 두 항목을 추가했다.

- **서버 결과 저장:** 실제 DB에 저장된 실행 결과의 개정과 저장 시각. 결과가 없으면 저장된 결과 없음.
- **재고 업무 반영:** 별도 처리·미연결 표시. 비행 성공·QR 판독 성공·서버 결과 저장을 재고 반영 성공으로 바꾸지 않는다.

장치의 오프라인 보관·전송 대기는 보고 필드가 없어 미확인으로 표시한다. 테스트 숫자를 실제 기체에 넣지 않았으므로 현재 화면은 종료 보고 대기 상태다.

## 서버 변경 — RV12

적용 API는 기존 `POST /api/drones/{drone_id}/companion-phase/`의 draft.4 **execution_result** 수신이다.

`Idempotency-Key`가 있는 요청에 다음 저장 확인 형식을 지원한다.

```json
{"stored":true,"key":"execution-test:2","sha256":"<수신한 원본 바이트의 SHA256>","duplicate":false}
```

- 키는 공백 없는 ASCII 1~200자. 장치 outbox에서 만든 원래 키와 원본 바이트를 재전송한다.
- 기체별 키를 유일하게 저장하며 장치 ID를 함께 보존한다.
- 결과 개정과 원본 바이트·SHA·저장 키를 한 DB 트랜잭션에 보존한다. commit이 끝난 후에만 `stored=true` 응답을 반환한다.
- 같은 장치·기체·키·원본이면 `duplicate=true`, 같은 SHA를 반환한다. 결과를 중복 저장하거나 저장 시각을 갱신하지 않는다.
- 같은 키에 다른 원본 바이트 또는 다른 장치면 HTTP 409 `IDEMPOTENCY_CONFLICT`. JSON 공백 차이도 원본 차이로 취급한다.
- 같은 실행·개정의 다른 내용은 기존 결과 검증에서 거부한다. 상위 결과 개정은 별도 보존한다.
- 현재 세션으로 인증한 재연결 요청은 해당 장치·기체의 이미 등록된 폐기 세션에서 생성된 종료 결과도 원래 문맥에 저장할 수 있다. 미수신 outbox 결과와 저장 응답 유실 재전송을 모두 처리한다.
- 과거 결과를 현재 비행의 telemetry·readiness로 바꾸거나 이전 명령을 실행하지 않는다. 원래 배정·배포본 검증은 유지한다.
- 서버가 모르는 세션·다른 장치의 세션·유효하지 않은 종료 결과는 저장하지 않는다. 이전 세션을 HTTP 인증 헤더로 쓰는 요청도 거부한다.
- 헤더가 없는 기존 draft.4 호출에는 기존 응답 구조와 현재 세션 검증을 유지한다. 새 receipt를 사용하려면 헤더를 추가해야 한다.
- HMAC은 현재 웹의 **7줄 서명**을 유지한다. 인증 헤더의 세션은 현재 세션이며, 과거 결과 본문의 실행 문맥은 변경하지 않는다.

## RV01~RV13 대조

| 항목 | 현재 반영 / 남은 작업 |
|---|---|
| RV01 마커·QR 고정 관계 | snapshot에 ArUco·장착 변환 참조 구조 존재. 실제 상대 배치 치수 수령·입력 UI 남음 |
| RV02 기체/라벨 XYZ 분리 | snapshot의 waypoint/scan 구분과 DESIGN 위치 카드 반영. 운영자 편집 화면은 후속 웹 요구사항에 남음 |
| RV03 라벨 작업 허용 구역 | scan_workspace 참조·라벨 연결 검증 존재. 실제 구역 제공·편집 및 Jetson 공간 검증 남음 |
| RV04 Scan 단계 | BT 화면에 B21 단계·의미 표시. 장치 ScanTask 구현은 별도이며 최신 Jetson 문서도 scan 미지원 |
| RV05 스캔 후 이탈 | BT 화면에서 판독과 이탈/복귀를 구분. 장치의 실제 경로 재검사·이탈 동작은 후속 |
| RV06 지연 QR 판독 | 스캔 시도·관측 시각·세션 경계와 scan-task-results 수신 구현 남음 |
| RV07 카메라 상실 | BOOT의 카메라 검사와 BT 스캔 상태 표시 존재. 카메라 고장·이탈 정책 장치 동작 검증 남음 |
| RV08 시간 한도 | QS-M1~M5 측정·승인 필요. 임의 시간값 추가하지 않음 |
| RV09 근접 스캔 안정 기준 | 별도 상대 정렬·각속도·안정 시간 승인 필요. 이동 yaw 허용값을 재사용하지 않음 |
| RV10 대상 QR·마커 매핑 | snapshot의 label/expected_qr_ref/marker 구조·참조 검증 존재. 실제 샘플·정답 자료와 편집 UI 남음 |
| RV11 Jetson 환경 | 새 전달 보고서는 CTest 7/7·HOST_OBSERVE를 보고함. 이 로컬 웹 작업에서 장치 환경·실비행 준비를 검증한 것은 아님 |
| RV12 저장/수락/업무 성공 | 이번에 execution_result 원본 저장 확인·재전송·충돌 처리와 서버 저장/재고 반영 분리 표시 보완. 개별 스캔 수신·재고 판정·장치 outbox 상태 표시는 남음 |
| RV13 라벨 정면 | snapshot의 outward_normal_map과 단위 벡터 검증 존재. 실측 방향·입력 UI·접근 공간 승인 남음 |

검수 기록의 미결 항목 전체를 구현 완료로 바꾸지 않는다. 필요한 실측값·실제 QR 자료도 합성 예제로 대신하지 않는다.

## 확인 및 운영 반영

- 관련 검사 15개 통과(0.176초, 테스트 DB 준비 별도), JS 문법 검사 통과.
- 신규 검사: 정확한 원본 SHA, 재전송 중복, 같은 키의 바이트 충돌, 개정 보존, 재연결 후 과거 결과 수신, 다른 장치/모르는 세션 거부, 저장 실패 시 전체 롤백.
- 기존 SQLite를 `.runtime/design-review-before-0026.sqlite3`에 백업한 뒤 `0026_behavior_result_receipt` 적용.
- 8876 로컬 서버를 재시작하고 로그인된 브라우저에서 변경 영역 확인.
- 실제 Jetson adapter와의 왕복 인수는 미실시. 새 전달 문서의 5줄 HMAC 후보와 현재 웹 7줄 HMAC 차이는 여전히 조율 대상이다.
- preparation/ack/일반 사건/launch-snapshot/scan-task-results용 receipt까지 구현한 것은 아니다.
- 서버 기록 자동 삭제 정책은 추가하지 않음. Jetson의 ULog 72시간 정책을 결과 원장에 적용하지 않음.

변경 코드: `apps/core/behavior.py`, `apps/core/models.py`, `apps/core/migrations/0026_behavior_result_receipt.py`, `apps/core/test_behavior.py`, `templates/drones/behavior.html`, `static/js/behavior.js`, `static/css/behavior.css` (모두 `dashboard/DroneStock-main/` 기준).

목록상 다음 문서는 **GLOSSARY.md**. 사용자 검수 후 진행한다.

## 이후 커밋·푸시 진행 규칙

2026-10-04 사용자 지시: 현재까지 완료한 작업을 커밋·푸시하고, 이후에도 MD별 작업 종료 때 자동으로 커밋·푸시한다. 변경 내용·남은 항목·검수 화면과 함께 커밋 및 푸시 결과를 보고한다. 다음 MD 착수는 기존의 사용자 검수 순서를 유지한다.

![로컬 결과 저장 구분](design-review-local-2026-10-04.jpg)
