# 웹·Jetson 합성 계약 예제
이 디렉터리는 웹과 Jetson이 같은 데이터로 개발하는 자료다.
요청서 검토와 격리된 mock 연동에 사용한다.

계약 후보: `1.1-draft.4` / 패키지 r2 / 전부 합성 REPLAY.
기준: [웹 전체 개편 요청서](../../WEB_REDESIGN_REQUEST_2026-10-04.md).
실제 서버에 POST하거나 실기체로 실행할 자료가 아니다.
고정 과거/시험 시각을 쓴다. 실제 인증 정보는 포함하지 않는다.

선택 기능 `replay_static_detour_v1`: 서버/Jetson 세션 양쪽에서 지원해야 한다.
snapshot의 `policy.global_detour_enabled=true`에는 해당 required capability,
`flight_boundary.status=PROVIDED`와 비어 있지 않은 volumes,
`no_fly_zones.status=PROVIDED`(장애물이 없으면 빈 목록)가 필요하다.
기존 예제의 false 정책과 고정 바이트/해시는 유지한다. 원본을 현장에서 바꾸지 않는다.
Jetson에는 승인된 시험 설정 `approved_replay_planner`를 설치한다:
resolution_m=0.25, max_nodes=8192, max_expansions=4096, max_length_m=50, budget_ms=50.
이 설정·허용 snapshot SHA가 없으면 준비를 거부한다. 경로는 현재 고도에서만 생성하며 필수 목표를 대체하지 않는다.
scope는 `ALLOWLISTED_SYNTHETIC_MAP_STATIC_DETOUR_ONLY`이고 실비행/실시간 LiDAR 승인과 구분한다.
내부 IPC `service_status.navigation`은 개정·최근 실패 사건을 제공한다.
웹 outbox의 `event_type=ROUTE_PLANNED`에는 route_revision·visited_tasks·합성 scope를 기록한다.
실제 경로 선분 배열/시각화 API와 LiDAR 장애물 갱신 계약은 후속이다.

## 1. 웹팀 요청 9종

| 파일 | 의미 |
|---|---|
| [mission_full.json](mission_full.json) | GET companion-mission 응답. 아래 불변 snapshot 파일을 해시로 참조 |
| [command_start.json](command_start.json) | 지상 AUTO_TAKEOFF 모의 START, 신규 수락 10초 |
| [command_active.json](command_active.json) | PAUSE/RESUME/CANCEL/LAND_NOW 각각의 대안 분기 |
| [result_accepted.json](result_accepted.json) | START 수락, 실행 ID 생성, 실제 이륙 전 flight_id=null |
| [result_rejected.json](result_rejected.json) | 수락 전에 준비 상실한 대안 분기. 같은 START가 이미 수락된 시나리오와 혼합 금지 |
| [telemetry.json](telemetry.json) | 스캔 중 모의 위치·제어권·스캐너 시도·원천 시각 |
| [readiness_ready.json](readiness_ready.json) | 33개 체크. 모의 실행 준비, flight_authority=false |
| [readiness_blocked.json](readiness_blocked.json) | 위치 UNKNOWN으로 준비 해제. preparation=null, can_start=false |
| [event_ready_revoked.json](event_ready_revoked.json) | 이전 준비 ID·개정을 무효화하는 사건 |

9개는 하나의 임무·기체 문맥을 공유한다.
상호 배타적인 성공/거부 예제를 전부 한 번에 순서 실행하지 않는다.
`manifest.json.scenarios`가 분기를 지정한다.

## 2. 전체 자료와 보조 예제

| 파일 | 의미 |
|---|---|
| [snapshot_content.json](snapshot_content.json) | 이동/scan/이동 3개 작업, 지도·라벨·구역·합성 변환·승인·참조 |
| motion-demo.json / scan-demo.json / body-demo.json | Jetson에 설치한 것으로 가정하는 REPLAY 설정 파일. 실측 수치 아님 |
| control_session.json | 서버 세션 협상 응답 후보 |
| command_submit.json | 브라우저→서버 최초 요청. client_request_key 보존 |
| plan.json | 모의 계획 보고. 기하/동역학 검증을 실제 수행한 계획은 아님 |
| operator_attestation.json | 계획·출발 문맥에 묶인 합성 운영 확인서 |
| preparation_report.json | 같은 계획/준비 ID를 가리키는 Jetson 보고 |
| launch_snapshot.json | 모의 실제 이륙 사건. 바닥 위치와 기체 기준점 높이를 구분 |
| scan_task_failed.json | 최종 판독 실패 저장, 대기점 복귀 중, revision 1 |
| scan_task_completed.json | 대기점 복귀 후 동일 결과 revision 2. 판독 결과는 여전히 실패 |
| scan_result_stored.json | 서버 저장 성공. 실패 판독이므로 업무 반영은 NOT_APPLICABLE |
| execution_result.json | 복귀·착륙 성공, 작업 2 성공·1 실패. 전체 작업 성공 아님 |
| manifest.json | 파일 목록·길이·SHA256, 합성 시나리오 |

`mission_full.json`에 전체 내용을 중복 삽입하지 않았다.
DEVICE_SNAPSHOT 응답은 `snapshot_content.json`의 실제 바이트다.
snapshot_ref의 byte_length/SHA256은 그 파일과 일치해야 한다.
서버는 파일을 재직렬화하거나 개행을 바꾸지 않고 제공한다.
프로파일 파일도 각 실제 파일 바이트의 해시로 대조한다.
`manifest.profile_files`는 이 묶음의 테스트 파일 위치다.
실제 서버 계약에서는 파일 이름으로 장치 경로를 지정하지 않고
승인 프로파일 ID/개정/해시로 Jetson 설치본과 대조한다.

## 3. 자료의 의도와 미확정 영역

- profile=REPLAY, 지원 실행=REPLAY_ONLY, 물리 출력 권한=false를 유지한다.
- fixture의 can_start=true는 격리된 모의 실행만 허용한다.
- 예제 센서·시간·거리·속도·안정 수치와 approval은 모두 합성이다.
- camera/좌표 변환은 mock 관측의 형태 예시다. 실제 광학 축·PX4 원점·UWB 측량을 대신하지 않는다.
- plan에는 모의 대기점/판독점/기록 복귀가 있다. 실제 통과 여유·바닥 접촉·이착륙 고도 예외는 승인된 비행 프로파일로 검증해야 한다.
- 도착/제동/센서 나이·시계 오차·FLIGHT 준비 수명은 실측 승인 대기다.
- JSON 구조 후보와 비행 정책의 확정을 구분한다. 웹팀과 API URL/필드/오류 대응 합의 후 버전을 고정한다.
- 이 묶음은 OpenAPI 또는 전체 JSON Schema를 제공한다고 주장하지 않는다. 아래 필드 계약과 실제 예제를 먼저 대조한다.

## 4. 필드 해석

외부 객체는 `contract_version`과 `type`을 함께 검사한다.
장치/시험 구분에는 `drone_id`와 `profile`을 사용한다.
프로파일 파일은 wire 메시지가 아닌 설치 설정 예제다.

| 객체 | 필수 값 | 선택/null 규칙 |
|---|---|---|
| context | boot/runtime/control_session, assignment/snapshot | execution·flight는 수락/이륙 전 null. preparation/revision은 START에서 필수, 실행 중 제어에는 null |
| control_request | 문자열 ID, 정수 seq≥1, action, UTC created/expires, context, payload | START payload.start_mode 필수. 나머지는 현재 빈 객체 |
| waypoint | task_id, type, position_m{x,y,z}, hold_s≥0 | yaw_deg 생략=이동 방향. 0°와 구별. null 입력은 이번 후보에서 거부 |
| scan | task_id, label_point_id, workspace_id, scan_profile_ref, approach_anchor | 기체 XYZ를 입력하지 않음 |
| geometry | id/revision, polygon_xy_m, z_min_m, z_max_m | 단순 다각형 3점 이상. 닫는 첫 점 반복 없음. 유한 수만 허용 |
| label | label_point_id/revision, position/reference, 단위 normal, expected_qr_ref, aruco | 정면 미상은 초안 저장 가능, 배포/준비는 거부 |
| readiness | 시간·상태·출력권한·context·checks·허용 명령 | 준비 상실이면 preparation=null·can_start=false |
| check | ID·상태·required/applicable/blocking·source·나이·seq | 관측 없으면 observed_at/source_age_ms=null. 빈 값으로 PASS 생성 금지 |
| result | control_request_id·status/code·개정·context·시각 | reason/operator_action, check IDs, field_errors는 실패 원인. 성공 시 빈 배열/null |
| scan result | 결과/스캔/작업 ID·개정·context·판독·이탈·시도 | raw_qr_data/decoded_at/reader_source는 최종 판독 실패 시 null 허용 |
| task_status | RUNNING/COMPLETED/PREEMPTED | COMPLETED는 작업 처리 종료. 성공 여부는 task_outcome에서 판정 |
| task_outcome | SUCCEEDED/FAILED/NOT_ATTEMPTED | RUNNING 중 아직 미확정이면 null |

timestamp는 UTC ISO 8601 문자열이다.
숫자 좌표는 m, yaw는 deg, 속도는 m/s, *_ms는 ms, *_s는 s다.
배터리는 0~100 pct다. 알 수 없는 값은 null/valid=false다.
`{id,revision}` 참조는 같은 snapshot 자산 또는 검증된 로컬 프로파일과 정확히 일치한다.
미지원 계약/필수 기능은 전체 거부한다.
지원 계약에서 허용되지 않은 실행 필드도 조용히 무시하지 않는다.
OpenAPI/Schema 작성 단계에서 추가 속성 정책을 메시지별로 고정한다.

## 5. 재현 순서

1. 네트워크/실제 PX4 출력이 없는 테스트 환경을 사용한다.
2. manifest 해시와 snapshot/profile 참조를 확인한다.
3. 서버 mock이 session→mission_full→snapshot_content를 제공한다.
4. mock Jetson이 plan→확인서→readiness를 보고한다.
5. 고정 시험 시계를 command_start.created_at에 맞춘다.
6. 수락 분기 또는 준비 해제/거부 분기를 선택한다.
7. 실행 분기는 telemetry→scan 결과 개정→execution 결과로 표시한다.
8. ACK 유실·동일 ID 재전송·옛 READY·단절을 E 시험으로 검증한다.

자료 검사 실행:

```text
python validate_fixtures.py
```

이 검사는 문서 데이터의 연결·해시·기본 규칙을 확인한다.
서버 API, C++ 실행기, 기하 계획, PX4 제어를 실행하지 않는다.
통과를 E01~E24 완료나 실기체 비행 승인으로 표시하지 않는다.
