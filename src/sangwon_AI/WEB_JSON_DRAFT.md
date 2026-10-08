# 웹 v1 확장 JSON 상세 초안
이 문서는 실제 필드·예제·결과 코드·명령 유효시간을 제안한다.
웹팀이 기존 API의 변경량과 샘플을 검토할 때 읽는다.

> 2026-10-04 외부 계약 검토 기준: [웹 전체 개편 요청서 r2](WEB_REDESIGN_REQUEST_2026-10-04.md)와 [draft.4 예제](contracts/web_v1_1_draft4/README.md)를 먼저 읽는다.
> 이 문서의 draft.2/부분 예제는 이전 근거다. 새 웹 개발에 혼용하지 않는다.

상태: `1.1-draft.2` 제안 / 2026-10-02 / 웹팀 수락 전.
동작 의미는 [WEB_CONTRACT_DRAFT.md](WEB_CONTRACT_DRAFT.md)를 따른다.
예제 ID·좌표·승인 자료는 합성 자료이며 운용값이 아니다.
UWB 형식은 포함하지 않는다. 전체 사양 수령 뒤 별도 어댑터로 연결한다.
2026-10-04 보완: [QR_SCAN_SPEC.md](QR_SCAN_SPEC.md)에 QR·ArUco 계약 요구를 추가했다.
아래 draft.2 JSON에는 scan 입력·결과가 아직 포함되지 않았다.
웹 회신 뒤 개정하며, 현재 기체의 지원을 뜻하지 않는다.

## 1. 버전과 적용 규칙

| 항목 | 제안 |
|---|---|
| contract_version | 문자열 `1.1-draft.2`. 합의 후 정식 번호 발급 |
| 기체 ID | 기존처럼 문자열. 숫자로 자동 변환하지 않음 |
| mission_db_id·mission_code | 기존 정수 ID·문자열 코드 의미 유지 |
| route_revision | 기존 전체 경로 개정 의미 유지, 같은 개정의 내용 변경 금지 |
| 임무 배포 상태 | 최상위 status=READY는 서버 자료 배포 상태. 기체의 system_readiness/mission_readiness와 별개이며 시동 조건이 아님 |
| 시간 | RFC3339 UTC 문자열, 경과 시간은 Jetson 단조 시각으로 관리 |
| 필수 기능 | required_capabilities 중 하나라도 미지원이면 전체 거부 |
| 신규 명령 | control_requests 배열. 기존 단일 control_action과 이중 실행 금지 |
| 미지원 행동 | 현재 구현은 scan 미지원이므로 전체 거부. QR_SCAN_SPEC.md에 확장 설계 |
| 값 검증 | 숫자 타입·유한성·단위·범위 검사. 문자열 숫자/중복 키/필수 null 거부 |

조회 요청에서 지원 버전을 알리는 `X-DS-Contract-Version` 헤더를 제안한다.
서버는 명시적으로 지원을 확인한 기체에만 확장 임무를 배포한다.
이 헤더의 인증 결합 방식도 웹팀과 확정한다.
구형 v1의 ‘알 수 없는 필드 무시’만으로 확장 임무를 실행시키지 않는다.

## 2. 전체 임무 조회 예제

기존 GET `/api/drones/{drone_id}/companion-mission/`의 확장 응답안이다.
지도는 첫 합성 시험에서는 inline으로 제공한다.
큰 지도 다운로드 방식은 같은 snapshot_id·개정 검증을 유지해 추후 분리할 수 있다.

```json
{
  "contract_version": "1.1-draft.2",
  "generated_at": "2026-10-02T03:00:00Z",
  "ok": true,
  "drone_id": "5",
  "status": "READY",
  "required_capabilities": ["mission_xyz", "arrival_yaw", "map_volumes", "command_session", "readiness_binding"],
  "snapshot": {
    "snapshot_id": "snapshot-demo-01",
    "mission_db_id": 17,
    "mission_code": "MISSION-DEMO-001",
    "route_revision": "route-demo-r1",
    "coordinate_frame": "WAREHOUSE_MAP",
    "length_unit": "m",
    "yaw_convention": {"unit": "deg", "zero_axis": "+x", "positive": "ccw"},
    "start_mode": "AUTO_TAKEOFF",
    "takeoff_z_m": 1.2,
    "start_yaw_deg": 0,
    "acceptance_profile_id": "flight-profile-demo-r1",
    "transform_ref": {"id": "transform-demo", "revision": "r1"},
    "route_tasks": [
      {"id": "A", "type": "waypoint", "x": 2, "y": 1, "z": 1.2, "yaw_deg": 90, "hold_s": 2},
      {"id": "B", "type": "waypoint", "x": 2, "y": 3, "z": 1.5, "hold_s": 0}
    ],
    "map": {
      "map_id": "warehouse-demo",
      "revision": "map-r1",
      "floor_reference_id": "floor-demo",
      "geometry_margin_applied": false,
      "obstacle_data_status": "PROVIDED",
      "obstacles": [
        {"id": "shelf-1", "kind": "SHELF", "polygon_xy_m": [[5,3],[6,3],[6,4],[5,4]], "z_min_m": 0, "z_max_m": 2}
      ],
      "overhead_data_status": "PROVIDED",
      "overhead_structures": [
        {"id": "ceiling-1", "polygon_xy_m": [[0,0],[10,0],[10,8],[0,8]], "z_min_m": 3, "z_max_m": 3.2}
      ],
      "altitude_zones": [
        {"id": "height-1", "polygon_xy_m": [[0,0],[10,0],[10,8],[0,8]], "z_min_m": 0, "z_max_m": 2}
      ],
      "flight_boundary": {"status": "NOT_PROVIDED", "polygon_xy_m": null},
      "no_fly_zones": {"status": "NOT_PROVIDED", "volumes": null},
      "yaw_validated_zones": [
        {"id": "yaw-zone-1", "polygon_xy_m": [[0,0],[10,0],[10,8],[0,8]], "z_min_m": 0, "z_max_m": 2, "approval_ref": "yaw-survey-demo-r1"}
      ]
    },
    "approvals": {
      "overhead_check": {"id": "check-demo-1", "operator_id": "operator-demo", "checked_at": "2026-10-02T02:59:00Z", "map_revision": "map-r1", "route_revision": "route-demo-r1"},
      "heading_confirmation": {"id": "heading-demo-1", "observed_yaw_deg": 0, "checked_at": "2026-10-02T02:59:30Z", "transform_revision": "r1"}
    }
  },
  "control_session_id": "session-demo-01",
  "control_requests": []
}
```

| 필드 | 계약 |
|---|---|
| snapshot_id | 임무·지도·승인 묶음의 불변 ID. 같은 ID 다른 내용은 충돌 |
| start_mode | AUTO_TAKEOFF 또는 RC_AIRBORNE_HANDOVER |
| takeoff_z_m | 두 모드 모두 필수. RC 인계는 현재 고도에서 인계 후 안전 조정 |
| start_yaw_deg·yaw_deg | 선택. 생략은 기본 방향, null은 오류, 0은 실제 목표 |
| route_tasks | 아래 waypoint 예제의 배열 순서가 필수 기체 방문 순서. ID 고유, 기체 XYZ 필수. scan 라벨 XYZ는 이 경로로 실행 금지 |
| hold_s | 0 이상 초 단위 체류. 기존 waypoint 0.7초 기본값을 적용하지 않음 |
| acceptance_profile_id | Jetson에 승인·설치된 도착 기준 참조. 웹 임의 완화 금지 |
| transform_ref | 승인된 좌표 변환 참조. 미설치/다른 개정이면 거부 |
| approval_ref | 관련 지도·기체 설정·측량 결과에 연결된 승인 자료 참조 |

예제의 +x=0°·반시계 양수는 **협의용 구체안**이다.
웹 좌표계 회의의 결정을 대신하지 않는다.
웹이 승인 참조 문자열을 보냈다는 사실만으로 승인을 신뢰하지 않는다.
Jetson에 저장된 해당 자료의 범위·개정·적용 기체를 대조한다.
heading_confirmation은 시작 당시 현재 방향 확인이며 목표 yaw와 별개다.
시작 요청 적용 시 신선도·실제 상태를 다시 검증한다.

다각형은 중복 끝점을 제외한 3개 이상의 점이며 마지막→첫 점으로 닫는다.
자기 교차·면적 0·높이 역전·비유한 값은 거부한다.
첫 초안은 구멍 없는 다각형과 높이 구간의 조합으로 표현한다.
구멍/복합 형상은 여러 볼륨 분할 또는 향후 스키마 확장이 필요하다.
겹친 고도 구역은 제한의 교집합을 적용한다. 빈 교집합은 경로 거부다.
기체 중심 제한과 장애물 점유 높이를 구분한다.
PROVIDED인 빈 금지구역 배열은 ‘없음’이며 NOT_PROVIDED/null과 다르다.
경계·금지구역이 모두 제공·검증되기 전 자동 우회는 꺼 둔다.

## 3. 명령 예제와 문맥

서버는 현재 제어 세션에서 생성된 요청을 GET의 control_requests로 전달한다.
아래는 자동 이륙 시작 요청 한 개다.

```json
{
  "control_request_id": "request-start-demo-01",
  "drone_id": "5",
  "action": "START",
  "control_session_id": "session-demo-01",
  "sequence": 1,
  "created_at": "2026-10-02T03:00:01Z",
  "expires_at": "2026-10-02T03:00:11Z",
  "context": {
    "boot_id": "boot-demo-01",
    "companion_session_id": "runtime-demo-01",
    "snapshot_id": "snapshot-demo-01",
    "preparation_id": "prepared-demo-01",
    "readiness_revision": 7,
    "execution_id": null,
    "flight_id": null
  }
}
```

| action | 필수 문맥 | 수락 후 완료 |
|---|---|---|
| START | boot_id·companion_session_id·snapshot_id·Jetson 발급 preparation_id·readiness_revision. RC 예외는 현재 flight_id도 필수 | 해당 임무의 최종 결과 |
| PAUSE | 활성 execution_id·flight_id | 실제 정지/위치 유지 확인 |
| RESUME | 같은 execution_id·flight_id, PAUSED 상태 | 재검증 후 임무 흐름 복귀 |
| CANCEL | execution_id, 비행 중에는 flight_id | 지상 종료 또는 복귀/상승 중 현 위치 착륙 완료 |
| LAND_NOW | 현재 flight_id. execution_id는 활성 임무가 있으면 필수 | 현재 위치 착륙·시동 해제 확인 |

START 전에 Jetson이 묶음을 검증하고 preparation_id를 상태로 보고한다.
모든 신규 명령은 현재 boot_id·companion_session_id·control_session_id에도 결합한다.
Python 상태 모니터의 정상 표시는 preparation_id를 발급하거나 대체하지 않는다.
새 지도·승인·변환 또는 준비 조건에 영향을 주는 기체 상태 변경으로 준비가 무효화되면 새 preparation_id가 필요하다.
지상 START의 flight_id는 null이다. 이미 비행 중이면 그 예외 문맥을 명시해야 한다.
명령은 현재 제어권 범위에서만 실행한다. LAND_NOW도 RC 제어권을 강제로 회수하지 않는다.
기존 return_to_home은 활성 임무 취소·복귀로, land는 LAND_NOW로 대응시키는 안이다.
이 호환 변환은 서버/Jetson 중 한 곳에서만 수행하고 원 요청 ID를 보존한다.

### 유효시간과 세션

**사용자 결정: 생성 후 10초를 초기 수락 유효시간으로 쓴다.**
서버는 expires_at=created_at+10초로 고정한다.
Jetson은 새 요청에 대해 created_at≤현재 시각<expires_at 조건과 시계 신뢰성을 검사한다.
미래 생성 시각·시계 오차 허용 범위는 M13 시험 뒤 확정한다.
현재 시계의 오차 범위 전체가 유효기간 안인지 확인할 수 없으면 새 요청을 거부하는 안이다.
시계가 불확실한 상태를 임의로 10초 연장하지 않는다.

반복 GET·재전송은 생성/만료 시각을 바꾸지 않는다.
이미 수락한 요청은 10초 뒤에도 실행하며 결과 조회가 가능하다.
수락 기록 조회는 만료·이전 세션 검사보다 먼저 한다.
같은 ID·같은 내용이면 기존 결과만 반환한다. 같은 ID 다른 내용은 충돌이다.
새 ID로 바꾼 재요청은 새 명령이며 현재 상태를 다시 검사한다.

제어 세션은 Jetson 부팅 세션과 별도다.
명령 조회 채널의 실패/단절 판정·재연결 시 기존 세션을 닫고 새로 확인한다.
서버가 단절 중 쌓은 요청을 새 세션에 자동 재지정하지 않는다.
운영자가 현재 상태·기존 요청 결과를 확인한 뒤 새 요청을 만든다.
단절 감지 전의 지연 요청은 10초 유효시간으로 제한한다.
더 짧은 단절을 식별할 수 있는지는 M13 시험으로 검증한다.
세션 발급·확인·결과 조회 경로는 아래 변경안으로 웹팀과 합의한다.

sequence는 세션 안에서 증가하는 양의 정수다.
새 요청의 역순은 STALE_COMMAND_ORDER로 거부한다.
수신 묶음에 LAND_NOW가 있으면 낮은 우선순위 명령을 실행 후 착륙하는 방식으로 처리하지 않는다.
먼저 안전·제어권과 요청을 중재하고 대체된 미실행 요청을 PREEMPTED로 기록한다.

## 4. 결과 구조와 코드

기존 control-action/ack/의 확장 본문안이다.
수락/최종 상태 변경마다 result_revision이 증가하며 재전송은 같은 값을 쓴다.

```json
{
  "contract_version": "1.1-draft.2",
  "drone_id": "5",
  "request_id": "request-start-demo-01",
  "action": "START",
  "result_revision": 1,
  "status": "ACCEPTED",
  "code": "OK",
  "execution_id": "execution-demo-01",
  "flight_id": null,
  "phase": "PREPARING",
  "reported_at": "2026-10-02T03:00:02Z",
  "details": [],
  "retryable": false
}
```

status는 ACCEPTED / REJECTED / COMPLETED / FAILED / PREEMPTED를 제안한다.
COMPLETED는 action의 완료이며 PAUSE 완료를 임무 성공으로 해석하지 않는다.
START 최종 결과에는 mission_result=SUCCEEDED/CANCELED/FAILED/MANUAL_HANDOVER를 추가한다.
PREEMPTED에는 superseded_by_request_id 또는 원인 코드를 포함한다.
retryable은 원인 해소 후 새 요청이 가능한지의 정보이며 통신 자동 재송신 지시가 아니다.

| code | 의미 | 처리 |
|---|---|---|
| OK | 해당 status의 처리 성립 | 실제 완료는 status·phase로 확인 |
| UNSUPPORTED_CONTRACT / UNSUPPORTED_CAPABILITY | 버전/필수 기능 불일치 | 배포 중단·계약 수정 |
| INVALID_PAYLOAD / INVALID_WAYPOINT / INVALID_MAP | 타입·값·형상 불량 | 대상 필드/점 ID를 details에 기록 |
| SNAPSHOT_CONFLICT / REQUEST_ID_CONFLICT | 같은 ID의 내용 변경 | 신규 실행 없이 충돌 표시 |
| MISSING_APPROVAL / TRANSFORM_UNAVAILABLE | 승인/변환 부족 | 준비 불가 |
| HEADING_UNALIGNED / OUTSIDE_VALIDATED_AREA | 방향 불일치/운용 범위 밖 | 입력·측량·센서 확인 |
| PREPARATION_STALE / STALE_CONTEXT | 준비/실행/비행 참조가 오래됨 | 현재 상태 조회 후 재요청 |
| COMMAND_EXPIRED / STALE_CONTROL_SESSION / STALE_COMMAND_ORDER | 기한/세션/순서 불일치 | 자동 재실행 없음 |
| CLOCK_UNTRUSTED | 기한을 신뢰성 있게 판정할 수 없음 | 동기화 상태 확인 |
| BUSY / INVALID_PHASE | 새 임무 불가/현재 단계에서 금지 | 현재 상태 표시 |
| CONTROL_NOT_OWNED / MANUAL_LOCKED | RC/PX4에 제어권이 있음 | 강제 회수 없음 |
| STORAGE_NOT_READY | 새 비행 기록 공간 부족 | 시작 거부 |
| PX4_NOT_READY / POSITION_NOT_READY | PX4/위치 품질 부족 | 준비 불가 |
| SYSTEM_NOT_READY / MISSION_NOT_READY | 시스템/해당 임무 필수 점검 미통과 | 차단 check_id·코드·운영자 조치 반환 |
| READINESS_STALE / STALE_RUNTIME_SESSION | 준비 관측 만료/다른 부팅·실행기 | 새 상태·세션·준비 ID 확인 뒤 운영자가 다시 요청 |
| MODE_TRANSITION_FAILED / ARM_REJECTED | 실제 모드 전환/시동 실패 | PX4 결과와 현재 상태를 details에 기록 |
| BLOCKED_TIMEOUT / POSITION_LOST / YAW_INVALID / LOW_BATTERY | 진행 중 중단·복귀·착륙 원인 | phase와 mission_result 별도 보고 |
| GOAL_EXPIRED / LAND_UNCONFIRMED / INTERNAL_ERROR | 내부 감시/착륙 확인/처리 이상 | 근거·실제 제어권 보고 |
| LOGGING_FAILED / GOAL_NOT_REACHED | 필수 기록 실패/목표 도달 실패 | 실패 원인과 복귀·착륙 상태를 보고 |

HTTP 전달 결과와 위 표의 기체 처리 결과는 구분한다.
결과 POST의 HTTP 200은 서버 저장 성공이며 기체 동작 성공이 아니다.
HTTP 분류 제안: 400=구문, 401/403=인증/권한, 404=대상 없음,
409=ID 내용 충돌, 422=의미 오류, 5xx=일시 서버 장애.
Jetson이 거부한 명령 결과도 정상적인 결과 기록으로 저장한다.
알려진 과거 결과는 단계가 바뀌어도 수락하며 같은 revision의 재전송은 중복 저장하지 않는다.
알 수 없는 요청 ID나 다른 기체의 결과는 거부한다.

## 5. 상태·사건·출발 기록

기존 WS에 다음 구조를 추가하는 안이다.
기존 x/y는 UWB 관측 의미를 유지하고 PX4 위치로 덮어쓰지 않는다.

```json
{
  "contract_version": "1.1-draft.2",
  "type": "telemetry",
  "drone_id": "5",
  "boot_id": "boot-demo-01",
  "companion_session_id": "runtime-demo-01",
  "control_session_id": "session-demo-01",
  "telemetry_seq": 100,
  "generated_at": "2026-10-02T03:00:10Z",
  "execution_id": "execution-demo-01",
  "flight_id": "flight-demo-01",
  "phase": "TAKING_OFF",
  "mission_result": null,
  "control_owner": "JETSON",
  "fc_connected": true,
  "fc_armed": true,
  "fc_mode": "OFFBOARD",
  "landed_state": "IN_AIR",
  "readiness_ref": {"companion_session_id": "runtime-demo-01", "readiness_revision": 8, "readiness_seq": 102},
  "preparation": {"ready": false, "preparation_id": null, "reason_codes": ["BUSY"]},
  "pose": {"frame": "WAREHOUSE_MAP", "source": "PX4_FUSED", "valid": true, "age_ms": 40, "x": 1, "y": 1, "z": 0.8, "yaw_deg": 0},
  "active_target": {"kind": "TAKEOFF", "waypoint_id": null, "x": 1, "y": 1, "z": 1.2, "yaw_deg": 0},
  "battery": 78,
  "warnings": [],
  "logging": {"live_recording": "ACTIVE", "can_start_new_flight": true, "ulog_collection": "NOT_STARTED"}
}
```

control_owner: NONE / JETSON / RC / PX4_FAILSAFE / UNKNOWN을 제안한다.
phase: IDLE / PREPARING / TAKING_OFF / HANDOVER / NAVIGATING / ROTATING / HOVERING /
PAUSED / BLOCKED_WAIT / RECOVERY_WAIT / RETURNING / LANDING / ENDED를 제안한다.
logging.can_start_new_flight는 기록 용량 판정이며 preparation.ready를 대신하지 않는다.
preparation은 기존 호환 요약이고 아래 readiness 메시지가 준비 판정의 상세 근거다.
두 자료가 불일치하거나 readiness_ref를 최신 상세 보고와 대조할 수 없으면 웹은 시작을 차단한다.
미상·낡은 pose는 valid=false, 쓸 수 없는 값은 null로 표시하고 마지막 값의 시각을 보존한다.
준비된 지상 상태에서는 preparation에 Jetson 발급 ID와 snapshot_id를 포함한다.

사건 POST에는 event_id·event_seq·execution_id·flight_id·previous_phase·phase·reason_code·occurred_at을 둔다.
출발 기록 POST에는 flight_id·snapshot_id·captured_at·liftoff_confirmed_at·지도/변환 개정을 둔다.
home_xy_m·ground_reference_id·pre_takeoff_yaw_deg·vehicle_reference_id도 포함한다.
home은 착륙 z 목표 0을 뜻하지 않는다. 최종 하강은 PX4 Land로 실행한다.
실제 이륙 전 기록과 이륙 확인 시각을 구분하고 웹에서 출발점을 재설정하지 않는다.

## 6. 웹 담당자에게 요청할 변경

| 연결점 | 요청할 변경 |
|---|---|
| companion-mission GET | 확장 버전 명시 확인, snapshot, control_requests, 세션 |
| control-action/ack POST | 위 status/code, result_revision, 과거 결과 중복 방지 저장 |
| companion-phase POST | 사건 ID·순서, 원인·현재 단계·최종 임무 결과 분리 |
| launch-snapshot POST | 실제 이륙점·이륙 전 yaw·비행 ID·변환 개정 고정 |
| WS telemetry | pose/active_target/control_owner/preparation/logging/readiness_ref, 관측 나이 |
| WS readiness | 부팅 단계·시스템/임무 준비·체크별 원인·최종 관측·준비 문맥 |
| 신규 연결점 제안 | 제어 세션 발급/확인, 요청 ID별 결과 GET |
| 웹 화면 | 현재 방향/목표 yaw 분리, 시작 거부 이유, 미전달/미확인 명령, 10초 만료 표시 |

신규 URL·정식 버전·JSON Schema·내용 해시 정규화는 웹 계약 합의 항목이다.
기존 URL 호환성 시험과 같은 샘플의 송수신 시험 뒤 정식화한다.

## 7. 부팅·시스템 준비·임무 준비 JSON

전용 `type=readiness`는 기존 Jetson→서버 WS에서 상태 변화 때와 주기적으로 보낸다.
초기 모의 시험 주기는 1Hz, 유효 나이는 2초로 제안한다. FLIGHT 값은 실제 지연 시험 후 별도 승인한다.
10Hz 위치 telemetry가 와도 readiness나 실제 센서가 오래됐으면 준비 유효성을 연장하지 않는다.
아래는 **합성 FLIGHT 승인 자료가 모두 있다고 가정한 계약 예제**이며 현재 Jetson의 상태가 아니다.
예제 checks는 축약 배열이다. 실제 보고는 승인 프로파일에 등록된 필수 체크 전부를 포함해야 한다.

```json
{
  "contract_version": "1.1-draft.2",
  "type": "readiness",
  "drone_id": "5",
  "boot_id": "boot-demo-01",
  "companion_session_id": "runtime-demo-01",
  "control_session_id": "session-demo-01",
  "readiness_seq": 101,
  "readiness_revision": 7,
  "generated_at": "2026-10-02T03:00:00Z",
  "source_age_ms": 0,
  "max_age_ms": 2000,
  "profile": {"mode": "FLIGHT", "id": "flight-profile-demo-r1", "approved": true},
  "boot_phase": "RUNNING",
  "system_readiness": {"state": "READY", "blocking_check_ids": [], "warning_check_ids": ["rc.available"]},
  "mission_readiness": {
    "state": "READY",
    "snapshot_id": "snapshot-demo-01",
    "preparation_id": "prepared-demo-01",
    "prepared_at": "2026-10-02T03:00:00Z",
    "valid_until": "2026-10-02T03:00:02Z",
    "can_start": true,
    "blocking_check_ids": [],
    "binding": {"map_revision": "map-r1", "route_revision": "route-demo-r1", "transform_revision": "r1", "profile_id": "flight-profile-demo-r1"}
  },
  "checks": [
    {
      "check_id": "px4.link",
      "scope": "SYSTEM",
      "required": true,
      "status": "PASS",
      "blocking": false,
      "code": "OK",
      "reason": "PX4 연결 및 대상 기체 식별 확인",
      "operator_action": "없음",
      "source": "px4_state_adapter",
      "last_seen_at": "2026-10-02T02:59:59.950Z",
      "source_age_ms": 50,
      "max_age_ms": 1000,
      "check_seq": 81
    },
    {
      "check_id": "rc.available",
      "scope": "SYSTEM",
      "required": false,
      "status": "WARN",
      "blocking": false,
      "code": "RC_UNAVAILABLE",
      "reason": "RC 연결 없음. 현재 수동 인계 불가",
      "operator_action": "비상 수동 인계가 필요하면 RC 연결 상태 확인",
      "source": "px4_state_adapter",
      "last_seen_at": "2026-10-02T02:59:59.900Z",
      "source_age_ms": 100,
      "max_age_ms": 1000,
      "check_seq": 62
    }
  ],
  "rc": {"availability": "UNAVAILABLE", "override_latched": false, "manual_policy_ref": "existing-px4-rc-profile", "px4_mode": "AUTO_LOITER"},
  "indication": {"system_ready": true, "mission_ready": true, "primary": "WEB", "scanner_led": "UNVERIFIED"}
}
```

| 필드 | 형식·검사 |
|---|---|
| boot_id | 문자열. OS 재부팅마다 변경, 서버 재접속으로 임의 변경하지 않음 |
| companion_session_id | C++ 실행기 기동마다 새 ID. Python 모니터 ID와 혼용 금지 |
| readiness_seq | 해당 실행기 세션에서 증가하는 정수. 오래된 보고는 최신 상태를 덮어쓰지 않음 |
| readiness_revision | 허가 관련 결과/문맥이 바뀔 때 증가. START가 현재 값에 결합 |
| source_age_ms/max_age_ms | 보고 생성 시 실제 준비 관측 나이/허용 한도. 캐시·전송 경과를 더해 판정 |
| prepared_at/valid_until | 특정 준비의 현재 유효 창. 같은 ID의 정상 재검증으로 유효 창 갱신 가능 |
| mission_readiness.can_start | Jetson 판정. 웹은 추가로 LIVE·시계·신선도·대상·권한을 확인해야 버튼 활성화 |
| checks | 프로파일의 필수 체크 누락은 UNKNOWN. 배열 일부만 온 것을 전체 정상으로 보지 않음 |
| required/blocking | required=true의 FAIL/UNKNOWN은 blocking=true. 웹이 변경할 수 없음 |
| last_seen_at | 해당 증거를 마지막으로 실제 관측한 UTC. 모니터 반복 출력 시각과 구분 |
| source_age_ms=null | 관측 미수신/나이 불명. 필수 항목은 UNKNOWN·차단 |
| 관측 만료 | 별도 STALE 결과 enum을 만들지 않고 status=UNKNOWN, code=INPUT_STALE로 표현 |
| check_seq | 원천 체크 갱신 순서. 값이 같아도 실제 새 관측이면 증가할 수 있음 |
| profile.mode | REPLAY/SITL/FLIGHT. 항상 화면 배지에 명시 |
| rc.availability | AVAILABLE/UNAVAILABLE/UNKNOWN. 미연결은 시작 허용 정책의 WARN |
| rc.override_latched | RC 비상 수동 인계 이후 같은 비행 자율 재개 금지 표시 |
| scanner_led | UNVERIFIED/UNSUPPORTED/AVAILABLE. 현재 LED 제어 가능 여부 미확정 |

준비 heartbeat가 갱신하는 유효 창은 현재 모든 필수 증거가 여전히 유효할 때만 연장한다.
체크가 낡았는데 준비 메시지만 계속 발행하는 경우 READY를 연장하지 않는다.
preparation_id는 READY 상실·기체/임무/변환 개정 변경·제어 세션 변경·실행기 재기동 때 폐기한다.
readiness_revision과 preparation_id가 같고 관측이 계속 정상이면 이전 heartbeat를 본 START도 최신 검사를 거쳐 수락할 수 있다.
단, START의 10초 자체 유효시간과 당시 최신 준비 유효성은 모두 충족해야 한다.

### 7.1 실제 모니터와 최종 준비 보고의 구분

초기 Python 진단 모니터는 `monitor_session_id`로 파일/진단 보고를 만들 수 있다.
그 보고는 장치 준비 평가의 보조 진단이며 C++ mission_readiness, preparation_id, START 수락 권한을 갖지 않는다.
현재 로컬 모니터 스키마는 `sangwon-host-health/1`이며 위 외부 계약 JSON과 동일하지 않다.
monitor_session_id/monitor_seq, boot_id, scope=HOST_DIAGNOSTICS_ONLY, state=BLOCKED,
can_start=false, flight_authority=false, companion_session_id/preparation_id=null을 사용한다.
웹 어댑터는 이 호스트 진단과 별도의 C++ preflight 결과를 받아 외부 readiness 메시지를 구성한다.
호스트 진단이 녹색이라는 이유로 비어 있는 C++ 세션/준비 ID를 임의 생성하지 않는다.
예: UWB 사양 미수령·장치 미연결·PX4 품질 미검증이면 ‘진단 수집 동작 중/시스템 준비 불가’다.
‘모니터가 정상 실행 중’을 ‘비행 준비 완료’로 바꾸지 않는다.
실제 monitor 출력 JSON에서 외부 readiness JSON으로의 변환은 웹 어댑터 구현 때 명시한다.

### 7.2 준비 상실·거부 예제

```json
{
  "contract_version": "1.1-draft.2",
  "drone_id": "5",
  "request_id": "request-start-demo-02",
  "action": "START",
  "result_revision": 1,
  "status": "REJECTED",
  "code": "SYSTEM_NOT_READY",
  "execution_id": null,
  "flight_id": null,
  "phase": "IDLE",
  "reported_at": "2026-10-02T03:00:03Z",
  "details": [
    {"check_id": "position.quality", "status": "UNKNOWN", "code": "POSITION_NOT_READY", "reason": "유효한 융합 위치 관측 없음", "operator_action": "위치 센서 연결·PX4 융합 상태 확인", "last_seen_at": null}
  ],
  "retryable": true
}
```

retryable=true여도 웹이 새 ID로 자동 START를 만들지 않는다.
원인 해소→최신 준비 확인→운영자의 새 시작 요청 순서다.
같은 ID의 기존 결과 조회/전송 재시도는 허용되며 새 동작을 발생시키지 않는다.

### 7.3 준비 사건

```json
{
  "contract_version": "1.1-draft.2",
  "drone_id": "5",
  "event_id": "event-ready-demo-01",
  "event_seq": 11,
  "event_type": "MISSION_READY",
  "boot_id": "boot-demo-01",
  "companion_session_id": "runtime-demo-01",
  "readiness_revision": 7,
  "snapshot_id": "snapshot-demo-01",
  "preparation_id": "prepared-demo-01",
  "execution_id": null,
  "flight_id": null,
  "previous_phase": "IDLE",
  "phase": "IDLE",
  "reason_code": "CHECKS_PASSED",
  "occurred_at": "2026-10-02T03:00:00Z"
}
```

event_type은 SYSTEM_READY / MISSION_READY / READY_REVOKED / MANUAL_OVERRIDE를 추가한다.
READY_REVOKED에는 scope=SYSTEM/MISSION와 reason_code·check_ids를 포함한다.
MANUAL_OVERRIDE는 control_owner=RC·현재 PX4 모드·비행 ID·same_flight_resume_allowed=false를 함께 보고한다.
RC 수동 인계 뒤에는 기존 RC/PX4 설정이 동작하며 Jetson이 임의로 Land/Kill 정책을 추가하지 않는다.
웹은 ‘수동 조종 중/동일 비행 자율 재개 불가’를 표시한다.

## 8. 요청 검증 순서와 상태 표시 불일치

1. 장치 인증·대상 기체·지원 버전·JSON 구문을 검사한다.
2. 같은 request_id의 기존 원장 결과를 먼저 확인한다. 같은 내용이면 결과만 반환한다.
3. 새 요청의 boot/runtime/control 세션·순서·10초 기한·시계 신뢰를 검사한다.
4. action별 execution/flight/preparation 문맥·제어권·현재 단계·안전 우선순위를 검사한다.
5. START는 최신 시스템/임무 준비·프로파일·필수 체크·지도/승인을 재검증한다.
6. C++ 임무 관리기가 수락 원장을 영속 기록하고 실행한다. 웹 어댑터는 그 결과를 전달한다. 물리 완료는 별도 상태로 확인한다.

다중 탭이 같은 준비 ID로 START를 각각 요청해도 첫 수락 뒤 나머지는 BUSY/STALE_CONTEXT로 거부한다.
이미 수락된 동일 요청 재시도는 BUSY로 바꾸지 않고 기존 결과를 돌려준다.
준비 유효성이 없다는 이유로 현재 비행 LAND_NOW 처리를 START 준비 완료까지 기다리게 하지 않는다.
LAND_NOW는 현재 제어권·비행 문맥·명령 신선도·안전 우선순위로 판정한다.
시스템 준비 상실은 새 임무 시작 금지 의미이며, 비행 중 실제 대응은 BT의 해당 고장 정책을 따른다.
예를 들어 웹 단절·RC 단절·LiDAR 단절은 기존 저장 임무 계속 정책을 유지한다.

## 9. scan 확장 협의 항목 — draft.2 미포함

다음 의미는 2026-10-04 사용자 결정이다. 필드명·JSON Schema는 웹 회신 뒤 별도 개정한다.
기존 waypoint 예제나 구형 scan XYZ 전달 코드를 그대로 사용하지 않는다.

| 입력/출력 의미 | 계약에 반영할 요구 |
|---|---|
| 작업 유형 | waypoint는 기체 XYZ, scan은 라벨 XYZ. 유형별 스키마와 필수 정보 검사 |
| 라벨 위치 | 지도 좌표 m, z는 공통 바닥에서 라벨까지 높이. 정확한 기준점은 웹/라벨 담당자 협의 |
| 라벨 정면 | 판독 가능한 통로 쪽 방향. 표현·좌표축·필수 필드는 QS-W7 회신으로 확정 |
| 기체 판독 pose | 라벨 위치·정면·선택 판독 거리·장착 변환으로 Jetson이 생성한 별도 목표 |
| 접근 대기점/작업 공간 | 웹이 라벨별 허용 구역 제공, Jetson이 구역/경로 검증 후 기체 대기점 산출. QS-W8의 외곽·z 범위·개정·연결·승인 필드 협의 |
| 체류와 판독 시간 | 일반 이동점 hold_s와 스캔 안정/판독 창·전체 예산을 구분. scan 체류 필드 적용 위치는 회신 필요 |
| 진행 | 접근→정렬→안정→스캐너 1~3→카메라→결과 저장→대기점 복귀→다음 목표 |
| 결과 | 판독/정렬 실패와 비행/이탈 실패를 구분. 성공/실패 결과 저장만으로 다음 작업 시작 처리 금지 |
| 장치/마커 실패 | 비행 중 획득 한도 실패·카메라 고장은 실패 기록·대기점 복귀 후 다음 목표. 사전 불량은 scan READY 거부 |

대상 라벨 위치와 기체 접근/판독 목표는 상태 메시지에서 별도 항목으로 표시한다.
웹의 선택 기체 yaw를 라벨 정면으로 자동 해석하지 않는다.
명령 유효시간·세션·원장·RC/PX4 우선순위는 기존 계약을 유지한다.
scan 지원 기능과 버전이 합의·구현되기 전에는 scan이 포함된 임무를 전체 거부한다.
