# 웹 서버 소스와 W01 W02 인증 세션 명세

작성일: 2026-10-04. 현재 저장소의 웹 서버 코드와 W01·W02 검수 문서를 기준으로 정리한 전달 자료입니다.
계약 버전은 `1.1-draft.4`이며 현재 범위는 `REPLAY`입니다. 웹 구현 검수 자료이고 실제 Jetson 연동은 검수 대기 상태입니다.

## 소스와 서버 주소

| 항목 | 정보 |
|---|---|
| Git 저장소 | https://github.com/ljh006008-blip/https---github.com-ljh006008-blip-Teamproject1 |
| 확인한 브랜치와 커밋 | `main`, `a828456` |
| 웹 서버 소스 | 저장소의 `dashboard/DroneStock-main/` |
| 로컬 소스 절대 경로 | `C:\Users\Lee\Desktop\Project\DroneStock\https---github.com-ljh006008-blip-Teamproject1\dashboard\DroneStock-main` |
| 검수 문서에 기록된 API 기본 주소 | `http://203.247.41.82:8876` |
| 검수 문서에 기록된 대시보드 | http://203.247.41.82:8876/platform/ |
| SSH 호스트·계정·포트·키 | 저장소 배포 문서에서 확인하지 못함 |

로컬에는 미커밋 변경도 있습니다. 원격 저장소의 커밋만으로 로컬 변경 전체를 재현할 수 있다고 보장하지 않습니다.
위 서버 주소는 W01·W02 문서에 기록된 주소이며 이번 정리에서 접속 여부를 재검증하지 않았습니다.
일반 API 경로 앞에는 `/platform`을 붙이지 않습니다.

## 관련 명세와 구현

| 자료 | 파일 |
|---|---|
| W01 요청·응답·기능 협상·세션 교체 | [WEB_W01_REVIEW_2026-10-04.md](WEB_W01_REVIEW_2026-10-04.md) |
| W02 파일 전달·세션 인증·서명 | [WEB_W02_REVIEW_2026-10-04.md](WEB_W02_REVIEW_2026-10-04.md) |
| 실제 API 라우팅 | [api_urls.py](../dashboard/DroneStock-main/apps/core/api_urls.py) |
| HTTP HMAC 검증 | [device_auth.py](../dashboard/DroneStock-main/apps/core/device_auth.py) |
| W01 세션 협상 | [control_sessions.py](../dashboard/DroneStock-main/apps/core/control_sessions.py) |
| W02 파일 전달 | [mission_snapshots.py](../dashboard/DroneStock-main/apps/core/mission_snapshots.py) |
| snapshot 내용 검증 | [snapshot_contract.py](../dashboard/DroneStock-main/apps/core/snapshot_contract.py) |
| 설정 | [settings.py](../dashboard/DroneStock-main/config/settings.py) |

## W01 장치 인증과 제어 세션 협상

```text
POST /api/drones/{drone_id}/control-sessions/
Content-Type: application/json
```

요청 예시의 장치·기체·boot/runtime 값은 설명용입니다. 실제 등록된 값으로 바꿉니다.

```json
{
  "contract_version": "1.1-draft.4",
  "type": "control_session_request",
  "drone_id": "TEST-DRONE-01",
  "profile": "REPLAY",
  "boot_id": "boot-demo-01",
  "runtime_session_id": "runtime-demo-01",
  "supported_contract_versions": ["1.1", "1.1-draft.4"],
  "capabilities": [
    "mission_xyz", "arrival_yaw", "map_volumes",
    "command_session", "readiness_binding",
    "scan_task_v1", "label_pose_v1", "scan_workspace_v1"
  ]
}
```

위 요청 필드는 모두 필수입니다. 세션 기능 `command_session`, `readiness_binding`이 필요하며, 나머지는 기체와 서버가 공통 지원하는 기능만 협상 결과에 저장합니다.

정상 응답은 HTTP `201`이며 `control_session_id`를 서버 UUID로 발급합니다. 주요 응답 필드는 다음과 같습니다.

```json
{
  "contract_version": "1.1-draft.4",
  "type": "control_session",
  "drone_id": "TEST-DRONE-01",
  "profile": "REPLAY",
  "boot_id": "boot-demo-01",
  "runtime_session_id": "runtime-demo-01",
  "control_session_id": "서버가 발급한 UUID",
  "negotiated_contract": "1.1-draft.4",
  "server_time": "서버 생성 시각",
  "supported_capabilities": ["command_session", "readiness_binding"],
  "flight_authority": false,
  "allowed_execution": "REPLAY_ONLY"
}
```

이 응답은 형식 예시입니다. 실제 `supported_capabilities`는 요청과 서버 지원 목록의 교집합입니다.

- 유효한 협상 POST마다 새 세션을 만듭니다. 같은 boot/runtime이어도 이전 세션을 폐기하고 이력을 보존합니다.
- 협상 실패는 기존 활성 세션을 변경하지 않습니다.
- `201`은 세션 저장 완료이며 임무 준비·명령 수락·비행 승인과 별도입니다.
- W01은 `DEVICE_AUTH_MODE=audit/off`에서도 장치 HMAC 서명을 필수로 검사합니다.
- 장치 인증 ID와 URL의 기체 ID가 서버 등록 범위와 일치해야 합니다.

## W02 세션에 묶인 임무 파일 전달

| 메서드 | 경로 | 기능 |
|---|---|---|
| GET | `/api/drones/{drone_id}/companion-mission/` | 현재 세션의 `assignment`, `snapshot_ref` 조회 |
| GET | `/api/mission-snapshots/{snapshot_id}/content/` | 배정된 원본 UTF-8 JSON 다운로드 |

필수 헤더는 다음과 같습니다.

```text
X-DS-Contract-Version: 1.1-draft.4
X-DS-Control-Session: 현재 서버 발급 UUID
X-DS-Device-ID: 등록된 인증 장치 ID
X-DS-Timestamp: 현재 Unix timestamp 초
X-DS-Nonce: 요청마다 새 값
X-DS-Signature: HMAC-SHA256 hex 서명
```

- W01에서 발급한 현재 세션을 사용합니다. 다른 기체·이전·폐기된 세션은 거절합니다.
- 배정이 없으면 `assignment`, `snapshot_ref`는 null입니다.
- 저장된 원본 바이트와 SHA256, 길이를 보존합니다. 같은 snapshot ID에 다른 바이트를 덮어쓰면 `SNAPSHOT_CONFLICT`입니다.
- 파일 응답은 `application/json; charset=utf-8`, 정확한 `Content-Length`, SHA256 ETag를 제공합니다.
- 응답의 캐시 정책은 `private, no-store, no-transform`입니다.
- 시험 배정은 `QUEUED`이며 파일 다운로드 성공은 비행 준비·START 승인을 의미하지 않습니다.

## HMAC 서명과 nonce 규칙

알고리즘은 HMAC-SHA256이며 서명은 hex 문자열입니다. UTF-8 문자열을 실제 줄바꿈 `\n`으로 연결하고 마지막에 줄바꿈을 추가하지 않습니다.

W01은 다음 5줄을 서명합니다.

```text
HTTP_METHOD
전체 path와 query
SHA256(raw_body)의 hex
Unix timestamp 초
nonce
```

W02는 다음 7줄을 서명합니다. W01의 5줄 서명으로 대체할 수 없습니다.

```text
HTTP_METHOD
전체 path와 query
SHA256(raw_body)의 hex
Unix timestamp 초
nonce
X-DS-Contract-Version 헤더의 값
X-DS-Control-Session 헤더의 값
```

`HTTP_METHOD`는 대문자입니다. path는 서버의 `request.get_full_path()`와 같아야 하며 scheme·host는 포함하지 않습니다.
GET도 빈 본문의 SHA256을 사용합니다. POST는 실제 전송할 원본 바이트를 해시하므로 서명 후 JSON 공백·개행·내용을 바꾸지 않습니다.

공통 인증 헤더는 `X-DS-Device-ID`, `X-DS-Timestamp`, `X-DS-Nonce`, `X-DS-Signature`입니다.
시각 오차 허용 기본값은 300초이며 실행 환경 설정에 따라 달라집니다.
이미 사용된 동일 장치 ID·timestamp·nonce 조합은 `409`로 거절합니다. 요청마다 새 nonce를 사용합니다.

## 등록과 오류 처리

실제 접속에는 사전 등록된 시험 기체, 장치 인증 ID와 HMAC secret이 필요합니다.
서버 설정 `AUTONOMY_REPLAY_DEVICES`는 인증 장치 ID와 시험 기체 ID의 매핑이고, `DEVICE_AUTH_KEYS`는 인증 장치 ID와 secret의 매핑입니다.
이 문서의 예시는 실제 접속 자격 증명을 발급하지 않습니다.

| HTTP | 주요 code | 의미 |
|---|---|---|
| 400 | `INVALID_REQUEST` | 누락·타입 오류·중복 키·허용하지 않는 필드 |
| 401 | `DEVICE_AUTH_FAILED` | 서명 누락·변조·시각 오류 |
| 403 | `DEVICE_SCOPE_MISMATCH` | 등록된 기체 범위 불일치 |
| 404 | `DRONE_NOT_FOUND` | 기체 미등록 |
| 409 | `DEVICE_AUTH_FAILED` | nonce 재사용 |
| 409 | `CONTEXT_MISMATCH` | 기체·버전·세션 문맥 불일치 |
| 409 | `SNAPSHOT_CONFLICT` | 동일 ID의 다른 원본 또는 저장 원본 불일치 |
| 422 | `UNSUPPORTED_CONTRACT`, `UNSUPPORTED_PROFILE`, `UNSUPPORTED_CAPABILITY` | 계약·프로파일·기능 미지원 |

새 계약 오류 응답 필드는 `contract_version`, `type`, `code`, `reason`, `operator_action`, `blocking_check_ids`, `field_errors`입니다.
이 표는 주요 오류 요약이며 상세 조건은 W01·W02 명세와 해당 코드에서 확인합니다.

권장 연동 순서는 **시험 기체·장치 등록 → W01 세션 협상 → W02 임무 참조 조회 → 파일 다운로드 → 원본 길이·SHA256 검증**입니다.
