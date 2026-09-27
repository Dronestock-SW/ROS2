# DroneStock Companion ↔ Platform API 계약 v1.0

기준일: 2026-09-11  
적용 범위: Platform 서버와 Jetson Companion 간 통신  
계약 버전 필드: `contract_version: "1.0"`

이 문서는 통신 계약을 확정한다. PX4/ROS2 내부 토픽명, EKF 구성, 실제 비행 제어기는 Companion 팀의 책임 범위이며 이 계약이 이를 변경하지 않는다.

## 1. 연결 정보

| 항목 | 값 |
|---|---|
| Platform 기본 주소 | 작업 PC와 Jetson이 같은 LAN이면 `http://<작업PC-LAN-IP>:8001` |
| 사람용 화면 | `<base>/platform/` |
| 사람용 로그인 | `<base>/platform/login/` |
| 기체 식별자 | `DRONESTOCK_DRONE_ID`; URL의 `{drone_id}`에 사용 |
| 임무 조회 주기 | 0.5초(2 Hz) |
| 텔레메트리 권장/기본 전송률 | 10 Hz |
| 명령 최대 정상 감지 지연 | 조회 주기 0.5초 + 네트워크 지연 |
| 서버 연결 끊김 표시 | 마지막 Companion 텔레메트리 수신 후 10초 |

`/platform/`은 API 기본 경로가 아니다. API와 WebSocket은 아래 경로를 기본 주소에 바로 붙인다.

## 2. API 목록

| 방식 | 경로 | 방향 | 역할 |
|---|---|---|---|
| GET | `/api/drones/{drone_id}/companion-mission/` | 서버 → Companion | 임무, 경로, 앵커, 제어 요청 조회 |
| WS | `/ws/drones/{drone_id}/` | Companion → 서버 | 최신 텔레메트리 송신 전용 |
| POST | `/api/drones/{drone_id}/companion-phase/` | Companion → 서버 | 임무 단계 보고 |
| POST | `/api/drones/{drone_id}/control-action/ack/` | Companion → 서버 | 제어 요청 수락 ACK |
| POST | `/api/drones/{drone_id}/launch-snapshot/` | Companion → 서버 | 출발 위치 등록 |
| POST | `/api/drones/{drone_id}/scan/` | Companion → 서버 | QR 결과 등록 |

HTTP 성공은 2xx이며 JSON의 `ok`가 명시적으로 `false`이면 실패다. 업무 오류는 400/404/409, 인증 오류는 401로 반환한다. 알 수 없는 JSON 필드는 무시한다. 선택 센서값은 측정하지 않았으면 `null` 또는 필드 생략을 허용하며 0으로 대체하지 않는다.

## 3. 장치 인증

브라우저 로그인과 장치 인증은 분리한다.

- 로컬 전환기: `DEVICE_AUTH_MODE=audit`이면 서명 없는 기존 장치도 허용한다.
- 운영: `DEVICE_AUTH_MODE=enforce`이면 아래 HMAC 헤더가 필수다.
- 비밀키는 ZIP, Git, 문서에 포함하지 않고 현장에서 환경변수로 입력한다.

```text
X-DS-Device-ID: <발급 ID>
X-DS-Timestamp: <Unix seconds>
X-DS-Nonce: <매 요청 고유값>
X-DS-Signature: <hex HMAC-SHA256>
```

서명 원문은 다음 다섯 줄이다.

```text
HTTP_METHOD
request path와 query 전체
SHA256(raw HTTP body)
unix_timestamp
nonce
```

Companion 환경변수:

```dotenv
DRONESTOCK_DEVICE_AUTH_ID=
DRONESTOCK_DEVICE_AUTH_SECRET=
```

둘 다 설정하면 Companion이 자동 서명한다. WebSocket은 현재 송신 전용 호환 채널이며 로컬망 또는 TLS(`wss`) 범위에서 사용한다.

## 4. 좌표와 센서 의미

| 필드 | 확정 의미 |
|---|---|
| `x`, `y` | UWB 창고 로컬 좌표의 검증된 최종 관측값, m |
| `coordinate_frame` | `UWB_ANCHOR_LOCAL` |
| 원점 | A1 |
| +x | A1 → A2 |
| +y | A1 → A3 |
| +z | 바닥에서 위쪽 |
| `fix` | 현재 x/y를 화면 및 위치값으로 사용할 수 있음 |
| `telemetry_verified` | UWB 관측 유효성. FC 전달 또는 EKF 융합 완료를 뜻하지 않음 |
| `xy_control_usable` | 태그가 보고한 로컬 진단값. `telemetry_verified`와 별도 |
| `current_z_m` | Companion이 선택한 현재 고도 |
| `current_z_source` | 실제 출처 문자열. PX4 사용 시 `px4_ekf2` 허용 |
| `battery` | 0~100 %, 미상은 null |
| `battery_voltage` | V, 미상은 null |

UWB는 XY 전용이어도 된다. UWB z, 3D/4D fix, IMU 상세, `final_rc`는 선택 필드다. PX4의 연결/시동/모드는 각각 `fc_connected`, `fc_armed`, `fc_mode`로 분리한다.

송신 시각/순서 필드:

- `sent_at_ms`: 메시지 생성 Unix ms
- `telemetry_seq`: Companion 프로세스 수명 동안 증가하는 순번
- `uwb_seq`: UWB 원본 순번(있는 경우)
- `uwb_age_ms`: Companion 수신 기준 UWB 관측 나이
- `t`: `sent_at_ms`의 구버전 별칭

## 5. 임무 조회 응답

```json
{
  "contract_version": "1.0",
  "generated_at": "2026-09-11T10:00:00+09:00",
  "ok": true,
  "drone_id": "5",
  "status": "ACTIVE",
  "mission_id": "MISSION-20260911-001",
  "mission_db_id": 17,
  "mission_code": "MISSION-20260911-001",
  "route_revision": "4c79278c01dc2a72",
  "route_tasks": [
    {"id": "P1", "type": "waypoint", "x": 1.2, "y": 2.0, "z": 1.2, "arrival_radius_m": 0.2}
  ],
  "coordinate_frame": "UWB_ANCHOR_LOCAL",
  "origin": "A1",
  "x_axis": "A1_TO_A2",
  "y_axis": "A1_TO_A3",
  "z_axis": "UP_FROM_FLOOR",
  "unit": "meter",
  "control_action": "return_to_home",
  "control_requested_at": "2026-09-11T10:00:01+09:00",
  "control_request_id": "2f67e81490b89b04d0d1"
}
```

- `mission_id`는 구버전 호환 필드다. 신규 코드는 `mission_db_id`와 `mission_code`를 사용한다.
- QR의 `mission_code`에는 문자열 `mission_code`를 넣는다.
- `route_revision`은 임무 ID와 전체 경로 내용의 해시다. 값이 바뀌면 전체 경로 새 버전이다.
- 현재 Companion은 비행 중 경로가 바뀌면 index 0부터 다시 동기화한다. 따라서 비행 중 경로 수정은 금지하고 HOLD/착륙 후 새 경로를 배포한다.
- 잘못된 경유지는 조용히 제거하지 않는 방향이 최종 목표지만, 현재 v1 서버 경로 생성 단계가 좌표를 검증한다. Companion에 직접 비정상 경로를 주입하지 않는다.
- SCAN은 `type: "scan"`으로 결정하며 `required_scan`은 서버 메타데이터다.
- 일반 waypoint 대기 0.7초, scan 최소 2초, hover는 `hold_s`를 따른다.

## 6. 텔레메트리 예시

```json
{
  "contract_version": "1.0",
  "type": "telemetry",
  "telemetry_source": "companion",
  "companion_link": true,
  "companion_id": "jetson-drone-05",
  "telemetry_seq": 381,
  "sent_at_ms": 1789092000123,
  "uwb_seq": 8812,
  "uwb_age_ms": 24,
  "mission_db_id": 17,
  "mission_code": "MISSION-20260911-001",
  "route_revision": "4c79278c01dc2a72",
  "active_waypoint_index": 0,
  "active_waypoint_id": "P1",
  "fix": true,
  "telemetry_verified": true,
  "x": 1.24,
  "y": 2.05,
  "current_z_m": 1.18,
  "current_z_source": "px4_ekf2",
  "battery": 78.0,
  "battery_voltage": 15.6,
  "fc_connected": true,
  "fc_armed": true,
  "fc_mode": "LOITER",
  "flight_state": "MOVE_TO_WAYPOINT",
  "t": 1789092000123
}
```

WebSocket 재연결 대기열은 최신 상태 1건만 유지한다. 재연결 뒤 오래된 위치 이력을 몰아서 보내지 않는다.

## 7. 단계, 명령 ACK, 출발점

단계 보고:

```json
{
  "contract_version": "1.0",
  "phase": "in_flight",
  "notes": "",
  "mission_db_id": 17,
  "mission_code": "MISSION-20260911-001"
}
```

제어 ACK:

```json
{
  "contract_version": "1.0",
  "action": "return_to_home",
  "request_id": "2f67e81490b89b04d0d1",
  "requested_at": "2026-09-11T10:00:01+09:00"
}
```

ACK는 요청을 Companion FSM이 받아 해당 상태로 진입했다는 뜻이며 물리적 복귀/착륙 완료가 아니다. 성공 응답을 받을 때까지 같은 요청 ID로 재시도한다. 서버는 오래된 요청 ID 또는 시각이면 409로 거부한다.

출발점 POST에는 `mission_db_id`, `mission_code`, `captured_at_iso`, `source`, x/y/z를 넣는다. 서버는 명시된 미션과 실제 미션이 다르면 409로 거부한다.

단계 대응은 현재 구현을 유지한다. `FAILSAFE_LAND → failed_returning` 등 비행 의미를 바꾸는 매핑은 별도 운영 합의 후 변경한다.

## 8. QR

필수/권장 요청:

```json
{
  "client_scan_id": "UUID",
  "raw_qr_data": "{\"schema\":\"drone-stock-item/v1\",\"code\":\"ITEM-001\"}",
  "mission_id": 17,
  "mission_code": "MISSION-20260911-001",
  "label_point_id": 3,
  "scan_source": "wifi",
  "scanned_at_iso": "2026-09-11T01:00:05Z"
}
```

- `client_scan_id`는 accepted/rejected 양쪽 모두 전역 중복 방지 키로 사용한다.
- 재시도 시 같은 ID를 유지한다.
- `ok:true, accepted:false`는 서버가 거부 결과를 DB에 정상 기록했다는 뜻이므로 재전송 대상에서 제거한다.
- 로컬 전용 `attempts`, `last_error`는 서버로 보내지 않는다.
- 네트워크 실패 시 로컬 저장 후 다음 지점 진행을 허용하는 것이 현재 운영 정책이다. 서버 수락 필수 정책으로 바꾸려면 별도 합의가 필요하다.

## 9. 앵커 배치

- `anchor_layout_version=4`는 좌표 개정 번호가 아니라 4-anchor 계약 형식 버전이다.
- `anchor_layout_id`는 A1~A4 역할/좌표 내용의 해시이므로 좌표가 바뀌면 반드시 달라진다.
- 정상 표시는 태그의 `tag_layout_synced=true`와 태그 `anchor_layout_id`가 서버 ID와 같을 때만 한다.
- 태그 재부팅 또는 불일치가 감지되면 같은 서버 layout ID라도 Companion이 다시 전송한다.

## 10. 현재 확정하지 않는 비행 정책

아래는 통신 문제가 아니라 비행 안전/운영 정책이므로 상대 팀과 별도 승인 후 변경한다.

- ACTIVE 비행 중 경로 교체 방식
- HOLD/CANCEL 수신 시 hover/return/land 중 선택
- 제어 action 우선순위와 만료 시간
- `FAILSAFE_LAND`의 서버 phase 명칭
- QR 서버 미수락 상태에서 다음 경유지 진행 허용 여부
- PX4 EKF/UWB 융합 완료를 나타낼 별도 필드

