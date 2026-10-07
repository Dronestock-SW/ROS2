# Platform 실제 연결 점검
이 문서는 2026-09-13 웹 연결 시험 결과다.
다음 통신 시험의 범위를 정할 때 읽는다.

## 결과

현재 companion 환경에서 서버 연결에 성공했다.
임무 조회와 WebSocket 연결까지 확인했다.

| 항목 | 실측 결과 |
|---|---|
| 시험 시각 | 2026-09-13 15:35:55 KST |
| 서버 | `http://203.247.41.82:8001` |
| GET `/platform/` | 302. 로그인 화면으로 이동 안내 |
| GET `/platform/login/` | 200. HTML 응답 |
| GET `/api/drones/5/companion-mission/` | 200. JSON `ok=true`, 계약 `1.0` |
| WS `/ws/drones/5/` | 101. WebSocket 연결 응답 검증 성공 |
| HTTP 요청 소요 | 약 58~134ms. 각 1회, 통계 아님 |
| 인증 | 장치 키 없이 요청. 현재 읽기 요청은 수락됨 |
| 송신 범위 | GET·WS 연결·WS 종료만 전송 |
| 미실행 | 텔레메트리·단계·ACK·QR POST·비행 명령 |

응답 원문과 소요 시간은 [시험 기록](evidence/platform_connection_20260913.json)에 있다.
WS 연결 성공은 텔레메트리 저장 성공과 다르다.
UI 위치 갱신과 서버 DB 기록은 확인하지 않았다.
인증 모드가 audit인지는 서버 설정을 보지 못했다.
기본 도구 환경은 소켓을 막아 연결 시험에 실패했다.
네트워크 실행 권한을 적용한 시험에서 성공했다.

## 현재 서버 입력

5번 드론에 실행할 임무와 경로가 없다.

| 값 | 실제 응답 |
|---|---|
| `drone_id`, `drone_name` | `5`, `Drone-5` |
| `status` | `HOLD` |
| `mission`, `mission_db_id` | null |
| `mission_code`, `route_revision` | 빈 문자열 |
| `route_tasks`, `planned_route_metric` | 빈 배열 |
| `control_action`, `control_request_id` | null |
| `home_point` | null |
| `flight_z_m` | 1.2m |
| `mission_target_alt_m` | null |
| `coordinate_frame` | `UWB_ANCHOR_LOCAL` |
| `x_axis`, `y_axis` | `A1_TO_A2`, `A1_TO_A3` |
| `target_z_semantics` | `DESIRED_ALTITUDE_NOT_EKF_Z_NOT_UWB_TAG_Z` |
| `yaw_policy` | `DRONE_HEADING_A1_TO_A2_FIXED` |
| `anchor_layout_id` | `7ad8893e9f2b` |
| `anchor_layout_valid` | false |
| `anchor_layout_warning`, `anchor_layout_warnings` | null, 빈 배열 |

새 `yaw_policy`는 수신값으로만 기록했다.
기체 방향 제어에 적용하지 않았다.
배치가 무효인 구체적인 사유는 응답에 없다.

## 앵커 설정 차이

서버 앵커 값은 기존 실측 기록과 일치하지 않는다.
기존 좌표는 잠정값이며 이번에 재측정하지 않았다.

| 앵커 | 서버 x, y, z (m) | 기존 기록 x, y, z (m) |
|---|---|---|
| A1 | 0.01, 0.06, 2.10 | 0.00, 0.00, 2.20 |
| A2 | 4.91, 0.00, 0.00 | 5.810, -0.430, 2.20 |
| A3 | 0.01, 19.99, 0.00 | 0.00, 4.630, 2.20 |
| A4 | 5.00, 4.95, 2.10 | 6.110, 4.357, 2.20 |

기존 좌표 근거는 [API 대조 기록](evidence/companion_api_v1_review_20260913.json)에 있다.
서버의 축 정의도 기존 직교 좌표와 다르다.
[API 회신 검토](../companion_platform_api_v1_review.md) 5절을 따른다.
수신 배치를 태그에 전송하지 않았다.
이유: 실제 설치 기준과 맞는지 확인하지 못했기 때문이다.

## 다음 가능한 시험

현재 연결을 바탕으로 통신 기능을 단계별 확인할 수 있다.

| 기능 | 이번 확인 | 다음 시험 |
|---|---|---|
| 웹 임무 수신 | 현재 HOLD JSON 수신 | 테스트 임무 등록 후 경로·개정값 변화를 읽기 전용 확인 |
| 위치·상태 표시 | WS 연결만 확인 | 실제 ROS2 관측을 필드로 변환하고 화면 반영 확인 |
| 복귀·착륙 요청 수신 | 현재 요청 없음 | 비행 출력 없이 요청 ID·시각 변화 확인 |
| QR 결과 등록 | 미시험 | 지정 테스트 라벨로 수락·거부·중복 ID 처리 확인 |
| 임무 실행 | 미시험 | 좌표·배치·ROS2/PX4 연동 검증이 선행 |

이번 결과를 자동 비행 가능 판정으로 쓰지 않는다.
이유: 센서와 제어 경로는 이번 시험에 포함하지 않았다.
