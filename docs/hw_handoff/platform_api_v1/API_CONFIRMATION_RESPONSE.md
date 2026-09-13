# Companion API 확인표 회신 — Platform 담당

검토 기준: 상대 측 `companion_platform_api_confirmation.md`, `companion_platform_api_dictionary.md`  
회신일: 2026-09-11  
최종 통신 규격: `companion_platform_api_contract_v1.md`

## C01–C08 연결/응답

| 번호 | 회신 |
|---|---|
| C01 | 경로와 방향이 맞다. GET 1, POST 4, 송신 전용 WS 1을 사용한다. |
| C02 | URL ID는 Platform의 `DroneUnit.drone_id`다. 현재 `5`는 해당 장비에 배정된 경우에만 1호기로 사용한다. ROS domain ID와는 별개다. |
| C03 | 브라우저 로그인과 분리된 HMAC 장치 인증을 사용한다. 로컬 audit는 unsigned 호환, 운영 enforce는 4개 `X-DS-*` 헤더가 필수다. |
| C04 | 현재 임무/복귀/착륙은 0.5초 GET polling으로만 받는다. WS는 Companion 송신 전용이다. |
| C05 | 제어 루프 20 Hz와 서버 송신을 분리한다. 텔레메트리 기본/권장 10 Hz, 명령 정상 지연은 0.5초+네트워크, UI disconnect는 10초다. |
| C06 | 2xx이면서 `ok`가 false가 아니어야 성공이다. 업무 오류는 400/404/409, 인증은 401이다. |
| C07 | 계약서 필수 필드를 따른다. 미지원 센서는 null/생략, 추가 필드는 무시한다. 미측정값을 0으로 보내지 않는다. |
| C08 | `contract_version: "1.0"`을 추가했다. 기존 필드는 제거하지 않아 구버전 호환한다. |

## C09–C22 좌표/센서/앵커

| 번호 | 회신 |
|---|---|
| C09 | x/y는 UWB 창고 로컬의 검증된 관측이다. PX4 EKF 위치와 의미가 다르며 향후 필요 시 별도 필드로 둔다. |
| C10 | `UWB_ANCHOR_LOCAL`, A1 원점, +x A1→A2, +y A1→A3, +z 바닥 위, 단위 m다. |
| C11 | `telemetry_verified`는 관측 유효성에 한정한다. FC 연결/armed/mode를 별도 추가했다. |
| C12 | `xy_control_usable`은 태그 진단값으로 보존하고 Companion 최종값은 `fix`/`telemetry_verified`다. |
| C13 | `sent_at_ms`, `telemetry_seq`, `uwb_seq`, `uwb_age_ms`를 추가했다. WS 큐는 최신 1건만 유지한다. |
| C14 | 서버는 마지막 Companion 텔레메트리 10초 후 disconnected로 표시한다. HTTP mission 연결 상태는 Companion 내부 진단으로 분리한다. |
| C15 | UWB XY-only를 허용한다. UWB z/3D/4D는 null 또는 생략 가능하다. |
| C16 | PX4 고도는 `current_z_source=px4_ekf2`를 허용한다. 근거 없는 confidence는 생략/null로 둔다. |
| C17 | MPU6050 객체는 선택이다. PX4 IMU를 같은 의미로 위장하지 말고 실제 출처/지원값만 보낸다. |
| C18 | battery는 0~100 %, voltage는 V, 미상은 null이다. |
| C19 | 의미가 다른 잔차는 같은 이름으로 보내지 않는다. `uwb4_residual_m`은 해당 계산을 실제 지원할 때만 보낸다. |
| C20 | 서버 ID와 태그 적용 ID를 구분하며 일치+synced 전에는 정상으로 표시하지 않는다. |
| C21 | version 4는 형식 버전이다. 좌표 개정은 content hash인 `anchor_layout_id`가 담당한다. |
| C22 | 좌표 변경 시 서버가 새 hash ID를 생성한다. 태그 재부팅/불일치 시 같은 ID도 재전송하도록 수정했다. |

## C23–C34 임무/경로/명령

| 번호 | 회신 |
|---|---|
| C23 | 실제 이동 경로는 `route_tasks`다. 최상위 target은 현재 활성 목표 표시/구버전 호환이다. |
| C24 | 현재 경로 revision 변경 시 index 0으로 재동기화한다. 따라서 v1에서는 비행 중 경로 수정 금지다. |
| C25 | `route_revision`을 추가했다. 수락/거부 handshake는 v1에 없으며 서버가 제공한 revision을 Companion이 표시/보고한다. |
| C26 | 서버 UI 입력은 전체 경로를 검증한다. Companion에 직접 비정상 배열을 주입하지 않는다. 엄격한 클라이언트 전체 거부는 차기 호환 변경이다. |
| C27 | 각 route task에 z를 명시한다. 서버의 `mission_target_alt_m`은 목표 고도이며 현재 고도나 EKF z가 아니다. |
| C28 | SCAN 판단은 `type=scan`; `required_scan`은 서버 메타데이터다. |
| C29 | 일반 0.7초, scan 최소 2초, hover 지정 `hold_s`를 유지한다. |
| C30 | `mission_db_id`(정수)와 `mission_code`(문자열)를 분리했다. QR은 둘 다 보내며 code에는 문자열 코드를 넣는다. |
| C31 | ACTIVE만 이륙 조건이다. HOLD/CANCEL의 비행 중 행동은 안전정책 합의 전 임의 변경하지 않는다. |
| C32 | `control_request_id`와 원래 `control_requested_at`을 추가했다. 반복 GET은 값을 변경하지 않는다. |
| C33 | 조회 실패 시 마지막 성공 수신 시각을 갱신하지 않도록 수정했다. 기존 timeout 기준 3초가 실제로 작동한다. |
| C34 | v1 action은 `return_to_home`, `land` 두 가지다. 우선순위/동시 처리 확장은 별도 합의다. |

## C35–C49 결과/QR/표시

| 번호 | 회신 |
|---|---|
| C35 | ACK에 `request_id`, `requested_at`을 추가했다. ACK는 상태 진입 수락이며 물리 완료는 phase로 보고한다. |
| C36 | HTTP 성공 전까지 동일 request ID로 재시도하도록 수정했다. 오래된 ID는 서버가 409로 거부한다. |
| C37 | phase에 `mission_db_id`, `mission_code`를 넣고 중복 키에도 미션을 포함하도록 수정했다. |
| C38 | 현재 매핑은 유지한다. FAILSAFE 명칭 변경은 운영/안전 상태표 승인 뒤 한다. |
| C39 | 출발 등록에 두 미션 식별자와 captured 시각이 포함된다. 서버가 명시된 미션을 검증한다. |
| C40 | `drone-stock-item/v1` JSON을 해석한다. 원문은 `raw_qr_data`로 보존한다. |
| C41 | `label_point_id`는 DB 정수 ID, `point_id`는 사람이 읽는 경유지 문자열이다. 업로드 연결에는 정수 ID를 우선 사용한다. |
| C42 | 서버는 `client_scan_id`로 accepted/rejected 모두 중복 방지한다. 재시도는 같은 UUID를 쓴다. |
| C43 | 새 판독 UUID는 별도 관측으로 기록될 수 있다. 임무/라벨 중복 정책은 업무 규칙 변경이므로 현 상태 유지다. |
| C44 | 현재는 네트워크 장애 시 로컬 저장 후 진행 허용이다. 수락 필수 정책은 별도 합의가 필요하다. |
| C45 | `ok:true, accepted:false`는 rejected 로그가 DB에 저장된 정상 처리라 버퍼에서 제거한다. |
| C46 | `attempts`, `last_error`를 업로드에서 제외하도록 수정했다. |
| C47 | `final_rc`는 선택 진단이다. PX4 실제 상태는 FC 필드로 분리했다. |
| C48 | `fc_connected`, `fc_armed`, `fc_mode`를 추가했다. |
| C49 | `mission_db_id`, `mission_code`, `route_revision`, `active_waypoint_index`, `active_waypoint_id`를 텔레메트리에 추가했다. |

## 상대 팀 적용 요청

1. 이 회신보다 `companion_platform_api_contract_v1.md`를 구현 기준으로 사용한다.
2. 서버 주소, drone ID, UWB/Pixhawk 장치 경로는 현장값으로 설정한다.
3. 운영 HMAC을 켜기 전 장치 ID/비밀키를 별도 안전 채널로 발급받는다.
4. ROS2/PX4 내부 값은 의미가 맞는 외부 필드에만 매핑하고 미지원값은 null/생략한다.
5. 비행 중 route revision 변경, HOLD/CANCEL, failsafe phase, QR offline 진행 정책은 별도 합의 전 현재 동작을 유지한다.

