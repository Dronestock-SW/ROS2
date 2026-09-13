# Platform API v1 회신 검토
이 문서는 9월 11일 회신과 ZIP의 대조 결과다.
통신 구현 범위와 재확인 항목을 정할 때 읽는다.

## 1. 결론

통신 계약은 구체화됐다. 바로 배포할 상태는 아니다.
좌표축 불일치와 코드의 남은 차이를 먼저 맞춘다.

| 항목 | 판단 |
|---|---|
| 기존 질문 49개 | 모두 상대 회신 수신. 구현 검증 완료와는 다름 |
| 내부 구조 | 상대 계약도 ROS2·PX4 내부 구조를 변경하지 않음 |
| 위치 표시 | UWB의 검증된 수평 관측값 사용 |
| 고도·IMU | PX4 출처 허용. 미지원 세부값은 null/생략 |
| 통신 | 임무 조회 2Hz, 상태 전송 기본 10Hz |
| 원래 의도한 비행 중 경로 변경 | v1에서 금지. 아직 충족하지 않음 |
| 먼저 맞출 값 | 창고 좌표축·태그 적용 배치·전체 임무 응답 |
| 이번 작업 | 문서 갱신·통신 함수 오프라인 대조 |
| 미수행 | 설치·서비스 기동·장비 접속·웹 서버 요청·제어 변경 |

이 문서의 확정값은 상대방이 정한 계약을 뜻한다.
우리 비행 정책까지 승인했다는 뜻은 아니다.
현재 기술 기준은 [로드맵](roadmap.md)을 유지한다.

## 2. 수신 자료와 근거

별도 계약서와 ZIP 내부 계약서의 내용이 일치한다.

| 자료 | 확인 결과 |
|---|---|
| [별도 계약서](hw_handoff/platform_api_v1/companion_platform_api_contract_v1.md) | 상대 기준일 2026-09-11, `contract_version="1.0"` |
| [새 ZIP](hw_handoff/platform_api_v1/DroneStock-Companion-Jetson-20260911.zip) | 실제 SHA256 아래 값 |
| ZIP SHA256 | `5DD97E663E450D60B8A88CA00B79D9A950DB5094CB07EE6863A65E96D24F528B` |
| ZIP의 `API_CONTRACT_V1.md` | 별도 계약서와 디코딩한 텍스트 일치 |
| [ZIP에서 꺼낸 회신 원문](hw_handoff/platform_api_v1/API_CONFIRMATION_RESPONSE.md) | C01~C49 회신 포함 |
| [기존 확인표](companion_platform_api_confirmation.md) | 질문 원문 유지. 상대 회신 49개 반영 |
| [9월 10일 필드 사전](companion_platform_api_dictionary.md) | 이전 버전의 검토 이력. 최신 변경은 이 문서 참조 |
| [오프라인 대조 결과](report/evidence/companion_api_v1_review_20260913.json) | 함수에 합성 입력을 넣어 확인한 결과 |
| 검토일 | 2026-09-13 |

새 ZIP의 예상 해시는 별도로 받지 않았다.
위 값은 수신 파일에서 계산한 식별값이다.
서버 코드·서버 테스트는 이번 ZIP에도 없다.
서버 검증·DB 중복 방지는 상대 회신으로만 확인했다.

소스 경로는 ZIP 내부 `tools/companion/` 기준이다.
서버의 현재 IP·실행 상태는 재확인하지 않았다.
예전 `203.247.41.82`를 현재 주소로 확정하지 않는다.
이유: v1 계약은 현장 PC의 LAN IP를 사용한다.

## 3. 반영을 확인한 변경

주요 통신 수정은 실제 ZIP 코드에도 들어 있다.

| 관련 질문 | 회신 내용 | 코드 확인 | 한계 |
|---|---|---|---|
| C03 | HTTP 장치 인증 추가 | `io/server_client.py:259`, HMAC 헤더 생성 | WS 인증 처리와 서버 검증은 별도 |
| C05 | 텔레메트리 10Hz | `config.py` 기본 10, `app.py:246` 전송 간격 제한 | HTTP가 같은 루프를 지연시킬 수 있음 |
| C06 | 2xx + 업무 결과 검사 | `io/server_client.py:238`의 `_post()`가 `ok` 검사 | GET·QR 처리와 세부 규칙이 다름 |
| C08 | 계약 버전 추가 | 파서·WS·단계·ACK·출발 등록에 `1.0` | 미지원 버전 거부는 없음 |
| C13 | 순서·나이·최신 1건 큐 | `app.py:462`, `ServerClient` 기본 큐 1 | 서버에서 순서 역행을 거부하는지는 미검증 |
| C25 | 경로 개정값 추가 | `RouteManager`가 `route_revision`을 키로 사용 | 변경 시 진행 번호 0으로 초기화 |
| C30 | 임무 DB ID·코드 분리 | GET 파서와 QR payload에 반영 | 로컬 일부 키는 구형 `mission_id` 사용 |
| C33 | 실패 때 성공 시각 보존 | GET 예외에서 `received_monotonic` 갱신 제거 | 3초 감시의 실시간 작동은 통합 검증 필요 |
| C35·C36 | 요청 ID ACK·실패 재시도 | `app.py:413`, 성공 때만 처리 키 기억 | 상태 이탈 후 재시도 누락 가능. 6절 참조 |
| C37 | 단계 중복 키에 임무 포함 | DB ID·코드·phase·notes로 키 생성 | 서버의 임무 검증은 미검증 |
| C39 | 출발 등록에 임무 식별자 | `app.py:310`에서 두 ID 추가 | 일부 로컬 중복 판정은 구형 ID 사용 |
| C46 | QR 로컬 관리값 미전송 | `OfflineScanBuffer.flush()`가 두 키 제거 | 직접 호출자가 넣는 추가 키까지 막는 것은 아님 |
| C48·C49 | FC·임무·진행 상태 추가 | `app.py:462`의 WS payload에 반영 | 복귀 시 활성 지점 보고에 차이 있음 |

상태 전송 제한과 제어 루프는 별도 설정이다.
그러나 임무 HTTP 조회는 여전히 주 루프에서 실행한다.
따라서 20Hz 제어와 10Hz 송신을 보장하지 않는다.
이유: HTTP 대기와 공유 요청 잠금이 지연을 만든다.

## 4. 추가·변경 필드의 의미

새 필드는 다음 경로로 들어오거나 나간다.
기존 필드는 [이전 사전](companion_platform_api_dictionary.md)에 있다.

```text
Platform GET → 임무 DB ID·코드·개정·요청 ID
                    |
                    v
              companion 임무 처리
                    |
                    +→ 상태 진입 → 요청 ID ACK
                    +→ 임무 단계 → 임무 ID 포함 POST
                    +→ QR 원문 → DB ID·코드 포함 POST

UWB 관측·PX4 상태 → 10Hz 제한 → 최신 1건 큐 → WS
```

### 4.1 연결·인증

HTTP 서명은 전송하는 본문 바이트를 기준으로 만든다.
이 절은 계약 설명이며 실제 키 설정은 하지 않았다.

| 값 | 1. 정의 | 2. 로직 | 3. 역할 | 4. 파이프라인 |
|---|---|---|---|---|
| `DRONESTOCK_TELEMETRY_HZ` | 송신 목표 빈도. Hz | 기본 10. 루프에서 주기 제한 | 서버 전송량 조절 | 환경변수 → 설정 → WS 큐 |
| `DRONESTOCK_DEVICE_AUTH_ID` | 발급 장치 ID. 문자열 | 비밀키와 모두 있을 때 서명 | 장치 식별 | 환경변수 → HTTP 헤더 |
| `DRONESTOCK_DEVICE_AUTH_SECRET` | 발급 공유 비밀키 | UTF-8 바이트로 HMAC 계산 | 요청 검증 | 환경변수 → 로컬 서명 |
| `X-DS-Device-ID` | 요청 장치 ID | 설정값 전달 | 서버 키 조회 | companion → HTTP → 서버 |
| `X-DS-Timestamp` | 요청 생성 Unix 초 | `int(time.time())` 문자열 | 요청 시각 검증 | 로컬 시계 → HTTP → 서버 |
| `X-DS-Nonce` | 매 요청 고유값 | UUID의 32자리 hex | 같은 요청 재사용 구별 | 요청 생성 → HTTP → 서버 |
| `X-DS-Signature` | HMAC-SHA256의 hex 결과 | 아래 5줄을 서명 | 본문·경로 검증 | 서명 → HTTP → 서버 |
| 서버 `DEVICE_AUTH_MODE` | 서버 인증 정책 | `audit`: 미서명 호환. `enforce`: 서명 요구 | 전환기·운영 구분 | 서버 설정 → 인증 처리 |

```text
대문자 HTTP_METHOD
요청 path + query
SHA256(실제 HTTP body 바이트)의 hex
Unix 초 문자열
nonce
```

줄 사이는 LF다. ZIP 코드는 끝 LF를 붙이지 않는다.
GET의 본문은 빈 바이트다.
POST는 UTF-8 JSON 바이트를 서명하고 그대로 보낸다.
허용 시계 오차·nonce 보존 시간은 계약에 없다.
서버와 공통 서명 예제를 추가로 맞출 필요가 있다.
WS 연결에는 이 헤더를 넣는 코드가 없다.
계약도 WS를 로컬망 또는 `wss` 호환 채널로 둔다.

### 4.2 임무 입력·결과 보고

임무 코드와 DB ID를 서로 바꾸어 쓰지 않는다.
이유: 문자열 코드와 정수 ID는 다른 식별자다.

| 값 | 1. 정의 | 2. 로직 | 3. 역할 | 4. 파이프라인 |
|---|---|---|---|---|
| `contract_version` | 통신 계약 버전 문자열 | `1.0`. GET에서 보관. 주요 출력에 추가 | 계약 식별 | GET → 로컬 상태 / companion → 서버 |
| GET `generated_at` | 서버 응답 생성 시각 | 예시에 있음. ZIP은 원본 JSON에만 보관 | 서버 시각 진단 | 서버 → GET 원본 |
| GET `coordinate_frame` | 서버 좌표계 문자열 | 계약 `UWB_ANCHOR_LOCAL`. ZIP은 원본에만 보관 | 목표 좌표 해석 | 서버 → GET 원본 |
| GET `origin` | 좌표 원점 | 계약 `A1`. ZIP은 직접 검증하지 않음 | 원점 설명 | 서버 → GET 원본 |
| GET `x_axis` | +x 방향 | 계약 `A1_TO_A2`. 우리 기준과 충돌 | 축 설명 | 서버 → GET 원본 |
| GET `y_axis` | +y 방향 | 계약 `A1_TO_A3`. ZIP은 직접 검증하지 않음 | 축 설명 | 서버 → GET 원본 |
| GET `z_axis` | +z 방향 | 계약 `UP_FROM_FLOOR` | 고도 기준 설명 | 서버 → GET 원본 |
| GET `unit` | 길이 단위 문자열 | 계약 `meter`. ZIP은 직접 검증하지 않음 | 단위 설명 | 서버 → GET 원본 |
| `mission_db_id` | 임무 DB 정수 ID/null | GET에서 읽음. WS·단계·출발 POST에 사용 | DB 임무 연결 | GET → 임무 상태 → 결과 보고 |
| `mission_code` | 임무 코드 문자열 | GET에서 읽음. WS·단계·출발·QR에 사용 | 임무 코드 연결 | GET → 임무 상태 → 결과 보고 |
| 구형 `mission_id` | 이전 호환 식별자 | 신규 필드와 별도. 일부 로컬 키에서 계속 사용 | 구형 코드 호환 | GET → 로컬 경로/QR/출발 키 |
| `route_revision` | 전체 경로의 개정 해시 문자열 | 값이 바뀌면 경로를 새로 동기화 | 경로 버전 대조 | GET → RouteManager → WS |
| `control_request_id` | 제어 요청 고유 ID | 중복 ACK 키로 우선 사용 | 요청 구별 | GET → 처리 → ACK `request_id` |
| ACK `request_id` | 수락한 제어 요청 ID | GET의 ID 복사 | 요청 수락 연결 | FSM 상태 진입 → POST |
| ACK `requested_at` | 원래 요청 시각 | GET `control_requested_at` 복사 | 이전 요청 대조 | GET → POST → 서버 |
| 출발 `captured_for_mission` | 구형 임무 식별값 | ZIP이 추가. 계약서 필수값에는 없음 | 이전 임무 문맥 | 로컬 상태 → 출발 POST |
| QR `mission_id` | QR API의 DB 정수 ID | 내부 `mission_db_id`를 이 이름으로 전송 | QR·DB 임무 연결 | GET → QR 문맥 → scan POST |

서버는 오래된 요청 ID·시각을 409로 거부한다고 회신했다.
하지만 이것은 명령 만료 시간 정의가 아니다.
명령의 수명·우선순위는 별도 미합의 항목이다.

### 4.3 텔레메트리 추가값

FC 상태와 UWB 관측 품질을 분리해서 표시한다.

| 값 | 1. 정의 | 2. 로직 | 3. 역할 | 4. 파이프라인 |
|---|---|---|---|---|
| `coordinate_frame` | 관측 좌표계 이름 | 계약은 `UWB_ANCHOR_LOCAL`. ZIP WS에는 누락 | 좌표 의미 구별 | 좌표 계약 → WS에 반영 필요 |
| `sent_at_ms` | 메시지 생성 Unix ms | payload 생성 때 기록 | 메시지 시각 | companion 시계 → WS |
| `t` | 구형 송신 시각 별칭 | `sent_at_ms`와 동일 | 이전 화면 호환 | companion → WS |
| `telemetry_seq` | 프로세스 내 송신 생성 번호 | payload 생성마다 +1. 프로세스 재시작 시 초기화 | 순서 진단 | 생성부 → WS |
| `uwb_seq` | 태그 관측 순서 번호/null | 파싱된 태그 번호 전달 | 센서 순서 진단 | 태그 → 상태 → WS |
| `uwb_age_ms` | companion 수신 이후 나이. ms/null | 로컬 단조 시계 차이 | 관측 최신성 표시 | 수신 시각 → 계산 → WS |
| `active_waypoint_index` | 현재 경로 배열 번호/null | 경로의 active가 있으면 index | 진행 표시 | RouteManager → WS |
| `active_waypoint_id` | 현재 경로 지점 문자열/null | 경로의 active에서 point_id | 지점 표시 | RouteManager → WS |
| `fc_connected` | FC heartbeat 연결 판정 | ZIP은 최근 heartbeat 2초 미만으로 계산 | FC 연결 표시 | MAVLink → FCState → WS |
| `fc_armed` | FC 시동 상태. 불리언 | FC 상태 전달 | 실제 시동 표시 | MAVLink → FCState → WS |
| `fc_mode` | FC 모드 문자열/null | 비어 있으면 null | 실제 모드 표시 | MAVLink → FCState → WS |

센서 측정 시각과 메시지 생성 시각은 다르다.
`uwb_age_ms`도 전파 측정부터의 전체 지연은 아니다.
재시작 후 seq 초기화를 서버가 어떻게 처리할지는 미확인이다.

### 4.4 유지하거나 선택으로 바꾼 값

선택값은 우리 장비가 실제 제공할 때만 보낸다.

| 값 | 상대 회신 | 우리 적용 판단 |
|---|---|---|
| `x`, `y` | 검증된 UWB 관측, m | `/uwb_pose` 의미와 맞음. 좌표축 수정 먼저 |
| `fix`, `telemetry_verified` | 최종 관측 유효성 | FC 전달·EKF 융합과 별개 |
| `xy_control_usable` | 태그 진단값 | RAW 계약에서 동명 값이 없을 때 처리 확인 필요 |
| `current_z_source` | `px4_ekf2` 허용 | PX4 고도 표시 가능. companion z 제어 추가 없음 |
| UWB z·3D/4D·IMU 상세·`final_rc` | 선택. null/생략 허용 | 기존 ROS2·PX4 구성에 없는 값을 만들지 않음 |
| `uwb4_residual_m` | 같은 계산을 지원할 때만 전송 | 우리 거리 제곱 차 잔차와 다른 값 |
| QR `ok=true, accepted=false` | 거부 기록을 DB에 저장한 정상 처리 | 재전송 버퍼에서 제거 가능. 서버 보존은 미검증 |
| QR 오프라인 완료 | 로컬 저장 뒤 다음 지점 진행 허용 | 상대의 현재 정책. 우리 임무 정책 채택은 별도 |
| `anchor_layout_version=4` | 4-anchor 형식 버전 | 좌표 개정과 분리 |
| `anchor_layout_id` | 앵커 역할·좌표의 해시 | 해시 규격·길이·실제 적용값 추가 확인 |

## 5. 좌표축 불일치

현재 계약의 두 축은 우리 배치에서 직각이 아니다.
이는 이름 변경만으로 해결할 수 없다.

| 기준 | +x | +y | A2 좌표 |
|---|---|---|---|
| 우리 `uwb_map` | A1→A3를 볼 때 오른쪽 | A1→A3 | 약 `(5.810, -0.430)m` |
| 상대 계약 | A1→A2 | A1→A3 | x축 위에 있다고 해석하게 됨 |

근거는 로컬 `docs/uwb_anchor_survey.md` 기록이다.
필요한 좌표는 [대조 기록](report/evidence/companion_api_v1_review_20260913.json)에 보존했다.
잠정 좌표에서 A1→A2는 x축 대비 약 −4.23°다.
A1→A2와 A1→A3 사이 각도는 약 94.23°다.
이는 기존 잠정 좌표로 계산한 값이다.
새 실측 정확도를 뜻하지 않는다.

```text
                +y = A1→A3
                ^
                |
A1 -------------+--------------------> 우리 +x
                  \
                   \ A1→A2 (약 -4.23°)
```

제안은 서버도 우리 직교 좌표 정의를 쓰는 것이다.
`+x = A1→A3에 수직인 오른쪽`으로 수정한다.
앵커 좌표도 실측 기반 불규칙 배치를 보존한다.
두 앵커 방향을 각각 x·y축으로 고정하지 않는다.
이유: 거리·경로 계산이 직교 좌표를 전제로 하기 때문이다.

## 6. 계약과 코드의 남은 차이

문서상 해결과 실행 코드의 해결을 구분해야 한다.

| 번호 | 발견한 차이 | 근거 | 필요한 정리 |
|---|---|---|---|
| R01 | 계약의 +x가 우리 축과 다름 | 5절, 계약 4·5절 | 좌표 정의와 서버 앵커 실제값 수정 |
| R02 | 비행 중 경로 수정 금지지만 HOLD 행동은 미정 | 계약 5·10절. `RouteManager.sync()`는 상태 검사 없음 | HOLD를 안전한 경로 교체 조건으로 쓰지 않음. 착륙·시동 해제 또는 명시적 교체 가능 상태 필요 |
| R03 | 계약 GET 예시는 전체 이륙 입력이 아님 | 예시 파싱 시 `launch_authorized=false`, `anchor_layout_valid=false`, anchors 0개 | 정상 임무의 전체 응답 JSON 필요 |
| R04 | 태그 배치 ID 불일치가 WS에서 숨겨짐 | `app.py:520` 부근. 서버 ID 우선, synced는 태그 bool 그대로 | 서버/태그 ID를 별도 전송. 두 값 일치 여부로 정상 표시 |
| R05 | 계약은 XY-only 허용. 번들은 기존 품질 조건 유지 | XY만 넣으면 `POSITION_ONLY`, resolver가 거부 | 우리 RAW 어댑터의 XY 전용 판정과 별개로 계약 적용. 번들 전체 설치로 해결되지 않음 |
| R06 | ACK 재시도는 해당 상태에 있을 때만 수행 | `app.py:413`. RETURN 실패 뒤 LAND에서는 복귀 ACK를 다시 보내지 않음 | 수락 기록과 대기 ACK를 상태 전환 뒤에도 유지할지 합의 |
| R07 | RETURN 중 활성 지점이 HOME 대신 경로 지점 | `_telemetry_payload()`가 FSM 목표 대신 route active 조회 | 실제 제어 목표와 표시용 경로 진행을 구분 |
| R08 | 재부팅 자체를 감지하는 코드가 없음 | `_sync_anchor_layout()`는 ID/bool 확인. reader는 재연결 시 이전 상태 유지 | 재연결·태그 재시작 후 이전 synced 무효화와 재확인 필요 |
| R09 | WS `coordinate_frame` 누락 | `_telemetry_payload()` 고정 필드 대조 | 필수 여부 확정 후 전송 추가 |
| R10 | GET `ok=false`도 파싱 후 성공 시각 갱신 | `poll()`·`parse_mission_payload()`. `LaunchGuard`는 ok 직접 미검사 | HTTP 성공과 유효 임무 수신을 분리할지 확정 |

R04는 합성 입력으로 재현했다.
태그 `tag-old`, 서버 `server-new`를 입력했다.
출력은 `server-new`와 `tag_layout_synced=true`였다.
서버는 이 payload만으로 실제 태그 ID를 알 수 없다.

R08은 불일치 보고 시 재전송 개선은 확인했다.
같은 ID라도 불일치하면 2초 간격으로 재시도한다.
다만 이전 synced 상태가 남으면 재부팅을 놓칠 수 있다.

QR 거부 기록 보존·10초 연결 표시·401/409 처리는 미검증이다.
이유: 서버 코드와 실행 응답이 이번 자료에 없다.

## 7. 추가로 확인할 계약값

남은 질문은 다음 항목으로 좁힐 수 있다.

| 항목 | 요청할 내용 |
|---|---|
| 좌표 | +x 정의 수정 승인·서버 A1~A4 실제 좌표 |
| 경로 변경 | 비행 중 변경을 후속 버전으로 둘지 결정. HOLD의 실제 행동 정의 |
| 응답 예시 | 승인·대기·취소·복귀 요청의 전체 GET JSON |
| 배치 | 서버 ID/태그 ID의 별도 필드명·해시 길이와 계산 규격 |
| 재전송 | 상태 이탈 후 ACK 보존·409 뒤 처리·동일 ID 재요청 응답 |
| 진행 표시 | RETURN 때 HOME과 경로 진행 번호를 어떻게 표시할지 |
| 인증 | 장치 발급 방식·허용 시계 오차·nonce 보존·공통 서명 예제 |
| 선택값 | RAW 태그에 없는 `xy_control_usable`의 null/생략 처리 |
| 서버 증거 | QR 중복 ID·거부 저장·임무 불일치 응답 예시 |
| 기체 ID | 서버 `5`의 실제 1호기 배정 확인 |

이는 통신 구현을 구체화하기 위한 확인 사항이다.
사용자에게 설치 승인을 요청하는 문서는 아니다.

## 8. 검증 범위

장비 없이 확인 가능한 통신 함수만 대조했다.

| 확인 | 결과 |
|---|---|
| 최신 1건 큐 | 합성 메시지 3개 뒤 마지막 1개만 남음 |
| HTTP 실패 후 시각 | 기존 성공 시각 유지 |
| 경로 개정 변경 | 진행 번호 0으로 초기화 |
| QR 재전송 | 로컬 두 키 제외. 거부 정상처리 시 버퍼 제거 |
| XY-only 입력 | 품질이 `POSITION_ONLY`가 되어 거부 |
| 배치 ID·복귀 목표·ACK | 6절 R04·R07·R06 재현 |
| 계약 예시 파싱 | 이륙 승인·앵커 정보 누락 확인 |
| 서버 인증·DB·실제 비행 | 미실행 |

ZIP 내부 경로는 Windows 역슬래시로 저장돼 있다.
이번에는 `/tmp`에서 경로를 정규화해 읽었다.
Python 기본 압축 해제는 Linux에서 폴더를 만들지 못했다.
실제 설치 도구의 해제 결과는 별도 확인이 필요하다.
설치 스크립트는 실행하지 않았다.
