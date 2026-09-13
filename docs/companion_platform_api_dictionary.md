# companion–Platform API 필드 사전
이 문서는 9월 10일 ZIP의 API 필드 사전이다.
이전 요청과 변경 내용을 비교할 때 읽는다.

9월 11일 회신을 받았다. 아래 표는 이전 코드 이력이다.
최신 내용은 [v1 회신 검토](companion_platform_api_v1_review.md)를 본다.
상대 원문은 [API 계약 v1](hw_handoff/platform_api_v1/companion_platform_api_contract_v1.md)이다.

## 1. 검토 기준

이 문서는 코드에서 확인한 계약 초안이다.
서버 측 필수값과 저장 규칙은 아직 미확정이다.
확인 질문은 [확인표](companion_platform_api_confirmation.md)에 모았다.

| 항목 | 기준 |
|---|---|
| 검토일 | 2026-09-10 |
| 원본 | `DroneStock-Companion-Jetson-20260910.zip`. 현재 workspace에는 없음 |
| SHA256 | `719AB4CC8FC43211C1C350E2D84060681B1EB4927DE7AC2FB9748DB85734C7BC` |
| 해시 확인 | 전달값과 실제 파일이 일치 |
| ZIP 내부 루트 | `DroneStock-Companion-Jetson/` |
| 코드 공통 경로 | 아래 소스 경로 앞에 `tools/companion/`을 붙임 |
| 포함 범위 | companion 클라이언트·제어·센서 코드 |
| 미포함 범위 | Django 서버·DB·웹 화면·테스트 코드 |
| 버전 문자열 | `__init__.py`의 `0.1.0` |
| 검증 범위 | 정적 코드 검토. 서버·장비 접속 검증 없음 |
| 배포 상태 | 설치·서비스 실행·비행 설정 변경 없음 |

표의 로직은 ZIP의 현재 동작이다.
우리 시스템에 채택했다는 뜻은 아니다.
우리 대응값은 10절의 제안으로 구분한다.
변수명이 같아도 의미가 같아야 연결할 수 있다.

| 표기 | 읽는 법 |
|---|---|
| 수 | 유한한 숫자. 단위는 별도 표기 |
| 불리언 | JSON의 `true` 또는 `false` |
| `null` | 값이 없거나 사용할 수 없음 |
| 기본값 | 클라이언트가 필드 누락 때 선택하는 값 |
| 미사용 | 파싱·보관하지만 해당 실행 로직에서 사용하지 않음 |
| 경로 | 값을 만든 곳 → 처리하는 곳 → 소비하는 곳 |

미지원 값을 0이나 정상으로 채우지 않는다.
이유: 미측정을 실제 측정으로 오해하게 된다.
서버의 `null`·필드 생략 허용은 확인 대상이다.

## 2. 연결과 API 목록

명령은 HTTP 조회로 받고, 상태는 WebSocket으로 보낸다.
WebSocket 수신 처리 코드는 ZIP에 없다.

```text
웹 조작 → Platform 서버
                 |
                 | GET 응답: 임무·경로·복귀/착륙 요청
                 v
             companion → 로컬 임무 처리 → PX4
                 |
                 +-- WebSocket → 위치·센서·임무 상태 → 웹
                 +-- HTTP POST → 임무 단계·명령 ACK
                 +-- HTTP POST → 출발 좌표·QR 결과
```

### 2.1 주소와 식별자

주소와 장치 경로는 서로 다른 설정이다.

| 값 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `DRONESTOCK_SERVER_URL` | 서버 기본 주소. 문자열 | 번들 값 `http://203.247.41.82:8001`. 끝 `/` 제거 | HTTP·WS 접속 기준 | 환경변수 → `AppConfig` → `ServerClient` |
| `/platform/` | 사람용 화면 경로 | companion API 주소에 붙이지 않음 | 운영 화면 | 브라우저 → 서버 |
| `/platform/login/` | 사람용 로그인 화면 | 클라이언트는 이 경로로 로그인하지 않음 | 사용자 로그인 | 브라우저 → 서버 |
| `DRONESTOCK_DRONE_ID` | 서버 드론 식별자. 문자열 | 번들 값 `5`. HTTP·WS 경로에 삽입 | 요청 대상 선택 | 환경변수 → `/drones/5/` |
| `DRONESTOCK_COMPANION_ID` | companion 장치 식별자 | 번들 값 `jetson-drone-05`. 비면 호스트명 | 장치 식별 | 환경변수 → WS `companion_id` |
| `DRONESTOCK_COMPANION_PLATFORM` | 장치 플랫폼 이름 | 번들 값 `jetson-orin-nano` | 환경 식별 | 환경변수 → WS `companion_platform` |
| `DRONESTOCK_UWB_PORT` | 로컬 UWB 장치 경로 | 번들 `/dev/ttyUSB0`. 우리 설정 `/dev/uwb` | 센서 접근 | 로컬 설정 → 시리얼 수신 |
| `DRONESTOCK_UWB_BAUD` | UWB 시리얼 속도 | `921600` | 센서 통신 | 로컬 설정 → 시리얼 수신 |
| `DRONESTOCK_PIXHAWK_DEVICE` | 로컬 FC 장치 경로 | 번들 `/dev/serial0`. 우리 실제 연결과 별도 확인 | FC 접근 | 로컬 설정 → MAVLink |
| `DRONESTOCK_PIXHAWK_BAUD` | FC 시리얼 속도 | 번들 `57600`. 우리 기존 USB 점검값과 다름 | FC 통신 | 로컬 설정 → MAVLink |

서버 ID `5`와 `ROS_DOMAIN_ID=1`은 별개다.
둘 다 같은 1호기에 대응하도록 설정할 수 있다.
이는 식별자 대응이며 topic 접두어 변경이 아니다.

### 2.2 API별 계약

아래 `{id}`에는 현재 `5`가 들어간다.
HTTP 요청·응답 본문은 JSON이다.

| 방식·경로 | 방향·역할 | 호출 시점 | 성공 판정·오류 처리 | 근거 |
|---|---|---|---|---|
| GET `/api/drones/{id}/companion-mission/` | 서버 → companion. 임무·목표 조회 | 기본 0.5초 간격 | HTTP 오류 예외·JSON 파싱. 실패 시 기존 임무를 남기고 연결 실패 표시 | `io/server_client.py:95` |
| WS `/ws/drones/{id}/` | companion → 서버. 텔레메트리 | 주 루프마다 큐에 추가. 기본 루프 20Hz | `send()` 성공만 확인. 서버 저장 ACK 없음 | `io/server_client.py:207`, `app.py:421` |
| POST `/api/drones/{id}/companion-phase/` | companion → 서버. 임무 단계 보고 | 해당 FSM 상태에서 시도 | HTTP 2xx면 성공. 본문 미검사 | `io/server_client.py:129`, `app.py:355` |
| POST `/api/drones/{id}/control-action/ack/` | companion → 서버. 명령 처리 ACK | 복귀·착륙 상태로 전환했을 때 | HTTP 2xx. 호출부는 실패해도 처리 키를 기억 | `io/server_client.py:141`, `app.py:377` |
| POST `/api/drones/{id}/launch-snapshot/` | companion → 서버. 출발 좌표 등록 | 이륙 승인 상태이며 `home_point`가 없을 때 | HTTP 2xx. 성공한 임무 ID 기억 | `mission/launch_guard.py:75`, `app.py:284` |
| POST `/api/drones/{id}/scan/` | companion → 서버. QR 결과 등록 | SCAN 중 판독 또는 저장분 재전송 | HTTP 2xx와 응답 `ok`·`accepted`를 구분 | `io/qr_camera_scanner.py:142` |

| 통신 값 | 정의 | 로직 | 역할 | 경로 |
|---|---|---|---|---|
| HTTP 일반 timeout | 요청 대기 제한 | GET·단계·ACK·출발 등록은 2초 | 응답 지연 제한 | 클라이언트 → HTTP 요청 |
| QR timeout | QR 업로드 대기 제한 | 5초 | 업로드 응답 제한 | 스캐너 → HTTP 요청 |
| `mission_poll_s` | 조회 간격. 초 | 기본 0.5. 환경변수 로딩 항목은 없음 | 임무 갱신 빈도 | 설정 → `poll_if_due()` |
| `loop_hz` | 루프 목표 빈도 | 기본 20. HTTP 대기 등으로 실제 빈도 저하 가능 | 상태 생성 빈도 | `AppConfig` → 주 루프 → WS 큐 |
| WS 큐 크기 | 대기 메시지 수 | 기본 200. 가득 차면 가장 오래된 1개 제거 | 일시 지연 흡수 | 텔레메트리 → 메모리 큐 → WS |
| WS 재연결 | 끊긴 연결 재시도 | 연결 timeout 3초. 실패 뒤 1초 대기 | 연결 복구 | WS 스레드 → 서버 |
| HTTP/WS 인증 | 기체 인증 수단 | 명시적 로그인·토큰·인증 헤더 설정 없음 | 서버 허용 방식 확인 필요 | companion → 서버 인증부 |

큐의 과거 메시지도 재연결 뒤 전송할 수 있다.
전송 예외가 난 메시지는 큐에 복원하지 않는다.
따라서 최신성·전달 보장은 별도 합의가 필요하다.

## 3. 서버 → companion: 임무 응답

경로 실행은 `route_tasks`를 사용한다.
최상위 `target_x`만 바꿔도 이동한다는 계약은 없다.
근거는 `io/server_client.py:242`다.

### 3.1 임무와 제어 요청

최상위 상태와 내부 `mission.status`는 다르다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `ok` | 응답 업무 성공. 불리언 | 기본 false. 상태에 보관하나 이륙 검사가 직접 참조하지 않음 | 응답 유효성 의도 | GET → `MissionState.ok` |
| `status` | 드론 임무 실행 상태. 문자열 | 기본 `HOLD`, 대문자화. 이륙 조건은 `ACTIVE` | 임무 시작 허용 판단 | GET → `LaunchGuard` |
| `mission` | 임무 세부 객체 | 객체가 아니면 빈 객체 | 임무 메타데이터 묶음 | GET → 파서 |
| `mission_id` | 임무 식별자. 문자열로 변환 | 최상위 값 → `mission.mission_code` → `mission.id` → 빈 문자열 | 경로·스캔·출발 등록 식별 | GET → 임무 상태 → 경로/QR |
| `mission.mission_code` | 서버의 임무 코드 | 최상위 ID가 없을 때 내부 ID 후보 | 사람이 쓰는 임무 코드 | GET → 내부 `mission_id` |
| `mission.id` | 서버의 임무 ID | 앞선 두 값이 없을 때 후보 | DB 임무 연결 의도 | GET → 내부 `mission_id` |
| `mission.status` | 서버 임무 세부 상태 | 문자열 보관. FSM에서 직접 사용하지 않음 | 상태 메타데이터 | GET → `mission_status` |
| `mission.launch_authorized` | 이륙 승인. 불리언 | 기본 false. 센서·경로 조건도 함께 검사 | 웹 승인 전달 | 웹 → GET → 이륙 검사 |
| `launch_blocked_reason` | 이륙 차단 사유. 문자열/null | 비어 있지 않으면 차단 사유에 추가 | 승인 불가 이유 전달 | 서버 → GET → 이륙 검사 |
| `mission_target_alt_m` | 임무 기본 목표 고도. m/null | `target_alt_m` 대체값 허용. 경유지 기본 z는 이 원본 필드만 사용 | 경유지·고도 기준 | GET → 임무/경유지 파서 |
| `target_alt_m` | 목표 고도 별칭. m/null | 임무 고도·최상위 목표 z 대체값 | 이전 필드 호환 | GET → 임무 상태 |
| `target_x` | 최상위 목표 x. m/null | `target_x_m`에 보관. 경로 실행에서 직접 사용하지 않음 | 목표 메타데이터 | GET → 임무 상태 |
| `target_y` | 최상위 목표 y. m/null | `target_y_m`에 보관. 경로 실행에서 직접 사용하지 않음 | 목표 메타데이터 | GET → 임무 상태 |
| `target_z` | 최상위 목표 z. m/null | `target_alt_m` 대체값. 경로 실행에서 직접 사용하지 않음 | 목표 메타데이터 | GET → 임무 상태 |
| `route_tasks` | 순서 있는 경유지 객체 배열 | 비어 있으면 `planned_route_metric`. 유효하지 않은 항목은 제거 | 실제 순회 경로 | GET → 경유지 파서 → `RouteManager` |
| `planned_route_metric` | 경로 배열 별칭 | `route_tasks`가 비었을 때 사용 | 이전 경로 형식 호환 | GET → 경유지 파서 |
| `home_point` | 복귀 지점. 객체/null | 경유지와 같은 파서. 기본 지점 ID `HOME` | 복귀 목적지 | GET → RETURN → 도착 검사 |
| `control_action` | 원격 제어 요청. 문자열 | 앞뒤 공백 제거·소문자화. 구현 동작은 `return_to_home`, `land` | 복귀·착륙 요청 | 웹 → GET → FSM → ACK |
| `control_requested_at` | 제어 요청 시각. 문자열/null | 날짜 파싱 없음. `action`과 결합해 로컬 중복 ACK 키 생성 | 동일 요청 구별 | GET → ACK 처리부 |
| `drone_name` | 드론 표시명. 문자열 | QR 처리 시 원본 응답에서 읽음 | QR 기록 표시 | GET 원본 → QR POST |
| `drone_id` | 응답의 드론 표시 후보 | `drone_name`이 비면 사용. 그마저 없으면 설정 ID | QR 드론명 대체 | GET 원본 → QR POST |

숫자 변환은 숫자 문자열도 받아들인다.
`NaN`·무한대는 기본값으로 바꾼다.
Python 불리언도 숫자로 변환될 수 있다.
서버 계약은 JSON 숫자로 제한할지 확인한다.
`or`로 별칭을 고르면 숫자 0이 대체될 수 있다.
특히 최상위 고도 필드가 이 동작에 해당한다.

### 3.2 경유지 값

현재 파서는 x·y·z가 모두 있어야 수락한다.
이는 우리 수평 제어 계약과 별도 조정할 사항이다.
근거는 `io/server_client.py:279`다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `x` | 경유지 x. m | 유한수 필요. 없으면 `target_x` | 수평 목적지 | GET 경유지 → `x_m` → 경로 |
| `y` | 경유지 y. m | 유한수 필요. 없으면 `target_y` | 수평 목적지 | GET 경유지 → `y_m` → 경로 |
| `z` | 경유지 고도. m | `z` → `target_z` → 최상위 `mission_target_alt_m` | 높이 목표 | GET 경유지 → `z_m` → 도착/고도 처리 |
| `target_x` | 경유지 x 별칭 | `x` 변환 실패 시 사용 | 형식 호환 | GET 경유지 → `x_m` |
| `target_y` | 경유지 y 별칭 | `y` 변환 실패 시 사용 | 형식 호환 | GET 경유지 → `y_m` |
| `target_z` | 경유지 z 별칭 | `z` 변환 실패 시 사용 | 형식 호환 | GET 경유지 → `z_m` |
| `point_id` | 경유지 식별자. 문자열 | `point_id` → `id` → 빈 문자열 | 스캔 위치·작업 구별 | GET → 경로 → QR 문맥 |
| `id` | 경유지 ID 별칭 | `point_id`가 비면 사용 | 형식 호환 | GET → `point_id` |
| `type` | 경유지 작업 유형 | 기본 `waypoint`. 소문자화. 특별 처리 `hover`, `scan` | 이동·대기·스캔 선택 | GET → 도착 검사 → FSM |
| `hold_s` | 지점 유지 시간. 초 | 기본 0. `hover`는 그대로, `scan`은 최소 2초. 일반 지점은 기본 0.7초 사용 | 도착 후 유지 조건 | GET → `RouteManager.update_arrival()` |
| `arrival_radius_m` | 수평 도착 반경. m | 우선 적용. 미제공 시 cm 별칭 또는 0.2m. 값 0은 실행 시 기본 반경 사용 | XY 도착 범위 | GET → 도착 검사 |
| `arrival_radius_cm` | 수평 반경 별칭. cm | m 필드가 없으면 100으로 나눔 | 단위 호환 | GET → `arrival_radius_m` |
| `required_scan` | 스캔 필수 의도. 불리언 | 기본 false. 저장하지만 FSM 스캔 여부에 사용하지 않음 | 현재는 메타데이터 | GET → `Waypoint` |
| `label_point_id` | 서버 라벨 지점 ID. 정수/null | 경유지 원본에서 QR 처리 시 정수 변환 | 스캔·선반 연결 | GET 경유지 → QR POST |

도착 조건은 수평 반경과 고도 오차를 함께 쓴다.
ZIP의 고도 허용 오차 기본값은 0.15m다.
`required_scan=true`만으로 SCAN에 들어가지 않는다.
현재 FSM은 `type=scan`을 검사한다.

### 3.3 경로 변경·시각의 의미

현재 변경 처리는 경로 전체를 다시 시작한다.
명령 유효기간과 경로 변경 버전은 정의돼 있지 않다.

| 값·상황 | 정의 | 로직 | 역할 | 경로 |
|---|---|---|---|---|
| 내부 경로 키 | 임무 ID와 경유지 원본의 SHA1 | JSON 키 정렬 후 계산. 메타데이터 변경도 키를 바꿀 수 있음 | 경로 변경 감지 | GET → `_route_key()` |
| `RouteManager.index` | 현재 경유지 번호. 0부터 시작 | 경로 키 변경 시 0으로 초기화 | 다음 목적지 선택 | 경로 키 → index → active |
| 기본 고도만 변경 | 원본 경유지에 z가 없는 경우 | 해석된 z는 달라져도 원본 경로 키는 같을 수 있음 | 변경 감지의 한계 | GET 기본 고도 → 파싱 결과 |
| `received_monotonic` | companion에서 응답을 처리한 시각 | 성공뿐 아니라 GET 예외 때도 갱신 | 수신 나이 계산 | HTTP 처리 → 임무 상태 |
| `age_ms` | 위 시각 이후 경과. ms | 로컬 단조 시계로 계산 | 오래된 서버 입력 감시 | 임무 상태 → 안전 검사 |
| `server_reachable` | HTTP 조회 성공 여부 | 성공 true, 예외 false. WS 연결과 독립 | 이륙 전 서버 확인 | HTTP 결과 → 이륙 검사 |
| `server_max_age_ms` | 서버 입력 나이 제한 | 기본 3000. 현재 안전 검사는 나이 기준 | 서버 결손 감시 | 임무 나이 → 안전 상태 |

HTTP 실패도 나이를 갱신하는 점에 주의한다.
실패 반복을 3초 만료로 잡는다고 보장할 수 없다.
서버가 같은 과거 임무를 보내도 나이는 새로 시작한다.
서버 생성 시각·만료 시각 계약이 추가로 필요하다.

## 4. 서버 → companion → 태그: 앵커 배치

배치 전송 성공과 태그 적용 확인은 다른 값이다.
근거는 `app.py:265`, `io/uwb_serial_reader.py:113`이다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `anchors` | 앵커 객체 배열 | 인식 가능한 역할을 정렬. 최대 4기 | 설치 좌표 전달 | GET → 정렬 → USB 명령 |
| `anchors[].role` | 앵커 역할 문자열 | `role` → `id` → `anchor_id` 순서. A1~A4 또는 아래 별칭 | 앵커 순서 지정 | 서버 → 정렬 함수 |
| `anchors[].id` | 역할 식별 별칭 | `role`이 비면 사용 | 역할 호환 | 서버 → 정렬 함수 |
| `anchors[].anchor_id` | 역할 식별 별칭 | 앞선 값이 비면 사용 | 역할 호환 | 서버 → 정렬 함수 |
| `anchors[].x` | 앵커 x. m | 유한수 필요. 소수 3자리 전송 | 설치 x 좌표 | 서버 → USB → 태그 |
| `anchors[].y` | 앵커 y. m | 유한수 필요. 소수 3자리 전송 | 설치 y 좌표 | 서버 → USB → 태그 |
| `anchors[].z` | 앵커 설치 높이. m | 유한수 필요. 소수 3자리 전송 | 거리 계산의 설치 기준 | 서버 → USB → 태그 |
| `anchor_layout_id` | 배치 식별자. 문자열 | 없으면 `layout_id`. 전송 이력은 ID로 비교 | 서버·태그 배치 대조 | GET → USB 및 UWB 상태 비교 |
| `layout_id` | 배치 ID 별칭 | 위 값이 비면 사용 | 형식 호환 | GET → 내부 배치 ID |
| `anchor_layout_version` | 배치 버전. 정수 | `layout_version` 별칭. 기본 0. 이륙 검사는 0 외 값이 4인지 검사 | 배치 형식 검사 의도 | GET → 이륙 검사 |
| `layout_version` | 배치 버전 별칭 | 위 값이 비면 사용 | 형식 호환 | GET → 내부 버전 |
| `layout_compact` | 압축 배치 문자열 | 기본 빈 문자열. USB 전송에 사용하지 않음 | 현재는 메타데이터 | GET → 임무 상태 |
| `anchor_layout_valid` | 서버 배치 유효 판정 | 기본 false. 참일 때만 전송 시도 | 배치 전송·이륙 조건 | GET → 동기화/이륙 검사 |
| `anchor_layout_warning` | 서버 배치 경고 | 보관. 이륙 검사에서 직접 사용하지 않음 | 현재는 메타데이터 | GET → 임무 상태 |

| 순서 | 인식 역할 |
|---|---|
| A1 | A1 포함 문자열, `ORIGIN`, `BASE_TOP` |
| A2 | A2 포함 문자열, `X_AXIS`, `X_FLOOR` |
| A3 | A3 포함 문자열, `Y_AXIS`, `Y_FLOOR` |
| A4 | A4 포함 문자열, `DIAG_TOP` |

```text
anchors[3기] → ANCHORS LAYOUT=<id> x1 y1 z1 ... x3 y3 z3\n
anchors[4기] → ANCHORS4 LAYOUT=<id> x1 y1 z1 ... x4 y4 z4\n
태그 응답 → layout_synced/tag_layout_synced → 배치 적용 상태
```

USB 전송 ID는 문자·숫자·`_`·`-`만 남긴다.
길이는 최대 31자로 자른다.
전송 이력에는 변환 전 ID를 저장한다.
같은 ID의 좌표 변경·태그 재시작 재전송은 미보장이다.
앵커 중복 역할 검사는 이 함수에 없다.
버전 4가 앵커 수인지 형식 버전인지는 확인한다.
우리 v1.8의 명령 지원 여부도 아직 미확인이다.

## 5. companion → 서버: WebSocket 텔레메트리

표는 `_telemetry_payload()`의 전체 고정 필드다.
중첩 객체는 하위 절에서 모두 정의한다.
근거는 `app.py:421`이다.

### 5.1 장치·현재 위치

`telemetry_verified`는 XY 판정 결과다.
서버 저장이나 PX4 융합 성공의 증거가 아니다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `type` | 메시지 종류 | 항상 `telemetry` | 메시지 분기 | 생성부 → WS → 서버 |
| `telemetry_source` | 발행 주체 | 항상 `companion` | 다른 송신원 구별 | 생성부 → WS → 서버 |
| `companion_link` | companion의 연결 표시 의도 | 생성 시 항상 true. 서버 수신 확인값이 아님 | 접속 표시 입력 | 주 루프 → WS → 웹 |
| `companion_platform` | 장치 플랫폼 문자열 | 설정값 사용 | 장치 환경 표시 | 설정 → WS → 웹 |
| `companion_id` | 장치 이름 | 설정값 또는 호스트명 | 장치 구분 | 설정 → WS → 웹 |
| `companion_version` | 프로그램 버전 문자열 | ZIP에서는 `0.1.0` | 버전 추적 | 패키지 → WS → 웹 |
| `fix` | companion XY 사용 가능 판정 | `bool(estimation.xy_control_usable)` | 좌표 표시의 유효성 | UWB → XY 검사 → WS |
| `telemetry_verified` | companion XY 검증 표시 | 위 `fix`와 같은 식 | 정상 좌표 표시 의도 | XY 검사 → WS → 웹 |
| `x` | companion 처리 후 x. m/null | UWB 좌표를 예측·보정. 실패 시 마지막 값 유지 가능 | 현재 수평 위치 표시 | UWB → `UwbPoseResolver` → WS |
| `y` | companion 처리 후 y. m/null | x와 같은 처리 | 현재 수평 위치 표시 | UWB → `UwbPoseResolver` → WS |
| `current_z_m` | ZIP이 선택한 현재 고도. m/null | 상태에 따라 ToF/UWB/FC/마지막 값 선택 | 고도 표시 | 센서 → `CurrentZResolver` → WS |
| `current_z_source` | 선택 고도의 출처 | `unknown`, `tof10120`, `uwb_z_control`, `fc_relative_alt`, `last_safe_z` | 고도 출처 표시 | 고도 선택 → WS |
| `current_z_trusted` | 선택 고도 신뢰 판정 | ToF·UWB 후보 true. FC 대체·마지막 값 false | 고도 신뢰 표시 | 고도 선택 → WS |
| `current_z_confidence` | 선택 고도 신뢰 점수 | ToF 입력값. UWB 기본 0.7. FC 0.35. 미확인 0 | 보조 품질 표시 | 고도 선택 → WS |
| `altitude_error_m` | 활성 목표−현재 고도. m/null | 두 값이 있을 때 계산 | 목표 고도 차이 표시 | 활성 경유지/복귀점 → 계산 → WS |
| `mission_target_alt_m` | 임무 기본 목표 고도. m/null | GET의 임무 기본값을 그대로 사용 | 기본 목표 표시 | GET → 임무 상태 → WS |
| `t` | 메시지 생성 시각. Unix ms | `int(time.time()*1000)` | 메시지 시각 표시 | companion 벽시계 → WS |

`t`는 센서 측정 시각이 아니다.
`x`·`y`가 숫자여도 오래된 값일 수 있다.
`altitude_error_m`의 목표는 활성 경유지일 수 있다.
따라서 임무 기본 고도와 직접 비교하면 안 된다.
이유: 서로 다른 목표를 참조할 수 있기 때문이다.

### 5.2 UWB 품질·거리·배치

이 그룹은 대부분 태그 상태를 전달한다.
최종 companion XY 판정과 다를 수 있다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `uwb_link` | 태그 측 UWB 연결 판정 | 입력값 → `pf` → 거리/XY 상태 대체값. 나이로 재계산하지 않음 | UWB 상태 표시 | 태그 → 파서 → WS |
| `xy_control_usable` | 파싱된 태그 XY 사용 가능 | 입력·별칭을 사용. `fix`+거리 정상+배치 동기화로 true 대체 가능 | 태그 품질 표시 | 태그 → `UWBState` → WS |
| `z_control_usable` | 파싱된 태그 z 사용 가능 | `z_control_usable` → `zcu` → 신뢰/제어 대체값 | 태그 z 품질 표시 | 태그 → `UWBState` → WS |
| `uwb_z_trusted` | 태그 z 신뢰 판정 | `z_trusted` 또는 `current_z_trusted` | UWB 고도 품질 표시 | 태그 → 파서 → WS |
| `uwb_z_residual_m` | 태그 z 잔차. m/null | `z_residual_m` 또는 `current_z_residual_m` | UWB 고도 진단 | 태그 → 파서 → WS |
| `pose3d_fix` | 태그 다차원 측위 판정 별칭 | 내부 `pose4d_fix`와 같은 값 | 이전 화면 호환 | 태그 → 파서 → WS |
| `pose4d_fix` | 태그 다차원 측위 판정 | `pose4d_fix` 또는 `pose3d_fix` | 태그 측위 진단 | 태그 → 파서 → WS |
| `pose_quality` | 태그 측위 품질 문자열 | 입력 대문자화. 없으면 XY/Z 상태로 생성 | 품질 등급 표시 | 태그 → 파서 → WS |
| `uwb4_residual_m` | 태그 다차원 잔차. m/null | 동명 → `z_residual_m` → `current_z_residual_m` | 태그 품질 진단 | 태그 → 파서 → WS |
| `d1` | 첫 앵커 거리. m/null | 내부 거리 배열 0번 | 거리 표시 | 태그 `ranges`/`d1` → WS |
| `d2` | 둘째 앵커 거리. m/null | 내부 거리 배열 1번 | 거리 표시 | 태그 `ranges`/`d2` → WS |
| `d3` | 셋째 앵커 거리. m/null | 내부 거리 배열 2번 | 거리 표시 | 태그 `ranges`/`d3` → WS |
| `d4` | 넷째 앵커 거리. m/null | 내부 거리 배열 3번 | 거리 표시 | 태그 `ranges`/`d4` → WS |
| `range_ok` | 거리별 정상 여부 배열 | 입력 배열 우선. 없으면 숫자 존재로 생성 | 거리 유효성 표시 | 태그 → 거리 파서 → WS |
| `layout_version` | 표시용 배치 버전. 정수 | 태그 값이 0이면 서버 버전 사용 | 배치 버전 표시 | 태그/GET → WS |
| `anchor_layout_id` | 표시용 배치 ID | 서버 ID 우선. 비면 태그 ID | 배치 식별 표시 | GET/태그 → WS |
| `tag_layout_synced` | 태그 보고 배치 적용 여부 | `layout_synced` → `tag_layout_synced` → false | 배치 적용 표시 | 태그 → 파서 → WS |

`pose_quality`의 값은 다음 네 가지다.
`NO_FIX`, `DEGRADED_FIX`, `POSITION_ONLY`, `CONTROL_GRADE`.
미제공 시 XY·Z 모두 가능해야 `CONTROL_GRADE`가 된다.
이 값들을 우리 RAW 계약에 그대로 적용하지 않는다.
이유: 우리는 UWB z를 관측하지 않기 때문이다.

`range_ok=true`가 실측 오차 보증은 아니다.
표시용 배치 ID는 실제 태그 ID를 숨길 수 있다.
동일 배치 확인 여부는 별도 합의해야 한다.

### 5.3 제어 진단·비행·배터리

제어 진단 객체는 PX4 위치 융합 확인값이 아니다.
현재 객체는 시동 전 제어 방식 검사 결과다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `control_mode` | 요청한 제어 방식 | 설정을 정규화. `fc_guided`도 `companion_rc`로 변경 | 제어 방식 표시 | 설정 → backend → WS |
| `selected_mode` | 실제 선택된 방식 | `companion_rc` 또는 `dry_run` | 선택 결과 표시 | backend 검사 → WS |
| `control_backend_probe_result` | 제어 방식 검사 객체 | 아래 하위 필드로 직렬화 | 준비 상태 진단 | backend → WS |
| `fc_guided_probe_result` | 위 객체의 이전 이름 | `control_backend_probe_result`와 같은 객체 | 화면 호환 | backend → WS |
| `failure_reasons` | 제어 검사 사유 객체 배열 | probe의 `reasons`와 동일 | 제어 방식 차단 이유 | backend → WS |
| `battery` | FC 배터리 잔량. %/null | FC 상태값 전달 | 배터리 표시 | FC → MAVLink → WS |
| `battery_voltage` | FC 배터리 전압. V/null | FC 상태값 전달 | 전원 상태 표시 | FC → MAVLink → WS |
| `flight_state` | companion FSM 상태 | 아래 상태 목록 중 하나 | 임무 실행 상태 표시 | FSM → WS |
| `takeoff_phase` | 이륙 제어 세부 단계/null | 제어기 단계 문자열 | 이륙 진행 표시 | 제어기 → WS |
| `safety_level` | companion 안전 판정 | `OK`, `WARN`, `DEGRADED_HOVER`, `FAILSAFE_LAND`, `EMERGENCY_CUT` | 안전 상태 표시 | 안전 검사 → WS |
| `safety_reasons` | 안전 사유 문자열 배열 | 센서·IMU 검사 결과. 정상은 `ok` 포함 가능 | 안전 판정 설명 | 안전 검사 → WS |
| `final_rc` | RC 출력 계산값 객체 | 아래 4개 채널만 포함 | 제어 진단 | 제어기 → RC 제한 → WS |
| `final_rc.roll` | roll RC 채널 정수 | 최종 제한 후 값. 각도 아님 | RC 진단 | RC 처리 → WS |
| `final_rc.pitch` | pitch RC 채널 정수 | 최종 제한 후 값. 각도 아님 | RC 진단 | RC 처리 → WS |
| `final_rc.throttle` | throttle RC 채널 정수 | 최종 제한 후 값. 고도 아님 | RC 진단 | RC 처리 → WS |
| `final_rc.yaw` | yaw RC 채널 정수 | 최종 제한 후 값. 방위각 아님 | RC 진단 | RC 처리 → WS |

backend가 RC를 중립값으로 바꿀 수 있다.
`final_rc`에는 그 교체 전 계산값이 들어간다.
따라서 실제 전송·실행 확인값으로 쓰지 않는다.
이유: 계산 이후 출력이 달라질 수 있기 때문이다.

| probe 하위 필드 | 정의·형식 | 로직 | 역할 | 경로 |
|---|---|---|---|---|
| `requested_mode` | 요청 방식 문자열 | 정규화된 설정값 | 설정 표시 | backend → probe → WS |
| `selected_mode` | 선택 방식 문자열 | 준비 검사 결과 | 준비 상태 표시 | backend → probe → WS |
| `fc_guided_ok` | 이전 guided 검사 표시 | 현재 backend가 false로 생성 | 호환용 필드 | backend → probe → WS |
| `fallback_allowed` | 제어 방식 대체 허용 표시 | 현재 backend가 false로 생성 | 호환용 필드 | backend → probe → WS |
| `reasons` | 실패 사유 객체 배열 | 준비 검사 결과 | 차단 원인 | backend → probe → WS |
| `reasons[].code` | 사유 코드 문자열 | 예: `FC_LINK_TIMEOUT`, `LOITER_MODE_PENDING` | 기계 판독 | 준비 검사 → WS |
| `reasons[].level` | 사유 수준 문자열 | 기본 `BLOCKER` | 차단 수준 표시 | `FailureReason` → WS |
| `reasons[].message` | 상세 설명 문자열 | 검사 함수에서 생성 | 사람용 설명 | 준비 검사 → WS |

시동 후에는 probe 결과를 갱신하지 않는다.
`failure_reasons`는 전체 비행 장애 목록이 아니다.

| `flight_state` 값 | 의미 |
|---|---|
| `SYSTEM_CHECK`, `READY` | 시스템 검사, 준비 |
| `ARMING`, `TAKEOFF_READY`, `TAKEOFF_INIT` | 시동, 이륙 준비, 이륙 초기화 |
| `WAIT_FOR_TAKEOFF` | 이륙 제어 완료 대기 |
| `MOVE_TO_WAYPOINT`, `WAIT_FOR_ARRIVAL` | 경유지 이동, 도착 유지 확인 |
| `SCAN`, `NEXT_WAYPOINT` | QR 판독, 다음 지점 선택 |
| `RETURN`, `LAND`, `POWER_OFF` | 복귀, 착륙, 시동 해제 단계 |
| `DEGRADED_HOVER` | 위치 품질 저하 대응 |
| `FAILSAFE_LAND`, `EMERGENCY_CUT` | 안전 착륙, 긴급 출력 차단 단계 |

`POWER_OFF`는 companion 전원 종료가 아니다.
ZIP FSM의 시동 해제 단계 이름이다.

| `takeoff_phase` 값 | 의미 |
|---|---|
| `TAKEOFF_ARMED_IDLE` | 시동 후 대기 |
| `TAKEOFF_BOOST` | 이륙 출력 증가 |
| `LIFTOFF_CONFIRM` | 지면 이탈 확인 |
| `CLIMB_CAPTURE` | 상승 상태 포착 |
| `ALTITUDE_HOLD_SETTLE` | 고도 유지 안정화 |
| `INITIAL_HOVER` | 초기 정지 비행 |
| `XY_STABILITY_CHECK` | 수평 안정성 확인 |

### 5.4 IMU 객체

이 값들은 별도 MPU6050 코드에서 나온다.
우리 PX4 IMU 값과 의미·좌표계를 맞춰야 한다.
근거는 `io/mpu6050_reader.py:144`다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `imu` | IMU 상태 객체 | 아래 값 묶음 | 센서 진단 | MPU6050 → WS |
| `imu.sensor` | 센서 모델명 | 항상 `MPU6050` | 출처 표시 | 생성부 → WS |
| `imu.health` | 수신 처리 상태 문자열 | 정상 샘플 `OK`, 오류 `ERROR`. 초기값 `UNKNOWN` | 센서 상태 표시 | reader → WS |
| `imu.calibrated` | 로컬 교정 완료 여부 | 정상 샘플에서 true | 시작 교정 표시 | reader 교정 → WS |
| `imu.sample_rate_hz` | 설정 샘플 주파수. Hz | `rate_hz` 전달. 실측 수신률 아님 | 설정 표시 | 설정 → reader → WS |
| `imu.roll_deg` | reader의 roll 각도. °/null | 가속도 각도와 자이로 적분을 결합 | 기울기 표시 | MPU6050 → reader → WS |
| `imu.pitch_deg` | reader의 pitch 각도. °/null | 위와 같은 결합 | 기울기 표시 | MPU6050 → reader → WS |
| `imu.roll_rate_dps` | x축 각속도. °/s/null | 자이로 환산·교정값 차감 | 회전 상태 | MPU6050 → reader → WS |
| `imu.pitch_rate_dps` | y축 각속도. °/s/null | 자이로 환산·교정값 차감 | 회전 상태 | MPU6050 → reader → WS |
| `imu.yaw_rate_dps` | z축 각속도. °/s/null | 자이로 환산·교정값 차감 | 회전 상태 | MPU6050 → reader → WS |
| `imu.accel_norm_g` | 가속도 벡터 크기. g/null | 세 축 제곱합의 제곱근. 정지 시 약 1 | 충격 진단 입력 | MPU6050 → reader → WS |
| `imu.linear_accel_z_mps2` | 보드 z 가속도에서 중력 상수 차감. m/s²/null | `az_g × G − G`. 자세 회전 보정 없음 | 수직 가속도 의도 | MPU6050 → reader → WS |
| `imu.jerk_mps3` | 연속 가속도 차이 크기/시간. m/s³/null | 세 축 차이의 크기를 dt로 나눔 | 충격 변화량 | reader 이력 → WS |
| `imu.vibration_score` | 가속도 크기의 표준편차. g/null | 창 길이 `max(10, rate_hz//2)` | 진동 진단 | reader 이력 → WS |
| `imu.motion_state` | 움직임 분류 문자열 | 아래 임계값 규칙. 실제 비행 모드 아님 | 진단용 분류 | reader → WS |
| `imu.tilt_ok` | 기울기 허용 판정 | `max(abs(roll),abs(pitch)) < 12°` | 기울기 진단 | reader → WS |
| `imu.rate_ok` | 각속도 허용 판정 | 세 축 절댓값 최대 < 120°/s | 회전 진단 | reader → WS |
| `imu.shock_detected` | 충격 판정 | 가속도 크기 ≥ 2g 또는 jerk ≥ 35m/s³ | 충격 표시 | reader → WS |
| `imu.drift_status` | 드리프트 상태 표시 | 정상 샘플에서 `STABLE` 고정 | 진단 표시 의도 | reader → WS |
| `imu.temperature_c` | 센서 온도. °C/null | 원시값/340 + 36.53 | 센서 온도 표시 | MPU6050 → reader → WS |

`tilt_ok` 등의 임계값은 reader에 고정돼 있다.
별도 안전 검사 설정과 다를 수 있다.
`drift_status=STABLE`은 드리프트 실측 증거가 아니다.

| `imu.motion_state` | 현재 reader의 판정 순서 |
|---|---|
| `TIPPED_OR_CRASHED` | 기울기 ≥ 18° 또는 가속도 크기 ≥ 2.5g |
| `BOUNCE_OR_CONTACT` | 앞 조건이 아니며 jerk ≥ 35m/s³ |
| `ON_GROUND_STABLE` | 기울기 < 8°, 각속도 < 30°/s, 진동 < 0.08g, 가속도 0.92~1.08g |
| `MOTOR_SPINNING` | 앞 조건이 아니며 진동 ≥ 0.08g, 각속도 < 80°/s |
| `LIFTOFF_CANDIDATE` | 앞 조건이 아니며 기울기 < 12°, 각속도 < 120°/s, jerk < 35m/s³ |
| `UNKNOWN` | 나머지·초기 상태 |
| `AIRBORNE_STABLE` | enum에는 있으나 현재 reader는 생성하지 않음 |

### 5.5 QR 진행 상태

스캔 완료와 서버 수락 완료는 다르다.
로컬 저장만 성공해도 다음 지점으로 진행한다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `qr_scan_status` | 스캔 단계 문자열 | `DISABLED`, `IDLE`, `SCANNING`, `UPLOADED`, `BUFFERED`, `REJECTED`, `FAILED` | 스캔 상태 표시 | QR 처리 → WS |
| `qr_scan_active` | 스캔 작업 활성 여부 | 요청 시 true. 수락·저장 완료 시 false | 판독 진행 표시 | FSM → QR → WS |
| `qr_scan_complete` | 다음 지점 진행 가능 여부 | 서버 수락 또는 로컬 저장이면 true | 임무 진행 조건 | QR → FSM 및 WS |
| `qr_pending_buffer_count` | 재전송 대기 건수. 정수 | JSONL 레코드 수 | 미전송 결과 표시 | 로컬 버퍼 → WS |
| `qr_last_error` | 최근 QR 상태 사유 문자열 | timeout·거부 코드·저장 사유 등 | 오류 설명 | QR 처리 → WS |

SCAN 밖에서는 QR 상태를 IDLE로 초기화한다.
완료 상태가 영구 보존된다는 계약은 없다.
재전송은 기본 15초 간격이다.
카메라·디코더 시작 실패 시 재전송 루프도 시작하지 않는다.

## 6. companion → 서버: 임무 단계·제어 ACK

단계 보고와 제어 ACK는 서로 다른 API다.
ACK는 물리적 복귀·착륙 완료가 아니다.

| API·필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| phase POST `phase` | 서버 임무 단계 문자열 | 아래 상태표로 변환 | 임무 단계 갱신 | FSM → POST → 서버 |
| phase POST `notes` | 설명 문자열 | 기본 빈 문자열. 장애 때 안전 사유를 쉼표로 연결 | 단계 변경 설명 | 안전 판정 → POST → 서버 |
| ACK POST `action` | 처리한 요청 문자열 | `return_to_home` 또는 `land` | 요청 처리 표시 | GET → FSM 상태 전환 → POST |

| FSM 상태 | 전송 `phase` |
|---|---|
| `WAIT_FOR_TAKEOFF`, `MOVE_TO_WAYPOINT`, `WAIT_FOR_ARRIVAL`, `SCAN`, `NEXT_WAYPOINT` | `in_flight` |
| `RETURN` | `returning` |
| `LAND` | `landing` |
| `FAILSAFE_LAND` | `failed_returning` |
| `EMERGENCY_CUT` | `failed` |
| `POWER_OFF` | `completed` |
| 나머지 | 단계 POST 없음 |

단계 POST의 중복 키는 `phase + notes`다.
프로세스 수명 동안 유지하며 임무 ID를 포함하지 않는다.
다음 임무에서도 같은 단계 전송을 생략할 수 있다.
ACK 본문에는 요청 시각·요청 ID·임무 ID가 없다.
ACK 실패 시 같은 요청을 재전송하지 않을 수 있다.
호출부가 HTTP 성공 여부를 사용하지 않기 때문이다.

## 7. companion → 서버: 출발 좌표

출발 좌표는 복귀점 등록을 위한 후보값이다.
서버의 실제 저장 방식은 ZIP으로 확인할 수 없다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `x` | 출발 x. m | XY 판정 통과값 필요 | 복귀점 x 후보 | XY 처리 → 출발 POST |
| `y` | 출발 y. m | XY 판정 통과값 필요 | 복귀점 y 후보 | XY 처리 → 출발 POST |
| `z` | 출발 고도. m | 신뢰된 현재 z → 사용 가능한 UWB z → ToF 고도 | 복귀점 z 후보 | 센서 선택 → 출발 POST |
| `current_z_m` | 출발 고도 별칭. m | `z`와 동일 | 서버 필드 호환 | 출발값 → POST |
| `current_z_trusted` | 출발 고도 신뢰 표시 | payload 생성 시 true 고정 | 출발값 품질 표시 의도 | 생성부 → POST |
| `fix` | 태그의 측위 판정 | `snapshot.uwb.fix`. WS의 `fix`와 출처가 다름 | 태그 측위 표시 | 태그 → 출발 POST |
| `captured_at_iso` | 등록값 생성 시각 | UTC ISO 8601 문자열. 측정 시각과 다름 | 출발 기록 시각 | companion 시계 → POST |
| `source` | 출발값 생성 주체 | 항상 `companion_local` | 출처 표시 | 생성부 → POST |

이 API 본문에도 임무 ID가 없다.
ToF 최종 대체 경로는 고도 존재만 확인한다.
따라서 true 표시만으로 최신성을 확정할 수 없다.

## 8. companion → 서버: QR 결과

실제 스캐너 요청과 보조 함수 요청을 구분한다.
근거는 `io/qr_camera_scanner.py:142`다.

### 8.1 요청값

QR 원문을 문자열로 보낸다.
우리 `/qr/item` 객체와 동일 형식은 아니다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `client_scan_id` | 스캔 요청 식별자. 문자열 | 새 판독 처리마다 UUID4 생성. 동일 업로드 재시도·저장분 재전송은 유지 | 중복 요청 구별 | 판독 → 요청/JSONL → 서버 |
| `raw_qr_data` | QR 원문 문자열 | 디코더 결과 그대로 | 재고 데이터 해석 | 카메라 → 디코더 → POST |
| `drone_name` | 드론 표시명 문자열 | 임무 응답의 표시명/ID/설정 ID | 기록 표시 | GET 문맥 → POST |
| `mission_code` | 임무 코드 문자열 | 실제 스캐너는 내부 `mission_id`를 복사 | 스캔·임무 연결 | GET → QR 문맥 → POST |
| `label_point_id` | 라벨 지점 정수 ID | 변환 가능할 때만 포함 | 스캔·선반 연결 | 경유지 원본 → POST |
| `scan_source` | 업로드 경로 문자열 | 최초 `wifi`, 로컬 저장 후 `offline` | 수집 경로 표시 | 스캐너/버퍼 → POST |
| `scanned_at_iso` | QR 판독 처리 시각 | UTC ISO 8601. 재전송해도 원래 값 유지 | 재고 관측 시각 | 판독 → JSONL/POST |
| `note` | 설명 문자열 | 지점 ID가 있으면 `waypoint=<id>` | 현장 추적 | 경유지 → POST |
| `mission_id` | 별도 임무 ID | 보조 `upload_scan()`만 선택 전송. 실제 카메라 경로는 미전송 | 서버 ID 연결 의도 | 보조 호출 → POST |
| `attempts` | 저장분 재전송 실패 횟수 | 최초 0. 실패마다 +1. 저장 레코드 그대로 POST됨 | 재전송 관리 | JSONL → POST |
| `last_error` | 저장분 마지막 실패 사유 | 저장·재전송 실패 때 갱신. POST에도 포함 | 재전송 진단 | JSONL → POST |

최초 업로드는 기본 3회 시도한다.
실패 뒤 기본 경로는 `/tmp/companion_scan_buffer.jsonl`이다.
버퍼 `push()` 자체에는 중복 ID 검사가 없다.
동일 QR 재판독은 새 ID를 만들 수 있다.
서버가 중복 저장을 막는지 별도 확인해야 한다.

### 8.2 응답값

HTTP 성공과 QR 업무 수락은 서로 다르다.

| 필드 | 1. 정의·형식 | 2. 로직 | 3. 역할 | 4. 경로 |
|---|---|---|---|---|
| `ok` | API 처리 성공. 불리언 | 스캐너가 참/거짓으로 검사. 비2xx 응답은 클라이언트가 false 강제 | 업로드 처리 판정 | 서버 응답 → 스캐너/버퍼 |
| `accepted` | QR 업무 수락. 불리언 | `ok` 참일 때 검사. 기본 false | 스캔 성공 판정 | 서버 → UPLOADED 또는 REJECTED |
| `rejection_code` | QR 거부 코드. 문자열 | `accepted=false`일 때 오류로 표시 | 거부 설명 | 서버 → QR 상태 → WS |
| `error` | 오류 설명 | 업로드·버퍼 오류 기록에 사용 | 재시도 진단 | 서버/클라이언트 → 로컬 상태 |
| `msg` | 오류 설명 별칭 | 버퍼 재전송 실패 시 `error`가 없으면 사용 | 재전송 진단 | 서버 → 버퍼 |
| `status_code` | HTTP 상태 코드. 정수 | 비2xx 때 클라이언트가 결과 객체에 추가 | HTTP 진단 | HTTP 응답 → 클라이언트 결과 |

2xx의 `{}`나 빈 본문은 `ok=true`가 아니다.
직접 업로드는 `ok=true, accepted=true`여야 완료다.
버퍼 재전송은 `ok=true`만으로 레코드를 제거한다.
따라서 `accepted=false` 처리 규칙을 맞춰야 한다.
20회 재전송 실패 시 `.jsonl.dead`로 이동한다.

## 9. UWB 로컬 입력과 웹 필드의 구분

UWB 시리얼 계약은 웹 API와 별개다.
웹 연동을 위해 태그 형식을 바꿀지는 미정이다.

| 로컬 입력값 | 정의 | ZIP 로직 | 역할 | 경로 |
|---|---|---|---|---|
| `type` | 태그 메시지 종류 | `uwb_pose`, `telemetry`만 수락 | 메시지 분기 | 태그 → UWB reader |
| `seq` | 태그 순서 번호 | 정수 파싱·진단 보관. resolver는 중복 번호를 검사하지 않음 | 수신 진단 | 태그 → 로그. WS 미전송 |
| 태그 `t` | 태그 시각 | resolver의 수신 나이 계산에 사용하지 않음 | 원본 진단 | 태그 → raw 보관 |
| `xy_control_fresh` | 태그 신선도 | 정확히 false일 때 거부. 누락은 이 검사 통과 | XY 판정 | 태그 → resolver. WS 미전송 |
| `xy_control_held` | 이전 좌표 유지 여부 | 정확히 true일 때 거부 | XY 판정 | 태그 → resolver. WS 미전송 |
| `range_cycle_ms` / `cycle_ms` | 거리 측정 주기. ms | 값이 있으면 90ms 초과 거부 | XY 판정 | 태그 → resolver. WS 미전송 |
| `control_pose_age_ms` | 제어 좌표 나이. ms | 값이 있으면 120ms 초과 거부 | XY 판정 | 태그 → resolver. WS 미전송 |
| `range_reject_mask` | 거부 거리 비트 | 값이 있으면 정수 변환 후 0 아닌 값 거부 | XY 판정 | 태그 → resolver. WS 미전송 |
| 로컬 UWB 수신 나이 | companion 수신 이후 시간 | 기본 120ms 초과 거부 | XY 판정 | 로컬 시계 → resolver |
| `reason` | 태그 상태 사유 | 사용 불가 사유·로그에 활용 | UWB 진단 | 태그 → 로컬 상태. WS 최상위 미전송 |

입력 ID와 설정 드론 ID의 일치 검사도 없다.
ZIP 파서는 태그 `drone_id`를 필터링하지 않는다.
XY 보정 오차가 1.2m를 넘으면 추적을 초기화한다.
그 좌표를 정상으로 반환하므로 점프 거부와 다르다.
첨부 요구명세의 표현과 코드 동작을 구분해야 한다.

## 10. 우리 ROS2 값의 대응 후보

아래는 구현 전 확인받을 의미 대응표다.
새로운 웹 통신·제어 기능은 아직 구현하지 않았다.
기준은 [로드맵](roadmap.md)과 [고도 정책](altitude_policy.md)이다.

```text
현재 관측: 태그 RAW → uwb_node → /uwb_pose [uwb_map]
                                      |
                            검증 후 PX4 관측 입력
                                      v
비행 상태:                     PX4 EKF2 → MAVROS

제안 연동: ROS2 관측·비행 상태 → 필드 변환 → WS → Platform
          Platform → GET 임무 → 검증 → 기존 목표 topic 경계
```

| 외부 필드·기능 | 우리 값·담당 후보 | 필요한 변환·조건 | 현재 상태 |
|---|---|---|---|
| 서버 드론 ID `5` | 1호기 설정과 대응 | `ROS_DOMAIN_ID=1`과 별도 관리 | 대응 합의 필요 |
| WS `x`, `y` | 관측 표시: `/uwb_pose.pose.pose.position.x/y` | `uwb_map` 원점·축과 서버 창고 좌표 일치 | 관측 노드 구현 |
| WS `x`, `y` 대안 | 비행 위치 표시: `/mavros/local_position/odom` | ENU→창고 고정 변환. 관측과 추정 구분 | FC 수신 이력. UWB 융합 미검증 |
| `fix`, `telemetry_verified` | 새 관측 수신·품질·나이 판정 후보 | 관측 유효/FC 전달/FC 융합을 별도 정의 | 동명 최종 판정 없음 |
| `uwb_link` | `/uwb/status.connected`, `watchdog_ok`, `cycle_age_s` | USB 연결과 무선 거리 유효성 구분 | 기존 진단 있음 |
| `pose_quality`, `xy_control_usable` | `/uwb/status.last_decision`와 새 관측 여부 | RAW 검사 결과를 외부 등급으로 합의 | 직접 대응 없음 |
| `d1`~`d4`, `range_ok` | `/uwb/raw`의 `raw_slant_m`, `valid_mask`, `failure` | 앵커 순서·정상 비트 확인 | RAW 진단 있음 |
| `uwb4_residual_m` | RAW 전처리의 `pair_residual_m` 후보 | 같은 잔차가 아님. 이름만 바꾸지 않음 | 외부 필드 정의 변경 필요 |
| `current_z_m` | PX4 고도 출력 후보 | 원점·양의 방향·출력 최신성 합의 | companion z 추정/제어 추가 없음 |
| UWB z·`pose3d_fix`·`pose4d_fix` | UWB는 x·y만 관측 | 생략/null/미지원 중 서버 허용값 합의 | 직접 대응 없음 |
| `current_z_source`, `current_z_trusted`, `current_z_confidence` | PX4 출처·상태 후보 | `px4_ekf2` 등 출처 추가는 제안. 신뢰점수 임의 생성 금지 | 별도 계약 필요 |
| `imu.*` | `/mavros/imu/data`, `/mavros/imu/data_raw` 후보 | 모델명, FLU 좌표, rad/s→°/s 확인 | 수신 이력. 변환 API 미구현 |
| IMU 진동·충격·교정 표시 | PX4 진단 중 실제 제공값 | MPU6050 계산값과 같은 뜻으로 간주하지 않음 | 제공 가능 항목 확인 필요 |
| `battery`, `battery_voltage` | MAVROS 배터리 출력 후보 | 잔량 0~1 비율이면 ×100. 미상값 처리 | 실제 topic·수신 확인 필요 |
| `flight_state` | `/flight_state` 경계 | PX4 모드·시동 상태와 임무 FSM 단계 분리 | 로드맵 경계. 통합 FSM 미완 |
| 목표·경로 수신 | `/target_pose`, `/target_valid` 경계 | 좌표 변환·허용/거부 사유. 경로 배열 계약 추가 필요 | 경계만 정의 |
| QR 원문 `raw_qr_data` | `/qr_reader/data`, `/qr_code/data`, `/qr/data` 후보 | 사용 입력 하나 선택. 원문 문자열 보존 | 판독 노드 있음 |
| QR 의미 데이터 | `/qr/item` | `schema`, `code`, `name` 등. 원문과 구분 | 파서 있음 |
| QR 업로드·결과 | `/mission_result` 경계 | 임무/라벨 ID 및 중복 처리 계약 | Phase 4 스키마 미정 |
| `final_rc`, 제어 backend 진단 | 직접 대응 없음 | PX4 제어 상태 진단으로 대체할지 합의 | ZIP의 RC 제어 도입 안 함 |
| 앵커 동기화 | 현재 실측 배치 설정·태그 RAW 상태 | 서버 배치의 기준 권한·태그 명령 지원 확인 | 태그 설정 명령 미확보 |

## 11. 소스 찾기

행동의 근거는 ZIP 내부 소스에서 찾는다.
줄 번호는 검토한 ZIP 버전에만 적용된다.

| 찾을 내용 | ZIP 내부 경로·함수 |
|---|---|
| HTTP·WS·응답 해석 | `io/server_client.py`: `ServerClient`, `parse_mission_payload`, `_parse_waypoint` |
| 전체 WS 필드 | `app.py:421`: `_telemetry_payload` |
| 단계·ACK·출발 등록 호출 | `app.py:284`, `app.py:355`, `app.py:377` |
| 경로 변경·도착 처리 | `mission/route_manager.py:26`, `:45`, `:72` |
| 이륙 조건·출발 payload | `mission/launch_guard.py:22`, `:75` |
| 임무 상태 전환 | `mission/mission_fsm.py`: `MissionFSM.update` |
| QR 요청·응답 처리 | `io/qr_camera_scanner.py:59`, `:142` |
| 저장분 재전송·삭제 | `io/offline_scan_buffer.py`: `push`, `flush` |
| 태그 입력·앵커 명령 | `io/uwb_serial_reader.py:113`, `:249` |
| XY·고도 값 생성 | `estimation/uwb_pose_resolver.py`, `estimation/current_z_resolver.py` |
| IMU 값 계산 | `io/mpu6050_reader.py:144` |
| 출력과 표시값 차이 | `control/backend.py`: `dispatch` |
| 상태 enum·probe 하위값 | `types.py` |
| 기본값·실제 번들 설정 | `config.py`, `deploy/companion.env.local` |

용어는 [용어 사전](glossary.md)을 따른다.
