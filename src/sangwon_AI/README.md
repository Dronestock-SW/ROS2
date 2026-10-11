# sangwon_AI 설계 문서
이 디렉터리는 실내 자율비행 행동 계층의 설계를 모은다.
설계 결정·구현 범위·검증 근거를 검토할 때 읽는다.

2026-10-10: 수동 수집 로그를 C++ 입력에 직접 연결했다.
`sangwon_capture_replay`는 원본 시각·좌표계·공분산을 검사한다.
UWB XY와 PX4 상태를 분리한다. FC 출력은 추가하지 않는다.
[관측 입력 계약](../../docs/architecture/cpp_observation_input.md)과
[준비 결과·실행 명령](../../docs/report/cpp_observation_preparation_20261010.md)을 따른다.

2026-10-08 전체 미션은 기존 geometry를 native 경로에 연결했다.
`sangwon_native_plan`은 지도·기체 여유·마커 보정 구역을 검사한다.
Python MissionChain이 유일한 FC 명령 주체다.
자세·고도·추정은 PX4가 맡는다.
[연결 범위](../../docs/architecture/mission_chain.md)와
[현장 절차](../../docs/runbooks/mission_chain_field.md)를 따른다.
기존 BT의 Offboard 출력은 실물 경로로 연결하지 않는다.
이유: 고도 책임과 명령 주체를 중복시키지 않는다.

2026-10-08: [기본 이착륙 시험](../../docs/runbooks/native_hover_test.md)을 먼저 완성한다.
native hover는 C++의 별도 출력 실행기다.
취소·응답 지연·실패 결과·RC 인계를 검사한다.
기존 BT·REPLAY 서비스의 실물 출력을 활성화하지 않는다.

2026-10-07: 기존 소스를 ROS2 통합 브랜치에 편입했다.
Tag별 PX4 XYZ 관측과 workspace 설치본 실행을 추가했다.
실제 비행 writer와 센서 스캔 연결은 미완료다.
[통합 기록](../../docs/report/flight_uwb_ai_integration_20261007.md)과
[지상 실행](../../docs/runbooks/flight_uwb_ai_ground.md)을 먼저 읽는다.

기존 설계: 내부 v1.0 / C++ REPLAY v0.1.0 + 실행 서비스 v0.2 단계.
현재 범위는 [구현 현황](IMPLEMENTATION_STATUS.md)을 먼저 읽는다.
웹↔Jetson 구현·부팅 적용: [서비스 구조](SERVICE_ARCHITECTURE.md), [실행 절차](WEB_JETSON_RUNBOOK.md).
웹팀 다음 전달: [장치 등록·연동 요청](WEB_JETSON_INTEGRATION_REQUEST.md).
컨텍스트 압축 후 재개: [DEVELOPMENT_HANDOFF.md](DEVELOPMENT_HANDOFF.md).
2026-10-04: [QR·ArUco 명세](QR_SCAN_SPEC.md)의 BT 스캔·다각형 경로 검사를 합성 REPLAY에 연결했다. 실제 카메라·스캐너 어댑터는 후속이다.
UWB 전체 사양은 ROS2 멀티태그 계약을 따른다.
웹 공동 인수·센서 실측·실기체 검증은 별도다.
웹 전달은 [전체 개편 요청서](WEB_REDESIGN_REQUEST_2026-10-04.md)부터 읽는다.
2026-10-04 실제 웹 화면을 조회하고 [화면 점검](WEB_UI_AUDIT_2026-10-04.md)·[같은 버전 JSON 예제](contracts/web_v1_1_draft4/README.md)를 추가했다.
자동 기동 적용 상태는 [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md)에 있다.
개발 실행 환경은 [JETSON_ENV.md](JETSON_ENV.md), 공동 검수는 [DESIGN_REVIEW.md](DESIGN_REVIEW.md)를 따른다.

| 문서 | 내용 |
|---|---|
| [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) | 현재 구현·시험 범위·다음 작업 |
| [BUILD_REPLAY.md](BUILD_REPLAY.md) | Jetson C++ 빌드·실행·시험 |
| [BOOT_PREFLIGHT_SPEC.md](BOOT_PREFLIGHT_SPEC.md) | 자동 연결·점검·임무 준비·READY 상세 계약 |
| [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md) | 적용한 자동 점검 서비스·조회·관리자 후속·복구 절차 |
| [RC_EMERGENCY_SPEC.md](RC_EMERGENCY_SPEC.md) | 기존 RC 설정 보존·수동 인계·동작 검수 |
| [DESIGN.md](DESIGN.md) | BT, 지도·고도·제어권·로그 정책 |
| [IMPLEMENTATION_BASELINE.md](IMPLEMENTATION_BASELINE.md) | C++ 핵심·Python 보조, 공통 계약·프로세스·SIM-01·구현 순서 |
| [BT_SPEC.md](BT_SPEC.md) | 노드별 입출력·전환·시간 제한·선점·복구 |
| [QR_SCAN_SPEC.md](QR_SCAN_SPEC.md) | ArUco 미세 정렬·호버·스캐너 3회·카메라 대체 판독·웹 추가 요청 |
| [PX4_INTERFACE.md](PX4_INTERFACE.md) | 목표 메시지·좌표 변환·모드/시동 전환·실패 처리 |
| [MEASUREMENT_REGISTER.md](MEASUREMENT_REGISTER.md) | 확정값·시험값·미정 파라미터·의사결정 목록 |
| [YAW_RESEARCH.md](YAW_RESEARCH.md) | 목표 yaw, 자력계, 방향 검증 구역 |
| [WEB_REQUIREMENTS.md](WEB_REQUIREMENTS.md) | 웹팀 전달용 개발 요청·완료 조건·회신 자료 |
| [WEB_HANDOFF.md](WEB_HANDOFF.md) | 웹 담당자 전달 순서·개발 범위·회신 양식 |
| [WEB_REDESIGN_REQUEST_2026-10-04.md](WEB_REDESIGN_REQUEST_2026-10-04.md) | 웹 전체 개편 기준: Jetson 책임·기존 화면 변경·전체 흐름·API·인수 조건 |
| [WEB_UI_AUDIT_2026-10-04.md](WEB_UI_AUDIT_2026-10-04.md) | 제공 사이트에서 직접 확인한 현행 화면과 검수 범위 |
| [WEB_CONTRACT_DRAFT.md](WEB_CONTRACT_DRAFT.md) | v1 확장 데이터·명령·상태·재연결 의미 계약 |
| [WEB_JSON_DRAFT.md](WEB_JSON_DRAFT.md) | 실제 JSON 초안·버전·10초 수락 기한·결과 코드 |
| [UWB_INTEGRATION_REQUEST.md](UWB_INTEGRATION_REQUEST.md) | 전체 사양 요청 목록, 수령 후 어댑터 검토 순서 |
| [JETSON_SETUP.md](JETSON_SETUP.md) | SSH 설치 현황·장치/권한 문제·환경 준비 절차 |
| [INTEGRATION_PLAN.md](INTEGRATION_PLAN.md) | 담당별 산출물·선행 조건·설계 완료 기준 |
| [VALIDATION_PLAN.md](VALIDATION_PLAN.md) | 지상·시뮬레이션·제한 비행 검증 순서 |
| [LOG_REVIEW.md](LOG_REVIEW.md) | 제공 수동 RC ULog의 확인 결과 |
| [GLOSSARY.md](GLOSSARY.md) | 이 설계의 용어 |
| [analysis/ulog_inspection.ipynb](analysis/ulog_inspection.ipynb) | 원본 ULog 읽기 전용 분석 |

코드 변경 시 MD 상세 계약과 구현 현황을 함께 검토한다.
실비행 전에는 UWB→PX4 융합, 지도·PX4 좌표 변환,
yaw·고도 정확도와 PX4 실패 동작을 검증한다.
UWB는 전체 사양을 받은 뒤 어댑터를 설계한다.
JSON 구체안은 작성했으며 웹팀 합의 전이다.
실기체 도착 허용 XY/Z·안정 시간·watchdog는 실측 프로파일로 승인한다.
내부 로직은 SIM-01로 재현한다. 핵심 C++·보조 Python 분담은 확정했다.

웹 협의는 WEB_REDESIGN_REQUEST_2026-10-04 → contracts/web_v1_1_draft4 → 검증 계획 순서다.
WEB_JSON_DRAFT.md의 draft.2는 이전 근거이며 이번 예제와 혼용하지 않는다.
W01~W18 요구사항과 전달 문서의 WH 검수 사례를 대조한다.
기존 v1 확장이 기준이며 실제 웹팀 수락은 별도다.

구현 준비는 IMPLEMENTATION_BASELINE → BT_SPEC → PX4_INTERFACE → MEASUREMENT_REGISTER 순서다.
현재 문서가 존재한다는 사실은 구현·연동·실비행 검증 완료를 뜻하지 않는다.
