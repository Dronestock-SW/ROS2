# Jetson 자동 기동·연결·비행 전 점검 명세
이 문서는 전원 투입부터 시스템 점검, 임무 준비, 준비 완료 알림까지의 계약이다.
자동 기동 서비스·상태 수집기·웹 준비 화면을 구현하고 검수할 때 읽는다.

상태: 내부 구현 명세 v1.0 / 2026-10-02 / 실기체 연결·비행 승인 전.
정책 기준은 [DESIGN.md](DESIGN.md), 비행 중 대응은 [BT_SPEC.md](BT_SPEC.md)다.
장치별 필드와 제어권은 [PX4_INTERFACE.md](PX4_INTERFACE.md)를 따른다.
설치 현황과 적용 증거는 [JETSON_SETUP.md](JETSON_SETUP.md)에 별도 기록한다.
2026-10-04 QR 보완: scan 임무의 준비 검사는 [QR_SCAN_SPEC.md](QR_SCAN_SPEC.md)를 포함한다.
리더·카메라·ArUco 검출·교정·마커/QR 배치·장착 변환·기록 상태를 확인한다.
카메라는 최초 정렬에도 필요하므로 대체 판독용이라는 이유로 선택 장치로 낮추지 않는다.
미세 조정 범위·한도·호버/판독 조건이 미검증이면 scan 임무 READY를 발급하지 않는다.
scan XYZ는 라벨 위치다. 라벨 정면·기준점·장착 변환으로 기체 대기점/판독 pose를 별도 산출한다.
라벨 방향 또는 검증된 작업 공간·진입/이탈 경로가 없으면 scan 준비를 거부한다.
전체 경계 미제공 시에도 웹에서 받은 라벨별 허용 구역의 외곽·높이·경로 연결·개정을 검증한다(QS-W8).
정적 장애물 검사는 라벨점이 아닌 기체 전체 이동 공간에 적용한다.
비행 중 마커/카메라 실패의 실패 기록·대기점 복귀·다음 목표 정책은 사전 장치 불량의 READY 거부와 구분한다.
이동 전용 임무와 QR 장치 요구를 구분한다. 현재 진단 서비스에 검사를 구현한 것은 아니다.
2026-10-05 관측 구현: C++ 구독기는 MAVROS State/ExtendedState/BatteryState/RCIn을 선택 `OBS_PX4_*` host 진단으로 제공한다. 구독 프로세스 PASS/fresh bridge 수신은 BP-C10 항공기/FC boot·C12 prearm·C13/14 위치/yaw·C15 failsafe·C18 수동 인계의 승인 근거가 아니다. 실제 BP 검사는 UNKNOWN이다. header 시각은 FC 원천 시각을 증명하지 않으며 RCIn RSSI/채널 수로 수동 조작/override를 판정하지 않는다.

## 1. 요구사항과 준비 완료의 의미

전원을 켜면 등록된 프로세스가 기동하고 연결·검사를 반복한다.
정상이면 웹에 준비 상태를 알리고 새 START를 기다린다.
전원 투입·프로세스 복구·준비 상태 진입 자체는 arm·이륙·이전 임무 재개의 조건이 아니다.

| ID | 요구사항 | 완료 증거 |
|---|---|---|
| BP-R01 | 사용자 로그인 없이 점검 서비스를 자동 기동 | 재부팅 후 서비스 세대·부팅 ID·최초 점검 결과 확인 |
| BP-R02 | 설정된 통신 대상에 자동 연결·재연결 | 실제 식별값·신선한 메시지·수신 나이 보고 |
| BP-R03 | 필수 검사 미실시·알 수 없음·오래됨을 정상으로 처리하지 않음 | 차단 코드와 원천 근거 확인 |
| BP-R04 | 호스트 상태와 실제 비행 준비를 구분 | 호스트 PASS여도 PX4/UWB 미검증이면 비행 준비 false |
| BP-R05 | 특정 임무·지도·변환·기체에 결합된 준비 결과 발급 | 다른 snapshot 또는 재부팅 뒤 기존 준비 ID 거부 |
| BP-R06 | START 직전에 다시 검사하고 기존 명령을 재사용하지 않음 | 지연 START·배터리 하락·부팅 변경 시험 |
| BP-R07 | 상태 악화·자료 만료 시 즉시 준비 표시 철회 | 웹·로컬 상태가 같은 readiness_revision 반영 |
| BP-R08 | 웹이 준비 완료와 실패 원인을 설명 | 점검별 결과·원인·복구 안내·갱신 시각 표시 |
| BP-R09 | 프로그램 재기동이 공중 제어권을 자동 회수하지 않음 | PX4 모드 유지·이전 목표 미발행·재개 잠금 |
| BP-R10 | 비행 중 고장은 기존 BT/PX4 정책으로 처리 | 부팅 점검기가 직접 경쟁 Land/arm 명령을 내지 않음 |

### 세 가지 단계

| 단계 | 의미 | 발급 주체 | 비행 시작 가능 여부 |
|---|---|---|---|
| HOST_CHECKED | OS·설정·저장·프로세스의 확인 가능한 항목 검사 | 호스트 점검 보조 프로세스 | 이것만으로 불가 |
| SYSTEM_CHECKED | 실제 기체 상태·품질·프레임·제어 경로까지 확인 | C++ readiness_manager | 특정 임무 준비 필요 |
| MISSION_PREPARED | 해당 snapshot의 경로·공간·승인·상태 검사 통과 | C++ mission_runtime | 새 유효 START 수락 뒤 실제 상태 재검사 |

웹 임무 응답의 기존 `status=READY`는 서버 자료 배포 상태다.
Python 호스트 진단은 `device_readiness`, C++ 기체 준비는 `system_readiness`,
임무 준비는 `mission_readiness`로 구분한다. 기존 `preparation.ready`는 같은 임무 준비의 호환 표현이다.
정확한 웹 구조는 [WEB_JSON_DRAFT.md](WEB_JSON_DRAFT.md)의 `1.1-draft.2`를 따른다.
설치 스크립트 성공, systemd의 active, heartbeat 수신은 각 단계의 전체 성공을 대신하지 않는다.
REPLAY/SITL의 준비 상태에는 실행 프로파일을 반드시 붙인다. FLIGHT 준비로 표시하지 않는다.

## 2. 기동 순서와 의존성

```text
전원 -> boot_id 생성 / 이전 런타임 준비 무효화
     -> 호스트·설정·저장 검사 + 웹 연결 재시도
     -> 장치 식별 -> MAVROS / DDS 수신 / 센서 어댑터
     -> PX4 상태·시각·프레임 정규화
     -> flight_guard (출력 잠금) + 필수 기록기
     -> planner + mission_runtime + readiness_manager
     -> 시스템 검사 -> 임무 snapshot 준비 -> 웹 준비 완료 -> START 대기
```

위 의존성은 준비 조건이다. 웹 장애 때문에 PX4 감시나 기록기의 기동을 막지 않는다.
flight_guard는 실제 목표를 발행하지 않는 출력 잠금 상태에서 먼저 준비할 수 있다.
MAVROS와 DDS가 같은 물리 직렬 포트를 각각 소유하도록 자동 설정하지 않는다.
실제 연결 채널이 없거나 소유자가 불명확하면 해당 연결을 BLOCKED로 둔다.
UWB는 전체 사양 수령 전 드라이버·baud·프로토콜을 추측해 기동하지 않는다.

| 기동 묶음 | 선행 조건 | 준비 확인 | 실패 시 |
|---|---|---|---|
| 호스트 점검기 | 설치된 설정·경로 | 현재 boot_id의 새 결과 | 상태 파일의 이전 PASS 무효화 |
| 웹 어댑터 | 네트워크 설정·인증 자료 | 대상 기체·계약 버전·제어 세션 확인 | 백오프 재연결, 비행 중 저장 임무 계속 |
| MAVROS | 확인된 포트/UDP·대상 ID·권한 | 올바른 PX4의 최신 state | 필수 입력 불명, 신규 시작 차단 |
| DDS 수신 | 펌웨어 호환 메시지·별도 확인된 채널 | 같은 PX4 문맥의 품질 메시지 | 필수 품질 UNKNOWN |
| 센서 어댑터 | 확정 장치 사양·식별 | 신규 관측·원천 시각·품질 | 위치 입력 상실 또는 기능 저하 |
| PX4 상태 어댑터 | MAVROS·DDS 입력 | 동일 기체·시간·프레임의 StateSnapshot | 오래된 값 혼합 금지 |
| 기록기 | 전용 데이터 경로·용량 | 필수 스트림 실제 쓰기·읽기 검증 | 새 시작 차단, 비행 중 복귀 사건 |
| flight_guard | 프로파일·직접 PX4 상태·세션 | 출력 독점·lease 검사·출력 잠금 확인 | 신규 시작 차단, PX4 독립 failsafe 경로 |
| C++ 임무/계획/준비 | 설정·원장·위 구성의 상태 | 현재 세대 heartbeat·계약 버전·처리 가능 | 신규 준비 발급 금지 |

부팅 중 자동 센서 교정·PX4 파라미터 변경·임의 모드 전환·arm 시험은 하지 않는다.
영구 설치·장치 권한·연결 설정은 확인된 배포본으로 적용하고 부팅 때는 대조한다.
향후 승인 프로파일의 지상 자동 적용은 부팅 진단과 구분한 설정 단계로 구현한다.
대상 식별·disarmed·백업·허용 목록·재조회 검증·중간 실패 처리 조건은
[PX4_PARAMETER_PROFILE.md](PX4_PARAMETER_PROFILE.md)를 따른다. 현재 writer는 미구현이다.

## 3. 세션과 상태 전환

| 식별자 | 수명과 용도 |
|---|---|
| boot_id | Jetson OS 부팅마다 변경. 이전 상태 파일·단조 시각 무효화 |
| process_generation | 해당 프로세스 재시작마다 변경. 늦은 응답·heartbeat 구분 |
| monitor_session_id | Python 호스트 점검기의 세션. 비행 준비 토큰 발급 권한 없음 |
| companion_session_id | C++ 실행기 기동 세션. Python 모니터 ID와 혼용 금지 |
| readiness_revision | 검사 결과나 준비 근거 변경마다 증가 |
| readiness_seq | 해당 C++ 실행기 세션에서 증가하는 보고 순서 |
| control_session_id | 웹과 재확인한 제어 세션. 미전달 명령 재실행 차단 |
| preparation_id | 특정 snapshot·기체·PX4 부팅·변환·프로파일·준비 근거에 결합 |
| frame_epoch | 같은 PX4 부팅 내 좌표 기준의 세대. reset이면 이전 준비 무효화 |

| 내부 기동 상태 | 진입 조건 | 가능한 다음 상태 |
|---|---|---|
| BOOTING | 이번 부팅의 검사 결과 없음 | CONNECTING, BLOCKED |
| CONNECTING | 설정된 필수 연결을 기다림 | CHECKING, BLOCKED |
| CHECKING | 현재 출처로 필수 검사를 평가 중 | STANDBY, BLOCKED, RECOVERY_LOCK |
| STANDBY | 시스템 검사 통과, 지상 대기 | CHECKING, BLOCKED, ACTIVE |
| BLOCKED | 필수 검사 FAIL/UNKNOWN/STALE 또는 미구현 | 복구 후 CHECKING |
| ACTIVE | 이미 정상 수락한 실행 문맥이 비행을 관리 | STANDBY, RECOVERY_LOCK |
| RECOVERY_LOCK | 재기동 후 공중/시동 상태 또는 이전 실행 미종료가 발견됨 | 실제 착륙·시동 해제·종료 대조 후 CHECKING |

STANDBY에서 임무별 preparation.ready를 별도로 발급한다.
외부 `system_readiness.state`는 CHECKING/READY/NOT_READY/FAULT,
`mission_readiness.state`는 NO_MISSION/CHECKING/READY/NOT_READY/BUSY를 쓴다.
내부 BLOCKED는 외부 NOT_READY다. 내부 ACTIVE는 임무 BUSY이며,
시스템 상태는 별도 최신 검사로 표시한다. FAULT는 실행기나 필수 검사 자체의 내부 오류다.
외부 boot_phase는 BOOTING/CONNECTING/CHECKING/RUNNING/FAULT이며 내부 상태와 어댑터에서 대응한다.
STANDBY/ACTIVE는 RUNNING에 대응하지만, BLOCKED는 원인별 CHECKING 또는 FAULT로 표시한다.
시작 허가는 boot_phase가 아니라 system_readiness와 mission_readiness로 판정한다.
새로운 상태 패킷마다 준비 ID를 바꾸지는 않는다. 근거 변경·필수 실패·만료·새 부팅이면 철회한다.
같은 임무가 준비돼도 START가 없으면 출력 잠금을 유지한다.
이미 비행 중에는 preparation.ready=false와 BUSY를 보고하되 device_readiness를 지상 상태로 되돌리지 않는다.
RECOVERY_LOCK은 재부팅 전에 기록한 비행을 재개하는 상태가 아니다.
예외 RC 인계는 같은 부팅에서 이륙 전 출발점이 기록되고 해당 비행이 추적될 때만 별도로 B05를 준비한다.
공중에서 켜진 Jetson이 출발점·이륙 전 yaw를 추정해 이 예외에 진입할 수 없다.

## 4. 검사 결과의 공통 계약

외부 검사 결과는 `PASS / WARN / FAIL / UNKNOWN` 중 하나다.
STALE은 내부 신선도 상태이며 외부에는 UNKNOWN과 `INPUT_STALE` 원인으로 보고한다.
적용 제외는 버전 관리된 프로파일이 해당 기능을 사용하지 않을 때만 가능하며 필수 목록에서 제외 근거를 남긴다.
검사 구현이 없으면 UNKNOWN이고, 실제 입력이 없는 모의 값을 PASS로 제출할 수 없다.

| 필드 | 계약 |
|---|---|
| check_id | 아래 BP-C 식별자, 이후에도 의미 유지 |
| result / severity | 관측 결과와 영향 구분. severity는 BLOCK_START 또는 WARN_ONLY |
| source / evidence | 실제 프로세스·장치·주요 관측·설정 개정. 비밀 키 원문 제외 |
| boot_id / process_generation | 결과를 만든 실행 문맥 |
| observed_mono_s / max_age_s | 동일 Jetson 부팅의 측정·유효 나이. 장치 시각은 정렬 뒤 사용 |
| reason_code / recovery_hint | 기계 판정 코드와 운영자가 할 조치 |
| profile_id / profile_revision | 적용 기준의 출처. FLIGHT에 모의 임계값 사용 금지 |

BLOCK_START 검사 하나라도 PASS가 아니면 준비를 발급하지 않는다.
예외로 WARN 결과를 허용하는 항목은 아래 표에서 명시한다.
허용 경고는 상태 메시지와 준비 화면에 남기며 전체 PASS로 숨기지 않는다.
과거 PASS는 max_age_s 경과 시 STALE이다. 수신 시각만 갱신해 원천 관측 나이를 지우지 않는다.
위 표는 내부 계약이다. 웹 직렬화에서는 result를 status로, reason_code를 code로,
recovery_hint를 operator_action으로, 초 단위 나이를 source_age_ms/max_age_ms로 변환한다.
필수성은 required, 현재 차단 여부는 blocking으로 보고한다.
위험한 명시적 FAIL은 안정화 시간을 기다리지 않고 즉시 반영한다.
복구 PASS 전환에 필요한 연속 정상 시간은 프로파일에 둔다.

### 호스트·배포 검사

| ID | 검사·출처 | 시작 영향 | 기준/신선도 | 비행 중 영향 |
|---|---|---|---|---|
| BP-C01 | 실행 모드·기체 ID·설정 스키마·배포 해시 | 불일치/누락 차단 | 기동 및 파일 변경 시 재검사 | 임의 hot reload 금지, 활성 설정 유지 |
| BP-C02 | ROS 도메인·패키지·라이브러리·메시지 호환 | 필수 모듈 누락 차단 | 기동 시 + 세대 변경 | 실제 서비스 상실은 모듈별 처리 |
| BP-C03 | 장치 식별·별칭·권한·포트 소유 | 대상/소유 불명 차단 | 연결 이벤트 및 주기 검사 | PX4/센서 상실 정책 |
| BP-C04 | 기록 경로 쓰기·필수 파일 검증·공간 여유 | 기록 불가/여유 미달 차단 | 실제 쓰기 진전·공간 주기 검사 | 필수 기록 실패면 복귀·불가하면 Land |
| BP-C05 | 명령 원장·이전 종료 결과 복원 | 원장 불명/쓰기 불가 차단 | 기동·수락 시 | 필요한 Land·제어권 반환은 저장을 기다리지 않음 |
| BP-C06 | Jetson 시계·웹 시계 오차·UTC 신뢰 | 새 웹 명령 시간 검증 불가 시 차단 | 시간 상태 주기 검사 | 단조 시계 기반 기존 임무 유지, 자동 삭제 보류 |
| BP-C07 | CPU 부하·메모리·온도·전원 경고 | 검증된 운용 한도 위반 시 차단 | 하드웨어별 프로파일 | 일정 초과만으로 임의 Land 추가하지 않음; 제어/기록 지연 영향 검사 |
| BP-C08 | 필수 프로세스 세대·버전·heartbeat | 누락/불일치 차단 | 직접 IPC 응답과 최대 나이 | BT/guard/기록/웹별 기존 실패 처리 |
| BP-C09 | 자동 기동 구성·중복 실행·출력 소유자 | 중복 출력자/구성 불명 차단 | 기동·프로세스 변경 | guard 권한 판단, 경쟁 출력 차단 |

공간 여유 기준은 기록률·최장 임무·수집 중 ULog·보존량에서 산출한다.
단순한 디스크 80% 수치 등을 실측 없이 비행 승인 기준으로 고정하지 않는다.
BP-C07의 장치별 온도·전원 정보가 조회되지 않으면 UNKNOWN 및 NOT_SUPPORTED를 근거로 기록한다.
지원되지 않는 측정값을 0도·정상 전원으로 꾸미지 않는다.
이 값의 FLIGHT 시작 차단 여부는 해당 기체의 승인 프로파일에서 정한다.

### 연결·기체·제어 검사

| ID | 검사·출처 | 지상 시작 영향 | 신선도/승인 근거 | 비행 중 대응 |
|---|---|---|---|---|
| BP-C10 | PX4 대상 ID·펌웨어·실제 부팅 문맥 | 다른 기체/불명 차단 | MAVROS+DDS 동일 기체 대조 | 연결/프레임 상실 정책 |
| BP-C11 | mode·armed·landed·PX4 자체 failsafe | AUTO_TAKEOFF는 지상·disarmed 확인 필수 | 승인된 상태 최대 나이 | RC/PX4 제어 우선, 상태 불명에서 자동 회수 없음 |
| BP-C12 | PX4 자체 prearm/health 검사 | 필요한 점검 미통과/근거 없음 차단 | 실제 펌웨어의 상태/사건 필드 대응을 검증 | PX4 자체 failsafe와 BT 대응 유지 |
| BP-C13 | 융합 위치·속도·고도 품질 | INVALID/UNKNOWN/STALE 차단 | 실측 위치 프로파일·원천 나이 | 유지 가능할 때 최대 8초 복구, 불가/만료 Land |
| BP-C14 | yaw 품질·reset·지도 방향 정렬 | 검증 구역·변환·품질 미승인 차단 | 측량본·현재 PX4 품질 | reset/명확한 상실이면 이동 중단 Land |
| BP-C15 | 배터리 자체 상태·잔량·PX4 독립 Land 설정 | 이상/안전 기준 미확인 차단 | 현재 배터리 인스턴스·최신 PX4 상태 | 30% 복귀, 자체 상실 Land, Jetson 잔량만 누락 시 기존 정책 |
| BP-C16 | UWB 실제 융합 경로·신규 관측 의미 | 사양/융합 검증 전 차단 | 전체 사양·동기·좌표·정확도 승인 | 신규 위치 관측 상실 정책 |
| BP-C17 | MAVROS 단일 출력·guard lease·failsafe 설정 대조 | 경로 미검증/설정 불일치 차단 | 설치·펌웨어·시험 승인 개정 | 오래된 목표 차단, PX4 독립 failsafe |
| BP-C18 | RC 연결 | 미연결 WARN 허용 | 최신 연결 상태 | 연결 상실만으로 중단하지 않음, 스틱 인계 우선 |
| BP-C19 | LiDAR 관측·장애물 기능 | 미연결은 기능 저하로 표시, 경로 기능 요구와 함께 검사 | 실제 원천 나이·능력 목록 | 별도 대기 없이 승인 경로 계속, 자동 우회 비활성화 |
| BP-C20 | 웹 연결·인증·계약·제어 세션 | 새 START 검증 불가 시 차단 | 재연결 때 세션 재확인 | 저장 임무 계속, 미전달 요청 자동 실행 금지 |

BP-C12는 arm을 시도해 거절 이유를 얻는 검사가 아니다.
읽기 전용으로 확인 가능한 PX4 점검 자료의 필드·의미를 펌웨어별로 검증한다.
자료가 없으면 UNKNOWN으로 남긴다. 오래된 STATUSTEXT의 부재를 정상으로 보지 않는다.
Jetson 점검 통과 후에도 PX4의 실제 arm 허용 검사가 최종적으로 수행된다.
arm 거절은 우회하지 않고 B03/B20의 지상 종료·늦은 응답 규칙으로 처리한다.
지상 잔량 30% 이하에서 출발했다가 즉시 복귀하는 흐름은 허용하지 않는다.
실제 시작 최소 잔량은 30% 복귀 기준보다 높은 검증값이 필요하며 프로파일에 둔다.

LiDAR가 없다는 상태를 정상으로 만들지 않는다. 자동 우회를 요구하는 임무는 기능 부족으로 준비할 수 없다.
기존 정책상 승인된 경로만 수행하는 기능 저하 운용은 별도 대기 없이 가능하며 해당 경고를 유지한다.
지상에서 배터리 관측이 없어도 출발하도록 비행 중 텔레메트리 상실 정책을 확대하지 않는다.

### 임무별 준비 검사

| ID | 검사 | 준비 무효화 조건 |
|---|---|---|
| BP-C21 | 불변 snapshot·필수 기능·전체 경유점·명령 계약 | 누락/미지원·다른 개정·동일 ID 내용 충돌 |
| BP-C22 | 지도·장애물·천장·고도 구역·경계/금지 구역 상태 | 형상 불량·다른 지도·요구 기능 불충족 |
| BP-C23 | 지도↔PX4 변환·기체 기준점·yaw 승인 구역 | PX4 재부팅·frame reset·측량/장착 변경 |
| BP-C24 | 이륙 수직 공간·시작 회전·모든 필수 점·복귀 경로 | 현재 위치/경로/장애물로 준비 근거가 달라짐 |
| BP-C25 | 운영자 상부 공간 확인·현재 heading 확인 | 해당 지도/경로/변환과 불일치·신선도 미충족 |
| BP-C26 | 실제 출발점·이륙 전 yaw·RC 예외 문맥 | 출발 기록 불명·RC 수평 이동·다른 비행 |
| BP-C27 | 도착·제동·속도·watchdog·안정 시간의 승인 프로파일 | 미정값·다른 기체·SITL 수치의 FLIGHT 재사용 |
| BP-C28 | 새 START에 대한 세션·10초 수락 만료·중복 검사 | 재연결 전 미전달 요청·기존 실행·오래된 준비 ID |

경계가 없으면 자동 우회를 끄고 승인 경로 및 검증된 복귀만 준비한다.
웹에서 지도 자료를 받았다는 사실만으로 공간 검사를 통과시키지 않는다.
BP-C28은 최종 명령 수락 시 검사다. 사전 준비 화면은 이후 새 START 수락을 보장하지 않는다.

## 5. 시간 기준과 검사 반복

| 항목 | 첫 구현 기준 | 적용 범위 |
|---|---|---|
| 호스트 점검 반복 | 2초 | OS 모니터용 공학적 시험값, flight_guard 주기를 대신하지 않음 |
| 호스트 상태 소비 만료 | 마지막 새 결과 후 10초 | 모니터 표시 철회 기준, 실기체 제어 품질 TTL과 별개 |
| 연결 재시도 | 1·2·4·8·최대 10초, 대상별 독립 | 통신·전원 상태를 바꾸지 않는 재연결 |
| 최초 연결 안내 | 30초 미연결이면 사유 표시, 이후 백오프 유지 | 30초 만료를 정상 준비 또는 장치 고장 확정으로 취급하지 않음 |
| 호스트 PASS 안정 | 연속 3회 정상 | UI 점검 흔들림 완화, 명시적 FAIL은 즉시 반영 |
| 준비 이벤트 전달 | 상태 전환 즉시 + 주기 상태로 재확인 | 기존 웹 세션/QoS 계약에 맞춤 |
| PX4·위치·배터리 TTL | 승인 프로파일 참조 | SIM-01은 모의 시험만, FLIGHT는 별도 검증 |
| START 수락 유효시간 | 생성 후 10초 초기 시험값 | 이미 수락한 임무 수행 시간과 무관 |

이 표의 호스트 수치는 장치 성능 실측값이 아니다.
상세 제어 나이·lease·착륙 모드 확인 시간은 [IMPLEMENTATION_BASELINE.md](IMPLEMENTATION_BASELINE.md)를 따른다.
실제 프로세스 간 시각 불일치·스케줄 지연·통신 지연을 측정하고 FLIGHT 기준을 승인한다.
통신 재시도는 읽기/연결 복구에 해당한다. arm·모드 요청을 이 백오프 루프에 넣지 않는다.

## 6. 재시작과 복구 책임

| 고장/재시작 | 지상 | 공중 또는 상태 불명 |
|---|---|---|
| 호스트 점검기 | 이전 상태 만료, 재검사 | 스스로 비행 명령 없음; 상태 전달 실패 보고 |
| 웹 어댑터 | 새 세션·준비 확인, 저장 START 자동 전달 금지 | 기존 C++ 임무 계속, 상태 재연결 |
| 기록기 | 필수 기록 재검증 전 시작 차단 | 복귀 잠금, 복구돼도 임무 재개 없음 |
| planner/mission_runtime | 세대 변경·준비 철회 후 대기 | guard의 lease 경로, 새 runtime은 RECOVERY_LOCK |
| flight_guard | 출력 잠금으로 시작, 상태 재확인 | 이전 의도 로드·자동 Offboard 재진입 없음, PX4 독립 failsafe |
| MAVROS/DDS/상태 어댑터 | 신선한 입력·같은 기체부터 재검증 | 재연결은 가능하지만 이전 목표 재발행/제어권 회수 없음 |
| Jetson 전체 재부팅 | 이전 준비 무효화·새 명령 대기 | 비행 문맥 미복원 상태로 자동 임무 재개 금지 |

감시용 프로세스가 재시작됐다는 이유만으로 비행 프로세스 전체를 함께 재시작하지 않는다.
Linux 서비스 자동 재시작은 프로세스 복구까지만 한다. 비행 권한은 runtime/guard가 별도로 판정한다.
새 guard가 기동 중 기존 guard와 겹치지 않도록 단일 인스턴스와 출력 소유권을 검사한다.
정상 가동 중 업데이트·서비스 재기동은 지상·disarmed 확인 뒤 적용한다.
뜻하지 않은 재기동 시 대응은 위 표로 검수한다.

공중에서 실행 문맥을 잃었을 때 살아 있는 guard는 기존 Land 정책을 수행한다.
guard까지 종료됐으면 PX4 Offboard 상실 동작에 맡기며, 새 프로세스가 경쟁 제어를 시작하지 않는다.
상태 불명에서는 임의 지상 판정·강제 disarm·READY 발급을 하지 않는다.

## 7. systemd 배포 계약

아래 이름은 전체 ROS 연동이 구현된 뒤의 서비스 구분안이다.
실제 적용한 파일·서비스명·enable 결과는 JETSON_SETUP에 기록한다.
현재 호스트 점검기만 설치했다면 나머지를 설치/동작 완료로 표시하지 않는다.

| 단위 | 목적 | 준비 의미 |
|---|---|---|
| sangwon-health-monitor.service | OS·설정·파일·기초 장치 상태 점검 | HOST_CHECKED 근거만 생성 |
| sangwon-connectivity.target | MAVROS·DDS·장치별 수신 구성 | 프로세스 기동과 실제 연결 상태 별도 |
| sangwon-flight-core.target | 상태 어댑터·guard·계획·임무·준비 검사 | 핵심 C++ 프로세스 준비, 출력 잠금 기본 |
| sangwon-support.target | 웹·기록·알림 어댑터 | 보조 서비스 분리 |
| sangwon-stack.target | 위 구성을 묶는 운영 기동 대상 | target active만으로 비행 READY 아님 |

서비스는 .bashrc에 기대지 않고 사용자·작업 경로·ROS setup·ROS_DOMAIN_ID=1을 명시한다.
실제 실행 파일이 `sd_notify`를 구현한 경우에만 Type=notify를 쓴다.
Type=simple이면 active는 실행 중이라는 의미이며 readiness 결과로 다시 확인한다.
After는 시작 순서, 각 프로그램의 실제 데이터 준비는 런타임 검사로 판단한다.
네트워크 준비 target만으로 웹 서버 연결 성공을 판정하지 않는다.
재시작 횟수·간격 상한을 두고 반복 실패를 웹/로컬 상태에 남긴다.
WatchdogSec는 해당 서비스가 실제 WATCHDOG 알림을 내는 구현에서만 사용한다.
systemd 재시작 감시는 제어 목표 lease나 PX4 failsafe를 대신하지 않는다.
최소 운영 권한으로 실행하고 장치 권한을 명시한다. 상시 root·모든 사용자 장치 쓰기로 우회하지 않는다.
사용자 systemd 서비스는 로그인 후 실행과 전원 투입 직후 실행을 구분한다.
로그인 전 자동 실행은 system 서비스 또는 해당 운영 계정의 linger 설정을 확인해야 한다.
사용자 서비스 enable만 성공하고 linger가 꺼져 있으면 BP-R01 완료로 표시하지 않는다.
권한 때문에 적용하지 못한 항목은 필요한 관리자 명령과 검증 절차를 JETSON_SETUP에 남긴다.
서비스 의미의 근거는 [systemd v249 서비스 문서](https://raw.githubusercontent.com/systemd/systemd/v249/man/systemd.service.xml),
로그인 전 실행 조건은 [loginctl linger 문서](https://raw.githubusercontent.com/systemd/systemd/v249/man/loginctl.xml)를 참고한다.
현재 적용한 호스트 서비스는 실제 notify 알림을 구현했다. READY=1은 진단 서비스 시작만 뜻한다.
2026-10-02 적용 결과는 사용자 서비스 enabled/active, Linger=yes이며 상세 증거는 JETSON_DEPLOYMENT에 기록한다.
콜드 부팅 시험은 별도 수행 전이며, 설정 적용 성공만으로 그 시험 통과를 주장하지 않는다.

## 8. 준비 알림과 웹 상태 계약

사용자 결정에 따라 첫 구현은 웹의 상태 배지·점검 목록·상태 전환 이벤트를 기준으로 한다.
스캐너 LED는 제어 가능 여부·전기적 연결·현재 용도를 확인한 뒤 별도 알림 어댑터 후보로 검토한다.
확인 전 scanner_led는 UNVERIFIED이며 LED가 있다는 사실만으로 제어 가능하다고 가정하지 않는다.
추가 LED/부저는 하드웨어와 표시 방식을 확정한 뒤 검토한다.
준비 확인을 위해 모터 회전·이륙·기체 이동·yaw 회전을 수행하지 않는다.
전원만 켜져도 들리는 소리를 비행 준비 완료 신호로 쓰지 않는다.

| 사용자에게 보일 상태 | 의미 | 시작 버튼 |
|---|---|---|
| 부팅 중 / 연결 중 | 새 기동 세대 검사 중 | 비활성 |
| 점검 필요 | 필수 항목 실패·불명·오래됨 | 비활성, 원인과 조치 표시 |
| 시스템 점검 완료 · 임무 대기 | SYSTEM_CHECKED, 아직 임무 준비 없음 | 비활성 |
| 임무 준비 완료 · 시작 대기 | 해당 snapshot의 preparation.ready=true | 현재 준비 근거와 경고 표시 후 활성 |
| 비행 중 / 수동 인계 / 복구 잠금 | 실행 상태 표시 | 새 START 비활성 |
| 연결 끊김 / 상태 오래됨 | 웹의 마지막 상태 만료 | 마지막 READY 철회, 새 요청 자동 실행 금지 |

READY 전환 이벤트는 `boot_id + preparation_id + readiness_revision`으로 중복 제거한다.
재연결로 같은 준비를 다시 수신해도 새 준비 완료처럼 반복 알림을 내지 않는다.
정상 상태가 철회되면 이미 보낸 이벤트가 남아 있어도 현재 배지와 시작 가능 상태를 즉시 바꾼다.
웹의 시작 버튼 비활성화와 별도로 Jetson이 최종 수락 검사를 수행한다.
알림 장치 고장 자체가 정상 비행 출력에 영향을 주지 않으며 웹에 별도 경고한다.
로컬 LED/부저의 필수성은 하드웨어 확정 전 미결정이며 비행 준비 승인 근거로 사용하지 않는다.
RC 비상 인계는 [RC_EMERGENCY_SPEC.md](RC_EMERGENCY_SPEC.md)를 따른다.
기존 RC/PX4 모드·스위치·kill 설정을 유지하고 Jetson이 수동 제어권을 반환한다.
RC 입력이 Jetson을 우회해 PX4에 도달하는 경로도 점검한다.
RC 미연결은 시작을 막지 않지만 웹에서 **현재 수동 인계 불가**를 명확히 경고한다.

### 호스트 모니터와 외부 상태의 연결

초기 구현은 [ops/health_monitor.py](ops/health_monitor.py)의 `sangwon-host-health/1` 진단이다.
아래는 핵심 필드만 뽑은 합성 예제다. 이 파일의 `system_ready=false`를 웹 READY로 승격하지 않는다.
실제 전체 필드·적용 방법은 [JETSON_DEPLOYMENT.md](JETSON_DEPLOYMENT.md)를 따른다.

```json
{
  "schema_version": "sangwon-host-health/1",
  "scope": "HOST_DIAGNOSTICS_ONLY",
  "boot_id": "boot-demo-01",
  "monitor_session_id": "monitor-demo-01",
  "monitor_seq": 17,
  "generated_monotonic_s": 40.0,
  "display_ttl_s": 10,
  "state": "BLOCKED",
  "system_ready": false,
  "mission_ready": false,
  "can_start": false,
  "flight_authority": false,
  "companion_session_id": null,
  "preparation_id": null
}
```

모든 호스트 항목이 정상이어도 이 모니터는 비행 준비 토큰을 발급하지 않는다.
실제 C++ 준비 수집기가 구현되면 `WEB_JSON_DRAFT.md` 7절의 정식 확장 구조로 보고한다.
그 구조에는 boot_id·companion_session_id·control_session_id·readiness_seq·readiness_revision,
system_readiness·mission_readiness·checks·rc·indication이 포함된다.
모니터를 연결하는 어댑터는 그 자료를 호스트 검사 근거로만 사용한다.
준비 허가와 토큰은 실제 PX4/UWB/임무 검사를 통과한 C++ 프로세스만 발급한다.
웹 check_id는 의미를 가진 문자열이며 BP-C ID와의 대응표를 어댑터에 고정한다.
현재 웹 예제의 px4.link는 BP-C10, position.quality는 BP-C13, rc.available은 BP-C18이다.
한 상세 검사 안에 독립 실패 원인이 여러 개면 외부 체크를 분리하되 원래 BP-C 추적 ID를 보존한다.
source_age_ms/max_age_ms는 외부 상태의 관측 나이/한도이며 전송·캐시 경과를 더해 판정한다.
나이가 불명인 필수 관측은 UNKNOWN으로 차단한다.
상태 메시지가 멎으면 서버/화면은 자체 만료 기준으로 오래된 준비 표시를 제거해야 한다.
민감한 인증 자료·실제 비밀번호는 evidence에 넣지 않는다.

## 9. 구현 작업과 완료 기준

| 순서 | 구현 작업 | 끝났다고 판단할 증거 |
|---|---|---|
| BP-I01 | 호스트 점검 CLI·주기 실행·원자적 상태 파일 | 오류/미구현 분리, stale·boot_id 검증, 실제 장치 제어 없음 |
| BP-I02 | 제한된 자동 기동 서비스와 상태 조회 | 재부팅/프로세스 실패 후 복구, 시작 명령 발생 없음 |
| BP-I03 | ROS 공통 CheckResult·ReadinessSnapshot | C++/Python 형식 일치·세대/신선도 시험 |
| BP-I04 | PX4 실제 상태/품질 어댑터·읽기 전용 점검 | 각 BP-C10~C17의 실제 원천·미상 처리 확인 |
| BP-I05 | C++ readiness_manager와 mission_runtime 연결 | snapshot 준비·철회·START 직전 재검사 |
| BP-I06 | 웹 준비 UI·결과/이벤트 연결 | 오래된 READY 제거, 원인 표시, 명령 중복 방지 |
| BP-I07 | 자동 기동 전체 서비스·모의 고장 시험 | 고장별 독립 재시작·공중 재개 차단·출력 단일 소유 |
| BP-I08 | UWB 전체 사양 반영·융합·비행 프로파일 검증 | 사양 대응표·실측·SITL/지상·제한 비행 승인 증거 |
| BP-I09 | 선택 로컬 LED/부저 | 하드웨어 식별·표시 규칙·상태 만료/재연결 시험 |

호스트 단계의 성공을 BP-I04 이후 완료로 올려 적지 않는다.
실장 미확인 항목은 임의 mock PASS로 대체하지 않는다.
모의 시험은 REPLAY/SITL 표시를 유지하며 실제 장치 출력 경로를 열지 않는다.

## 10. 수락 시험

아래는 구현할 시험이다. 이 문서 작성 자체로 통과한 것으로 기록하지 않는다.

| ID | 입력·고장 | 기대 결과 |
|---|---|---|
| BP-T01 | 전원 투입, PX4/UWB 미연결 | 호스트 검사 반복, 비행 준비 false, arm/목표 발행 0회 |
| BP-T02 | OS·모든 프로세스 active, 위치 품질 미검증 | SYSTEM_CHECKED 발급 금지, 원인 표시 |
| BP-T03 | 현재 boot_id와 다른 PASS 파일 | 폐기, 새 검사 전 UNKNOWN |
| BP-T04 | 모니터 정지·상태 10초 초과 | 호스트 상태 STALE, 웹 준비 표시 철회 |
| BP-T05 | PX4 다른 system ID 또는 DDS 다른 부팅 | 데이터 혼합 거부, 시작 차단 |
| BP-T06 | 필수 위치 최신 값이지만 원천 시각 오래됨 | STALE/UNKNOWN, 수신 시간 갱신으로 통과 불가 |
| BP-T07 | 전부 통과, 임무 자료만 서버 READY | 임무 검증 및 새 START 전 arm 없음 |
| BP-T08 | preparation.ready 후 배터리/위치 악화 | 준비 철회, 지연 START 거부 |
| BP-T09 | 이전 준비 ID·다른 snapshot START | 거부, 실제 전환 요청 없음 |
| BP-T10 | Wi-Fi 재연결 뒤 오래된 미전달 START | 자동 실행 없음, 새 상태·세션 요구 |
| BP-T11 | 비행 중 웹만 종료·복구 | 저장 임무 계속, START 중복 없음 |
| BP-T12 | 비행 중 BT 종료, guard 생존 | lease 만료 후 기존 정지/Land 처리 |
| BP-T13 | 비행 중 guard 종료·재기동 | PX4 독립 실패 처리, 새 guard 출력 잠금·권한 회수 금지 |
| BP-T14 | 비행 중 Jetson 재부팅 | RECOVERY_LOCK, 기존 목표/임무 재개 없음 |
| BP-T15 | 지상 기록 경로 쓰기 실패 | 준비 불가·복구 이유 표시 |
| BP-T16 | 비행 중 필수 기록 실패 | 복귀 잠금, 이후 기록 복구돼도 임무 재개 없음 |
| BP-T17 | RC 미연결, 나머지 기준 통과 | RC 경고 유지, RC 연결만으로 시작 차단하지 않음 |
| BP-T18 | LiDAR 상실, 승인 경로 운용 | 별도 대기 없이 기존 정책, 자동 우회 꺼짐, 상실 기록 |
| BP-T19 | PX4 자체 배터리 센서 상실 vs Jetson 잔량만 누락 | 두 원인과 기존 대응을 구분 |
| BP-T20 | 웹 마지막 READY 후 상태 스트림 정지 | 수신 측 만료로 시작 비활성, 재접속 때 현재 상태 재조회 |
| BP-T21 | READY 이벤트 중복·세대 변경 | 같은 이벤트 중복 알림 없음, 이전 세대 준비 무효 |
| BP-T22 | 알림 어댑터 정지 | 비행 출력 경로 지연 없음, 알림 장애 별도 표시 |
| BP-T23 | PX4 arm 거절·늦은 arm 응답 | 거절 우회 없음, B03/B20 종료 규칙 확인 |
| BP-T24 | FLIGHT에서 SIM-01 및 모의 PASS 주입 | 프로파일/출처 검사로 차단 |
| BP-T25 | 자동 기동의 포트 점유 충돌 | 임의 기존 프로세스 종료 없음, 연결 차단 원인 보고 |
| BP-T26 | 준비 후 NTP 시각 이동 | 단조 타이머 유지, 새 명령 UTC 검증 불가 시 수락 거부 |
| BP-T27 | 지상 자동 기동 중 불필요한 출력 관찰 | setpoint/arm/mode/직접 모터 명령 0회 |
| BP-T28 | ULog 사후 수집 실패만 발생 | 경고 유지, 다른 준비 검사 통과 시 다음 임무 허용 |

각 결과에는 소프트웨어·설정 개정, 대상 기체/시뮬레이터, 입력 증거, 기대/실제 결과를 남긴다.
호스트 자동 기동 검증과 실기체 failsafe 검증은 별도 결과로 관리한다.
