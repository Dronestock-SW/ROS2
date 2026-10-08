# 웹 전체 개편 요청서 — Jetson 요구사항과 공동 개발 기준
이 문서는 웹 화면·서버와 Jetson을 함께 개발하기 위한 요청서다.
화면 흐름, API, 담당 범위와 인수 조건을 정할 때 이 문서부터 읽는다.

작성: 2026-10-04 / 개정: r2 / 수신: 웹 UI·UX·서버 담당자.
검토 근거: 웹팀 `WEB_HANDOFF_REQUESTS.md`와 현재 sangwon_AI 설계.
웹팀 회신은 자료 요청서다. 구현 완료 보고서로 해석하지 않았다.

## 1. 요청 결론과 문서 우선순위

**웹은 ‘작성·배정·준비·실행·결과’ 전 과정을 지원해야 한다.**
Jetson은 임무 검증·비행 목표 생성·실행 판정을 맡는다.
PX4는 실제 위치·고도·자세 제어와 자체 비상 동작을 맡는다.
웹의 시작 버튼, 서버 ACTIVE, 연결 성공만으로 이륙하지 않는다.

| 구분 | 이번 전달 상태 |
|---|---|
| 운용 정책 | 사용자 합의 반영. 아래 J 요구와 명령 규칙을 구현 기준으로 사용 |
| 화면·서버 계약 | 구체적인 후보안 제공. 웹팀의 기존 코드 대응표와 수락 필요 |
| JSON 예제 | [같은 버전 합성 자료](contracts/web_v1_1_draft4/README.md) 제공 |
| 현재 구현 | C++ 합성 REPLAY. 실제 웹·비행·QR 통합 완료를 뜻하지 않음 |
| 실제 웹 화면 검수 | 제공 사이트 로그인 후 조회 완료. [현행 화면 점검](WEB_UI_AUDIT_2026-10-04.md)과 §4.0 반영. 저장/비행/API 실행 시험은 미수행 |
| 실측 자료 | UWB 전체 사양, 좌표·센서·제동·정렬 승인값 대기. 임의 기본값으로 승인하지 않음 |

이번 외부 계약 후보는 `1.1-draft.4`이다.
아직 정식 API 버전이나 배포된 기능이 아니다.
기존 v1의 HTTP 조회·WS 상태 전송 구조를 확장한다.
r1의 draft.3 부분 예제에서 배정·snapshot 전송·접근 앵커 구조가 바뀌어 버전을 올렸다.

읽는 순서와 충돌 해석은 다음과 같다.

1. 이 요청서: 웹 개편의 범위·의미·우선순위.
2. `contracts/web_v1_1_draft4/`: 이번 필드·합성 예제·시험 시나리오.
3. `QR_SCAN_SPEC.md`, `BT_SPEC.md`, `BOOT_PREFLIGHT_SPEC.md`: Jetson 상세 동작.
4. `WEB_REQUIREMENTS.md`: W01~W18, D01~D10 요구 추적.
5. `WEB_CONTRACT_DRAFT.md`, `WEB_JSON_DRAFT.md`: 이전 설계 근거. draft.2 예제는 구현에 혼용하지 않는다.

이전 `WEB_ADDITIONAL_REQUESTS_2026-10-04.md` r1은 이 요청서로 대체한다.
기존 내부 문서의 외부 필드와 충돌하면 이번 후보 계약을 따른다.
후보 계약 합의가 물리적 비행 승인이나 기존 런타임 변경을 뜻하지는 않는다.

## 2. Jetson이 먼저 보장해야 할 요구사항

웹 개발에 필요한 책임 경계를 아래와 같이 고정한다.

| ID | Jetson 책임 | 웹이 받아야 할 출력/거부 사유 |
|---|---|---|
| J01 | 부팅 후 서비스·장치 자동 연결, 자체 점검 | boot_phase, check 목록, 원인·운영자 조치 |
| J02 | 전체 불변 임무를 내려받아 검증 | assignment 상태, snapshot ID·해시, 검증 오류의 필드 경로 |
| J03 | 지도·실측 변환·센서/기체 프로파일 검증 | 승인 누락·좌표 불일치·미지원 기능 차단 |
| J04 | 일반 기체 이동점과 QR 라벨 위치를 구분 | 실제 기체 목표, 라벨 위치, 접근 대기점 |
| J05 | 이동·고도 변경·회전·스캔 진입/이탈·복귀 검사 | plan ID와 검증된 구간, 불가 구간/원인 |
| J06 | 시스템 준비와 해당 임무 준비를 따로 판정 | preparation ID, readiness 개정·유효기간, can_start |
| J07 | 새 START를 원장에 한 번 수락, 직전 재검사 | 요청별 ACCEPTED/REJECTED, execution ID |
| J08 | C++ 행동 트리와 독립 guard로 실행 | 비행 단계, 활성 작업, 출력 소유권·RC 잠금 |
| J09 | ArUco 정렬·호버·스캐너 최대 3회·카메라 대체 | 스캔 단계, 시도 수, 판독 결과, 대기점 복귀 결과 |
| J10 | 단절 중 수락 임무 계속, 결과 영속 저장 | 재연결 후 동일 ID 결과 동기화, 중복 실행 없음 |
| J11 | 귀환·착륙과 실제 종료 확인 | 비행 결과, 작업 결과, landed/disarmed 근거 |
| J12 | 필수 기록·ULog 수집·보존 상태 관리 | 시작 차단과 경고의 구분, 수집/전송 상태 |

현재 J 항목 전체를 구현했다고 표시하지 않는다.
지원 여부는 배포된 실행기가 capabilities로 보고한다.
미구현 기능을 쓰는 임무는 전체 거부한다.
스캔 작업을 삭제하고 이동점만 실행하면 작업 순서가 달라진다.

### 2.1 입력과 판정의 소유자

| 자료 | 생성/관리 | 최종 판정 | 웹 화면의 역할 |
|---|---|---|---|
| 임무 순서·대상·이륙 높이·체류·선택 yaw | 운영자/웹 | Jetson 전체 검증 | 작성·오류 수정 |
| 지도·라벨·장애물·높이 제한·허용 구역 | 웹에 저장 | 측량/운영 승인 + Jetson 기하 검사 | 편집·개정·승인 이력 |
| UWB 원시 형식·측정 품질 | UWB 제작 담당 | 사양 수령 후 Jetson 어댑터 검증 | 장치 원시 패킷을 직접 해석하지 않음 |
| 카메라/스캐너 장착·QR–ArUco 배치 | 영상/기체 담당 | 실측 승인 | 파일 등록·참조·미수령 표시 |
| 제동·도착·정렬·watchdog 수치 | 비행/Jetson 담당 | 시험 후 프로파일 승인 | 승인 프로파일 선택, 임의 완화 금지 |
| readiness·허용 명령·실제 방문 완료 | Jetson | Jetson/PX4 상태 | 최신 결과 표시. 브라우저 거리 계산으로 완료 처리하지 않음 |
| 재고/업무 반영 | 웹 서버 | 웹 업무 규칙 | 판독 결과와 업무 결과를 따로 표시 |

기체 외형은 약 50×50cm 쿼드콥터다.
실제 프로펠러 포함 외곽·센서 오프셋·여유는 실측 자료로 등록한다.
웹은 호버 추력이나 모터 출력을 보내지 않는다.
목표 속도는 승인된 motion_profile_ref를 선택한다.
사용자 속도 입력 UI는 승인 범위와 추가 필드를 합의한 뒤 제공한다.

## 3. 전체 사용자 흐름

설정은 최초/개정 시 수행하고, 임무는 다음 흐름으로 반복한다.

```text
[사이트 설정] 지도·라벨·구역·측량본·기체 프로파일 등록
      ↓
[임무 초안] 이동점/라벨 순서·이륙 높이·체류·선택 yaw 작성
      ↓ 서버 형식/참조 검사
[배포본 생성] 불변 snapshot 저장
      ↓ 운영자가 기체 선택·배정
[Jetson 수신] 전체 다운로드 → 해시/버전/참조 확인
      ↓ 경로 계산·점검
[시작 준비] 계획 미리보기·상부 공간/현재 방향 확인
      ├─ 차단: 이유와 수정 화면 연결 → 새 개정/재검사
      └─ 신선한 임무 READY → 운영자의 새 START
                                  ↓ Jetson 최종 재검사
[실행 관제] 이륙 → 이동/스캔 → 복귀 → 착륙 확인
      ↓
[결과] 비행 / 각 작업 / 결과 동기화 / 업무 반영 확인
      ↓ 운영자가 선택
[다음 임무 초안] 실패 라벨 재작업 등. 자동 시작 없음
```

### 3.1 상태별 처리 순서

| 단계 | 운영자/웹 | 서버 | Jetson | 다음 화면 |
|---|---|---|---|---|
| 부팅 | 기체 상세 진입 | 마지막 상태와 최신 연결 구분 | 자동 연결·점검. 출력을 활성화하지 않음 | 점검 진행 |
| 초안 저장 | 미완성 자료도 저장 가능 | draft 개정·낙관적 충돌 검사 | 영향 없음 | 편집 계속 |
| 배포본 생성 | 최종 순서 확인 | 필수값·참조 검사 후 snapshot 고정 | 영향 없음 | 기체 배정 |
| 배정 | 대상 기체 선택 | assignment 생성. START 생성 금지 | 해당 배정을 받아 전체 저장·검증 | 준비 진행 |
| 계획 검토 | Jetson 계산 경로·대기점 확인 | plan을 원문대로 저장·전달 | 안전 지점/기체 목표 산출 | 운영 확인 |
| 운영 확인 | 상부 공간·현재 heading 확인 | 해당 계획/출발 문맥에 묶인 확인서 저장 | 확인서·실제 상태 재검사 | READY 또는 차단 |
| 시작 | 준비 ID에 묶인 START 1회 요청 | 요청 원자 저장·10초 만료·전달 | 원장/세션/최신 상태 확인 후 수락 | 요청 중 → 실제 단계 |
| 실행 | 진행 감시·허용 명령만 요청 | 명령/결과/사건 저장 | 임무 수행·비상 중재 | 관제 |
| 종료 | 종료 결과 확인 | 결과 동기화·업무 처리 | landed/disarmed 확인·로그 수집 | 결과 |

오프라인에서는 기체를 선택한 임무 초안까지만 저장한다.
배정 확정에는 최신 기체 상태와 기존 실행 종료 확인이 필요하다.
이미 배정된 자료를 재접속 후 수신하는 동작과 비행 시작은 별개다.
비행 중에는 새 배정·교체를 거부한다.
오프라인이라 비행 여부를 모르면 ‘교체 대기’로 숨겨 배정하지 않는다.

## 4. 화면별 UI·UX 변경 명세

실제 메뉴명은 웹팀이 제안한다. 아래 기능과 연결 흐름은 유지한다.

### 4.0 현재 화면에서 바꿀 부분

2026-10-04 제공 사이트를 직접 조회했다.
기존 기능을 활용하되 다음 의미 충돌을 먼저 수정한다.
이 표의 ‘미확인’은 서버에 기능이 없다는 단정이 아니다.

| ID | 현재 화면·관측값 | 구체적 변경 요청 | 연결 요구 |
|---|---|---|---|
| UI-D01 | 경로 설정: 지도 클릭·공통 이동 Z=120cm·경로 적용 | 임무 초안 저장→배포본 생성→기체 배정→준비→START를 분리. 작업별 XYZ/hold/yaw 편집 | U04~06, J02/J06/J07 |
| UI-D02 | 라벨 포인트: ‘작업 Z(CM) = 드론 스캔 고도’ | 라벨 QR 중심의 바닥 기준 z로 수정. 기존 값 의미 확인 후 수동/검증 migration. 기체 판독 z는 Jetson 출력 | U03, J04 |
| UI-D03 | 라벨: 스캔 필수/선택, 순서 | 지도 라벨 속성과 임무 작업 순서를 분리. snapshot에 선택한 scan은 필수 시도, 최종 판독 실패 후 이탈/다음 목표 정책 적용 | U03/U04, J09 |
| UI-D04 | 라벨: 북/동/남/서, 도착 반경 cm | 방향이 ‘라벨 정면’인지 확인하고 지도 축 기반 법선으로 명시 변환. 반경은 기체 도착/안정/판독 거리로 분리 | U03, J03/J04 |
| UI-D05 | 창고: 비행 고도=120cm·라벨 접근 오프셋=50cm | 이륙/경유점/라벨 높이 필드 분리. 50cm는 시험용 프로파일 후보로 표시. 웹 생성 접근점을 비행 목표로 확정하지 않음 | U02~04, J05 |
| UI-D06 | 장애물: 좌상단 X/Y·가로/세로만 관측 | 실제 점유 z_min/z_max·다각형·천장·고도 구역·스캔 허용 구역·개정 추가 | U02/U03, J05 |
| UI-D07 | 실시간 좌표: cm/기준 앵커, UWB 삼각측량/Z | 지도 m/PX4 융합 위치를 주 표시. cm 입력을 유지하면 명시 단위 변환. UWB 진단은 별도. 앵커 선택으로 제어 좌표계 변경 금지 | U01/U07, J03 |
| UI-D08 | 운영 제어: 출발 승인 토글·긴급 복귀·즉시 착륙 | 지속 승인 플래그를 새 START로 대체. PAUSE/RESUME/CANCEL 추가, 준비/만료/요청 결과/RC 잠금 표시 | U06/U07, J06~08 |
| UI-D09 | 편집 페이지: ‘프론트 자동 반영 중지’ | 편집 폼 보호는 유지. 연결/READY/권한/비상 상태는 계속 갱신. 비행 제어는 관제 화면 중심으로 이동 | U04/U07 |
| UI-D10 | scan-flow: CSI Camera→QR Decode→Retry Buffer→API/DB→Live Web | 비행 작업 단계와 업로드 흐름을 두 줄로 분리. ArUco/호버/스캐너 3회/카메라 QR/대기점 이탈을 표시 | U07/U08, J09 |
| UI-D11 | QR 생성: ArUco DICT_4X4_50 ID 0 고정·px 크기 조절 | label_point_id와 QR 업무 키 연결. 인쇄 배치 개정·실측 마커 m 크기·QR 상대 pose 관리. 동일 ID 모호성 검증 | U03, J09 |
| UI-D12 | 조사 진행률: 품목 완료/미완료/오류 중심 | 실행별 비행 결과·작업 결과·전송·업무 반영 분리. 실패/미시도 scan도 집계 | U08, J10/J11 |
| UI-D13 | 지상국: PC 포트/LoRa 브리지 시작·중지 | PC 브리지 관리와 Jetson 자동 부팅 점검 분리. 브리지 중단이 Jetson 고장을 뜻하지 않게 표시 | U01/U09, J01 |
| UI-D14 | 지도·앵커 표시 사이 일부 좌표 불일치 관측 | 동일 지도/변환 개정·단위·좌표 출처를 대조하고 통일. UI 조회만으로 UWB 원시값을 정정하지 않음 | U02/U09, J03 |

기존 QR 데이터는 화면상 `drone-stock-item/v1`이다.
품목 code만으로 물리 라벨을 유일하게 구분할 수 있는지 웹팀이 확인한다.
동일 품목의 여러 선반 라벨은 label_point_id/위치 연결을 추가한다.
ArUco ID 0 고정은 현재 카메라 시험 설정으로 표시돼 있다.
이를 라벨 고유 ID로 가정하지 않는다.
고유 마커 할당 또는 지역/QR 결합 식별 정책을 공동 승인한다.
모호한 마커를 보고 다른 선반으로 미세 조정하지 않도록 차단한다.

### 4.1 목표 화면 구성

| 화면 ID | 화면/주요 사용자 | 필수 구성·동작 | 오류·빈 상태·다음 이동 |
|---|---|---|---|
| U01 | 기체 목록·상세 / 운영자 | 연결·프로파일·제어권·현재 임무, 자동 점검 진행, 체크별 원인/조치 | 미수신은 UNKNOWN. 시스템 READY·임무 없음은 ‘임무 배정 필요’ |
| U02 | 지도·설정 / 지도 담당 | 원점/직교 축/m/z 기준, 장애물 외곽·높이, 천장·z 제한, yaw 검증 구역 | 미제공과 빈 목록 구분. 변경은 새 개정. 사용 중 비행에는 미적용 |
| U03 | 라벨·스캔 구역 / 지도·영상 담당 | QR 위치/ID·정면, 기대 QR 규칙, ArUco 배치 참조, 허용 공간 | 라벨 정면/장착/구역 미정 표시. 자료 담당과 연결 |
| U04 | 임무 편집 / 운영자 | 이동점 추가·라벨 작업 추가 분리, 순서, takeoff_z_m, hold_s, 선택 yaw, 프로파일 | 잘못된 작업 자동 삭제 금지. 필드 경로·작업/지도 위치로 오류 이동 |
| U05 | 배포·배정 / 운영자 | 초안/불변 배포본 구분, 지도·프로파일 개정·기체 capabilities 확인 | 미지원/비행 중은 배정 거부. 배정 완료는 ‘수신 대기’ |
| U06 | 준비·시작 / 운영자 | 다운로드/검증 진행, Jetson 계획, 현재 위치/heading, 운영 확인, 시작 차단 이유 | READY 유효시간 상실 즉시 시작 해제. 확인서 변경/무효화 이유 표시 |
| U07 | 실시간 관제 / 운영자 | 실제 기체·라벨·기체 목표·방문 순서·스캔 단계, 명령 결과, RC/PX4 상태 | 단절 시 마지막 확인 시각. 결과 미확인을 성공으로 바꾸지 않음 |
| U08 | 결과·이력 / 운영자 | 비행·작업·업로드·업무 반영 4개 요약, 라벨별 실패, 로그 수집 상태 | 실패 라벨로 새 초안 생성. 이미 판독한 결과의 재전송은 재스캔 아님 |
| U09 | 계약·장치 진단 / 개발·관리자 | 배포 버전·capabilities·세션·사건/요청 조회, 인증 발급/회수 이력 | 원시 비밀키 표시 금지. 일반 운영 화면에 내부 파라미터 편집 노출 금지 |

### 4.2 지도 표시 규칙

라벨과 기체 목표를 다른 모양으로 표시한다.

```text
○ 필수 이동점   ▣ QR 라벨   ◇ 접근 대기점   △ 기체
실선: 검증된 이동 구간     점선: 미검증/초안 구간
정면 화살표: 라벨 → 통로   기체 화살표: 실제/목표 yaw
```

- 화면 확대·축척과 무관하게 좌표는 m다.
- 높이 입력에는 ‘바닥 → 기체 기준점’ 또는 ‘바닥 → QR 중심’을 적는다.
- 기체는 라벨 좌표로 이동하지 않는다. Jetson이 판독 위치를 계산한다.
- 웹이 임의로 0.5m를 빼서 안전한 접근점이라고 표시하지 않는다.
- Jetson 계획에는 plan_id, snapshot_id, 검증 개정, 생성 시각을 함께 표시한다.
- 실제 위치는 PX4 융합 위치를 지도에 변환한 값이다.
- UWB 원시 관측은 별도 레이어로 표시한다. 관측 없음은 null이다.
- 높이·회전 공간도 검사한다. 평면 선만 표시해 검증 완료로 간주하지 않는다.
- 승인 구역 밖, 선반 위 비행 미검증, 형상 부족이면 임무를 거부한다.

### 4.3 준비 화면 구성

```text
기체 TEST-01 / REPLAY                 연결 LIVE / 제어권 NONE
시스템 점검  [준비됨]                임무 점검 [차단됨]
임무 이름 · 지도 개정 · 3개 작업      [계획 보기]

차단 1: 카메라 장착 변환 미승인       [자료 등록 화면]
경고 1: RC 미연결                    비상 수동 인계 불가

상부 공간 확인 [미확인]  현재 방향 확인 [미확인]
[재검사]                            [시작 비활성]
```

체크 행은 상태·필수 여부·원인·조치·관측 나이를 제공한다.
PASS/FAIL/UNKNOWN/WARN을 색상과 텍스트로 함께 표시한다.
RC 미연결 WARN만으로 시작을 막지 않는다.
호스트 진단 정상은 FLIGHT READY로 표시하지 않는다.
준비 알림은 새 preparation/개정에서 한 번만 발생시킨다.
READY_REVOKED 또는 신선도 만료 시 배지와 버튼을 해제한다.
LED는 별도 하드웨어 확인 항목이다. 모터로 준비 완료를 알리지 않는다.

### 4.4 명령 버튼과 결과 표시

Jetson의 `allowed_commands`와 각 명령의 거부 이유를 따른다.
웹은 권한·연결·문맥·신선도 조건을 추가로 검사한다.
버튼이 켜져 있어도 Jetson이 실행 직전 다시 검사한다.

| 실제 상태 | START | PAUSE | RESUME | CANCEL | LAND_NOW |
|---|---|---|---|---|---|
| 지상·임무 READY·미실행 | 새 요청 가능 | 불가 | 불가 | 배정 해제 UI 사용 | 불가 |
| START 수락 후 지상 준비 | 불가 | 불가 | 불가 | 준비/시동 중단·지상 확인 | 불가 |
| 자동 이륙 상승 | 불가 | 위치 유지 검증 시 | 일시정지 상태만 | 현 위치 착륙 | 가능 |
| 이동·회전·호버·스캔 | 불가 | 유지 가능한 상태만 | 일시정지 상태만 | 출발점 복귀·착륙 | 가능 |
| 상승 중 일시정지 | 불가 | 불가 | 재검사 후 | 현 위치 착륙 | 가능 |
| 일반 일시정지 | 불가 | 불가 | 재검사 후 | 복귀·착륙 | 가능 |
| RETURNING/LANDING | 불가 | 불가 | 불가 | 기존 동작 유지·NO_OP | 복귀 중 가능 / 착륙 중 NO_OP |
| RC 비상 인계·수동 잠금 | 불가 | 불가 | 불가 | 비행 제어 요청 거부 | 거부·RC 설정에 맡김 |
| PX4 failsafe/제어권 불명 | 불가 | 불가 | 불가 | 권한 없는 요청 거부 | Jetson 권한 검증 실패 시 거부 |
| STALE/OFFLINE | 불가 | 불가 | 불가 | 새 요청 불가·전달 불가 표시 | 새 요청 불가·RC 안내 |

NO_OP은 기존 동작을 바꾸지 않았다는 결과다.
물리적 착륙 완료를 뜻하지 않는다.
LAND_NOW는 현재 위치 PX4 Land 요청이다. 모터 Kill 기능이 아니다.
명령 확인창에는 취소=복귀, 긴급 착륙=현 위치 의미를 표시한다.
RC 인계 후에는 수동 RC 설정을 그대로 사용한다.
같은 비행에서 웹으로 자율 제어권을 회수할 수 없다.

## 5. 임무 데이터와 좌표 계약

임무 데이터의 의미는 아래처럼 고정하고 표현은 후보 JSON으로 제안한다.

| 항목 | 필수 의미/검사 |
|---|---|
| frame | WAREHOUSE_MAP, m, 오른손 직교 좌표, z는 한 층 공통 바닥에서 위쪽 |
| yaw | 후보: 지도 +x가 0°, 위에서 본 반시계 +, [-180,180). 웹·측량 담당 수락 필요 |
| takeoff_z_m | 별도 자동 이륙 높이. 첫 경유점 z와 별개 |
| start_yaw_deg | 선택. 있으면 자동 이륙 후 공간 확인·회전 |
| waypoint | 기체/PX4 기준점 XYZ, 고유 task_id, 선택 도착 yaw, hold_s. 순서대로 모두 방문 |
| scan | 라벨·허용 구역·스캔 프로파일 참조. 기체 이동 XYZ로 읽지 않음 |
| label | QR 중심 좌표 후보, 통로 쪽 단위 법선, 기대 QR 규칙, 고정 ArUco 배치 개정 |
| map volume | 실제 외곽과 점유 z 구간. 기체 여유를 웹에서 미리 빼지 않음 |
| altitude zone | 영역별 기체 기준점 z 최저·최고. 겹치는 제한은 더 좁은 교집합 사용 |
| scan workspace | 기체 전체·여유가 들어갈 물리적 허용 공간. 지도·라벨·승인 개정 참조 |
| launch record | 실제 이륙 전 위치·방향·기체 기준 높이. 웹 마커보다 PX4/UWB 검증값 우선 |
| approvals | 대상 ID·개정·근거·유효성·승인자. 빈 값이나 합성 자료는 FLIGHT 승인 아님 |

거리 40~60cm와 yaw ±10°는 초기 시험 조건이다.
카메라/스캐너의 광학 기준점에서 라벨 면까지 거리를 정의한다.
기체 중심에서의 거리로 바꾸려면 실측 장착 변환이 필요하다.

### 5.1 접근 대기점과 재사용 지도 객체

지도 객체에는 특정 임무 task_id를 저장하지 않는다.
같은 라벨 구역을 다른 임무에서도 사용할 수 있어야 한다.
r1의 `scan_workspaces.entry_anchor_task_id` 제안은 폐기한다.

이번 후보는 scan 작업에 `approach_anchor`를 둔다.

| kind | 의미 | 제약 |
|---|---|---|
| LAUNCH | 출발 XY에서 이륙/인계 후 검증한 높이의 대기점 | 첫 작업 scan에 사용 가능. 바닥 좌표로 비행시키지 않음 |
| PREVIOUS_TASK_EXIT | 바로 앞 작업의 실제 완료 출구를 대기점 기준으로 사용 | waypoint면 그 기체 목표, scan이면 앞 작업의 복귀 대기점 |

첫 작업은 LAUNCH만 허용한다.
나머지는 PREVIOUS_TASK_EXIT와 바로 앞 task_id를 함께 사용한다.
미래 참조·순환·임의 작업 건너뛰기는 거부한다.
Jetson은 기준점이 해당 구역과 연결 가능한지 검사한다.
안전한 대기점을 만들 수 없으면 임무를 거부한다.
필요한 경우 운영자가 앞에 이동점을 추가한 새 개정을 만든다.
서버가 몰래 이동점을 삽입하지 않는다.
연속 scan도 같은 대기점에서 각각 검증된 진입·이탈을 수행할 수 있다.

### 5.2 경계가 없는 첫 버전

전체 비행 경계 W13은 조건부 P1이다.
미제공이면 일반 자동 우회 기능을 끈다.
승인된 이동 구간과 검증된 기록 복귀 경로를 사용한다.
스캔에는 별도로 라벨별 국소 허용 구역이 반드시 필요하다.
국소 구역을 창고 전체의 우회 허가로 확장하지 않는다.
장애물에 막혀 10초 안 해결하지 못하면 복귀를 시도한다.
복귀 불가면 PX4 Land를 요청한다.

## 6. API 전체 흐름과 연결점

**URL은 후보다. operation_id와 의미를 먼저 공동 고정한다.**
기존 URL도 현재 서버 코드에 존재하는지 웹팀 확인이 필요하다.
인증은 기존 v1 체계의 기체 한정 권한을 재사용할 수 있는지 검토한다.
운영자 인증과 기체 인증은 분리한다. 비밀키는 샘플에 넣지 않는다.

### 6.1 브라우저 ↔ 서버

| operation_id | 후보 method/path | 입력 → 결과 | 소유/주요 규칙 |
|---|---|---|---|
| WEB_MAP_REVISION | POST `/api/maps/{map_id}/revisions/` | 지도·라벨·구역 → 새 개정/필드 오류 | 웹. 기존 개정 수정 금지 |
| WEB_MISSION_DRAFT | POST/PATCH `/api/missions/`·`/{id}/` | 초안·base_revision → 새 초안 개정 | 웹. 동시 수정은 409 |
| WEB_PUBLISH | POST `/api/missions/{id}/snapshots/` | 초안 개정 → snapshot_ref | 웹. 전체 자료 고정·원본 바이트 보존 |
| WEB_ASSIGN | POST `/api/drones/{id}/mission-assignments/` | snapshot_id·base_assignment_revision → assignment | 웹. 최신 상태·기존 실행 종료 확인. 이륙 명령 아님. 비행 중 교체 금지 |
| WEB_UNASSIGN | DELETE `/api/drones/{id}/mission-assignments/{assignment_id}/` | 배정 개정 → 해제 결과 | 수락 실행/비행 없음 확인 필요. 실행 중이면 CANCEL 사용 |
| WEB_RECHECK | POST `/api/drones/{id}/preparations/recheck/` | assignment·현재 문맥 → 재검사 요청 ID | 비행 명령 아님. 준비 ID는 Jetson만 발급 |
| WEB_ATTEST | POST `/api/drones/{id}/operator-attestations/` | plan·출발 문맥·현재 방향/상부 확인 → 확인서 | 웹. 의미가 바뀌면 새 확인 필요 |
| WEB_COMMAND | POST `/api/drones/{id}/control-requests/` | action·context·client_request_key → 저장된 명령 | 서버가 ID·seq·시각 발급. HTTP 201은 비행 수락 아님 |
| WEB_COMMAND_RESULT | GET `/api/drones/{id}/control-requests/{request_id}/` | ID → 전달 상태 + Jetson 결과 | UI가 응답 유실을 조회로 해결 |
| WEB_STATE | GET `/api/drones/{id}/autonomy-state/` | 없음 → 최신 상태·원천 시각·연결/준비·plan | 브라우저 재접속 초기 동기화 |
| WEB_HISTORY | GET `/api/drones/{id}/executions/{execution_id}/` | 실행 ID → 요청/작업/종료/업로드 결과 | 페이지/사건 cursor 보존 |

브라우저 실시간 전달 방식은 기존 웹 구성을 따른다.
WebSocket/SSE 선택은 웹팀이 회신한다.
브라우저가 PX4 또는 Jetson 제어 포트에 직접 연결하지 않는다.
동시 운영자는 서버가 직렬화한다. 늦은 요청은 BUSY/문맥 오류로 거부한다.

### 6.2 Jetson ↔ 서버

| operation_id | 연결점 | 요청/응답 의미 |
|---|---|---|
| DEVICE_SESSION | 신규 POST `/api/drones/{id}/control-sessions/` | boot/runtime·profile·지원 버전/기능 → session ID·서버 시각·협상 결과 |
| DEVICE_MISSION | 기존 GET `/api/drones/{id}/companion-mission/` | assignment·snapshot_ref·recheck/attestation·현재 세션의 control_requests 반환 |
| DEVICE_SNAPSHOT | 신규 GET `/api/mission-snapshots/{snapshot_id}/content/` | 불변 UTF-8 JSON 파일 바이트. SHA256·길이는 snapshot_ref와 대조 |
| DEVICE_PREPARATION | 신규 POST `/api/drones/{id}/preparation-reports/` | assignment 수신/검증/차단, plan, readiness 사건 저장 |
| DEVICE_STATE | 기존 WS `/ws/drones/{id}/` | telemetry/readiness/health. 명령 전송 채널로 사용하지 않음 |
| DEVICE_ACK | 기존 POST `/api/drones/{id}/control-action/ack/` | 명령 ID별 수락·거부·완료/실패/선점, 개정 원장 저장 |
| DEVICE_EVENT | 기존 POST `/api/drones/{id}/companion-phase/` | event_id·seq·원천 시각·실제 단계/최종 결과 |
| DEVICE_LAUNCH | 기존 POST `/api/drones/{id}/launch-snapshot/` | 실제 이륙점·방향·flight_id·변환·기체 문맥 |
| DEVICE_SCAN_RESULT | 신규 POST `/api/drones/{id}/scan-task-results/` | 실패 포함 판독/이탈 결과를 개정별 저장 |
| DEVICE_SCAN_QUERY | 신규 GET `/api/drones/{id}/scan-task-results/{task_result_id}/` | 저장 개정·업무 처리 상태 조회 |

snapshot_ref의 URI는 인증된 서버의 허용 경로로 제한한다.
요청서 샘플은 자료 전체를 한 JSON에 넣는다.
전체 파일의 실제 UTF-8 바이트에 SHA256을 계산한다.
파싱 후 재직렬화한 내용에 해시를 계산하지 않는다.
동일 snapshot_id의 다른 바이트는 SNAPSHOT_CONFLICT다.
장치별 실행 프로파일 본체는 승인된 Jetson 설정이며 ID/개정/해시를 대조한다.
이후 대형 자료를 분리하려면 동일한 불변 파일/해시 규칙으로 계약 개정한다.

### 6.3 배정·계획·확인서·준비의 연결

`assignment`는 전송과 검증의 진행을 추적한다.
상태는 QUEUED/FETCHING/VALIDATING/AWAITING_OPERATOR/READY/BLOCKED/EXECUTING/CLOSED다.
서버 저장 성공만으로 READY로 올리지 않는다.
READY는 현재 Jetson 보고와 신선도에서 파생한다.

`plan`은 snapshot·기체 설정·변환·출발 문맥과 결합한다.
각 segment에 작업 ID·기체 목표 종류·XYZ/yaw·승인 공간 참조를 둔다.
프리뷰 계획은 모터 명령이나 실시간 제어 API가 아니다.

`operator_attestation`은 snapshot_id·plan_id·launch_context_id·확인 항목에 묶는다.
처음에는 CHECKING/확인 대기 상태여도 계획을 볼 수 있다.
확인서 수신 뒤 Jetson이 재검사하여 preparation을 만든다.
계획/출발 방향/승인 근거가 바뀌면 확인서를 무효화한다.
동일 근거를 heartbeat로 갱신했다고 재확인을 계속 요구하지 않는다.
확인서의 최대 나이는 운용 프로파일에 포함한다. FLIGHT 값은 승인 대기다.

## 7. 식별자·명령·시각·재연결 규칙

식별자는 UI 라벨과 분리하고 아래 발급 책임을 고정한다.

| 항목 | 발급자 | 유효 범위/변경 시점 |
|---|---|---|
| mission_db_id / mission_code | 서버 | 기존 DB 정수 ID / 사람이 읽는 코드 |
| snapshot_id / assignment_id | 서버 | 불변 배포본 / 기체 배정 회차 |
| task_id / label_point_id | 웹 편집·서버 검증 | snapshot 안 고유 작업 / 개정된 라벨 |
| boot_id / runtime_session_id | Jetson OS / C++ 실행기 | 재부팅 / 실행기 재시작 |
| control_session_id | 서버 협상 | boot/runtime 결합. 재연결 시 새 세션 |
| client_request_key | 브라우저 | 한 번의 운영자 의도. 응답 유실 재시도에도 동일 |
| control_request_id / command_seq | 서버 | 최초 원자 저장 시 발급. 세션 내 증가 |
| plan_id / launch_context_id | Jetson | 검증 계획 / 실제 이륙 준비 근거 변경 |
| preparation_id / readiness_revision | Jetson | 준비 근거 / 준비 상태 의미 변화 |
| execution_id | Jetson | START 최초 수락 시 생성 |
| flight_id | Jetson | 실제 이륙 감지 시 생성. RC 예외 이륙은 START 전 존재 |
| task_result_id / client_scan_id | Jetson | 실행 내 스캔 작업 결과 / 논리적 판독 건 |
| event_id / event_seq | Jetson | 사건 중복 제거 / runtime 내 순서 |

`control_request_id`를 새 계약의 요청 필드명으로 통일한다.
기존 ACK `request_id`는 서버 어댑터에서만 명시 매핑한다.
샘플의 문자열 ID를 숫자로 강제 변환하지 않는다.

### 7.1 명령 수락

1. 서버는 같은 client_request_key와 같은 내용이면 기존 명령을 반환한다.
2. 같은 키의 다른 내용은 HTTP 409다. 새 명령으로 만들지 않는다.
3. 최초 생성 UTC부터 10초를 새 수락 기한으로 둔다.
4. Jetson은 원장에 있는 요청을 먼저 확인한다. 같은 ID·같은 원문이면 기존 결과를 반환한다.
5. 신규 요청만 세션·순서·시각·문맥·준비·권한·단계를 검사한다.
6. 시계 불확실 시 CLOCK_UNTRUSTED로 거부한다. 만료 시간을 늘리지 않는다.
7. START 수락을 영속 기록한 뒤 실행한다. 수락 후 기한이 지나도 임무는 계속한다.

START는 snapshot·assignment·preparation·준비 개정을 포함한다.
지상 AUTO_TAKEOFF에서는 execution_id/flight_id가 null이다.
RC 예외 시작은 `start_mode=RC_HANDOVER`와 실제 flight_id를 포함한다.
이 경우 수직 이륙·제자리 호버·출발 기록·직전 방향 확인이 필요하다.
인계 후 현재 높이에서 시작하여 안전한 지점에서 다음 높이로 조정한다.

PAUSE/RESUME/CANCEL은 실행 문맥을 포함한다.
공중에서는 flight_id도 일치해야 한다.
LAND_NOW는 현재 flight_id와 Jetson 제어권을 검사한다.
이전 비행의 지연된 착륙 명령을 새 비행에 적용하지 않는다.

### 7.2 결과·연결 상태

| 층 | 상태/의미 |
|---|---|
| 서버 전달 | CREATED/AVAILABLE/DELIVERY_UNKNOWN/RESULT_RECEIVED/EXPIRED_UNCONFIRMED |
| Jetson 처리 | ACCEPTED/REJECTED/COMPLETED/FAILED/PREEMPTED |
| 현재 비행 | PREPARING/TAKING_OFF/HANDOVER/NAVIGATING/ROTATING/HOVERING/PAUSED/BLOCKED_WAIT/RECOVERY_WAIT/RETURNING/LANDING/ENDED 등 |
| 연결 | LIVE/STALE/OFFLINE. 원천 시각과 마지막 서버 수신 시각을 둘 다 보존 |

10초가 지났다는 이유만으로 ‘기체가 실행하지 않았다’고 확정하지 않는다.
ACK가 유실됐을 수 있어 EXPIRED_UNCONFIRMED로 표시한다.
재연결 시 기존 요청 결과와 현재 실행을 먼저 조회한다.
이미 수락한 명령 결과는 옛 세션이어도 기록을 받는다.
새 세션에서는 옛 미수락 요청을 전달·실행하지 않는다.
브라우저가 새 ID를 만들어 자동 재시도하지 않는다.

서버 원장은 재시작 후에도 보존한다.
동일 ID/개정·동일 내용은 중복 성공이다.
동일 ID/개정·다른 내용은 409 충돌이다.
낮은 개정은 이력으로 처리하고 최신 상태를 덮어쓰지 않는다.
종료된 실행 ID·중복 제거 표식은 재전송 가능 기간보다 오래 보존한다.
정확한 보존 기간·용량 상한은 웹팀 운영 회신 항목이다.
원장이 없어진 장치에서는 새 FLIGHT START를 허용하지 않는다.

## 8. 점검과 상태 DTO

체크 목록은 Jetson이 제공하며 웹이 센서 판정을 재구현하지 않는다.

| 그룹 | 체크 ID | 내용 |
|---|---|---|
| 호스트 | BP-C01~09 | 설정·의존성·장치·저장·원장·시계·자원·프로세스·단일 출력 |
| 기체 | BP-C10~20 | PX4 문맥·모드·health·융합 위치·yaw·배터리·UWB·guard·RC·LiDAR·웹 |
| 임무 | BP-C21~28 | 묶음·지도·변환·전체 경로·운영 확인·출발점·승인 프로파일·명령 문맥 |
| 스캔 추가 후보 | QS-C01~05 | 카메라/스캐너, 장착/배치, 라벨 식별/정면, 구역/진입·이탈, 정렬/판독 프로파일 |

QS 체크는 scan 없는 임무에서 `applicable=false`다.
기체 필수 체크를 임의 제외하는 용도로 쓰지 않는다.
BP-C28은 준비 단계에서 세션/수락 가능 문맥을 검사한다.
실제 신규 요청의 만료/중복 검사는 요청 도착 때 추가한다.

체크 필드: check_id, status, required, applicable, blocking,
code, reason, operator_action, source, observed_at,
source_age_ms, max_age_ms, check_seq.
FAIL/UNKNOWN이 필수이면 시작을 막는다.
WARN 허용은 RC 미연결 등 명시된 정책에만 적용한다.
웹이 missing check를 PASS로 채우지 않는다.

readiness 필드: system_state, mission_state, profile,
flight_authority, can_start, allowed_execution,
snapshot/assignment/preparation, revision,
generated_at, valid_until, context, checks, allowed_commands.
host 진단은 flight_authority=false이고 실제 시작 권한을 발급하지 않는다.
REPLAY 예제의 can_start는 격리된 모의 실행에만 적용한다.
실기체 시작 버튼은 FLIGHT 권한·지원 프로파일·실제 준비가 모두 필요하다.

telemetry에는 실제 모드·제어권·RC 잠금·배터리·융합 위치와
UWB 원천 관측·활성 기체 목표·스캔 진행을 각각 보낸다.
정지 좌표가 반복되어도 새 관측 시각이 있으면 새 측정일 수 있다.
UWB가 직전 값을 재전송한 자료는 새 관측으로 융합하지 않는다.
UWB 세부 DTO는 전체 장치 사양 수령 뒤 어댑터에서 확정한다.

후보 통신 주기: 임무 조회 2Hz, telemetry 10Hz, readiness 1Hz.
합성 시험 readiness 유효기간은 2초다.
센서 최대 나이·시계 허용 오차·FLIGHT 준비 수명은 시험 승인값이 필요하다.
서버 수신 시각으로 오래된 원천 값의 나이를 0으로 초기화하지 않는다.

## 9. 스캔·결과·업무 처리

스캔 판독과 비행 경로 완료를 독립적으로 저장한다.

```text
대기점 → 검증된 접근 → ArUco 획득/미세 조정 → 호버 안정
  → 스캐너 최대 3회 → 실패 시 카메라 QR 판독
  → 결과 로컬 저장 → 검증된 대기점 복귀 → 다음 작업
```

카메라는 ArUco 정렬 때부터 필요하다.
카메라 QR 디코딩만 스캐너 3회 실패 뒤 시작한다.
판독창을 열었으면 1회로 센다. 일시정지/재정렬로 횟수를 초기화하지 않는다.
최종 QR 실패·마커 획득 한도 실패·카메라 고장은 실패 기록 후 이탈한다.
비행 상태와 이탈 경로가 정상일 때만 다음 목표로 진행한다.
이탈 실패를 라벨 실패 하나로 덮고 다음 목표로 이동하지 않는다.

| 결과 항목 | 값/판정자 | UI 의미 |
|---|---|---|
| flight_outcome | SUCCEEDED/CANCELED/FAILED/MANUAL_HANDOVER, Jetson | 귀환·착륙과 제어권 종료 |
| work_outcome | ALL_SUCCEEDED/PARTIAL_FAILURE/FAILED/INCOMPLETE/NOT_APPLICABLE, Jetson | 필수 작업의 실제 완료 집계 |
| scan_outcome | SUCCEEDED/FAILED/NOT_ATTEMPTED, Jetson | QR 일치 판독 여부 |
| egress_status | NOT_STARTED/IN_PROGRESS/COMPLETED/FAILED/PREEMPTED, Jetson | 대기점 복귀 상태 |
| upload_status | PENDING/STORED/CONFLICT, 저장 결과 | 서버 기록 동기화 여부 |
| business_status | NOT_APPLICABLE/PENDING/APPLIED/REJECTED, 서버 | 재고 등 업무 반영 여부 |

예: 10개 중 8개 판독, 복귀·착륙 성공은
‘비행 완료 / 라벨 8 성공·2 실패’다.
전체 작업 성공으로 표시하지 않는다.
아직 실행하지 않은 작업이 있으면 work_outcome=INCOMPLETE다.
모든 작업이 종료됐고 성공/실패가 섞이면 PARTIAL_FAILURE다.
판독 성공이어도 이탈이 실패하면 해당 작업 정상 완료가 아니다.
동일 scan 작업의 재시도 3회는 3개 업무 결과로 만들지 않는다.

`task_result_id`는 논리 작업당 고정한다.
판독 확정 결과를 revision 1로 저장할 수 있다.
이탈 완료/실패 후 같은 ID의 revision 2로 갱신한다.
판독 내용은 확정 후 임의 덮어쓰지 않는다.
실패 결과도 raw_qr_data=null로 저장한다.
다른 QR은 TARGET_MISMATCH이며 정상 재고 반영으로 보내지 않는다.
시각/장치 세션/작업 세대를 검사해 이전 작업의 늦은 판독을 배제한다.

새 scan-task-results API를 새 경로의 단일 저장 창구로 제안한다.
기존 `/scan/` 재고 처리 코드는 서버 내부에서 한 번만 연결한다.
Jetson이 두 API에 동일 판독을 이중 전송하지 않는다.
서버는 client_scan_id별 업무 반영을 한 번만 수행한다.
응답 유실 시 같은 ID/개정으로 재전송하고 기존 업무 결과를 반환한다.
업무 거부는 비행 중 자동 재스캔 사유가 아니다.

ULog 검증 사본의 비행 종료 후 72시간 삭제 정책은
QR 작업 결과·명령 원장·미전송 결과에 적용하지 않는다.
결과 보존/업무 이력 보존은 별도 서버 정책으로 회신한다.

## 10. 비상·실패 UX와 최소 결과 코드

웹은 이미 결정된 비행 정책을 설명하고 실제 결과를 표시한다.

| 상황 | 동작 | 코드/표시 예 |
|---|---|---|
| 배터리 30% 이하 | 즉시 복귀 시도, 불가 시 Land | BATTERY_RETURN |
| Jetson에만 잔량 미수신 | PX4 상태 정상 시 임무 계속 | BATTERY_TELEMETRY_UNAVAILABLE |
| PX4 자체 배터리 상태 상실 | Land 요청 | PX4_BATTERY_INVALID |
| 위치 상실 | 유지 가능한 경우 최대 8초 재검사. 복구 후 자동 재개, 아니면 Land | POSITION_RECOVERY_WAIT / POSITION_LOST |
| yaw reset/품질 상실 | 이동 중단·Land | HEADING_INVALID |
| LiDAR 단절 | 별도 대기 없이 승인 경로 진행, 우회 비활성 | LIDAR_DEGRADED |
| 목표 도달 제한 초과 | 해당 목표 실패·복귀, 불가 시 Land | TARGET_TIMEOUT |
| 필수 기록 실패 | 복귀·착륙. 다음 시작 차단 | RECORDING_FAILED |
| ULog 수집만 실패 | 경고, 다음 임무 허용 | ULOG_COLLECTION_FAILED |
| Jetson/Offboard 송신 상실 | PX4 독립 Land. 1초는 시험 시작값 | OFFBOARD_LOSS |
| RC 수동 인계 | Jetson 출력 중단·같은 비행 재개 금지 | MANUAL_LOCKED |

신규 요청 거부 코드는 최소 다음 목록을 지원한다.

- 계약/자료: UNSUPPORTED_CONTRACT, UNSUPPORTED_CAPABILITY, SNAPSHOT_CONFLICT, INVALID_MAP, INVALID_WAYPOINT, INVALID_SCAN_TASK, MISSING_APPROVAL, TRANSFORM_UNAVAILABLE.
- 준비: SYSTEM_NOT_READY, MISSION_NOT_READY, READINESS_STALE, PREPARATION_STALE, STORAGE_NOT_READY, OPERATOR_CONFIRMATION_REQUIRED, DEVICE_STATE_UNAVAILABLE.
- 명령: COMMAND_EXPIRED, CLOCK_UNTRUSTED, REQUEST_ID_CONFLICT, STALE_COMMAND_SEQUENCE, STALE_RUNTIME_SESSION, STALE_CONTROL_SESSION, CONTEXT_MISMATCH.
- 단계/권한: BUSY, INVALID_PHASE, MANUAL_LOCKED, CONTROL_NOT_OWNED.
- 스캔: SCAN_GEOMETRY_UNAVAILABLE, MARKER_UNAVAILABLE, CAMERA_UNAVAILABLE, QR_UNREADABLE, TARGET_MISMATCH, SCAN_EGRESS_FAILED.

거부에는 code·reason·operator_action·blocking_check_ids·field_errors를 포함한다.
모르는 코드는 그대로 표시하고 성공으로 취급하지 않는다.
HTTP 400=형식 오류, 401/403=인증/권한, 404=자료 없음,
409=ID/개정/상태 충돌, 413=크기 초과, 422=내용 검증 실패를 후보로 둔다.
HTTP 저장 성공과 Jetson 명령 REJECTED는 구별한다.

## 11. 기존 v1 마이그레이션 요청

웹팀은 아래 대응표를 실제 코드/DB 기준으로 채워 회신한다.

| 기존 계약 | 이번 변경 | 이행 요구 |
|---|---|---|
| ACTIVE로 임무 실행 | 새 START+preparation | ACTIVE 상태만으로 실행 경로 제거 |
| control_action 한 개 | ID/세션/문맥을 가진 control_requests | 구형·신형을 동시에 실행하지 않음 |
| HOLD 중 경로 갱신 | 수락 snapshot 고정 | PAUSED에도 새 경로 교체 차단 |
| XYZ만 있는 점 | waypoint/scan 구분 | 기존 점을 라벨 좌표로 자동 변환하지 않음 |
| UWB_ANCHOR_LOCAL | 측량된 WAREHOUSE_MAP | 축 이름만 바꾸지 않음. 승인 변환 필요 |
| x/y UWB | pose_fused와 uwb_observation | 기존 의미 보존 또는 명시 버전 전환 |
| `/scan/` 성공 위주 | 실패/이탈도 기록하는 작업 결과 | 중복 재고 반영 방지·이전 데이터 보존 |
| 고정 대기 0.7초 등 | hold_s와 도착 안정 조건 분리 | 실측 전 도착 성능 승인값으로 사용 금지 |
| 임시 최신 상태 | 내구성 명령/결과/사건 원장 | 재시작·재접속·역순 처리 시험 |

신형 기능은 테스트 기체/테스트 환경에서 먼저 켠다.
구형 기체에는 기존 계약만 제공한다.
계약 미지원 기체가 신형 작업을 무시하고 움직이지 않도록 거부한다.
롤백은 새 시작을 차단하고 진행 비행의 현 계약을 유지하는 방식으로 한다.
비행 중 snapshot/명령 파서 버전을 교체하지 않는다.
실제 DB migration과 롤백 스크립트는 웹팀 코드 검토 후 작성한다.

## 12. 양쪽 동시 개발 순서와 완료 조건

기체 완성을 기다리지 않고 합성 계약으로 양쪽을 개발한다.

| 단계 | 웹 담당 산출물 | Jetson 담당 산출물 | 공동 완료 조건 |
|---|---|---|---|
| G0 계약 검토 | 실제 API/DB/화면 대응표·수정 제안 | 이 문서·fixture·미지원 목록 | 버전·operation·필드 의미·오류 의미 수락 |
| G1 합성 흐름 | U01~U09 화면·서버 저장·mock 기체 | fixture 재생·명령/결과 어댑터 테스트 | 초안→배정→READY→모의 실행→결과 전체 연결 |
| G2 실패 시험 | 단절/재시작·중복·복수 사용자 UI | 원장·세션·guard·준비 해제 주입 | 아래 E 시험의 요청/응답·화면 증거 |
| G3 실장 연결 | 실제 환경·인증·관제 | PX4/UWB/카메라/스캐너·환경 검증 | 프로펠러 없는 점검·물리 인터페이스 검증 |
| G4 제한 비행 | 운영 확인·기록·장애 대응 화면 | 승인된 위치/yaw/제동/스캔 수치 | 비행 담당 승인 후 제한된 현장 시험 |

현재 시작 가능한 작업: UI 와이어프레임, 도메인 모델, 서버 원장,
후보 계약 검토, 합성 fixture 기반 양방향 송수신 개발.
UWB 수신/융합은 전체 사양을 받은 뒤 개발한다.
실측 미정이어도 UI는 ‘자료 대기’를 구현할 수 있다.

### 12.1 인수 시험

| ID | 입력/사건 | 기대 결과 |
|---|---|---|
| E01 | 부팅 + 옛 ACTIVE/START | 자동 시동 없음, 새 준비/새 요청 필요 |
| E02 | 시스템 정상 + 임무 없음 | NO_MISSION, START 비활성 |
| E03 | 저장→배정→다운로드→계획→확인 | 단계별 표시, 수락 전 flight_id 없음 |
| E04 | 누락/변조/다른 개정 snapshot | 전체 거부, 필드/해시 오류, 작업 삭제 없음 |
| E05 | READY 뒤 원천 관측 정지 | 반복 WS와 무관하게 준비 해제·시작 거부 |
| E06 | double click·POST/ACK 응답 유실 | 같은 client_request_key/명령 ID, 실행 한 번 |
| E07 | 10초 만료·시계 불명·옛 세션 | 신규 수락 거부. 기존 수락 결과 조회 가능 |
| E08 | Wi-Fi 단절 중 수행·재연결 | 임무 계속, 미전달 명령 자동 실행 없음 |
| E09 | RC 예외 수직 이륙 인계 | 출발 기록·현재 방향 확인. 수평 이동/기록 없음 거부 |
| E10 | RC 비상 인계 | 기존 RC 설정 유지, 같은 비행 재개 거부 |
| E11 | 지상/상승/일반/복귀 중 취소 | 중단/현 위치 Land/복귀 Land/NO_OP 구별 |
| E12 | 첫 scan·연속 scan | 앵커 참조 유효, 라벨 XYZ 직접 목표 없음 |
| E13 | 스캐너 성공·3회 실패 | 성공 시 종료, 실패 후만 카메라 QR. 정렬 카메라는 선사용 |
| E14 | 최종 QR 실패·마커/카메라 실패 | 실패 기록→검증된 대기점 복귀→다음 목표 |
| E15 | 판독 성공 후 이탈 실패 | 판독/이탈 분리, 다음 목표로 강제 진행 없음 |
| E16 | QR 결과 중복/응답 유실/역순 개정 | 결과 보존, 재고 한 번, 최신 개정 역행 없음 |
| E17 | 일부 scan 실패·정상 착륙 | 비행 성공과 PARTIAL_FAILURE/FAILED 작업 결과 구별 |
| E18 | 실행 중 지도 수정/새 배정 | 현재 snapshot 유지, 새 배정 거부 |
| E19 | ULog 실패 vs 필수 기록 실패 | 경고 허용 vs 복귀/다음 시작 차단 |
| E20 | 서버/어댑터/C++ 재시작 | 원장 복원, 공중 자동 재개 없음, 현재 상태 재동기화 |
| E21 | REPLAY/SITL/호스트 진단 | 실기체 READY·출력 권한으로 승격되지 않음 |
| E22 | 전체 경계 없음·국소 스캔 구역 있음 | 일반 우회 비활성, 구역 내 접근만 검증 |
| E23 | 다른 QR·늦은 이전 판독 | 목표 오인/중복 업무 반영 없음 |
| E24 | 두 운영자 동시 시작/옛 편집 저장 | 원자 충돌 검사, 한 실행만 수락 |

위 항목은 시험 요청이다. 완료 표시가 아니다.
기존 WT/WH/WA 사례는 이 표와 함께 추적한다.
증거는 요청/응답 원문·원천/수신 시각·화면 캡처·실행 로그다.
새 UI 화면이 보이는 것만으로 연동 완료로 판정하지 않는다.

## 13. 웹팀이 회신할 내용

다음 회신을 받으면 서로 다른 가정을 줄이고 구현을 시작할 수 있다.

| 회신 ID | 요청 자료 | 누가 작성 |
|---|---|---|
| R01 | 이번 조회 화면을 기준으로 프런트/백엔드 코드 경로·UI-D01~14 실제 구현 대응 | 웹 |
| R02 | U01~U09 목표 와이어프레임·메뉴 이동·오류/빈/단절 화면 | 웹 UI·UX |
| R03 | operation별 실제 URL/method·request/response·기존 코드 대응 | 웹 서버 |
| R04 | draft.4 후보 필드·ID·좌표·방향·scan 접근 anchor 수락/수정표 | 웹+Jetson |
| R05 | 불변 snapshot/배정/원장/사건/scan 업무 DB 모델·중복/원자 처리 | 웹 서버 |
| R06 | 계정 역할·기체 인증 발급/회수·환경 분리·테스트 기체 ID | 웹 서버 |
| R07 | 최대 payload·호출 제한·timeout·원장/결과 보존·재연결 기준 | 웹 서버+Jetson |
| R08 | 지도·라벨 법선·국소 구역·승인 등록 지원 일정, 전체 경계 일정 | 웹+지도 담당 |
| R09 | E01~E24 구현 담당·일정·시험 증거, 미지원 사유 | 양쪽 |

답변 양식:

| 요구 ID | 기존 지원 근거 | 수락/수정 필요/미지원 | 변경안·영향 | 담당 | 예정일 | 시험 ID |
|---|---|---|---|---|---|---|
| Uxx / Jxx / operation_id | 파일·함수·화면 |  |  |  |  | Exx |

### 13.1 우리 쪽에서 제공할 자료와 보류

| 기존 요청 | 이번 제공/후속 | 현재 상태 |
|---|---|---|
| 상세 문서 6종 | 배포 묶음에 원문 포함, 이 요청서를 앞에 배치 | 제공 |
| D01 필드 대응 | §11 의미 대응. 실제 코드 대응은 R03 | 우리 안 제공·웹 회신 필요 |
| D02~D05 JSON | 같은 문맥 9종 + snapshot/plan/session/scan/result 보조 파일 | 합성 예제 제공 |
| D06 승인 자료 | DTO·합성 예제. 실제 승인본은 측량/영상/비행 담당 | 실측 대기 |
| D07 환경/인증 | 비밀 없는 절차/격리 요구. 실제 주소·발급은 R06 | 웹 회신 필요 |
| D08 계획 | G0~G4·R01~R09·E01~E24 | 담당자/일정 기입 필요 |
| D09 점검 | BP-C01~28 + QS-C01~05 합성 상태 | 구조 제공·물리 나이 기준 미정 |
| D10 준비 UI 증거 | U06와 E02/E05/E21 인수 조건 | 실제 화면 구현 후 증거 요청 |
| UWB | 전체 사양을 받은 뒤 어댑터로 수용 | 형식/정확도/융합 미확정 |
| 장착·판독/도착/제동 수치 | MEASUREMENT_REGISTER와 QR_SCAN_SPEC | FLIGHT 승인 전 필수 |

## 14. 전달할 핵심 문장

Jetson이 임무를 받아 검증하고 기체 목표를 생성하는 것을 기준으로 웹을 개편해 주세요.
초안 저장, 불변 배포본, 기체 배정, Jetson 계획/점검, 운영 확인, 새 START, 실시간 관제, 결과 확인을 연결해야 합니다.
라벨 좌표는 기체 목표가 아니며, 스캔 접근·ArUco 정렬·대기점 복귀는 Jetson이 판단합니다.
이 문서의 U/J/API/E 항목에 대해 현재 코드 대응, 변경 제안, 담당과 일정을 회신해 주세요.
후보 계약과 합성 fixture를 먼저 공동 수락하고 양쪽 개발을 병행하겠습니다.
실제 웹 화면은 조회했으며, 외부 계약 수락과 서버·기체 동작 시험은 남아 있습니다.
