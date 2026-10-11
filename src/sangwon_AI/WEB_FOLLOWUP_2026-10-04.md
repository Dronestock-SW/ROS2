# 웹 검수 2~10차 후속 및 공동 연동 순서

기준일: 2026-10-04. 최신 수신 자료는 `references/web/`에 원문 그대로 보관한다.
웹팀의 자체 시험 결과와 Jetson 공동 인수 결과를 구분한다. 관리자 페이지에서는 작업하지 않는다.

## 0. Git 소스 직접 대조 — 추가 확인

사용자 GitHub 인증 후 비공개 저장소를 조회했다. 작성자 `JihoLee`, 최신 확인 커밋
`aa67a531f4c75b8d6132c0cdb7ab8e9b4ba7791e` (2026-10-04 17:24 KST)에서 API를 대조했다.
이어 `0ca0eb0f1a8920374c3d3ff8db50463f459e7815` (17:28 KST)까지 확인했고 추가 변경은 구현 범위 안내 문구다.
9차 연동 대응표는 `bdda580dd6d4f32fd32fe69b8729666c5b1ed24d` (17:33 KST)와 대조했다. 문서·링크 변경이며 추가 API 배포가 아니다.
10차 배포 검토는 `9a3ce221eb1151068dba9b2f465778459ab54322` (17:36 KST)와 일치하며 문서 추가만 있다.
원문 링크: [Git 커밋](https://github.com/ljh006008-blip/https---github.com-ljh006008-blip-Teamproject1/commit/aa67a531f4c75b8d6132c0cdb7ab8e9b4ba7791e).
아래 표는 MD 보고를 실제 소스로 보완한 결과이며, 실행 중 서버의 배포 커밋은 별도 확인해야 한다.

| 파일 | 확인한 차이 | 조치 |
|---|---|---|
| `apps/core/control_sessions.py`, `device_auth.py` | W01 8필드·201·5줄 / 후속 7줄 서명 확인 | Jetson 코드 반영 완료 |
| `apps/core/consumers.py` | 성공 ACK는 type/accepted만 반환. telemetry에도 ACK가 생성됨 | Jetson은 두 종류 ACK를 각각 수신하도록 수정. 기체/문맥/seq 필드 추가를 웹팀에 요청 |
| `apps/core/snapshot_contract.py` | 전체 지도·참조·정책 필수, map_volumes 필수 capability | 기존 축약 runtime fixture는 웹 배포본과 호환되지 않음. W02 배정 요청 전에 공통 전체 형식 검증기를 맞춰야 함 |
| `apps/core/control_sessions.py` | replay_waypoints_v1은 서버 capability 목록에 없음 | 이 기능명을 넣은 mock 파일을 운영 W02에 바로 배정 요청하지 않음 |
| `apps/core/behavior.py` | execution_result 원본·receipt를 같은 트랜잭션에 저장 | receipt 형식 구현은 소스로 확인. 일반 event는 여전히 다른 본문 |
| `apps/core/readiness.py` | 관측 시각이 보고 생성 시각보다 미래면 CLOCK_UNTRUSTED | Jetson 시각/관측 시각 검증과 공동 시험에 반영 |

**W02의 현재 한계:** 등록 후 세션·무배정 조회는 시험할 수 있다. 그러나 임무 파일 수신까지 성공하려면 전체 snapshot 형식과 기능 협상이 먼저 맞아야 한다. 현재 mock 성공을 실제 W02 배포본 수신 성공으로 표현하지 않는다. 서버 검증을 느슨하게 하거나 Jetson의 미구현 map_volumes를 지원한다고 광고해서 통과시키지 않는다.

6차 용어 검수는 `ac66f31`과 일치한다. heading/목표 yaw, 임무 실행 ID/실행기 세션 ID, 서버 저장/재고 반영을 분리한 표현을 수락하며 API 키 변경은 없다.
7차 BASELINE 검수는 `aa67a53`의 미래 관측/공백 출처 UNKNOWN 판정과 일치한다.
8차 STATUS 검수는 `0ca0eb0`의 REPLAY·스캔 연결 미완료 안내와 일치한다. Jetson 구현을 대신한 변경은 아니다.
8차의 SVC 5개 설명은 이전 전달본 기준이다. 이번 전달본은 BP/QS 33개 **보고 형식**을 추가했으며 실제 검사 구현 미완료 상태는 그대로 명시한다.
9차 W01~W18/D01~D10 대응표를 수신했다. 특히 W06 제어 요청 생성·ACK 원장·ID별 조회, preparation/scan API가 미구현이고 control_requests가 빈 배열임을 재확인했다. 공동 모의 임무 완료는 이 경로까지 구현한 뒤 판정한다.
9차의 ‘Jetson 5줄 서명만/점검 5개’ 설명은 이전 전달본 기준이다. 이번 r2에서 W01 5줄·후속 7줄과 33개 보고 구조를 반영했다. 전체 snapshot 차이와 TEST 등록은 여전히 미해결이다.
10차에서 지적한 service unit·설치 스크립트·health_monitor.py 및 배포 증거 누락도 이번 r2 필수 포함 목록에 추가했다. 실제 콜드 부팅, host_health→전체 C++ 점검 연결, 물리 포트 권한·센서 검증은 미완료다. 원격 서비스 재시작 시험으로 이를 대체하지 않는다.

## 1. 반영된 변경과 남은 차이

| 항목 | 웹팀 보고 | 이번 Jetson 대응 / 남은 작업 |
|---|---|---|
| W01 세션 | 필수 8필드, HTTP 201, 5줄 HMAC | `supported_contract_versions`, `capabilities`로 요청 수정. 응답 계약·기능 교집합·REPLAY_ONLY·비행 권한 false·문맥·시각 검증 |
| W02 파일 수신 | 현재 세션의 7줄 HMAC, 불변 원본 다운로드 | 현재 세션/계약 헤더 및 원본 길이·SHA256 검사 구현. 임의 1.1→draft.4 변환 없음 |
| BOOT 점검 | BP 28 + QS 5개, 세션·순서·만료 검증 | 33개 보고 구조와 UNKNOWN 차단 구현. 실제 하드웨어 점검은 아직 미완료이며 REPLAY 가정을 별도 표시 |
| readiness ACK | `readiness.ack`, `telemetry.ack` 응답 | 실제 소스는 type/accepted만 반환. 현재 Jetson의 문맥·순서 결합 ACK와 불일치 확인. 아래 보완 JSON 요청 |
| BT 화면 | 단계·RC 잠금·착륙 관측·종료 결과 분리 | 표시 구현과 실제 BT/센서 구현을 구분. 현재 실행기는 승인 합성 waypoint만 지원 |
| 위치 화면 | PX4 융합 / 원시 UWB / 목표 카드 분리 | 추가 요청: Jetson이 변환한 지도 UWB `uwb_map_position` 표시·지도 마커 연동. 기존 카드 구현만으로 이 필드나 기존 지도 연동 완료가 아님 |
| 실행 결과 저장 | 검수 5차에서 `stored/key/sha256/duplicate`, 트랜잭션·재전송·충돌 처리 추가 | 이전 receipt 응답 차이는 문서상 해소. 현재 Jetson `companion-phase`에는 일반 `event`가 나가므로 본문 타입 차이는 여전히 남음 |
| 나머지 outbox | preparation/명령 ACK/일반 사건/launch/scan receipt 미구현 | 현재 full 어댑터와 실제 웹의 전체 쓰기 호환은 미완료. 미지원 경로를 성공 처리하거나 로컬 원장을 지우지 않음 |
| 소스 전달 | 이전 ZIP에 서비스/빌드 파일 누락 보고 | 이번 r2 ZIP은 소스·빌드·테스트·배포·계약을 실제 포함하며 파일별 SHA256 manifest와 필수 경로를 검증 |

**주의할 실제 차이:** `execution_result`의 receipt가 구현됐어도 `event_type=EXECUTION_ENDED` 사건을 그대로 받아 준다는 뜻은 아니다. C++에서 실제 종료 관측·작업별 결과로 `execution_result`를 생성하는 연결과, 일반 사건의 별도 수신 계약이 필요하다. 예제 JSON의 성공 결과를 실측 결과로 복사하지 않는다.

## 2. 먼저 함께 완료할 범위: W01 → W02

새 `integration_stage=receive_only`는 다음 순서로만 진행한다.

```text
격리 REPLAY core → 장치 HMAC W01 POST → 서버 세션 검증
→ W02 assignment GET → snapshot 원본 GET → 길이/SHA256 검증
→ 로컬 준비 검증 → 수신 상태 기록
```

- START/PAUSE/RESUME/CANCEL/LAND_NOW를 처리하지 않는다.
- WS 보고 및 outbox POST를 하지 않는다. 로컬 준비 사건은 원장에 보존한다.
- 수신 완료는 비행 준비가 아니다. 미지원 scan/지도 snapshot은 수신 가능해도 준비 검증에서 거부될 수 있다.
- `SNAPSHOT_RECEIVED`는 바이트 수신 성공, `preparation_error`는 별도 결과다.
- 해당 설정은 수동 공동 시험용이며 현재 자동 부팅 HOST_OBSERVE 설정에는 적용하지 않았다.

파일: `config/companion.web-receive.example.json`, `config/web.receive-only.example.json`.
실행 절차: [WEB_JETSON_RUNBOOK.md](WEB_JETSON_RUNBOOK.md) §8.

## 3. 웹 담당자에게 우선 요청

### P0 — 실제 시험 기체와 장치 등록

| 필드 | 값 |
|---|---|
| 시험 기체 | `TEST-DRONE-01` |
| 인증 장치 ID | `jetson-b602b05a946e459d8938f1dd44320aa9` |
| 프로파일 | `REPLAY`, 실제 비행 권한 false |
| 계약 | `1.1-draft.4` |
| 서명 | W01 5줄 / W02·WS 7줄 |
| 현재 상태 | Jetson에서 별도 ID·키 생성, 서버 등록 미확인 |

웹 서버의 `AUTONOMY_REPLAY_DEVICES`, `DEVICE_AUTH_KEYS`에 등록하고 완료 여부·지원 주소를 회신한다.
기존 기체 5용 ID와 이번 TEST용 ID는 서로 다른 키다. 비밀은 Jetson의 `.runtime/private/replay-device.env`에 권한 0600으로만 보관한다.
키 전달은 서버 담당자와 별도 경로로 진행한다. 본 요청서·Git·ZIP에는 비밀을 포함하지 않는다.
관리자 웹 로그인이나 GitHub 로그인은 장치키 등록을 대신하지 않는다.

### P1 — 수신 인수 자료

1. 등록된 TEST 기체의 W01 201 응답과 W02 배정/무배정 응답 형식.
2. 공동 인수용 **전체** waypoint snapshot과 필수 capability 목록을 확정한다. `contracts/runtime_replay/waypoint_snapshot.json`은 축약 mock 전용이며 현재 W02에 배정할 수 없다. Jetson 전체 snapshot/지도 검증 구현과 승인 SHA 추가를 먼저 한다. 실제 지도나 라벨 데이터를 TEST 기체에 섞지 않는다.
3. 최신 커밋 SHA와 실제 8876 실행 서버의 배포 커밋 SHA. push 성공만으로 운영 서버 반영을 가정하지 않는다.
4. Git 저장소 읽기 권한. 아직 미커밋 변경이 있으면 검수 문서에 구분한다.

공동 합격 조건: 세션 생성 → 배정 참조 → 원본 파일 → 길이·SHA 일치 → 로컬 준비 결과 확인. 운영 웹에 START를 생성하는 시험은 별도 단계다.

### P2 — 보고와 결과 경로 연결

- readiness/telemetry ACK에 아래 식별 필드를 추가한다. 현재 성공 응답은 type/accepted만 있어 Jetson이 문맥 결합 수락으로 처리하지 않는다.
- `execution_result` 본문 검증과 receipt는 실제 송수신으로 검증한다. 현재 헤더는 새 세션, 재전송 본문은 원래 실행 문맥·원본 바이트를 유지한다.
- preparation report, command ACK, 일반 사건의 지원 경로와 저장 receipt를 제공한다. 지원되지 않은 outbox를 삭제하거나 전송 완료로 바꾸지 않는다.
- 스캔 결과 수신·QR 업무 판정·재고 반영은 별도 구현이다. 비행 성공 또는 서버 저장 성공을 재고 성공으로 표시하지 않는다.
- LoRa는 생존 신호만 표시한다. 변환 좌표는 Wi-Fi telemetry로 받으며 UWB 사양 전 값은 null/무효다.

ACK 보완 요청 예시(실제 수신 보고의 값을 그대로 돌려준다):

```json
{
  "type": "readiness.ack",
  "accepted": true,
  "contract_version": "1.1-draft.4",
  "drone_id": "TEST-DRONE-01",
  "context": {
    "boot_id": "received-boot-id",
    "runtime_session_id": "received-runtime-id",
    "control_session_id": "authenticated-current-session-id"
  },
  "readiness_seq": 7
}
```

telemetry는 `type=telemetry.ack`, `telemetry_seq`로 동일하게 연결한다. 거부는 accepted=false와 기계 판독 code를 반환한다. 문맥 검증 실패 시 받은 문맥을 검증된 값처럼 돌려주지 않는다.
Jetson은 매 보고마다 해당 ACK를 받은 뒤 다음 보고를 보내고, timeout/다른 종류/순서 불일치에서는 연결을 닫는다. HTTP 저장 receipt와 WS 수신 ACK는 서로 다른 증거다.

## 4. 현재 검증 범위

- 최종 Jetson CTest 7/7 통과(23.70초, 전송 단위 10개 포함). 추가 W01/W02 receive-only 시험에서 원본 일치, 유효 START 무시, WS·결과 POST 없음 확인.
- C++ 보고에 공통 생성 시각을 적용해 합성 관측 시각이 자신의 보고보다 미래가 되지 않도록 수정했다. 반복 상태 수집으로 이 경계를 검증하고 설치 바이너리를 갱신했다.
- 전송 단위 시험에서 W01 5줄/W02 7줄의 원본 바이트·쿼리·세션 결합 서명과 ACK 문맥 검증 확인.
- 실제 서버에 대한 이전 서명 GET은 401이었다. 실제 W01 세션·W02 수신 합격 기록은 아직 없다.
- PX4 없는 개발 Jetson이며 UWB 전체 사양은 수령 대기다. CTest는 실비행 증거가 아니다.
- 자동 서비스는 HOST_OBSERVE, can_start=false, flight_authority=false를 유지한다.

## 5. 최신 수신 자료

- [W01/W02 인증·세션](references/web/WEB_SERVER_W01_W02_AUTH_SESSION.md)
- [BT 검수](references/web/LOCAL_BT_SPEC_REVIEW_2026-10-04.md)
- [위치 표시 검수](references/web/LOCAL_DESIGN_REVIEW_2026-10-04.md)
- [5차 결과 저장·재전송 검수](references/web/LOCAL_DESIGN_REVIEW_ITEMS_2026-10-04.md)
- [6차 용어 검수](references/web/LOCAL_GLOSSARY_REVIEW_2026-10-04.md)
- [7차 구현 기준·관측 품질 검수](references/web/LOCAL_IMPLEMENTATION_BASELINE_REVIEW_2026-10-04.md)
- [8차 구현 범위 안내 검수](references/web/LOCAL_IMPLEMENTATION_STATUS_REVIEW_2026-10-04.md)
- [9차 연동 대응표·자료 목록](references/web/LOCAL_INTEGRATION_PLAN_REVIEW_2026-10-04.md)
- [10차 배포 증거·서비스 전달 검토](references/web/LOCAL_JETSON_DEPLOYMENT_REVIEW_2026-10-04.md)

위 자료의 자체 시험 통과 수치는 웹팀 보고다. 우리 공동 송수신 인수 통과 수치로 합산하지 않는다.
# 최신: 서버 직접 구현으로 전환

사용자가 웹 Git 직접 수정·commit·push를 승인했다. 최신 상태는
[WEB_BACKEND_INTEGRATION_2026-10-04.md](WEB_BACKEND_INTEGRATION_2026-10-04.md)와 웹 개발 브랜치
`feat/jetson-multidrone-integration`을 따른다. 아래 소스 차이표는 기존 main에 대한 수신 당시 기록이다.
feature branch에서 명령 생성/ACK 원장/receipt/문맥·seq ACK/합성 waypoint 계약을 구현했다.
Git 반영과 실행 서버 배포를 구분하며 기존 heartbeat는 API와 origin/main을 읽기 전용으로 확인한다.
