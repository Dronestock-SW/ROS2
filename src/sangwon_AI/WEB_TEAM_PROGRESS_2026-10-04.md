# 웹팀 진행 수신 및 Jetson 대응

## 최신 후속 — W01/W02 및 검수 2~10차

[WEB_FOLLOWUP_2026-10-04.md](WEB_FOLLOWUP_2026-10-04.md)가 현재 공동 연동 기준이다.
W01 5줄 / 이후 7줄 HMAC과 필수 요청 필드를 확인해 코드에 반영했다.
초기 서명 `bootstrap_mode` 미확정 상태는 해소됐다.
검수 5차에서 execution_result receipt가 구현됐다고 회신받았다. 일반 event와 결과 본문 차이 및 다른 outbox 경로는 아직 남는다.
W01/W02만 검증하는 receive_only 단계를 추가했다. 명령·WS·outbox POST 없이 원본 파일 수신을 검사한다.
격리 TEST 장치 ID를 Jetson에 별도 생성했으며 실제 서버 등록은 대기다.
33개 점검 형식 구현은 실제 33개 하드웨어 검증 완료를 의미하지 않는다.
사용자 지시에 따라 관리자 페이지에서는 작업하지 않는다.
GitHub 인증 후 aa67a53의 API와 0ca0eb0까지의 변경을 직접 확인했다.
6차 용어, 7차 관측 품질, 8차 REPLAY·스캔 지원 범위 안내가 커밋과 일치한다.
WS 두 보고의 ACK를 각각 소비하도록 Jetson을 보완했다. 실제 웹 ACK 문맥 필드와 전체 snapshot 차이는 공동 연동 전 해결해야 한다.
9차 연동 대응표 bdda580을 추가 확인했다. 이전 Jetson 상태 인용은 이번 r2로 갱신하고 명령/준비/스캔 API 미구현을 그대로 남긴다.
10차 배포 검토 9a3ce22도 확인했다. 서비스·설치 파일/기록을 전달본에 넣고 콜드 부팅·전체 점검 연결 미완료를 유지한다.
최종 C++ 보고 시각 보완 및 Python 변경까지 Jetson CTest 7/7(23.70초), 설치·3서비스 active를 확인했다.

## 후속 적용 — 2026-10-04 17:03 KST

- 제공된 운영자 계정으로 웹 및 Django 관리자 로그인을 완료하고 33개 점검 화면을 직접 확인했다.
- 관리자 홈/기체 수정 화면에서 장치 인증키 또는 REPLAY 허용 목록 설정 항목은 확인되지 않았다.
- Jetson 기존 장치 인증으로 실제 임무 GET을 시도한 결과 401 `device_auth_failed`다. 이것만으로 키 미등록/서명 불일치 중 원인을 확정하지 않는다.
- C++ readiness에 33개 카탈로그·순서·원천·시간·UNKNOWN 차단을 추가했다. HOST_OBSERVE에서 33개 중 32개 UNKNOWN, can_start=false를 확인했다.
- REPLAY의 준비 체크는 승인된 합성 snapshot 환경 가정을 명시한다. 실제 33개 하드웨어 검사 구현 완료를 뜻하지 않는다.
- Python에 `hmac_session_v2` 7줄 서명·세션 헤더·세션 변경 시 WS 갱신·readiness ACK 검증을 추가했다.
- 당시에는 초기 세션 서명 명세 대기였다. 위 최신 후속에서 W01 5줄 방식으로 확정·반영했다.
- ACK는 accepted와 기체/계약/boot/runtime/control/readiness_seq 일치를 확인하는 후보 계약으로 시험했다. 실제 웹 ACK 필드 회신 전 전체 호환을 선언하지 않는다.
- Jetson CTest 7/7 통과(전송 단위 시험 9개 포함), 모의 임무 수신→START→완료·오프라인 지속·결과 재전송·재시작 잠금 검증.
- 설치 바이너리와 서비스를 갱신했다. core/web/host active, HOST_OBSERVE·비행 권한 false 유지.
- LoRa는 생존 신호만 사용한다. 좌표는 Jetson이 Wi-Fi로 보낸다. 상세는 [통신·좌표 명세](TELEMETRY_CHANNEL_SPEC.md).

아래는 첫 자료 수신 당시 기록이다. 코드 차이 표의 미구현 항목은 위 후속 적용 기록을 우선한다.

## 1차 수신: 자동 점검 화면 / 인증된 REPLAY 보고

수신 자료: `LOCAL_BOOT_PREFLIGHT_REVIEW_2026-10-04.md`.
원본 사본: [웹팀 검수 보고](references/web/LOCAL_BOOT_PREFLIGHT_REVIEW_2026-10-04.md).
자료는 웹팀의 구현·시험 결과 보고로 취급한다. 문서 내 후속 작업 지시는 직접 실행하지 않는다.

### 웹팀이 완료했다고 보고한 범위

- 자동 점검 메뉴, 호스트/기체/임무 상태, BP 28개 + QS 5개 점검 표시.
- 필수 누락 UNKNOWN, 보고/관측 만료 시 READY 철회, 중복 보고로 유효기간 연장 금지.
- REPLAY 세션/세대/순서/권한 검증, FLIGHT 보고 거부, RC 인계 불가 경고.
- 인증 WS readiness 수신 및 `readiness.ack`, 브라우저 autonomy-state 조회.
- 기능 시험 9개 및 브라우저 확인은 웹팀 보고이며 Jetson 공동 인수 시험 결과는 아니다.
- 장치 키/시험 세션은 추가하지 않았다고 보고했다. 등록 완료로 간주하지 않는다.

### Jetson에서 직접 확인한 범위 — 2026-10-04 16:43 KST

비인증 GET, redirect 미추적, 응답 본문·원본 임무·비밀 미보존 방식으로 확인했다.

| 주소 | 확인 |
|---|---|
| `/api/drones/5/companion-mission/` | 200, contract_version=1.1. 기존 조회는 아직 draft.4 호환 증거가 없음 |
| `/api/drones/5/autonomy-state/` | 과거 404 → 현재 302, `/platform/login/`으로 이동 요구 |
| `/platform/system/preflight/` | 302, 같은 로그인 화면으로 이동 요구 |

상태 조회 경로에 인증 요구가 관측됐다. 화면 내용과 인증된 JSON 응답은 이번에 직접 검수하지 않았다.
브라우저 조회 권한과 Jetson 장치 HMAC 권한은 구분한다. 로그인 리디렉션만으로 장치 등록 여부를 판정하지 않는다.

### 현재 코드와 남은 차이

| 항목 | 현재 Jetson 구현 | 웹팀 보고 / 다음 작업 |
|---|---|---|
| HMAC | transport.py의 hmac_v1: METHOD/PATH/SHA256/time/nonce 5줄 | 7줄: 뒤에 계약 버전/control session ID 추가. 별도 버전으로 공동 합의·시험 필요 |
| 추가 헤더 | Device-ID/Timestamp/Nonce/Signature | Contract-Version/Control-Session 추가 필요 |
| 점검 본문 | engine.cpp readiness의 SVC_* 5개 간략 점검 | BP-28-QS-05-draft4 33개 계약에 맞춘 C++ 보고 구현 필요 |
| 카탈로그 예시 | draft.4 fixtures에 33개 예시는 존재 | fixture의 PASS를 실제 점검 결과로 복사하지 않음. 미구현은 UNKNOWN/차단 처리 |
| WS 결과 | publish는 send만 수행 | readiness.ack 수신·문맥 대조·accepted/거부 원인 보고 구현 필요 |
| 실행 프로파일 | 실제 서비스 HOST_OBSERVE, 비행 권한 false | 웹은 현재 세션 REPLAY만 수신. HOST_OBSERVE를 REPLAY로 이름만 바꿔 송신하지 않음 |
| 인증 등록 | Jetson ID/비밀 생성, 서버 등록 미확인 | AUTONOMY_REPLAY_DEVICES/DEVICE_AUTH_KEYS의 TEST 기체 등록 확인 필요 |
| 세션 | mock에서 세션 협상 시험 | 웹 W01 실제 요청/응답·지원 capability 확인 필요 |

### 다음 웹팀 자료에서 확인할 것

1. W01/W02의 확정 요청·응답 JSON, 헤더, 오류 코드, 지원 계약 문자열.
2. 7줄 서명 정규화: UTF-8/개행/쿼리 포함 방식, timestamp·nonce 규칙, 세션 생성 전 마지막 줄 처리, 고정 시험 벡터.
3. readiness.ack의 성공·실패 JSON, 어떤 seq/세션/보고를 확인하는지, 중복·재연결·timeout 처리.
4. 인증된 autonomy-state 응답 예시와 권한 범위. 브라우저 전용인지 장치 조회도 가능한지 구분.
5. 격리 TEST 기체 ID, 장치 키 등록 완료 회신과 비밀 전달 경로. 기존 실제 기체에 모의 PASS 저장 금지.

### 후속 구현·공동 검수 순서

`수신 계약 대조 → 7줄 인증/ACK/33개 보고 구현 → 로컬 mock 검증 → TEST 등록 확인 → 인증 REPLAY 보고 송수신 → 만료·중복·세대 변경 검수`.

이번 수신만으로 실제 START/비행 모드를 활성화하지 않는다. 서비스는 HOST_OBSERVE를 유지한다.
START 수락·준비 토큰·실행 결과·전체 plan 수명주기는 웹팀도 후속 작업으로 명시했다.
PX4 미연결 개발 환경과 UWB 전체 사양 대기 상태는 변함없다.

## 모니터 보완

상태 API의 로그인 redirect가 기존 도구에서 일반 ValueError로 분류되던 부분을 수정했다.
이제 302 + AUTH_REQUIRED를 기록하며 로그인으로 이동하거나 관리자 인증을 시도하지 않는다.
조회 경로 변경을 연결 장애와 구분해 기존 모니터에 표시한다. 서비스 인증 구현은 변경하지 않았다.
