# 로컬 수정 1차 — BOOT_PREFLIGHT_SPEC.md

원본: `C:/Users/Lee/Desktop/WEB_TEAM_HANDOFF_2026-10-04/BOOT_PREFLIGHT_SPEC.md`

검수 화면: http://203.247.41.82:8876/platform/system/preflight/

## 이번에 반영한 웹 범위

- 왼쪽 **드론 관리 → 자동 점검** 메뉴. 기체 상세에도 바로가기 추가.
- 호스트 / 기체 / 임무 준비 상태를 구분. BP-C01~28, QS-C01~05 총 33개 항목.
- 항목별 통과·경고·실패·미확인, 필수·차단·해당 없음, 원인·조치·관측 시각·원천·나이/한도 표시.
- 보고가 없거나 필수 항목이 누락되면 미확인. 기존 heartbeat로 점검 통과를 생성하지 않음.
- 보고/원천 시각이 만료되면 서버와 화면 양쪽에서 준비 완료 철회. 화면은 단조 시계로 전송 경과도 포함.
- 현재 boot/runtime/control 세션에 연결된 REPLAY 보고만 수신. 다른 기동 세대, 이전 순서, 중복 체크, FLIGHT 주입 거부.
- 동일 보고 재수신으로 수신 시각을 연장하지 않음. 준비 알림은 기체 + boot/preparation/revision 기준으로 탭 세션 안에서 중복 제거.
- RC 경고 시 **현재 수동 인계 불가** 표시. scan 미적용 항목은 해당 없음으로 구분.
- 브라우저의 기체 배정 권한 적용. REPLAY 표시와 실기체 비행 권한 분리.

원문 대응: 8절의 웹 상태 배지·점검 목록·상태 전환, BP-R03/04/07/08, BP-I06의 조회 부분.

## 아직 완료로 판단하지 않는 부분

이 문서 전체 또는 BP-I06 전체가 완료된 것은 아니다. 이번 검수 단위는 **로컬 점검 조회 화면 + 인증된 REPLAY 상태 연결**이다.

- 실제 Jetson 자동 기동·33개 검사 수행·PX4/UWB/센서 연결은 장치 측 구현 및 현장 확인 필요.
- FLIGHT/SITL 세션, 실기체 비행 권한, 실제 준비 토큰/START 명령·최종 수락·중복 실행 방지는 후속 계약 작업.
- preparation report·plan·운영 확인서의 전체 수명주기는 후속 WEB 요구사항 작업.
- RC 수동 인계/공중 재기동/복구 잠금 등의 실제 기체 상태 전환 시험은 미실시.
- 현재 로컬 기체는 새 계약 점검 보고가 없어 **점검 보고 대기 / 미확인**이 정상. 모의 통과를 실제 기체에 저장하지 않았음.
- Vercel에는 이번 변경을 배포하지 않았음. 다른 원본 MD의 신규 구현은 검수 후 순차 진행.

## 연결 방법

- 장치: 기존 `WS /ws/drones/{drone_id}/`에 W01 현재 REPLAY 세션으로 연결.
- 업그레이드 요청에 W02와 동일한 7줄 HMAC 사용: `GET`, 전체 경로, 빈 본문 SHA256, timestamp, nonce, 계약 버전, control session ID.
- `X-DS-Device-ID`, `X-DS-Timestamp`, `X-DS-Nonce`, `X-DS-Signature`, `X-DS-Contract-Version`, `X-DS-Control-Session` 헤더 필수.
- 메시지는 draft.4 `readiness` 구조 / `BP-28-QS-05-draft4` 카탈로그. ACK는 `readiness.ack`의 `accepted`와 거부 사유.
- 장치 등록 범위는 기존 `AUTONOMY_REPLAY_DEVICES`, 키는 기존 `DEVICE_AUTH_KEYS`. 이번 작업에서 로컬 기체에 키·시험 세션을 추가하지 않음.
- 웹: `GET /api/drones/{id}/autonomy-state/`로 조회. 브라우저 1초 갱신, 자체 만료 250ms 검사. 보고 TTL은 원본 valid_until과 10초 중 짧은 값.
- 전송 연결 방식/조회용 응답 필드는 웹 측 초안 구현이며 장치 측 공동 수락은 남아 있음.

## 확인 결과

- 이번 기능 테스트 9개 통과: 누락/만료/중복/관측 나이/세션 폐기/다른 문맥/RC 경고/접근 권한/HMAC 경계 등.
- Django 시스템 검사, migration 모델 일치, JavaScript 구문 검사 통과.
- 로컬 SQLite에 `0024_drone_readiness_report` 적용. 기존 기체·임무 데이터를 바꾸지 않고 별도 상태 저장 테이블 추가.
- 로컬 계정으로 실제 브라우저 확인: 자동 점검 메뉴, 3단계 카드, 33개 미확인 항목 정상 표시.
- 화면 상단에 노출되던 기존 템플릿 주석의 문법을 수정.
- 사용자 검수 대기. 다음 MD로 자동 진행하지 않음.
