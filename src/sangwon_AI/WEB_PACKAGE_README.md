# 웹 담당자 전달 묶음
이 문서는 전달 파일의 읽는 순서와 범위를 안내한다.
UI·UX·서버 개편과 Jetson 병행 개발을 시작할 때 읽는다.

작성일: 2026-10-04 / 요청서 r2 / 외부 계약 후보 1.1-draft.4.

## 먼저 읽을 파일

1. [웹 전체 개편 요청서](WEB_REDESIGN_REQUEST_2026-10-04.md): Jetson 요구·전체 흐름·기존 UI 변경 14개·API·담당·인수 조건.
2. [실제 웹 조회 결과](WEB_UI_AUDIT_2026-10-04.md): 현재 화면에서 확인한 사실과 미검증 범위.
3. [같은 버전 JSON 자료](contracts/web_v1_1_draft4/README.md): 요청한 9종, 전체 snapshot 및 결과 등 23개 JSON 자원과 manifest.
4. [검수 기록](reports/WEB_HANDOFF_REVIEW_2026-10-04.md): 이번 검사 결과와 외부 회신/실측 대기 항목.

웹팀이 요청한 WEB_REQUIREMENTS, WEB_JSON_DRAFT, WEB_CONTRACT_DRAFT,
DESIGN, IMPLEMENTATION_STATUS, VALIDATION_PLAN도 포함했다.
이전 문서의 draft.2와 r1 draft.3 예제는 설계 근거다.
새 구현에는 전체 요청서와 draft.4 후보만 대조한다.
실제 URL/필드/기존 코드 대응은 웹팀과 함께 수락해야 한다.

## 회신 요청

전체 요청서 §13의 R01~R09를 작성해 달라.
UI-D01~14 및 각 operation_id에 현재 코드 위치와 변경안을 연결한다.
G0 계약 수락 후 UI·서버·Jetson mock 개발을 병행한다.
기체 실측과 UWB 전체 사양 수령은 별도 실장 단계다.

## 묶음의 범위

- 문서, 합성 JSON, 오프라인 자료 검사 스크립트만 포함한다.
- 상세 문서의 구현 소스·환경 파일·과거 실행 로그 링크는 저장소 참조다. 이 묶음에 모두 포함하지 않는다.
- 로그인 비밀과 실제 인증 토큰은 포함하지 않는다.
- 저장/시작/착륙 등 실제 서버 변경을 수행한 결과물이 아니다.
- JSON 검사 통과는 물리 비행 승인이나 E 인수 시험 완료를 뜻하지 않는다.
