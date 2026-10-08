# 로컬 수정 6차 — GLOSSARY.md

원본: `C:/Users/Lee/Desktop/WEB_TEAM_HANDOFF_2026-10-04/GLOSSARY.md`

**상태: 용어집 대조 및 기존 웹 화면의 용어 정리 완료.**

## 변경 위치

1. 비행 행동 상태 → 위치·관측·목표 / 임무 실행 ID / 실행 종료 결과
   - http://203.247.41.82:8876/platform/drones/5/behavior/#position-observations
2. 자동 점검 → 하단 현재 준비 근거·세션 정보 펼치기
   - http://203.247.41.82:8876/platform/drones/5/preflight/

## 적용한 용어

| 원문 용어 | 화면 반영 |
|---|---|
| 목표 yaw / 현재 heading | Jetson 목표는 **목표 방향(yaw)**, PX4 추정값은 **현재 방향(heading)**, UWB 원시값은 **관측 방향(yaw)**으로 구분. 도달할 방향과 추정한 현재 방향이라는 설명 추가 |
| 임무 실행 ID / 비행 ID | 현재 실행 제목을 **임무 실행 ID**로 명확히 하고, 재접속·재시도에도 같은 실행 번호를 유지한다는 의미와 실제 이륙~착륙 기록을 연결하는 비행 ID의 의미 설명 |
| 제어 세션 / 준비 ID | 자동 점검 하단에 새 웹 요청의 연결 문맥과 검증한 임무·상태 식별자라는 설명 추가 |
| 불변 snapshot / 임무 묶음 | **불변 배포본(snapshot)**으로 표기하고 임무·지도·승인 자료를 고정한 묶음이라는 설명 추가 |
| 임무 배정 | 자동 점검 문맥의 배정 ID를 **임무 배정 ID**로 표기 |
| 실행 프로파일 | 두 화면의 프로파일 배지를 **실행 프로파일**로 통일. REPLAY 모의 실행 전용 의미 유지 |
| ULog | **PX4 비행 기록(ULog) 수집**으로 표시 |
| 수락 ACK | 요청 처리 결과이며 실제 이륙·복귀·착륙 완료와 구분한다는 설명 반영 |

실행기 프로세스 세션인 `runtime_session_id`는 **실행기 세션 ID**, 임무 1회 실행인 `execution_id`는 **임무 실행 ID**로 구분한다.

좌표 값·단위·좌표계·API 키를 바꾸지 않았다. 현재 heading도 보고된 좌표계를 기준으로 읽으며, 원시 yaw를 북쪽 기준 heading으로 변환했다고 표시하지 않는다.

## 나머지 용어의 처리

행동 트리·공유 상태·선점·목표 lease·행동 세대·flight_guard·uXRCE-DDS·속도 프로파일·linger·venv 등은 설계/장치 구현을 설명하는 용어다. 별도 웹 기능이나 새 용어집 메뉴를 요구하는 문서가 아니므로 기존 페이지에서 필요한 설명만 반영했다.

특히 flight_guard의 독립 프로세스는 설계 정의다. 새 Jetson SERVICE_ARCHITECTURE의 현재 REPLAY 구현은 Runtime/Guard가 같은 프로세스이므로, 용어집 정의를 실제 구현 완료로 표시하지 않았다.

## 확인

- 변경 JS 2개의 문법 검사 통과.
- 로컬 서버를 재시작하고 로그인된 두 화면에서 문구 확인.
- 문구·설명만 바뀌어 별도 테스트 추가나 전체 테스트 재실행은 하지 않음.
- API 수신·인증·DB·비행 제어·재고 처리는 이번 변경 대상이 아님.
- 사용자 지시에 따라 이 문서 단위로 커밋·푸시한 뒤 검수 보고.

수정 파일: `templates/drones/behavior.html`, `templates/system/preflight.html`, `static/js/behavior.js`, `static/js/preflight.js` (모두 `dashboard/DroneStock-main/` 기준).

다음 문서는 **IMPLEMENTATION_BASELINE.md**. 사용자 검수 후 진행한다.

![비행 행동 상태 용어](glossary-local-2026-10-04.jpg)

![자동 점검 세션 용어](glossary-preflight-local-2026-10-04.jpg)
