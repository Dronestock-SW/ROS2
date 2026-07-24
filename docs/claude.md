# Dronestock — AI 에이전트 작업 규칙

이 워크스페이스에서 작업하는 모든 AI 에이전트(Claude Code, Codex 등)는 아래를 따른다.

## 작업 시작 전 필수 참조 (우선순위 순)
1. docs/roadmap.md — 기술 방향의 유일한 기준. 구 8주 WBS는 초안으로 격하됨. 충돌 시 roadmap이 이긴다
2. docs/altitude_policy.md — z(고도) 처리 원칙. companion에서 z 제어 코드 작성 절대 금지
3. docs/glossary.md — 용어 정의. 문서/주석 작성 시 이 용어 체계를 따른다
4. docs/equipment_inventory.md — 보유 하드웨어와 정확한 모델명 (예: TFmini가 아니라 TFmini Plus)

## 개발 규칙
- 빌드: colcon build --symlink-install (workspace 루트에서), 빌드 후 source install/setup.bash
- build/, install/, log/ 디렉토리는 자동 생성물 — 절대 수정 금지
- 새 코드는 src/ 아래 패키지에만 작성
- Python 한글 출력 시 UTF-8 인코딩 명시 (파일 I/O, print, matplotlib 전부)
- 커밋 메시지: "Phase/작업ID: 내용" 형식
- 자세제어 관련 코드 작성 금지 — 자세는 PX4 전담 (roadmap 핵심 결정 4)
- 웹 관련 개발 보류 중 — API 경계 topic(roadmap 참조)만 유지

## 새 용어 등장 시
코드나 문서에 glossary에 없는 기술 용어를 도입하면, 같은 PR에서 docs/glossary.md에 항목을 추가한다.