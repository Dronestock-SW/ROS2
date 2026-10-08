# Jetson 소스 전달본 r2

이전 전달본의 서비스·빌드 파일 누락 보고를 반영해 **실제 파일을 포함하고 압축 해시를 검증**했다.

1. 최신 대응은 `WEB_FOLLOWUP_2026-10-04.md`를 먼저 읽는다. 웹 검수 1~10차 및 Git 9a3ce22까지 대조했다.
2. 서비스 실행은 `WEB_JETSON_RUNBOOK.md`, 구조는 `SERVICE_ARCHITECTURE.md`를 따른다.
3. 포함: CMakeLists.txt, C++ src/include, Python adapter/transport/mock, tests, trees, deployment, config, contracts, 운영 도구와 문서.
4. 제외: 실행 중 원장·원본 임무, 비밀키, `.runtime`, `.venv`, `.build`, 웹팀 Git checkout, 빌드 바이너리.
5. `HANDOFF_MANIFEST.json`에 압축본의 실제 파일 경로·바이트 길이·SHA256이 있다.

`python/sangwon_web/adapter.py`, `src/service/engine.cpp`, `CMakeLists.txt`는 필수 포함 검사를 통과했다.
Linux/Jetson에서 의존성을 준비해 빌드해야 한다. Windows에서 물리 비행 실행용 프로그램이 아니다.
`ops/build_source_handoff.py`로 같은 선택 규칙의 전달본을 다시 만들 수 있다.

현재 가능: HOST_OBSERVE 자동 기동, 합성 waypoint REPLAY, 로컬 모의 HTTP/WS 연동, W01/W02 수신 전용 단계.
현재 미완료: 실제 서버 TEST 등록, 전체 snapshot/map_volumes 준비 검증, 웹 문맥 포함 ACK, 전체 명령/결과 경로, PX4/UWB/스캔 비행 연결.
웹팀의 execution_result 저장 receipt 구현과 Jetson 일반 event 본문 차이는 후속 요청서에서 구분한다.
서버의 축약 fixture 예외 허용, 미구현 capability 광고, 실제 기체 5에 합성 보고 주입으로 연동을 통과시키지 않는다.

이 압축본을 받은 사실은 실제 서버 배포·등록·인수·비행 승인 완료를 의미하지 않는다.
