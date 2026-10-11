# 웹 서버 직접 구현 · 개발 Jetson 공동 연동

기준일: 2026-10-04. 사용자가 웹 소스를 직접 수정·커밋·push하고 화면 담당자는 UI/UX를 진행하도록 승인했다.

## 소스 위치·소유

- Jetson 정본: `src/sangwon_AI`. C++가 임무·제어권·준비·원장 권한을 갖고 Python은 전송·도구를 담당한다.
- 웹 개발 clone: `.runtime/web-development`, 브랜치 `feat/jetson-multidrone-integration`.
- 저장소: `ljh006008-blip/https---github.com-ljh006008-blip-Teamproject1`.
- `.runtime/web-source`는 이전 checkout과 origin/main을 확인하는 읽기 전용 참조다. 개발 clone과 구분한다.
- 웹 repo의 `companion/sangwon_AI`는 정본의 소스 전달본이다. `ops/export_web_repository.py`로 명시된 파일만 내보낸다. runtime·build·venv·개인 키·외부 clone은 포함하지 않는다.
- 서버 구현/Jetson 계약은 이 작업에서 담당한다. 웹 화면 담당자는 repo의 `docs/UI_UX_COMPANION_HANDOFF.md`로 연결한다.

## 실제 완료한 흐름

W01 HMAC 세션 → 불변 합성 snapshot 게시·배정 → W02 파일 길이·SHA 확인 → C++ preparation → WS readiness 문맥·seq ACK → 새로운 웹 START → C++ 모의 이동·복귀·착륙 → command_result / execution_result → 서버 동일 바이트 receipt.

두 TEST 기체에서 서로 다른 C++ runtime·SQLite 원장·socket·장치 세션을 사용했다. 각 임무는 필수 waypoint 2개를 방문했다. 한 기체의 Python adapter를 임무 중 중단한 뒤 C++가 모의 임무를 완료하고, 새 W01 세션으로 재연결하여 원래 문맥의 outbox를 저장하는 것을 확인했다.

- 서버는 접수와 실행기 수락을 구분하며 신규 수락 TTL은 10초다.
- 미전달 PENDING은 새 세션에서 SUPERSEDED이고 자동 재발행하지 않는다.
- 기체별 scope, 원래 세션 문맥, 동일 의도 중복, 원본 receipt 충돌, 역할, stale 상태를 검증한다.
- 닫힌 배정을 이력으로 보존하고 다음 임무 배정을 분리한다.
- Fleet 조회는 기체 ID별·페이지별이며 LoRa 생존과 Jetson Wi-Fi 위치를 분리한다.
- 전체 map 형식은 유지하고 합성 `replay_waypoints_v1` 형식은 명시적으로 분기한다. C++의 승인 SHA 허용 목록을 완화하지 않았다.
- REPLAY 보고 시계 오차 예산은 최대 250ms다. 점검/보고 유효기간에서 차감한다. 관측 시각이 자체 보고 생성보다 미래인 경우는 여전히 UNKNOWN이다.

## 검증 경계

공동 시험은 실제 Windows Django/Daphne와 SSH로 연결한 실제 개발 Jetson의 C++·Python 프로세스로 수행했다. 서버는 `127.0.0.1`에만 열고 SSH 역방향 tunnel을 사용했다. 테스트 DB·사용자·HMAC 키·두 TEST 기체는 격리했다.

**모의 PX4이며 물리 비행 출력은 없다.** UWB 좌표를 생성하거나 실제 서버 8876을 변경하지 않았다. `HOST_OBSERVE`, `can_start=false`, `flight_authority=false` 부팅 설정은 유지한다.

현재 waypoint 서비스는 scan·map 계획·실제 FC 출력을 구현 완료하지 않았다. 전체 설계의 QR/ArUco/우회 기능이 실행 가능해졌다고 표시하지 않는다. UWB 전체 사양 수신 후 어댑터를 선정한다.

## 재현

웹 repo backend의 `config.companion_test_settings`는 격리 SQLite/메모리 Channels 시험 전용이다. 운영 서버 settings로 사용하지 않는다.

```text
python dashboard/DroneStock-main/manage.py test apps.core --settings=config.companion_test_settings
python scripts/test_jetson_companion_e2e.py --ssh-host <Jetson-host> --remote-package <package-root> --report .runtime/jetson_companion_e2e.json
```

공동 시험은 Jetson의 `.venv/bin/python`, `.build/colcon-build/sangwon_ai_replay/sangwon_companiond`와 `tests/django_companion_peer.py`가 필요하다. 소스 설치·빌드는 BUILD_REPLAY.md/JETSON_DEPLOYMENT.md를 따른다. 알려진 fixture 키는 격리 시험에서만 사용한다.

최종 테스트 결과·Git revision은 웹 repo의 `docs/JETSON_COMPANION_INTEGRATION.md`와 `docs/COMPANION_ACCEPTANCE_2026-10-04.json`에 기록한다.

## 다음 적용

1. feature branch 리뷰·머지 후 실행 서버에 배포한다. push를 배포 완료로 간주하지 않는다.
2. 서버 0027 migration, 비공개 장치키/TEST 등록, 운영자 기체 배정, `provision_replay_drone` CLI를 적용한다. 관리자 페이지는 사용하지 않는다.
3. 화면 담당자가 새 게시/명령/조회 API와 REPLAY 표시를 연결한다.
4. 실제 서버와 등록된 Jetson의 공동 수신·ACK·receipt를 확인한다. HOST 부팅에서 자동 START하지 않는다.
5. UWB 전체 사양·PX4 연결·파라미터 프로필·하드웨어 점검·스캔/지도 실행 기능·실측/시뮬레이션 승인 후 실기체 단계로 진행한다.
