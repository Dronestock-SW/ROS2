# 로컬 검토 10차 — JETSON_DEPLOYMENT.md

원본: `C:/Users/Lee/Desktop/WEB_TEAM_HANDOFF_2026-10-04/JETSON_DEPLOYMENT.md`

**상태: 문서 검토·기존 웹 대응 확인 완료 / 추가 웹 수정 없음 / Jetson 원격 배포·콜드 부팅 시험 미실시.**

## MD 내용 간단 요약

- Jetson 호스트 점검 서비스의 자동 기동, 장애 후 재시작, 상태·로그 조회 및 되돌리기 절차를 정리한다.
- 서비스가 active여도 비행 준비 완료는 아니다. 센서·권한·최신 기동 세대·실제 비행 점검을 별도로 확인해야 한다.
- 당시 호스트 서비스 적용·재시작은 확인됐지만, 실제 전원 재투입과 전체 장치 연결·비행 준비 검증은 남아 있다.

## 이번 처리와 확인할 화면

원문 6절의 웹 요구는 READY·차단 원인·갱신 나이·RC 인계 상태 표시다. 해당 웹 부분은 기존 BOOT_PREFLIGHT/BT 구현에 반영돼 있어 화면을 추가하지 않았다.

- [자동 점검](http://203.247.41.82:8876/platform/drones/5/preflight/): 호스트 / 기체 / 임무 준비, 항목별 원인·조치·관측 나이, RC 인계 가용 여부.
- [비행 행동 상태](http://203.247.41.82:8876/platform/drones/5/behavior/): 장치가 보고한 제어권·수동 인계 잠금·착륙 관측.
- 이번 검수 산출물은 이 보고서다. 앱 코드·DB·서비스 설정은 변경하지 않았다.

## 웹 요구 대응

| 원문 요구 | 기존 웹 처리 | 적용 범위 |
|---|---|---|
| systemd active/READY를 비행 READY로 변환하지 않음 | 자동 점검에서 호스트·기체·임무 준비 분리. “프로세스 실행만으로 비행 준비가 완료되지는 않습니다” 안내 | heartbeat나 서비스 실행만으로 점검 통과를 생성하지 않음 |
| 미연결·미구현·미수신은 UNKNOWN | 점검 보고·필수 항목·관측 출처/시각이 없거나 만료되면 미확인·차단 표시 | 현재 인증된 REPLAY 점검 보고 수신 범위 |
| 이전 기동 상태·오래된 준비 폐기 | 현재 boot/runtime/control 문맥 확인, 보고와 원천 관측 만료 처리 | Jetson 로컬 host_health.json을 웹에서 직접 읽는 기능은 없음 |
| 세션 구분 | 웹의 실행기 세션은 runtime_session_id | 호스트 monitor_session_id를 실행기 세션으로 대체하지 않음. 호스트 파일→웹 보고 변환은 별도 연동 대상 |
| 상태 나이와 원인 표시 | 항목별 원천·관측 시각·나이/한도·오류 코드·원인·조치 표시 | 호스트의 2초 주기·10초 TTL을 PX4 센서나 명령 유효시간으로 사용하지 않음 |
| RC 상태 | 자동 점검의 RC 가용 여부와 행동 화면의 수동 인계 잠금 분리 | 실제 RC 하드웨어 인계 시험은 미실시 |
| 준비 알림·철회 | 상태 전환 안내, 탭 세션 내 중복 억제, 만료/연결 끊김 시 준비 표시 해제 | 영속 사건 이력·전체 공동 검수는 남음 |
| 진단으로 비행 권한 발급 금지 | 상태 API의 flight_authority=false, can_start=false 및 시작 버튼 비활성 | 실제 START·실기체 비행 승인은 이번 작업 범위 밖 |

근거: [BOOT_PREFLIGHT 검수](LOCAL_BOOT_PREFLIGHT_REVIEW_2026-10-04.md), [BT 검수](LOCAL_BT_SPEC_REVIEW_2026-10-04.md), [관측 품질 보완](LOCAL_IMPLEMENTATION_BASELINE_REVIEW_2026-10-04.md), [연동 대응표](LOCAL_INTEGRATION_PLAN_REVIEW_2026-10-04.md).

코드 대조: [readiness.py](../dashboard/DroneStock-main/apps/core/readiness.py), [preflight.js](../dashboard/DroneStock-main/static/js/preflight.js), [자동 점검 템플릿](../dashboard/DroneStock-main/templates/system/preflight.html).

## 배포 기록과 현재 확인 범위

원본은 2026-10-02 호스트 모니터 배포 및 2026-10-04 환경 확인 기록이다. 기록된 장치·권한·서비스 상태를 현재 원격 장치의 실시간 상태로 표시하지 않았다.

| 자료 | 확인 내용 | 해석 |
|---|---|---|
| 원본 JETSON_DEPLOYMENT.md | 호스트 모니터 enabled/active, Linger=yes, 장애 재시작 확인. 장치 별칭·dialout 권한·콜드 부팅은 미완료로 기록 | 전달자의 당시 기록. 이번에 SSH로 확인한 결과가 아님 |
| 원본 reports/host_monitor_deployment.json | 현재 전달 폴더의 해당 경로에 파일 없음 | 원본 문서에 적힌 배포 증거를 이 파일로 직접 대조하지 못함 |
| 추가 패키지 WEB_JETSON_RUNBOOK.md | host/core/web 서비스 구조와 HOST_OBSERVE 조회, 격리 REPLAY 절차 | 기존 호스트 모니터 단독 배포보다 확장된 운영 자료로 함께 검토 |
| 추가 패키지 reports/WEB_JETSON_BOOT_CHECK_2026-10-04.json | supervised_restart=PASS, new_runtime_session=true, target_enabled=enabled, services_active 3개가 active, Linger=yes | 제공된 JSON 파일의 기록 확인 |
| 같은 boot check | core_profile=HOST_OBSERVE, web_state=CONNECTED_READ_ONLY, observed_contract=1.1, physical_output=false | 기록 시점의 조회 상태. 현재 draft.4 쓰기·WS 연동 완료 증거가 아님 |
| 같은 boot check | auto_start_observed=false, cold_boot_test=NOT_RUN | 실제 전원 재투입·무로그인 최초 기동 시험 완료로 처리하지 않음 |

추가 패키지의 `deployment/`에는 env.sh·run_env.sh·requirements 2개, `ops/`에는 Python 환경 설치·진단 도구가 들어 있다. 문서가 참조하는 서비스 unit·설치 스크립트·health_monitor.py가 해당 디렉터리에 없어, 이 전달 폴더만으로 서비스 설치를 재현할 수 있다고 안내하지 않는다.

## Jetson 측에 남은 확인

- 계획된 전원 재투입 때 로그인 이전 자동 기동, 새 boot/monitor/runtime 문맥, 과거 준비 상태 폐기 및 비행 출력 없음 확인.
- 실제 장치 연결 뒤 포트 식별·프로세스 권한·센서 원천 데이터 확인. 원본에 기록된 dialout 미포함을 현재 상태로 단정하지 않음.
- 호스트 진단을 전체 C++ 비행 점검 및 웹 계약에 연결. 장치 파일 존재나 호스트 PASS만으로 비행 READY를 만들지 않음.
- 장치 인증·서명·snapshot·준비 계약 차이는 INTEGRATION_PLAN 보고서의 선행 조건에 따라 공동 검수.
- 원본 되돌리기는 호스트 모니터 한 개에 대한 절차다. 이후 core/web 서비스까지 있는 구성의 전체 복구 절차로 사용하지 않으며, 사용자 서비스 전체에 영향을 주는 Linger 변경도 실제 운영 구성을 확인해야 한다.
- 스캐너 LED·음향·모터 알림은 이번 웹 요청의 추가 구현 대상이 아니다. 물리 표시가 필요하면 장치 지원 여부와 준비 해제 연동을 별도로 확인한다.

## 이번 확인 및 전달

- 원문 전체, 추가 운영 절차와 boot check JSON, 기존 웹의 준비 표시·차단·시각/세션 처리 코드를 대조했다.
- 새 코드가 없어 추가 테스트·서버 재시작·화면 캡처는 하지 않았다. 위 링크는 기존 검수 화면이다.
- SSH 접속, 서비스 설치/중단, 그룹·Linger·udev 변경, 모의 START, 실제 기체 출력, Vercel 배포를 수행하지 않았다.
- 사용자 지시에 따라 이 MD 검수 보고서를 커밋·푸시한다.

다음 문서는 **JETSON_ENV.md**. 사용자 검수 후 진행한다.
