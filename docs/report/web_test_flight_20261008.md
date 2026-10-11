# 웹 전체 비행 순서의 클라우드 검증

2026-10-08 구현·확인 기록이다. 현장 시험 전에 완료 범위를 확인할 때 읽는다.

웹 이륙·경유지 이동·착륙 순서를 구현했다.
RAW UWB·실측 높이·FC 관측 전달을 연결했다.
클라우드 ROS 통합시험 네 조건을 통과했다.
실물 Jetson·PX4 융합·비행은 미실시다.

## 구현 범위

요청 좌표와 센서 관측을 분리했다.
PX4 EKF2와 기존 고도 제어를 유지했다.

| 변경 | 결과 |
|---|---|
| `drone_mission` | 웹 v1 검사·순서 FSM·MAVROS 명령 연결 |
| 로컬 웹 | start·복귀·착륙, HTTP 미션·ACK·단계, WS 텔레메트리 |
| B_TF 높이 | 승인한 XY와 같은 시각의 태그 XYZ 표시 |
| FC 브리지 | `/uwb/btf_pose` 선택, EV_CTRL·공분산·지연·장착값 검사 |
| 실물 launch | 센서·브리지·미션·플랫폼, 단일 MAVROS |
| 재요청 | 명령 전 디스크 기록, 반복 요청·재시작 재이륙 방지 |
| 중단 | UWB·높이·FC·웹·목표 상실 때 착륙 요청 |
| 수동 조종 | 자동 명령 중지. 늦은 응답으로 재개하지 않음 |
| 공중 재시작 | 자동 모드이면 착륙 요청. 경로 복원 미실시 |
| arm 취소 | 후속 takeoff 차단. disarm 수락·새 상태로 확인 |
| 경유지 반복 | 같은 XY에서도 도착 대기 시간을 새로 시작 |

기본 execute와 bridge_enabled는 false다.
물리 확인 항목도 false로 보존했다.
FC 파라미터를 자동 변경하지 않는다.
운영 웹의 start 지원·QR 스캔·LoRa는 별도 작업이다.

## 빌드와 테스트

Ubuntu 22.04 / ROS2 Humble / Python 3.10에서 검사했다.
클라우드 실행은 DOMAIN_ID 99·localhost 제한이다.
저장소 6개 패키지 빌드가 성공했다.
실물 launch의 인수 조회도 성공했다.

| 패키지 | 통과 | 실패 | skip |
|---|---:|---:|---:|
| drone_uwb | 302 | 0 | 0 |
| drone_demo | 116 | 0 | 0 |
| drone_bringup | 55 | 2 | 1 |
| drone_platform_link | 11 | 0 | 0 |
| drone_mission | 58 | 0 | 0 |
| 합계 | 542 | 2 | 1 |

현재 pytest XML은 총 545개·오류 0개다.
기존 실패는 bringup의 flake8·pep257다.
flake8은 E128 3건과 E501 1건이다.
pep257은 D213 16건이다.
이번 기능과 무관한 원본은 수정하지 않았다.
colcon test-result의 종료 코드는 1이다.
실패를 통과로 기록하지 않았다.

미션 검사는 정상 외에도 다음을 포함한다.

| 조건 | 확인 |
|---|---|
| 잘못된 좌표·배치·revision·중복 ID | arm 미실행 |
| 오래된 요청·소비한 요청·재시작 | 재이륙 미실행 |
| FC·센서·높이·브리지·정렬 누락 | 시작 차단·비행 중 착륙 요청 |
| 목표 송신 성공만 있음 | 도착·성공 미확정 |
| 적용 목표 변경·피드백 만료 | 착륙 요청 |
| 명령 거부·무응답·착륙 미확인 | 성공 미확정 |
| 수동 모드·arm 도중 취소·늦은 응답 | 후속 자동 이륙 차단 |
| 요청 기록 쓰기 실패 | arm 전에 차단 |
| 브라우저 텔레메트리 만료 | 과거 비행 상태 제거 |

증빙은 [evidence/web_test_flight_20261008](evidence/web_test_flight_20261008/)에 있다.
패키지별 XML·실행 로그 원본은 validation.tar.gz에 보존했다.
집계 JSON도 함께 보존했다.
웹 JavaScript 구문 검사와 git diff --check도 통과했다.
브라우저 화면의 시각 검토는 미실시다.

## 실제 ROS·HTTP·WebSocket 연결시험

FC만 프로토콜 모사 노드다. 비행 시뮬레이터가 아니다.
B_TF·브리지·미션·플랫폼은 실제 ROS 프로세스다.
RAW 거리·ToF·IMU·TIMESYNC를 ROS로 발행했다.
웹 HTTP start 요청으로 시퀀스를 시작했다.

| 시행 | 확인 결과 |
|---|---|
| 정상 | arm → takeoff → reposition → land, 지상·disarm 확인 |
| UWB 단절 | 착륙 요청, 실패 보고, 센서 복구 후 재이륙 없음 |
| 웹 단절 | 2초 수신 만료 뒤 착륙 요청, 실패 보고 |
| 공중 미션 노드 재시작 | ledger 유지, 재arm 없이 착륙, 주체 상실 보고 |

FC 입력 z=0·Z 공분산 1e6도 검사했다.
이륙·수평 목표에 NaN 고도를 사용했다.
PX4 목표의 새 시각·XY 일치를 검사했다.
이 검사는 실제 EKF 수신·융합 증거가 아니다.

기존 데모 ROS 임무 검사도 통과했다.
위치 479개·UWB 439개·공백 상태 40개를 수신했다.
목표 QoS·ToF 만료·ARRIVED 전이를 확인했다.

PX4 v1.16.0 공개 소스의 NaN 처리도 대조했다.
실물 펌웨어 버전을 확인한 것은 아니다.

| 참조 | 근거 |
|---|---|
| [takeoff.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/navigator/takeoff.cpp#L96) | 고도 미지정 시 현재 고도 + MIS_TAKEOFF_ALT |
| [navigator_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/navigator/navigator_main.cpp#L263) | 수평 reposition의 NaN 고도는 현재 고도 유지 |

## 실물 확인과 접속 상태

사용자는 어제 일부 적용했다고 확인했다.
현재 설정은 켜진 Jetson·FC에서 읽어야 알 수 있다.
과거 params를 현재 설정으로 단정하지 않는다.
실물 정렬·지연·장착 위치·EV 융합은 미확인이다.
전역 원점과 실내 전역 목표 지원도 미확인이다.
실제 datalink failsafe·다른 GCS 영향은 별도 시험한다.

SSH 설정은 `arialhanho@100.110.163.94`다.
초기 프록시 CONNECT 22 결과는 403이었다.
사용자는 Jetson 전원 투입을 알렸다.
직후 검사는 8초 TimeoutError였다.
제한을 늘린 검사는 CONNECT 200 뒤 SSH 인사말이 없었다.
사용자는 Tailscale 브라우저 로그인도 완료했다.
이후 SSH도 인증 전에 연결이 종료됐다.
사용자 기기 화면은 Jetson이 Connected임을 보여 줬다.
IP는 `100.110.163.94`로 기록과 같았다.
MagicDNS `user-desktop.tail720b90.ts.net`은 프록시 403이었다.
클라우드 VPN 접속·SSH 인증·로그인은 미확인이다.
SSH 실패를 기체 오프라인으로 판정하지 않는다.
기록은 [live_access.json](evidence/web_test_flight_20261008/live_access.json)이다.

읽기 전용 지상 수집기를 추가로 준비했다.
저장소 경로는 `src/drone_mission/tools/observe_ground.py`다.
상태·XYZ·ToF·MAVROS mirror 수집을 합성 ROS로 확인했다.
시리얼 열기·명령·설정 변경은 하지 않는다.
수신과 FC 융합 검증은 별개다.

기준 HEAD는 `deb940f612d7ee5198762aa73c7d7f22140beb61`다.
기능은 클라우드 작업 파일과 배포 패치에 보존했다.
이후 로컬 세션 인계를 위한 별도 브랜치를 만들었다.
브랜치는 `codex/web-flight-handoff-20261008`이다.
Jetson 적용은 미실시다.
다음 작업은 [현장 절차](../runbooks/web_test_flight.md)를 따른다.

## 로컬 세션 인계 때 추가로 확인한 사항

원격 main 밖에서 어제의 실물 통합 기록을 찾았다.
`codex/flight-uwb-ai-integration-20261007`이다.
확인한 HEAD는 `1cc7e3b7fbfe39a693968925bbec507db854380d`다.
이 브랜치에는 C++ AI와 TDMA·Tag A/B 연결이 있다.
이번 작업과 추적 변경 파일 16개가 겹친다.
현재 구현에 병합하지 않았다.

Tag B/domain 2·앵커 0.15m·이륙 1.3m가 기록돼 있다.
`COM_RC_OVERRIDE=2`와 RC 채널 0 관측도 기록돼 있다.
600초 UWB 연속성 실패와 B_TF·vision 출력 0회가 남아 있다.
이는 오늘 실물 상태를 직접 읽은 결과가 아니다.
현재 클라우드 기본값으로 그 설정을 교체하지 않는다.
이유: 장치·배치·명령 권한과 검증 범위가 다르다.

[기계 판독 대조 기록](evidence/web_test_flight_20261008/handoff_review.json)과
[다음 세션 절차](../runbooks/local_jetson_handoff_20261008.md)에 인계했다.
실물 비행 준비 완료로 판정하지 않았다.
