# 로컬 세션·Jetson 작업 인계

클라우드 구현을 로컬 SSH 세션으로 이어받는 절차다.
새 세션은 이 문서부터 읽고 실제 장비와 대조한다.

현재 판정은 **실물 비행 보류**다.
웹 전체 순서는 클라우드에서 구현·검사했다.
어제의 실물 통합 브랜치는 main에 없었다.
두 코드의 설정·입력 계약을 통합해야 한다.
기존 Jetson 작업을 이 브랜치로 바로 교체하지 않는다.
이유: 기체·앵커·RC·이륙 설정이 서로 다르다.

## 1. 보존된 코드부터 읽기

전달 브랜치는 `codex/web-flight-handoff-20261008`이다.
기준은 `deb940f612d7ee5198762aa73c7d7f22140beb61`이다.
main에 직접 커밋하거나 병합하지 않는다.

로컬의 새 빈 경로에서 검토 저장소를 준비한다.
기존 작업 폴더를 덮지 않는다.

```bash
git clone --branch codex/web-flight-handoff-20261008 \
  https://github.com/Dronestock-SW/ROS2.git ROS2-web-flight-review
cd ROS2-web-flight-review
git status --short --branch
git rev-parse HEAD
```

읽는 순서는 다음과 같다.

1. `AGENTS.md`, `docs/roadmap.md`, `docs/altitude_policy.md`.
2. `docs/glossary.md`, `docs/equipment_inventory.md`.
3. [현재 명령·관측 구조](../architecture/web_test_flight.md).
4. [클라우드 검증 기록](../report/web_test_flight_20261008.md).
5. 아래 어제의 통합 브랜치와 실제 Jetson 설정.

다른 세션은 `/workspace/ros2-cloud`가 있다고 가정하지 않는다.
필요한 지상 수집기는 이번 브랜치에 포함했다.
[수집기](../../src/drone_mission/tools/observe_ground.py)와
[합성 수집 근거](../report/evidence/web_test_flight_20261008/ground_observer_synthetic.json)를 따른다.
후자는 실물 로그가 아니다.

## 2. 어제의 통합 코드와 대조

다음 원격 코드를 확인했다. 이번 브랜치에 병합하지 않았다.

| 브랜치 | 확인한 commit | 포함 내용 |
|---|---|---|
| `codex/flight-uwb-ai-integration-20261007` | `1cc7e3b7fbfe39a693968925bbec507db854380d` | Tag A/B·TDMA·UWB 보호 조건·C++ AI·비전 보존·실물 기록 |
| `codex/uwb-web-handoff-20261007` | `7cfc482feee49d82ece1c3987af920587a1a920f` | TDMA 시각·웹 관측 계약. 위 통합 브랜치에 포함 |

어제 통합의 핵심 문서는 다음과 같다.
링크는 확인한 commit에 고정했다.

- [통합 구조](https://github.com/Dronestock-SW/ROS2/blob/1cc7e3b7fbfe39a693968925bbec507db854380d/docs/architecture/flight_uwb_ai_integration.md).
- [낮은 앵커·FC 설정·수신 실패 기록](https://github.com/Dronestock-SW/ROS2/blob/1cc7e3b7fbfe39a693968925bbec507db854380d/docs/report/anchor_low015_bench_20261007.md).
- [실물 지상 절차](https://github.com/Dronestock-SW/ROS2/blob/1cc7e3b7fbfe39a693968925bbec507db854380d/docs/runbooks/flight_uwb_ai_ground.md).
- `src/sangwon_AI/`의 설계·부팅·BT·실행 상태 문서.

다음 값은 어제의 기록이다. 오늘의 실물 조회가 필요하다.

| 항목 | 이번 클라우드 기본값 | 어제 통합 기록 |
|---|---|---|
| 태그·ROS domain | Tag A / ID 5 / domain 1 | Tag B / ID 6 / domain 2 |
| 앵커 높이 | 2.2m | 임시 0.15m. 복원 여부 확인 필요 |
| 이륙 상승 높이 | 기대값 0.6m | FC `MIS_TAKEOFF_ALT` 약 1.3m |
| RC 인계 | 실제 모드 변화 감시 | `COM_RC_OVERRIDE=2`. 자동 모드 스틱 인계 비트 꺼짐 |
| UWB 융합 허용 | `EKF2_EV_CTRL=1` 요구 | 저장·재부팅 후 1 확인. 실제 융합은 미확인 |
| 장착값 | Tag A 렌즈→태그 +0.12m 기록 | Tag B는 미측정·확인값 false |
| 원본 거리 시각 | main 기반 수신 경로 | TDMA 각 거리 시각·A/B 계약 추가 |

어제 600초 UWB 기록은 연속성 검사에 실패했다.
1초 창의 최소 수신은 0Hz였다.
기준 미달은 16창, 연속 누락은 최대 58개였다.
JSON 오류 43회, B_TF pose와 FC vision 출력은 0회였다.
이 기록은 해결 전제로 넘길 수 없다.
RC 16채널이 모두 0인 관측도 남아 있다.
RC 메시지 수신만으로 조종기 작동을 판정하지 않는다.

로컬에서 코드 차이를 읽는다.

```bash
git fetch origin \
  refs/heads/codex/flight-uwb-ai-integration-20261007:refs/remotes/origin/codex/flight-uwb-ai-integration-20261007
git diff HEAD origin/codex/flight-uwb-ai-integration-20261007 -- \
  src/drone_uwb src/drone_platform_link src/drone_bringup
git show origin/codex/flight-uwb-ai-integration-20261007:docs/report/anchor_low015_bench_20261007.md
```

별도 통합 브랜치에서 두 작업을 합친다.
수신 시각·태그·배치 선택·EV 전달 조건을 보존한다.
충돌 난 파일을 한쪽 버전으로 통째 덮지 않는다.
이유: 양쪽이 같은 bridge·B_TF·platform 파일을 바꿨다.
현재 추적 변경 파일 16개가 겹친다.
[대조 근거](../report/evidence/web_test_flight_20261008/handoff_review.json)에 경로를 남겼다.

`drone_mission`은 로컬 웹 시험 FSM이다.
어제 `sangwon_AI`는 C++ BT·스캔·가드·재생 코드를 보존한다.
두 실행기를 별개 명령 발행자로 동시에 켜지 않는다.
실제 PX4 출력 주체와 명령 중재를 하나로 확정한다.
비전 속도 제안도 그 주체를 통해 전달해야 한다.

## 3. 로컬 PC에서 SSH 연결

Jetson은 사용자 화면에서 Connected 상태를 확인했다.
클라우드에서는 SSH 인증 전에 연결이 종료됐다.
클라우드 VPN·프록시 문제의 정확한 원인은 미확정이다.
이 실패로 Jetson의 SSH 장애를 확정하지 않는다.

로컬 PC의 Tailscale 클라이언트 연결을 확인한다.
브라우저 관리 화면 로그인은 VPN 경로 확인과 별개다.

```bash
tailscale status
tailscale ping 100.110.163.94
ssh arialhanho@100.110.163.94
# IP 대신 쓸 수 있는 확인된 MagicDNS
# ssh arialhanho@user-desktop.tail720b90.ts.net
```

`arialhanho`는 사용자가 지정한 SSH 계정이다.
과거 문서의 Linux 계정은 `pgyxn`이었다.
접속 오류가 인증 단계라면 실제 계정·인증 방식을 확인한다.
호스트 키는 기존 신뢰 경로로 대조한다.
암호·개인키·장치 인증값을 Git이나 인계 문서에 넣지 않는다.

## 4. 교체 전에 실물 상태 보존

처음에는 프로펠러를 제거하고 현재 상태를 읽는다.
원격 계정과 저장소의 `AGENTS.md`부터 확인한다.
기존 서비스·설정·변경 파일을 보존한다.

| 읽을 대상 | 남길 근거 |
|---|---|
| 계정·OS·workspace | 실제 사용자, 경로, ROS·MAVROS·PX4 버전 |
| Git | HEAD·branch·status·diff·추적되지 않은 설정 |
| 서비스·포트 | MAVROS·UWB·platform·AI의 실행 주체와 시리얼 소유자 |
| 기체·배치 | 실제 Tag ID, domain, firmware, layout ID, 네 앵커 높이 |
| FC | 전체 params와 현재 RC·고도·EV·failsafe 설정 |
| 기존 실행 근거 | `.integration-evidence/` 등 어제의 로컬 기록 |

FC 파라미터 원본과 설정 파일의 복구본을 먼저 만든다.
비밀값이 든 env·장치별 flash 백업은 Git에 넣지 않는다.
기존 서비스를 중복 실행하거나 포트를 강제로 빼앗지 않는다.

수집기는 기존 ROS 환경을 사용한다.
아래 예는 실제 Tag B/domain 2를 확인한 경우다.
Tag A/domain 1이면 두 domain 인수를 모두 1로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
python3 /path/to/review/src/drone_mission/tools/observe_ground.py \
  --domain 2 --seconds 10 > ground-observation.json
```

이 도구는 기존 토픽과 MAVROS parameter mirror를 읽는다.
시리얼 열기·arm·이륙·파라미터 쓰기를 하지 않는다.
토픽 이름과 실제 발행자·수신 횟수를 함께 확인한다.
mirror의 값은 FC 직접 조회·적용값과 다시 대조한다.

## 5. 명령 흐름의 검토 지점

검증할 전체 경로는 다음과 같다.

```text
웹 새 START -> HTTP GET -> ROS assignment -> 계약·시각·기체 검사
 -> 요청 ID 영구 기록 -> PX4 arm -> native takeoff
 -> AUTO.LOITER -> 수평 목표 -> 실제 목표 반영
 -> PX4 위치·속도·유지 시간으로 도착 -> 다음 경유지
 -> native land -> ON_GROUND + disarmed -> 완료 보고

UWB 원본 + ToF·IMU·TIMESYNC -> B_TF -> XY 관측 -> PX4 EKF2
                                  +-> 태그 XYZ 표시
```

| 경계 | 통과 증거 |
|---|---|
| 웹 수신 | 올바른 기체·좌표축·단위·배치·revision·요청 ID |
| 재요청 | 같은 ID로 재이륙하지 않음. 원장 기록 실패 때 시작 차단 |
| arm | 일반 arm 수락과 실제 armed 상태 |
| takeoff | 실제 펌웨어의 NaN 처리·이륙 설정과 AUTO.LOITER 진입 |
| 이동 | 송신 성공 뒤 새 FC 목표 피드백 일치 |
| 도착 | PX4 위치 반경 0.15m·속도 0.1m/s 이하·1초 유지 |
| 착륙 | 최신 ON_GROUND와 disarmed를 모두 확인 |
| 보고 | ACK·목표 적용·도착·착륙 완료를 각각 구분 |

고도·자세 제어는 PX4가 맡는다.
UWB XY와 태그 XYZ 표시는 비행용 독립 EKF가 아니다.
전역 원점이 없으면 현재 이동 경로는 시작할 수 없다.
가짜 원점·GNSS·측정값으로 게이트를 통과시키지 않는다.

## 6. 실물 비행 전 해소할 항목

미확인 항목을 true로 바꿔 진행하지 않는다.
현재 파일의 확인값과 `execute` 기본값은 false다.

| 항목 | 필요한 검증·완료 조건 |
|---|---|
| 입력 계약 통합 | Tag A/B·TDMA·layout·domain이 전 경로에서 일치 |
| UWB 연속성 | 어제 0Hz 창·JSON 오류 원인 해결과 장시간 재측정 |
| 좌표·시각·장착 | 실제 +X/+Y 이동, ENU/NED 변환, FC FRD 레버암, ToF FLU, 지연 대조 |
| EKF 수평 융합 | ULog의 EV 수신·융합·innovation과 독립 위치 기준 대조 |
| 위치 유지·표류 | 실제 PX4 Hold에서 위치·속도·편향 추세 측정 |
| 고도 | 현재 FC 설정·ToF 융합·기준점·실제 상승 높이 대조 |
| 수평 명령 | 실제 펌웨어·원점에서 DO_REPOSITION 적용·정지·도착 확인 |
| RC 조종권 | 유효 채널·모드 스위치·스틱 인계와 자동 재획득 금지 |
| 단일 명령 주체 | FSM·BT·비전·기존 서비스의 동시 명령 방지. 현재 전역 잠금은 없음 |
| FC 자체 대응 | companion 종료·USB 상실·FC 재연결·RC 상실·저전압의 실제 정책 확인 |
| 다른 GCS | QGroundControl 연결이 datalink 상실 감지를 가리는지 확인 |
| 장애물 | 현재 사각형·구간 길이 검사에는 선반·장애물 회피가 없음 |
| 스캔·AI 임무 | C++ BT·ArUco·QR·스캔 창·실제 PX4 writer 연결 및 공동 시험 |

이번 웹 FSM은 `waypoint`·`hover`만 받는다.
`scan`과 운영 웹의 START 지원은 완성 범위가 아니다.
현재 경로는 Offboard stream 방식도 아니다.
Offboard-loss 파라미터만으로 프로세스 상실을 처리했다고 판정하지 않는다.

| 고장 | 현재 코드의 처리 | 실물에서 남은 확인 |
|---|---|---|
| UWB·높이·PX4 상태 노후 | 실행 중 착륙 요청 | 위치 상실 상태에서 FC의 착륙 가능 여부 |
| 웹 조회 2초 상실 | 착륙 요청·자동 재개 없음 | 실제 통신 단절·지연과 FC 반응 |
| 목표 변경·피드백 0.5초 상실 | 착륙 요청 | FC stream 주기·실제 목표 피드백 |
| 명령 거부·무응답 | 진행 차단·중단 처리 | 착륙 거부·통신 상실은 소프트웨어가 강제 해결하지 못함 |
| 수동 모드 전환 | 후속 명령 중지 | 실제 RC 설정·스위치와 조종권 회수 |
| arm 중 취소 | 이륙 취소·일반 disarm | 수락 후 새 지상 상태 확인 |
| 공중 실행기 재시작 | 자동 모드면 착륙 요청 | 실제 재시작·중복 실행·독립 감시 |
| 프로세스·전원 종료 | FSM 자체가 멈춤 | PX4 자체 failsafe와 RC가 담당해야 함 |

착륙 요청 성공은 추락 방지 보장이 아니다.
위치·추력·배터리·링크가 상실되면 FC 대응도 제한된다.
명령 수락만으로 물리 동작 성공을 기록하지 않는다.

## 7. 다음 세션의 진행 순서

1. 로컬 Tailscale·SSH와 실제 계정·workspace를 확인한다.
2. 어제 코드·FC params·장착·기체 선택을 보존하고 읽는다.
3. 별도 통합 브랜치에서 두 구현의 계약·명령 주체를 정리한다.
4. 통합 코드의 빌드·회귀·명령 순서 검사를 다시 실행한다.
5. 프로펠러 제거 상태에서 센서·RC·FC 설정과 전달을 검증한다.
6. 실제 PX4 SITL에서 정상·단절·재시작·거부·수동 인계를 검사한다.
7. 실제 Hold 안정성과 모든 비행 조건을 확인한다.
8. 조종자가 준비된 현장에서 짧은 단일 구간부터 시험한다.
9. ULog와 companion 원본을 함께 분석한 뒤 범위를 늘린다.

프로세스 종료·링크 상실 주입은 SITL·무프로펠러 지상에서 먼저 한다.
첫 실물 비행에 고장 주입 시험을 합치지 않는다.
이유: 기체의 기본 반응과 상실 대응이 아직 검증되지 않았다.
[실행 명령](web_test_flight.md)은 통합·설정 대조 후 사용한다.

## 8. 검증 근거와 새 세션 요청

클라우드에서 6개 패키지 빌드가 성공했다.
전체 545개 중 542개 통과·기존 lint 2개 실패·1개 skip이다.
미션 58개에는 ROS·HTTP·WS 시나리오 4개가 포함된다.
정상·UWB 단절·웹 단절·공중 재시작을 검사했다.
FC는 모사 노드다. 실제 EKF·물리 비행 증거는 아니다.
검사 로그는 저장소의 [근거 폴더](../report/evidence/web_test_flight_20261008/)에 있다.

새 세션에는 다음 요청을 전달한다.

```text
Dronestock-SW/ROS2의 codex/web-flight-handoff-20261008 브랜치를 읽고
docs/runbooks/local_jetson_handoff_20261008.md부터 이어서 작업하세요.
로컬 Tailscale/SSH로 Jetson의 현재 코드와 FC 설정을 먼저 확인하세요.
어제 codex/flight-uwb-ai-integration-20261007의 Tag B/domain 2,
앵커 0.15m, 이륙 1.3m 기록과 현재 실제 설정을 대조하세요.
UWB 연속성·EV 융합·RC 인계·단일 명령 주체·PX4 failsafe가 미검증입니다.
main에 직접 작업하지 말고 별도 통합 브랜치에서 구현·시험하세요.
웹 수신→미션→이륙→이동→도착→착륙과 실패 경로를 끝까지 검증하세요.
확인값을 임의로 true로 바꾸거나 이륙을 자동 실행하지 마세요.
각 결과는 실제 근거와 미실시 항목을 구분해 기록하세요.
```
