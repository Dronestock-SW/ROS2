# Gazebo 수평 목표 연결 구조
이 문서는 목표 명령과 도착 판정의 연결 구조다.
SITL에서 한 목표 이동을 연결할 때 읽는다.

## 구현 상태

**관측과 목표 이동을 같은 실시간 실행기에 연결했다.**
현재는 구현과 로컬 모의 메시지 시험까지다.
WSL 배포와 실제 실행 검증은 남아 있다.
PX4 명령 접수·실제 이동·정지 검증도 미실시다.
기본 설정의 송신은 비활성이다.

```text
목표 UWB 지도 x·y (PX4 기준점)
  → 허용 구역·이동 거리·현재 FC 상태 검사
  → 관측 연결과 같은 지도→NED 변환
  → PX4 전역 원점을 기준으로 위도·경도 변환
  → COMMAND_INT / DO_REPOSITION
  → PX4가 수평 이동·고도 유지 수행

명령 ACK + PX4의 실제 목표 메시지
  → 명령 접수·목표 반영 판정

PX4 위치·속도 + 추정 상태 + UWB 신선도
  → 기존 도착 상태 판단 재사용
  → 거리·속력·유지시간 충족 판정

Gazebo 정답
  → 별도 실제 도착 오차 평가 (후속 연결)
```

## 선택한 시험 후보

첫 구현 후보는 `MAV_CMD_DO_REPOSITION`이다.
현재 Hold 상태에서 다음 수평 목표만 요청한다.
고도·방향 항목은 NaN으로 보낸다.
PX4가 명령 수신 시점의 현재 고도를 선택한다.
고도 오차를 계산하는 companion 제어기는 없다.
다음 목표도 같은 방식으로 처리한다.

동작 근거는 대상 커밋의
[Navigator](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/navigator/navigator_main.cpp)와
[Commander](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/commander/Commander.cpp)다.
명령 정의는 [MAVLink](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION)를 따른다.
고도 유지와 실제 이동의 검증은 이후 시행에서 한다.

| 항목 | 현재 처리 |
|---|---|
| 이동 속도 | 시험값 0.30m/s |
| 1회 이동 거리 | 현재 위치에서 최대 1.00m |
| 지도 허용 구역 | x 0.50~5.30m, y 0.50~3.90m |
| 목표 기준점 | 관측 변환 후의 PX4 기준점 |
| 모드 변경 | 요청하지 않음. 이미 Hold여야 함 |
| 시동·이륙 | 이 모듈에서 수행하지 않음 |
| 고도·방향 | PX4 선택에 맡김 |
| 시계·원점 변경 | 기존 명령의 재사용 거부 |
| 송신 예외 | 같은 패킷 자동 재전송 없음 |

허용 구역은 불규칙 앵커 배치 내부의 시험 사각형이다.
벽 충돌·기체 크기·제동거리를 검증한 경계는 아니다.
실제 시험 경로와 목표 목록은 실행 전에 기록해야 한다.

## 좌표 정밀도

전역 좌표 형식은 GNSS 융합 활성화를 뜻하지 않는다.
PX4가 사용하는 전역 원점과 위치 유효성은 필요하다.
원점이 없으면 이 경로는 실행할 수 없다.
임의의 위도·경도로 원점을 대체하지 않는다.
이유: 관측과 목표가 서로 다른 지점을 가리키게 된다.

지도 변환은 관측 모듈의 함수를 그대로 사용한다.
목표는 이미 PX4 기준점이다. 장착 보정을 재적용하지 않는다.
전역 변환은 구면의 방위정거 투영식이다.
지구 반경은 대상 PX4와 같은 6,371,000m다.
[PX4 좌표 변환 근거](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/lib/geo/geo.cpp)를 따른다.

위도·경도는 1e7을 곱한 정수로 전송한다.
이를 E7 형식이라 한다.
명령은 `COMMAND_INT`를 사용한다.
`COMMAND_LONG`의 단정밀도 좌표는 사용하지 않는다.
이유: cm 단위 목표를 구분하기 어렵기 때문이다.

양자화 후 목표를 NED로 되돌려 차이를 기록한다.
`target_quantization_error_m`가 이 차이다.
원점 메시지 자체의 반올림 오차는 포함하지 않는다.
[GPS_GLOBAL_ORIGIN 송신 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/mavlink/streams/GPS_GLOBAL_ORIGIN.hpp)도 E7 반올림을 사용한다.
따라서 이 수치는 전체 좌표 정확도가 아니다.
UWB의 7cm 측위 목표와도 다른 지표다.

## 연결할 입출력

송신 전에는 그 시점의 상태로 검사를 다시 수행한다.
`NavigationReadiness`는 실시간 상태 전달 형식이다.
설정 파일에서 유효성을 임의로 켜는 용도가 아니다.
`SITLNavigationSession`이 같은 MAVLink 연결에서 채운다.

| 모듈 | 입력 | 출력 |
|---|---|---|
| `reposition_fields` | 목표·현재 지도 위치, 정렬·전역 원점, FC 상태 | 좌표 변환·정밀도·명령 필드 |
| `send_reposition_fields` | 검사를 통과한 새 필드, SITL 송신 객체 | 1회 COMMAND_INT 전달 |
| `RepositionProgress` | ACK, 새 FC 목표, 해당 시각 도착 판정 | 접수·반영·도착을 구분한 상태 |
| `PX4MissionMonitor` | ODOMETRY, ESTIMATOR_STATUS, UWB 시각 | 수평 도착·입력 실패 상태 |

명령 준비에는 다음 상태가 필요하다.
시동·공중·Hold·수평 위치/속도 유효성을 확인한다.
자세 해 유효성·전역 위치 유효성도 확인한다.
관측 정렬 때의 원점 시각·초기화 카운터가 같아야 한다.
FC 입력 최대 나이는 0.20초다.
UWB 입력 최대 나이는 0.25초다.
WSL 시뮬레이션과 관측 융합의 확인도 필요하다.

`ESTIMATOR_STATUS`의 자세 해 플래그를 읽는다.
이는 uORB의 `heading_good_for_control`과 다르다.
MAVLink에 없는 그 값을 참으로 만들어 넣지 않는다.
현재 기체가 공중 Hold인지도 함께 확인한다.
모드 허용과 비행 제어의 최종 판단은 PX4가 한다.
UWB 융합 확인은 별도 로그 근거가 필요하다.

## 도착 판정 재사용

기존 `MissionMonitor`에 PX4 입력 선택을 추가했다.
기본 데모 동작은 기존 위치 차분 방식을 유지한다.
PX4 입력은 FC 속도를 필수로 받는다.
위치가 잠시 같은 값이어도 속도가 높으면 도착을 거부한다.
body FRD 속도는 자세로 NED에 회전한 뒤 사용한다.
미상 속도를 0으로 대체하지 않는다.

| 조건 | 초기 시험값 |
|---|---:|
| 도착 진입 / 해제 | 0.15 / 0.20m |
| 속력 상한 | 0.10m/s |
| 연속 유지 | 1.00초 |
| 입력 복구 대기 | 0.50초 및 각 입력 3개 |
| 명령 응답 대기 | 2.00초 |
| FC 목표 메시지 최대 나이 | 0.50초 |

도착 반경은 비행 시험용 값이다.
UWB 측위 오차 7cm 기준을 바꾸지 않는다.
복구·유지시간은 기존 상태 판단을 재사용한다.
PX4와 UWB 표본 시각의 원점을 서로 빼지 않는다.
각 입력의 표본 시각과 수신 나이를 따로 사용한다.
PX4 초기화·시각 역행은 시행을 중단 상태로 유지한다.
새 정렬과 새 판정 인스턴스가 필요하다.

ACK만으로는 목표 반영이나 도착을 인정하지 않는다.
새 `POSITION_TARGET_GLOBAL_INT`가 목표와 맞아야 한다.
이 메시지의 정수 변환 차이는 E7 한 단위까지 허용한다.
그 뒤 최신 PX4 도착 판정도 통과해야 한다.
Gazebo 정답 기준의 실제 도착은 별도 평가로 남는다.
후속 [파일 평가기](uwb_gazebo_flight_evaluation.md)를 추가했다.
Gazebo 정답은 실행 중 도착 판단에 넣지 않는다.
새 기록의 원래 위치 표본 시각을 기준으로 대조한다.
실제 사용자 비행 기록의 평가는 아직 미실시다.

## 실시간 연결과 중단 처리

`drone_demo.sitl_navigation`이 관측 기록기를 호출한다.
UDP 수신 소켓과 수신 루프는 관측 기록기 하나다.
원문 메시지는 관측 검사 뒤 임무 연결기로 전달한다.
목표 송신도 같은 연결을 사용한다.
추가 ROS 설치나 두 번째 UDP 수신기는 필요 없다.

```text
단일 MAVLink 수신 → 관측 연결 검사 → 임무 상태 갱신
가상 RAW·ToF·자세 → 선택 모델 관측 송신 → 관측 신선도 갱신
신선한 FC 상태 + 관측 + 설정 확인
  → 0.5초 연속 입력 확인 → 첫 목표 송신
  → 명령 접수 + 목표 반영 + 도착 판정
  → 다음 목표 → 기록한 출발점 → PX4 착륙 요청
```

상태는 WAITING → MOVING → LANDING → LANDED다.
착륙 옵션을 끄면 마지막 도착에서 COMPLETED가 된다.
실패하면 ABORTED로 남으며 자동 재개하지 않는다.
복구 시험은 새 관측 연속성과 정렬을 확인해 다시 시작한다.
지상 시동·이륙 연결은 아직 이 실행기의 범위 밖이다.

첫 이동 전 0.5초를 세 시계에서 각각 확인한다.
Ubuntu 경과시간·PX4 표본 진행·UWB 표본 진행이다.
각 입력은 추가로 3개 이상 새로 들어와야 한다.
도착 유지시간은 PX4 위치 표본의 시각으로 잰다.
정지한 표본을 타이머만으로 도착 처리하지 않는다.

| 상황 | 구현한 요청 | 아직 확인할 결과 |
|---|---|---|
| UWB 만료, PX4 위치·속도·Hold 유효 | 좌표 없는 DO_REPOSITION으로 제동점 요청 | 실제 감속·정지 위치 |
| 사용자 종료·시간 제한, 유효한 PX4 상태 | 같은 제동점 요청 | 명령 반영·정지 |
| 위치 무효·원점 reset·시각 이상 등 | NAV_LAND 요청 | PX4 모드·하강·접지 |
| ACK·목표 반영 없음 | 2초 뒤 중단·착륙 요청 | 거절 원인과 실제 대응 |
| 반영된 목표 메시지 중단 | 0.5초 뒤 중단·착륙 요청 | 실제 대응 |
| 실행 프로세스 자체 종료 | PX4의 GCS 단절 설정에 의존 | 별도 프로세스 강제 종료 시험 |

착륙 요청의 근거는 대상
[Commander의 NAV_LAND 처리](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/commander/Commander.cpp)다.
명령 접수만으로 착륙을 완료 처리하지 않는다.
최근 접지 상태와 disarmed를 함께 수신해야 한다.
그래도 `flight_valid`와 `fusion_verified`는 자동 승격하지 않는다.
Gazebo 정답·ULog의 독립 검증이 남기 때문이다.

송신 모드는 1Hz GCS heartbeat를 보낸다.
실행 중 파라미터를 읽지만 쓰지는 않는다.
필요한 SITL 초기 설정은 아래와 같다.

| PX4 파라미터 | 요구값 | 목적 |
|---|---:|---|
| NAV_DLL_ACT | 3 | GCS 단절 때 Land |
| COM_DL_LOSS_T | 5초 | 대상 버전의 최소 허용 대기 |
| COM_DLL_EXCEPT | bit 1 꺼짐 | Auto Hold에서 단절 예외 해제 |
| COM_FAIL_ACT_T | 0초 | 추가 failsafe 대기 없음 |
| COM_POS_FS_ACT | 0 | 위치 상실 때 PX4 Descend 정책 |

근거는 대상 버전의
[commander_params.yaml](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/commander/commander_params.yaml)이다.
다른 GCS의 heartbeat를 받으면 목표 송신을 막는다.
다른 GCS가 없다는 사실을 무수신만으로 입증할 수는 없다.
QGroundControl 등 별도 GCS가 연결돼 있으면
이 프로세스의 종료가 GCS 단절로 잡히지 않을 수 있다.
따라서 실제 프로세스 단절 시험이 남아 있다.

## 시계와 기록

PX4에는 20Hz로 TIMESYNC 조회를 보낸다.
실제로 받은 PX4 부팅 시각만 사용한다.
Ubuntu 경과시간으로 PX4 시각을 외삽하지 않는다.
왕복 50ms 초과 응답은 사용하지 않는다.
PX4 시각이 200ms 동안 진행하지 않으면 입력을 거부한다.
늦게 온 이전 조회 응답과 시각 역행은 구분한다.

시계 응답보다 새 위치 표본은 잠시 보류한다.
최대 64개 이력에서 나이를 확인한 최신 표본을 고른다.
표본 나이와 수신 후 경과시간 중 큰 값을 쓴다.
이는 시각 동기화 정확도나 비행 성능의 증명은 아니다.

기존 관측 파일에 아래 시행 기록을 추가한다.

| 파일 | 기록 |
|---|---|
| navigation_settings.json | 목표 송신 조건·한계 복사본 |
| navigation_plan.json | 목표·복귀·착륙·시간 제한 |
| mission_config.json | 도착·입력 만료·복구 기준 |
| navigation_manifest.json | 모드·임무 소스 해시 |
| navigation_events.jsonl | 시각·상태·명령·ACK·반영·도착·실패 |
| navigation_summary.json | 요청 수·완료 구간·종료 상태 |

`input_index.json`이 이 파일들의 해시도 보존한다.
종료 요청에서 오류가 나도 연결 해제와 기록을 계속한다.
정리 오류는 capture.json의 cleanup_errors에 남긴다.

## 검증과 남은 작업

2026-10-02 로컬 검증 결과는 다음과 같다.

| 확인 | 결과 |
|---|---|
| 데모·도착·목표·관측 연결 관련 시험 | 124개 통과 |
| 실제 `pymavlink 2.4.50` 시험 | 7개 통과. 목표 4개와 기존 관측 3개 |
| 패킷 시험 범위 | 목표는 메모리 왕복, 관측은 메모리·UDP loopback |
| 빌드 | `colcon build --symlink-install`, 5개 패키지 성공. 8.69초 |
| 실제 WSL·PX4 비행 | 미실시 |

처음 데모 시험 수집은 ROS 모듈 경로 누락으로 실패했다.
ROS 설치 설정과 작업공간 설정을 적용했다.
기존 PYTHONPATH를 보존한 재실행에서 통과했다.
시험 중 외부 컴퓨터나 실제 PX4에는 명령을 보내지 않았다.
PX4 응답은 시험 코드가 생성했다.
실제 비행제어기 응답이나 EKF 융합 결과가 아니다.

위 표는 모듈만 구현했을 때의 이력이다.
같은 날 실시간 실행기 연결 뒤 관련 시험 142개가 통과했다.
실제 pymavlink 형식의 모의 PX4 메시지를 사용했다.
정상 순회·복귀·착륙 판정과 단절·무효 입력을 확인했다.
단일 수신 연결과 종료 오류 뒤 기록 보존도 확인했다.
초기 재시험 1개는 시험 준비 상태의 불일치로 실패했다.
응답을 이미 받은 상태로 무응답을 시험한 코드였다.
처음부터 응답을 주지 않도록 수정한 뒤 통과했다.
통합 빌드는 5개 패키지가 9.77초에 완료됐다.
설치 경로의 새 모듈 import도 통과했다.
센서 오류 확인을 임무 tick보다 앞에 배치했다.
이 마지막 변경 뒤 CLI 관련 3개 시험을 다시 통과했다.

새 통합 갱신본은
`/home/pgyxn/uwb_gazebo_navigation_runtime_20261002.zip`이다.
[실행 절차와 제작 확인](uwb_gazebo_navigation_runbook.md)을 따른다.
기존 모듈 전용 ZIP은 변경하지 않았다.
아래 이전 배포 기록과 새 통합본을 구분한다.

남은 항목은 사용자 WSL 배포·연결·융합 검증이다.
이후 실제 한 목표, 순회·복귀·착륙을 시험한다.
단절 시 요청과 실제 정지·모드 대응도 대조해야 한다.
명령 판정을 중단해도 기체가 멈췄다고 보지 않는다.
PX4는 이미 받은 목표를 계속 수행할 수 있기 때문이다.
이전 관측 연결 ZIP에는 이 변경이 포함되지 않는다.
새 모듈 묶음은 다음 경로에 별도로 준비했다.
`/home/pgyxn/uwb_gazebo_sitl_target_modules_20261002.zip`
같은 이름의 `.zip.sha256`으로 해시를 대조한다.
27일 기본 묶음에 덮어쓰는 갱신본이다.
기존 관측 갱신 파일도 함께 포함한다.
이 이전 묶음에는 새 라이브 실행기가 포함되지 않는다.

기본 ZIP과 갱신본을 임시 폴더에 겹쳐 확인했다.
목표 모듈 import와 기존 관측 CLI 도움말이 통과했다.
SHA-256은 다음과 같다.
`9ca9b15ca7ec42154525d9ad86e1f66c61915bc9e7709c39e6861f3e54a97b25`
이 문서의 제작 완료 기록은 ZIP 작성 후 추가했다.
ZIP 내부 문서는 작성 직전 상태다. 코드 파일은 동일하다.

배포 후의 최소 확인은 아래와 같다.
현재 사용자 WSL에 배포한 상태로 해석하지 않는다.

```bash
cd ~/uwb_sim
sha256sum -c uwb_gazebo_sitl_target_modules_20261002.zip.sha256
unzip -o uwb_gazebo_sitl_target_modules_20261002.zip -d uwb-gazebo-equipment
cd uwb-gazebo-equipment
export PYTHONPATH="$PWD/src/drone_uwb:$PWD/src/drone_demo${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -c "from drone_demo.sitl_mission import PX4MissionMonitor; from drone_uwb.integration.sitl_target_contract import reposition_fields; print('Target modules OK')"
```
