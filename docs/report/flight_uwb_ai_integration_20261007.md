# 2026-10-07 비행·UWB·AI 통합 기록
통합 변경과 시험 근거를 보존하는 보고서다.
코드 검토와 다음 실물 시험을 준비할 때 읽는다.

통합 코드를 빌드하고 Tag B·PX4 수신을 확인했다.
실물 비행이나 UWB 위치 정확도 완료를 뜻하지 않는다.
실제 SITL, FC 외부 위치 융합, 비행은 미실시다.

후속 앵커 0.15m·FC 설정·웹 시험은
[당일 후속 기록](anchor_low015_bench_20261007.md)에 별도로 남겼다.
아래 수치는 최초 통합 당시의 기록이다.

## 브랜치와 기존 작업 보존

| 항목 | 값 |
|---|---|
| 원본 경로 | `/home/arialhanho/Desktop/ROS2` |
| 원본 브랜치·HEAD | `main`, `0257c32` |
| 기존 수정 | `config/udev/99-dronestock.rules` |
| 기존 미추적 작업 | `src/sangwon_AI` |
| UWB 기준 | `7cfc482feee49d82ece1c3987af920587a1a920f` |
| 가져온 최신 main | `deb940f` |
| 통합 merge | `892b3a5` |
| 작업 브랜치 | `codex/flight-uwb-ai-integration-20261007` |
| 통합 경로 | `/home/arialhanho/ROS2-integration-20261007` |

`git fetch origin` 후 별도 worktree를 만들었다.
기존 별도 비행 브랜치는 없었다.
UWB 브랜치를 기반으로 최신 main을 merge했다.
원본의 udev 수정도 그대로 포함했다.
AI 원본 소스 191개를 allowlist로 가져왔다.
Windows의 canonical AI와 191개 해시가 일치했다.
개인 키·실제 임무·runtime·빌드 산출물은 제외했다.

백업은 `~/ros2-integration-backups/20261007`에 있다.
tracked patch, 상태, AI 전체 tar, 소스 해시를 보존했다.
tar 내부 목록은 정상 확인했다.
실행 중 바뀐 runtime 파일과 socket은 원자적 백업이 아니다.
원본 runtime과 관측 서비스는 그대로 유지했다.

기존 사용자 서비스 다섯 개가 실행 중이었다.
core·health·web·perception·PX4 observer다.
기존 실행 환경에 PX4/Gazebo/MAVROS는 없었다.
시험용 MAVROS와 노드는 별도 프로세스로 실행했다.

## 변경 코드

| 경로 | 변경 이유 |
|---|---|
| `drone_uwb/integration/ros/frames.py`, `bridge.py` | B_TF 선택·원본 시각·전체 XY 공분산·좌표 변환·지상 gate |
| `config/runtime/uwb_tag_a/b.yaml` | 기체 ID/domain·미확인 실측값·출력 비활성 |
| `mavros_btf_bench.yaml`, `bench_probe.py` | PX4 XYZ·RC·FC 설정 읽기·반복 조회 |
| `btf_node.py` | launch 정상 종료 시 예외 정리 |
| `integration/ground_audit.py` | 기록의 600초 전송·연속 누락 판정 |
| `src/sangwon_AI` | 기존 AI 전체 소스 편입·PX4 XYZ 구독·기체별 설정 |
| AI deployment helper | workspace 설치본과 기존 설치본 지원 |
| `companion_observe.launch.py` | Tag별 UWB·MAVROS·AI 관측 조립 |
| `drone_bringup/package.xml` | 잘못된 XML 주석 수정·ament 의존성 명시 |
| 기존 bringup Python 13개 | 기존 lint 오류 수정. 비전 계산 로직 유지 |

기존 package.xml 주석에 이중 하이픈이 있었다.
이 때문에 colcon이 일반 Python 패키지로 인식했다.
빌드는 성공해도 ROS 패키지 검색이 실패했다.
수정 후 `ros.ament_python`과 `ros2 pkg prefix`를 확인했다.

bridge는 안테나 기준점을 유지한다.
PX4가 실측 `EKF2_EV_POS_X/Y/Z`로 보정한다.
B_TF의 ToF→안테나 FLU와 별도로 확인한다.
Z는 미관측이며 FC 수평 융합만 허용한다.
미측정 확인값은 모두 false로 유지했다.

## 빌드·단위·합성 시험

| 범위 | 결과 | 증거 파일 |
|---|---|---|
| ROS 패키지 6개 | 빌드 성공 | `initial-build.log`, `final-build.log`, `bringup-ament-build.log` |
| UWB | 337 통과 | `uwb-final-tests.log` |
| 기존 비행 경로 | 116 통과 | `drone_demo-tests.log` |
| 웹 관측 전송 | 14 통과 | `drone_platform_link-tests.log` |
| bringup·QR·정렬 | 57 통과, 1 skip | `bringup-compose-tests.log` |
| AI C++·ROS·웹 | 등록 25개 모두 통과 | `ai-ctest.log`, `ai-ctest-rerun.log`, `ai-ctest-final.log` |

AI는 최초 20개가 통과했다.
5개는 Python 실행 경로 때문에 실행하지 못했다.
재시험에서 3개가 통과했다.
나머지 2개는 subprocess의 websockets 부재로 실패했다.
의존성이 있는 Python을 CMake에 지정했다.
최종 두 시험도 통과했다.
실패·복구 로그를 모두 보존했다.
이것은 단일 25개 연속 실행 기록은 아니다.
bringup skip은 기존 copyright 검사 TODO다.

이번 Jetson에서 사용한 추가 인자는 다음과 같다.

```bash
-DFETCHCONTENT_SOURCE_DIR_BEHAVIORTREE_CPP=/home/arialhanho/Desktop/ROS2/src/sangwon_AI/.build/replay/_deps/behaviortree_cpp-src
-DSANGWON_WEB_PYTHON=/home/arialhanho/Desktop/ROS2/src/sangwon_AI/.venv/bin/python
```

기존 BT 소스의 고정 SHA를 확인했다.
`2a8a226fbbd99f524f0796e0d8a3145773c61c06`이다.
YDLIDAR submodule도 고정 버전을 가져왔다.
`4ef70d3f32a85704ade0be54b214f3763b1ab3e8`이다.
드라이버의 unused-parameter 경고는 남는다.
추가 pymavlink는 통합 checkout의 `.test-deps`에 설치했다.
원본 서비스 가상환경의 패키지는 바꾸지 않았다.

정확한 재실행 명령은 [지상 절차](../runbooks/flight_uwb_ai_ground.md)에 있다.
테스트 원본은 통합 경로의 `.integration-evidence`에 있다.
MAVLink loopback과 ROS 합성 발행은 실제 SITL이 아니다.

## 실물 지상 1차

Tag B/domain 2를 615초 수신했다.
별도 probe는 600초간 기록했다.
FC는 connected, armed=false, AUTO.LOITER였다.

| 입력 | 600초 수신 수 |
|---|---:|
| UWB 기본 XY | 23,360 |
| PX4 local pose | 17,750 |
| PX4 IMU | 29,518 |
| TIMESYNC | 5,942 |
| downward_0 | 5,857 |
| B_TF XY | 0 |
| vision_pose/pose_cov 출력 | 0 |

원본 유효 4거리+TDMA는 평균 39.393 Hz였다.
그러나 1초 창 최소값은 0 Hz였다.
600개 창 중 14개가 35 Hz에 못 미쳤다.
최대 연속 누락은 113개였다.
따라서 10분 전송 인수 기준은 실패다.
동시에 빌드·시험이 진행됐지만 원인으로 단정하지 않는다.
UART JSON 파싱 오류도 31개였다.

B_TF는 `tof_to_tag_mount_unconfirmed`로 보류했다.
평면 바닥 확인도 false다.
마지막 PX4 XYZ는 약 (0.162,-16.921,0.527)m였다.
이는 FC local ENU이며 창고 좌표가 아니다.
ToF 거리와 PX4 Z는 서로 다른 기준을 가진다.

반복 bridge 조회의 FC 값은 다음과 같았다.
EV_CTRL=0, EV_DELAY=0, EV_NOISE_MD=0이다.
EV_POS_X/Y/Z도 모두 0이었다.
FC 값은 변경하지 않았다.
probe 초기 일회 조회의 null 문제는 반복 조회로 수정했다.

## 대략 배치와 후속 확인

사용자가 제시한 방향은 A1 원점·A2 +X·A3 +Y다.
크기는 약 4~5×5m이며 높이는 무작위다.
기존 설정은 6.3×4.6m, 높이 2.2m다.
이 차이를 측정 완료로 처리하지 않았다.
재연결 원본에서 ID 6과 네 거리 응답을 확인했다.
한 표본의 거리는 4.341/2.614/3.498/3.698m였다.
태그 자체 XY는 `inconsistent_ranges`로 무효였다.
거리 수신 성공과 유효한 XY는 다른 결과다.

통합 실행의 30초 AI 보고에서 domain 2를 확인했다.
state·extended_state·RC·local_position은 LIVE였다.
배터리 입력은 INVALID_MAVROS_MESSAGE였다.
잘못된 값을 정상으로 대체하지 않았다.
can_start·flight_authority·physical_output은 false다.

## 실물 지상 2차·LiDAR

빌드·시험이 끝난 뒤 통합 launch로 재측정했다.
Tag B 수신 615초와 probe 600초를 기록했다.
원본 유효 4거리+TDMA는 평균 38.682 Hz였다.
1초 창 최소는 0 Hz, 미달 창은 23개였다.
최대 연속 누락은 64개로 인수 기준에 실패했다.
따라서 원인을 빌드 부하만으로 설명할 수 없다.

| 항목 | 2차 결과 |
|---|---:|
| 원본 600초 유효 4거리+TDMA | 23,209 |
| probe 기본 XY | 185 |
| probe B_TF XY | 10 |
| probe PX4 local pose | 17,813 |
| probe downward_0 | 5,886 |
| probe IMU | 29,572 |
| probe TIMESYNC | 5,974 |
| FC 관측 전달 | 0 |

B_TF 10개는 `original_fit_consistent`였다.
기존 알고리즘은 일치한 네 거리 결과를 그대로 쓴다.
불일치 후보 22,012개는 높이 부재로 보류했다.
장착 확인값은 여전히 false였다.
따라서 10개 발행은 ToF 장착 검증의 증거가 아니다.
bridge의 출력은 계속 비활성이었다.

최종 PX4 XYZ는 약 (0.167,-16.933,0.511)m였다.
probe의 원본 나이는 약 45ms였다.
30초·590초 AI 보고 모두 위치를 LIVE로 표시했다.
배터리는 WARN, 비행 권한은 false를 유지했다.
FC 파라미터는 반복 probe 조회로도 읽었다.
EV_CTRL=0과 장착·지연 값이 1차와 같았다.

UART 파싱 오류는 56개였다.
원시 바이트에 중간에서 잘린 JSON을 확인했다.
첫 부분 행은 연결 시 절단으로 볼 수 있다.
그 이후에도 끊기거나 합쳐진 행이 있었다.
해당 시간의 커널 로그에서 USB reset은 찾지 못했다.
펌웨어 출력 직렬화·UART/USB·호스트 지연을 분리 진단해야 한다.
임의로 복원한 JSON을 유효 측정으로 넣지 않았다.

LiDAR는 뒤이어 독립적으로 30초 실행했다.
실제 장치는 Tmini Pro로 응답했다.
`/dev/lidar:230400`에서 scan 262개를 받았다.
첫·마지막 scan 기준 9.979 Hz였다.
각 scan의 유효 거리는 최소 367개였다.
시작 시 checksum 오류 6개와 점 개수 경고가 있었다.
SDK가 intensity bit를 조정한 뒤 수신을 이어갔다.
이 짧은 수신은 장착 TF·위치 정확도·경로계획 증거가 아니다.

통합 launch와 probe는 정상 종료했다.
LiDAR도 해당 프로세스에만 종료 신호를 보냈다.
최종 확인에서 세 직렬 포트의 시험 소유자는 없었다.
원본 AI 191개 해시는 백업 시점과 같았다.
원본 서비스 5개도 최초 PID와 active 상태를 유지했다.

요약값과 원본 증거 해시는
[기계 판독 기록](evidence/flight_uwb_ai_20261007.json)에 있다.

## 남은 검증

| 범위 | 상태 |
|---|---|
| 실제 SITL | Jetson에 px4/Gazebo 없음. 미실시 |
| Windows WSL | Ubuntu-22.04 중지. 기존 환경을 임의 시작하지 않음 |
| FC 외부 관측 융합 | EV_CTRL=0, 비활성. 미검증 |
| ARM·이륙·호버·이동 | 이번 작업에서 미실시 |
| 앵커 배치·거리 편향 | 실제 XYZ·ID·다점 오차 실측 필요 |
| 장착 기준점 | ToF→안테나 FLU와 FC→안테나 FRD 각각 필요 |
| 시간·방향 | 지연·시계 정렬·창고→ENU 실측 필요 |
| 카메라·바코드 | 현물 라벨·교정·스캔 창/원본 시각 연결 필요 |
| LiDAR | 약 10Hz 수신. 시작 checksum·장착 TF·경로 반영 별도 |
| AI 실제 출력 | 물리 writer·명령 권한 중재·RC 반환 검증 필요 |
| 부팅 | 원본 5개 관측 서비스 보존. 통합판 cold boot 미실시 |
| 웹·LoRa | 이번 실제 웹 PC/LoRa 공동 시험 미실시 |

현재 구현과 책임은 [통합 기준](../architecture/flight_uwb_ai_integration.md)을 따른다.
