# C++ LiDAR 관측 준비 기준
상단 LiDAR를 기존 C++ 설계에 연결하는 기준이다.
동시 로그 수집과 후속 스캔매칭을 준비할 때 읽는다.

## 현재 구현

C++ 관측 경계까지 준비했다. PX4 aiding은 미구현이다.
기존 `CaptureObservations`와 schema 1을 확장했다.
별도 Python 임무 실행기나 companion EKF를 만들지 않았다.

```text
YDLIDAR /scan + /tf + /tf_static
       ↓ 선택 수집 (--with-lidar)
manifest + events.jsonl
       ↓ C++ CaptureObservations
lidar_scan: 원본 frame·시각·거리·유효점 마스크
       ↓ 후속 구현
시각/장착 검증 → 자세 보상 → 상대 스캔매칭
       ↓ 후속 전달 경로 검증
PX4 관측 입력 → PX4 EKF2 → Position 제어

C++ sangwon_AI: 임무·권한·실행 상태
Python: ROS 수집·전송 보조
하방 ToF·IMU: PX4 고도와 자세 관측
```

임무 runtime에는 현재 스캔을 자동 주입하지 않는다.
이유: 장착·정합·추정 오차가 아직 검증되지 않았다.

## 구현 계약

| 항목 | 현재 동작 |
|---|---|
| 원본 기록 | LaserScan 전체, TF 전체를 선택 수집 |
| /tf_static | TRANSIENT_LOCAL 구독, 이미 나온 정적 TF 수신 |
| C++ /scan | sensor frame 보존. 세계 좌표로 간주하지 않음 |
| 기하 | 각도·거리 범위·표본 수·강도 배열 길이 검사 |
| 무효 거리 | NaN/Inf/범위 밖을 null로 구분. 빈 공간으로 해석하지 않음 |
| 시각 | 중복·역행·미래·수신 시 250ms 초과 거부 |
| 광선 시각 | 첫 광선 header와 time_increment 보존. 미래 광선 거부 |
| time_increment=0 | 개별 광선 시각 미확인. 임의 보간 금지 |
| 공백 | 원본 나이+수신 후 경과로 만료 |
| TF 활용 | 원본 기록만. C++ 장착 변환 승인·적용은 후속 |
| 권한 | can_start / physical_output_enabled / px4_aiding_ready=false |

`sample_valid`는 자료 형식·신선도 검사다.
위치 정확도나 스캔매칭 성공을 뜻하지 않는다.
원본 강도 배열은 events에 보존한다.
C++의 현재 요약은 거리·기하·시각을 제공한다.

LaserScan header는 첫 광선 측정 시각이다.
정의는 [ROS 메시지 원문](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/LaserScan.msg)을 따른다.
제조사 드라이버의 실제 시각 의미는 따로 검증한다.
이번 변경은 센서 드라이버를 자동 실행하지 않는다.

## 후속 상대 이동 계약

스캔매칭 관측은 C++ 자료형으로 연결한다.
아래는 구현 전 확정할 필드다.

| 필드 | 필요 근거 |
|---|---|
| source_id / boot_id / reset_counter | 다른 장치·새 지도·재시작 혼합 방지 |
| 시작·끝 측정 시각 | 수신 시각으로 측정 시각을 대체하지 않음 |
| parent / child frame | lidar 센서와 FC 기준점을 구분 |
| Δx / Δy / Δyaw | 두 스캔 사이 관측한 상대 이동 |
| 공분산·정보 행렬 | 벽 방향 편향·통로 퇴화를 표현 |
| 유효점·정합 잔차·겹침률 | 잘못 맞춘 스캔과 누락을 구분 |
| 장착·바닥/스캔 면 근거 | 기울어진 2D 스캔을 평면 이동으로 오인하지 않음 |
| source_lineage | UWB로 보정한 결과를 독립 UWB 검증에 재사용하지 않음 |

상단이라는 위치만으로 장착 TF를 확정하지 않는다.
이전 전방 14cm 기록을 현재 실측값으로 재사용하지 않는다.
임의 TF나 0 공분산으로 준비 상태를 통과시키지 않는다.

## PX4 입력 경계

UWB와 LiDAR가 같은 EV 토픽에 번갈아 발행하면 안 된다.
이유: 원점·시각·센서 기준점이 섞인다.
기존 단일 관측 발행자 계약을 유지한다.

위치 UWB와 속도 LiDAR를 조합하는 방안은 후보일 뿐이다.
동일 측정 시각·frame·상관관계 처리가 필요하다.
XY만 관측한 속도에서 vz를 만들어 넣지 않는다.
현재 PX4 펌웨어의 수용 조건부터 확인한다.
필요하면 별도 aiding 경로의 펌웨어 변경을 검토한다.
기존 roadmap의 PX4 EKF2 융합 책임을 유지한다.
외부 입력 경계는 [PX4 문서](https://docs.px4.io/main/en/ros/external_position_estimation)를 참고한다.

LiDAR를 하방 ToF나 목표 Z 제어로 사용하지 않는다.
광류의 그림자 취약성을 보완하는 수평 관측 후보다.
유리·균일한 복도·움직이는 물체는 별도 평가한다.

## 수집과 검증

운영 배포 후 명시적으로 선택한다.
기존 수집기의 기본 동작은 바꾸지 않았다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1
python3 -m drone_uwb.integration.ros.manual_capture record \
  --tag B --with-lidar --output /private/new-capture --seconds 180
```

부팅 수집기는 `MANUAL_CAPTURE_LIDAR=1`을 지원한다.
기본값은 0이다. 현장 환경 파일은 Git 밖에 둔다.
기존 수집기와 동시에 실행해 중복 기록하지 않는다.
보존할 장착 설정은 기존 `--config`로 함께 기록한다.
미확인 장착값을 생성하는 단계는 없다.

| 검증 | 결과 |
|---|---|
| C++17 GCC, -Wall -Wextra -Wpedantic -Werror | 관측 테스트·재생 실행기 빌드 성공 |
| C++ 관측 계약 시험 | 통과. 무효점·각도·시각·공백·권한 포함 |
| 파일 재생 시험 | 5개 통과 |
| 실제 C3 기록 재생 | 150,694행. LiDAR=MISSING, 출력 권한=false |
| ROS/Python 관련 검사 | 격리 Jetson에서 68개 통과 |
| colcon symlink 빌드 | 격리 Jetson drone_uwb 성공 |
| WSL 전체 CMake 구성 | SQLite3 개발 의존성 부재로 미완료 |
| 실제 상단 LiDAR 동시 기록·스캔매칭·SITL·FC 융합 | 미실시 |

기존 세 공중 기록에는 /scan이 없었다.
이번 변경만으로 그 비행의 LiDAR 분석은 할 수 없다.
실측 장착·개별 광선 시각·독립 이동 대조가 다음 단계다.

## 병합 경계

작업 브랜치는 `codex/mission-flight-flow-20261008`이다.
작업 시작점은 `1a1e73a`다.
조회 시 `origin/feature_uwb=5a3adb0`은 그 조상이었다.
당시 feature 전용 커밋 0개, 작업 브랜치 전용 8개였다.
이번 변경은 분석·지상 세션·수집과 C++ 입력으로 나눠 커밋한다.
main이나 feature_uwb에 실제 병합하지 않았다.

후속 병합에서는 관측 계약 파일의 의미 충돌을 확인한다.
원시 로그·현장 보정·부팅 환경 파일은 Git에 넣지 않는다.
운영 Jetson checkout은 이번 격리 검증으로 교체하지 않았다.

```bash
git fetch origin
git log --oneline origin/feature_uwb..origin/codex/mission-flight-flow-20261008
git diff --stat origin/feature_uwb...origin/codex/mission-flight-flow-20261008
```
