# 공용 데모 기준
이 문서는 장비 없이 쓰는 데모의 기준이다.
새 기능에 시험 입력을 연결할 때 읽는다.

## 범위

데모 입력을 한 패키지에서 가져다 쓴다.
패키지 이름은 `drone_demo`다.
실행 순서는 [데모 실행 절차](demo_procedure.md)를 따른다.
도착·수신 공백 판단은 [임무 상태 기준](demo_mission_design.md)을 따른다.

```text
데모 설정 + 기존 앵커 좌표
              |
         가상 이동 경로
          /          \
 데모 위치·목표점   가상 UWB 거리
                         |
                  기존 UWB 전처리
                         |
                    /uwb_pose
          \              /
            개발 중인 기능
```

가상 이동은 시간에 맞춘 직선 이동이다.
목표점은 데모가 함께 제공한다.
제어기의 행동으로 이동하는 물리 모델은 아니다.
EKF2·보상·비행 안정성 검증은 SITL에서 한다.
이유: 이 데모에는 추력·관성·자세 동역학이 없다.

## 공용 설정

임시값은 `config/demo.json`에서 바꾼다.

| 항목 | 첫 기본값 | 의미 |
|---|---|---|
| `z_min_m` / `z_max_m` | 0.2m / 2.2m | 가상 z 범위. 실측값 아님 |
| `z_period_s` | 8초 | 사인 파형 한 주기 |
| `start_xy_m` | [2.09, 1.68]m | 기존 기준점에서 가져온 시작 위치 |
| `target_xy_m` | [3.09, 2.68]m | 임의의 시험 목표점 |
| `rate_hz` | 40Hz | 가상 거리 주기. 실측 측정률 아님 |
| `duration_s` | 12초 | 실행 뒤 자동 종료 |
| `start_hold_s` | 2초 | 이동 전 대기 |
| `speed_m_s` | 0.25m/s | 가상 경로의 이동 속도 |
| `range_noise_stddev_m` | 0.02m | 예시 거리 잡음. 실측 교정값 아님 |
| `seed` | 7 | 동일 입력 재현용 값 |
| 앵커 높이 | 2.2m | 기존 잠정 배치 파일 사용 |

z는 1.2m에서 시작해 2.2m, 0.2m를 순서대로 지난다.
식은 1.2 + sin(2πt/8) m이다.
실제 ToF 측정이나 PX4 고도 추정값이 아니다.
유효한 `sensor_msgs/Range` 입력이 `/tof/range`에 들어오면
데모 노드는 가상 위치·목표·UWB 발행을 모두 종료한다.
토픽은 `real_tof_topic` 실행 인자로 바꿀 수 있다.
수동 차단에는 `synthetic_z_enabled:=false`를 쓴다.

앵커는 기존 불규칙 배치를 사용한다.
직사각형이나 비행 허용 구역으로 간주하지 않는다.
새 앵커 파일을 만들며 좌표를 중복 관리하지 않는다.
이유: 실측 좌표의 기준 파일을 유지하기 위해서다.

## 시나리오

한 설정에서 세 가지 입력을 재현한다.

| 이름 | 위치 변화 | UWB 입력 |
|---|---|---|
| `stationary` | 시작 위치 유지 | 연속 거리. 목표도 시작 위치 |
| `move` | 2초 후 목표까지 직선 이동 | 연속 거리 |
| `gap` | `move`와 같은 경로 | 4초 이상, 5초 미만에 수신 중단 |

수신 공백에는 새 UWB 좌표를 발행하지 않는다.
가상 위치는 데모의 정답으로만 계속 나온다.
그 정답을 공백 보완 추정 결과로 해석하지 않는다.
이유: 센서 없이도 생성기가 알고 있는 값이다.

## ROS2 입력 계약

ROS2 데모는 DOMAIN_ID 99에서만 실행한다.
1호기·2호기의 DOMAIN_ID 1·2와 구분한다.
같은 PC의 시험 노드는 `ROS_LOCALHOST_ONLY=1`을 쓴다.
이유: 가상 목표가 실기 임무 입력과 섞이지 않게 한다.

| Topic | 메시지 | 값의 의미 |
|---|---|---|
| `/demo_pose` | `PoseStamped` | 생성기가 아는 위치. z는 가상 사인 파형 |
| `/target_pose` | `PoseStamped` | 데모 목표. 고도 제어 명령 아님 |
| `/uwb/raw` | `String` JSON | 가상 RAW 계약. `demo=true` 포함 |
| `/uwb_pose` | `PoseWithCovarianceStamped` | 기존 전처리를 통과한 x·y |
| `/demo_status` | `String` JSON | 시나리오·시간·거부 사유·출처 |

모든 위치의 좌표계는 `uwb_map`이다.
이는 PX4 ENU와 정렬했다는 뜻이 아니다.
`/uwb_pose`의 z는 미관측 자리값 0이다.
z·회전 분산은 기존 노드와 같은 1e6이다.
임시 높이를 UWB의 고도 관측으로 넣지 않는다.
이유: 같은 높이의 앵커로 계산하는 관측은 x·y다.

`/target_pose`는 목표를 한 번 저장 발행한다.
늦게 연결한 구독자는 transient local QoS를 쓴다.
`/uwb_pose`와 `/uwb/raw`는 sensor data QoS다.
나머지 데모 상태는 reliable QoS다.

MAVROS·PX4 브리지는 이 데모에서 기동하지 않는다.
시리얼 포트와 서버에도 접속하지 않는다.
이유: 개발용 입력 생성에 실기 연결은 필요 없다.

## 다음 단계에서 가져다 쓸 부분

입력 출처를 필요한 시점에 하나씩 교체한다.

| 개발 단계 | 데모에서 재사용 | 실제 입력으로 전환할 때 |
|---|---|---|
| 거리·목표 판정 | 데모 위치와 목표 | 현재 위치는 PX4 추정값 사용 |
| UWB 공백 대응 | `gap`와 상태 기록 | 실제 수신 나이·결손으로 교체 |
| UWB 전처리 | 기존 `Processor`와 가상 RAW | 실제 `uwb_node`로 교체 |
| 고도 표시 | 가상 z, `demo_sine` 출처 표시 | 검증한 PX4 고도와 출처 사용 |
| LiDAR 기능 | 입력 연결 자리를 추후 추가 | 실제 T-mini Pro `/scan` 사용 |
| IMU·EKF2·보상 검증 | 시나리오와 평가 조건 | PX4 SITL의 물리·센서 모델 사용 |

이번 버전은 IMU·LiDAR 값을 만들지 않는다.
없는 센서의 신뢰도를 숫자로 채우지도 않는다.
이유: 해당 측정과 모델은 아직 연결하지 않았다.
T-mini Pro의 수평 스캔을 하방 고도로 쓰지 않는다.
이유: 측정 평면과 높이 방향이 다르다.

## 근거

프로젝트의 기존 제어 책임을 유지한다.

- [로드맵](roadmap.md) 결정 2·4·7·10.
- [고도 원칙](altitude_policy.md).
- [기존 앵커 좌표](uwb_anchor_survey.md).
- [9월 13일 서버 수신값](report/platform_connection_20260913.md).
- [ROS2 Humble Domain ID 공식 원문](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Domain-ID.rst).
- [ROS2 Humble QoS 공식 원문](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).
