# UWB·MAVROS·PX4 파라미터 연결

위치 전달 경로와 파라미터 API의 설명서다. FC 설정을 조회하거나 UWB 연결 상태를 확인할 때 읽는다.

## 연결 구조

MAVLink는 FC와 통신하는 규약이다.
우리 ROS 2 코드는 MAVROS의 토픽·서비스를 사용한다.
MAVROS가 이를 MAVLink 메시지로 처리한다.

```text
위치 관측 경로
UWB RAW -> uwb_node -> /uwb_pose
                         |
                   uwb_px4_bridge
                 좌표·시각·설정 검사
                         |
            /mavros/vision_pose/pose_cov
                         |
               MAVROS vision_pose 플러그인
                         |
              MAVLink VISION_POSITION_ESTIMATE
                         |
                   PX4 위치 추정

설정 조회 경로
ROS 2 서비스 -> MAVROS param 플러그인
                         |
              MAVLink 파라미터 프로토콜
                         |
                   PX4 파라미터
```

관측 토픽에는 계속 변하는 위치·시각·공분산을 보낸다.
파라미터에는 어떤 관측을 융합할지 등의 설정을 저장한다.
좌표를 `EKF2_EV_CTRL` 같은 파라미터에 쓰는 방식이 아니다.
현재 A 파일 계산 결과는 위 실시간 토픽에 연결하지 않았다.

MAVROS 2.14.0의 `pose_cov` 구독·메시지 변환을 확인했다.
ENU→NED 변환도 이 플러그인이 수행한다.
근거: [공식 vision_pose 소스](https://github.com/mavlink/mavros/blob/2.14.0/mavros_extras/src/plugins/vision_pose_estimate.cpp).

## 확인한 버전과 범위

2026-09-27에 설치 파일·코드·저장 로그를 읽었다.

| 대상 | 확인값 |
|---|---|
| 현재 설치 MAVROS | 2.14.0, ROS 2 Humble |
| 9월 26일 ULog 펌웨어 | 버전 바이트 `1,17,0,255` |
| ULog 펌웨어 커밋 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 과거 9월 6일 시험 | PX4 1.16.0 기록. 이번 ULog와 구분 |
| 현재 기체에 직접 질의 | 미실시 |
| 파라미터 변경 | 미실시 |

이 설명의 FC 파라미터는 ULog의 커밋 정의를 우선 대조했다.
현재 기체 버전·실행 설정은 별도로 조회해야 한다.

## 현재 브리지가 확인하는 설정

우리 브리지는 FC 설정 세 개를 읽어서 전달 여부를 판단한다.
FC 설정을 자동으로 변경하지 않는다.

| PX4 파라미터 | 의미 | 현재 코드의 전달 조건 | 9월 26일 로그 |
|---|---|---|---|
| `EKF2_EV_CTRL` | 외부 위치의 융합 항목 | 정확히 `1`: 수평 위치만 | `0` |
| `EKF2_EV_DELAY` | IMU 대비 외부 관측 지연, ms | `expected_ev_delay_ms`와 일치 | `0.0` |
| `EKF2_EV_NOISE_MD` | 관측 불확실성 선택 방식 | `0`: 메시지 분산 사용, 파라미터 하한 적용 | `0` |

`EKF2_EV_CTRL`은 비트 조합이다.
값 `1`, `2`, `4`, `8`은 각각 수평·수직·속도·yaw다.
우리 코드의 `1` 검사는 수평 관측만 받으려는 조건이다.
이 표는 현재 값을 변경하라는 지시가 아니다.
공식 의미는 [해당 펌웨어 정의](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/params_external_vision.yaml)에 있다.

추가로 확인할 설정도 있다.

| PX4 파라미터 | 확인 목적 | 9월 26일 로그 |
|---|---|---|
| `EKF2_EVP_NOISE` | 외부 위치 불확실성의 하한·대체값, m | 약 `0.1` |
| `EKF2_EV_POS_X/Y/Z` | 외부 위치 센서의 기체 기준 장착 위치, m | 모두 `0` |
| `EKF2_GPS_CTRL` | GNSS 융합 허용 항목 | `7` |
| `EKF2_OF_CTRL` | 광학흐름 융합 허용 | `1` |
| `EKF2_RNG_CTRL` | 하방 거리 융합 방식 | `1`: 조건부 |
| `EKF2_HGT_REF` | 높이 기준 센서 선택 | `1` |

GNSS·광학흐름·거리 설정의 정의는 각 공식 소스와 대조했다.
[GNSS](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/params_gnss.yaml),
[광학흐름](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/params_optical_flow.yaml),
[거리 센서](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/params_range_finder.yaml)를 참고한다.
높이 기준의 일반 설명은 [공식 파라미터 문서](https://docs.px4.io/main/en/advanced_config/parameter_reference#EKF2_HGT_REF)에 있다.
융합 허용 설정과 실제 융합 동작은 별도로 확인한다.

ROS 쪽 브리지 설정은 별도다.
`enabled`·좌표·시각·장착 확인 값은 현재 모두 `false`다.
근거: [uwb.yaml](../src/drone_uwb/config/uwb.yaml).
`EKF2_EV_CTRL` 하나만 바꿔도 자동 전달되는 구조는 아니다.
코드의 판정은 [frames.py](../src/drone_uwb/drone_uwb/frames.py)에 있다.

## 파라미터 API

설치된 ROS 2 인터페이스와 MAVROS 2.14.0 소스를 확인했다.

| 목적 | ROS 2 서비스 | 서비스 형식 |
|---|---|---|
| FC 값 다시 받기 | `/mavros/param/pull` | `mavros_msgs/srv/ParamPull` |
| 저장된 값 읽기 | `/mavros/param/get_parameters` | `rcl_interfaces/srv/GetParameters` |
| 이름 목록 읽기 | `/mavros/param/list_parameters` | `rcl_interfaces/srv/ListParameters` |
| 값 변경 API | `/mavros/param/set_parameters` | `rcl_interfaces/srv/SetParameters` |
| MAVROS 전용 변경 API | `/mavros/param/set` | `mavros_msgs/srv/ParamSetV2` |

`get_parameters`는 MAVROS의 로컬 보관 값을 반환한다.
매번 FC에 새 읽기 요청을 보내는 API가 아니다.
`pull`의 `force_pull=true`는 FC 목록을 다시 가져온다.
값 변경 API는 이번 작업에서 호출하지 않았다.
근거: [공식 param 플러그인](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/param.cpp).

설치된 `mavros_msgs/srv/ParamGet`은 deprecated 표시가 있다.
현재 ROS 2에서는 위 표의 표준 파라미터 API를 사용한다.
기존 ROS 1 예제의 `/mavros/param/get` 호출을 그대로 옮기지 않는다.
이유: 현재 플러그인이 제공하는 서비스 형식이 다르다.

MAVLink 계층의 메시지는 다음과 대응한다.

| 메시지 | 역할 |
|---|---|
| `PARAM_REQUEST_LIST` | 전체 목록 요청 |
| `PARAM_REQUEST_READ` | 개별 값 요청 |
| `PARAM_VALUE` | 값 응답·변경 통지 |
| `PARAM_SET` | 값 변경 요청 |

근거: [MAVLink 파라미터 프로토콜](https://mavlink.io/en/services/parameter.html).
MAVROS의 조회 캐시와 MAVLink 개별 요청을 같은 동작으로 보지 않는다.

## 기체 연결 환경에서 읽는 명령

기존 MAVROS가 FC에 연결된 터미널에서 실행한다.
아래 명령은 파라미터를 변경하지 않는다.
이번 작업에서는 서비스 형식만 확인했다.
기체 대상 호출 결과를 확보한 것은 아니다.

```bash
source /opt/ros/humble/setup.bash

ros2 service list -t | rg '/mavros/param/'

ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull \
  '{force_pull: true}'
```

`success: true`와 수신 개수를 확인한 뒤 조회한다.

```bash
ros2 service call /mavros/param/get_parameters rcl_interfaces/srv/GetParameters \
  "{names: ['EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD', 'EKF2_EVP_NOISE', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z', 'EKF2_GPS_CTRL', 'EKF2_OF_CTRL', 'EKF2_RNG_CTRL', 'EKF2_HGT_REF']}"
```

응답 `values`는 요청 이름과 같은 순서다.
`type=2`는 `integer_value`, `type=3`은 `double_value`를 읽는다.
`type=0`이면 보관된 값이 없는 상태다.
실제 파라미터가 0이라는 뜻으로 해석하지 않는다.

하나만 읽는 간단한 형태도 있다.

```bash
ros2 param get /mavros/param EKF2_EV_CTRL
ros2 topic echo /mavros/state --once
ros2 topic echo /uwb/bridge_status --once
```

MAVROS의 이름 공간이 다르면 경로를 맞춰야 한다.
우리 코드는 `/mavros`를 기준으로 작성돼 있다.
브리지는 약 1초마다 세 설정의 캐시를 조회한다.
최근 캐시 조회 성공이 FC 전체 값 재수신을 뜻하지 않는다.
근거: [bridge.py](../src/drone_uwb/drone_uwb/bridge.py).

## 9월 26일 로그에서 추가 확인한 사실

이 파일에서는 외부 위치 융합이 꺼져 있었다.
현재 기체나 최근 보고한 호버링의 설정으로 단정하지 않는다.

| 항목 | 저장 로그에서 관찰한 값 |
|---|---|
| `EKF2_EV_CTRL` | `0`. 로그 내 해당 설정 변경 기록 없음 |
| `cs_ev_pos/hgt/vel/yaw` | 기록된 두 EKF 인스턴스에서 모두 `0` |
| `cs_opt_flow` | 기록된 두 인스턴스에서 모두 `1` |
| `cs_gnss_pos` | 기록된 두 인스턴스에서 모두 `0` |
| `cs_rng_hgt` | `0`과 `1` 모두 기록 |
| `vehicle_status.nav_state` | 137개 표본 모두 `1` |
| `nav_state_user_intention` | 137개 표본 모두 `1` |

해당 펌웨어에서 `nav_state=1`은 `ALTCTL`이다.
Position 모드는 `2`다.
근거: [같은 커밋의 VehicleStatus 정의](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleStatus.msg).
융합 플래그는 [EstimatorStatusFlags 정의](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/EstimatorStatusFlags.msg)와 대조했다.

사용자가 보고한 Position 호버링 성공은 별도 이력으로 유지한다.
이 파일이 그 성공 시험을 기록했다고 확인하지 못했다.
[추출값·파일 해시](report/evidence/mavros_parameter_api_20260927/ulog_parameters.json)에 근거를 보존했다.
