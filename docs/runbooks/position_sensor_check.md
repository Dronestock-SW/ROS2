# UWB·옵티컬 플로우·ToF·IMU 점검
실물 센서의 수신과 PX4 융합을 나눠 확인하는 절차다.
위치 유지·직진 시험 전에 읽는다.

센서가 연결됐다고 위치 융합이 완료되지는 않는다.
현재 UWB→PX4 전달과 장착·정렬 검증은 미완료다.
수신·융합·방향·독립 오차를 순서대로 확인한다.

```text
UWB 네 거리 -> B_TF -> 안테나 XY
                ^         |
          ToF + IMU 자세   | 좌표·시각·레버암 확인
                          v
flow + ToF + IMU ------> PX4 EKF2 <--- UWB 수평 관측
                          |
                    위치·속도·자세
                          |
              목표 방향·횡방향 편차·도착 판정
```

| 입력 | 담당 역할 | 이번 코드의 상태 |
|---|---|---|
| UWB | 지도 기준 수평 위치 | B_TF 계산 있음. FC 전달 기본 잠김 |
| ToF+IMU 자세 | 거리 기하와 안테나 높이 보정 | 장착값·평면 검증 후 사용 |
| flow+거리 | 수평 속도·짧은 구간 이동 | PX4가 융합. companion 독립 적분 없음 |
| IMU | 자세·속도 예측·회전 성분 보정 | PX4 전담. B_TF는 자세만 사용 |
| 지도 정렬 | 지도 +X/+Y와 FC 좌표 대응 | 실측 전 확인값 false |
| 비행 제어 | 위치 유지·고도·자세 | PX4 전담 |

flow로 UWB 원시 좌표를 덮어쓰지 않는다.
이유: 독립 관측의 편향·불일치 근거를 보존해야 한다.
융합된 PX4 위치와 UWB 원본을 별도로 기록한다.
IMU 자세 안정과 지도상의 직진은 다른 검사다.

## 1. FC 읽기

MAVROS 실행 중에는 ROS 경유 읽기를 사용한다.
실행기·센서 수집·직렬 포트 소유자를 유지한다.
고정된 `param show`·`listener`만 요청한다.
매 요청 직전 FC 연결·disarm·상태 시각을 검사한다.
명령 누락·불완전한 응답에서는 즉시 중지한다.
이유: 이전 응답을 다음 항목에 붙이면 오판한다.

```bash
cd /home/arialhanho/ROS2-review-20261008-codex
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1
PYTHONPATH=/home/arialhanho/ROS2-integration-20261007/.test-deps:$PYTHONPATH \
python3 src/sangwon_AI/ops/px4_sensor_readback.py \
  --ros-domain 2 --output .review/sensor-readback-ros.json
```

기본 UAS1·FC system 1/component 1을 대조한다.
다른 기체에 확인 없이 주소를 재사용하지 않는다.
`/uas1/mavlink_source`를 구독한다.
`/uas1/mavlink_sink`에는 진단용 SERIAL_CONTROL만 보낸다.
파라미터 변경·ARM·이륙·모드 명령은 허용하지 않는다.
셸 release 패킷은 빈 데이터만 허용한다.
원시 MAVLink 큐는 고속 메시지에 대비해 1024개다.
이 수신은 시각 보정이나 비행 승인으로 세지 않는다.

직렬 직접 읽기는 포트 소유자가 없을 때만 쓴다.
`--ros-domain`을 생략하면 기존 직렬 경로다.
포트를 중복으로 열지 않는다.
이유: 읽기 충돌도 센서·명령 링크를 훼손할 수 있다.

```bash
cd /home/arialhanho/ROS2-review-20261008-codex
fuser /dev/pixhawk
PYTHONPATH=/home/arialhanho/ROS2-integration-20261007/.test-deps \
python3 src/sangwon_AI/ops/px4_sensor_readback.py \
  --output .review/sensor-readback.json
```

이 도구는 disarmed 상태를 요구한다.
고정된 `param show`·`listener`만 요청한다.
시동·모드·파라미터 쓰기 기능은 없다.
불완전한 셸 응답은 다음 쿼리에 붙이지 않는다.
여러 EKF 인스턴스가 있으면 각각 읽는다.

2026-10-09 받침대 정지 상태에서 36건을 읽었다.
flow와 거리 융합은 두 EKF 인스턴스 모두 확인했다.
EV 관측·융합은 없었다. [현장 기록](../report/field_readiness_20261009.md)을 따른다.

| 단계 | 읽을 항목 | 확인할 내용 |
|---|---|---|
| 설정 | `EKF2_OF_CTRL`, `EKF2_RNG_CTRL`, `EKF2_EV_CTRL` | 허용값. 융합 완료 증거와 구분 |
| flow 수신 | `sensor_optical_flow`, `vehicle_optical_flow` | 시각·품질·적분 시간·회전·거리 가용성 |
| 거리 수신 | `distance_sensor` 각 인스턴스 | 실제 거리·min/max·방향·품질 |
| 융합 | `estimator_status_flags` | `cs_opt_flow`, `cs_rng_hgt`, `cs_ev_pos` |
| 실제 갱신 | 각 `estimator_aid_src_*` | `fused`, `time_last_fuse`, innovation |
| UWB 전달 | `vehicle_visual_odometry` | 최신 원본 시각·수평 관측·공분산 |
| FC 출력 | `vehicle_local_position`, `vehicle_attitude` | 유효 위치·속도·heading·원점 |

2026-10-08 지상 조회에서는 flow 수신이 있었다.
`cs_opt_flow=false`, `cs_rng_hgt=true`였다.
외부 위치 토픽은 발행되지 않았다.
ToF는 1~3mm로 센서 최소 0.10m보다 작았다.
바닥 가림·기체 높이·거리 유효성을 먼저 대조한다.
이 상태만으로 공중 flow 고장을 판정하지 않는다.

PX4는 flow 사용 시 바닥 높이 범위도 검사한다.
범위를 벗어나면 품질값이 있어도 융합하지 않는다.
근거: [해당 펌웨어의 flow 융합 조건](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp).

## 2. 모터 없는 방향·거리 시험

프로펠러 제거·배터리 분리 상태로 먼저 시험한다.
기체를 고정 지그에서 센서 유효 높이에 놓는다.
측정된 높이와 ToF를 비교한다.
바닥 무늬·조명·센서 가림도 함께 기록한다.

| 시험 | 독립 기준 | 기록할 비교 |
|---|---|---|
| 정지 | 지그·바닥 표식 | flow 속도·IMU 자세·UWB XY의 시간 추세 |
| +X 이동 | 표식 사이 0.5m | 지도 XY·PX4 ENU 부호·직교축 변화 |
| +Y 이동 | 표식 사이 0.5m | 같은 비교. X/Y 축 바뀜 검사 |
| 방향 회전 | 표시한 두 기체 방향 | heading·지도 회전·flow body 축 대응 |
| 왕복 | 같은 시작 표식 | 원점 복귀·누적 편향·UWB 재정착 |

실측 장착 벡터와 좌표 회전을 저장한다.
검증되지 않은 0 벡터·0도는 그대로 잠근다.
원시 거리·ToF·자세·FC 융합을 같은 시각에 기록한다.
앵커가 꺼져 있으면 UWB 단계는 미실시다.

## 3. 위치 유지와 직진 시험

기본 이착륙 검증 후 단일 짧은 구간으로 늘린다.
전체 이동 미션을 첫 실물 시험에 합치지 않는다.
이유: 현재 독립 위치 오차와 융합 검증이 남아 있다.

직진 판정은 목표선에 대한 횡방향 편차를 사용한다.
기체 heading만으로 이동 방향을 판정하지 않는다.
UWB·PX4 궤적과 독립 바닥 표식을 함께 대조한다.
자세·속도·높이는 PX4가 제어한다.
실물 허용 오차는 독립 측정 결과로 확정한다.
[통합 기준](../architecture/flight_uwb_ai_integration.md)과
[기본 이착륙 절차](native_hover_test.md)를 따른다.
