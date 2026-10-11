# GPS 없는 실내 UWB 수평 융합 절차
GPS·지리적 원점 없이 UWB를 PX4에 넣는 절차다.
현재 관측 서비스를 재시작하거나 복구할 때 읽는다.

수평 위치는 UWB를 융합한다. 고도·자세는 PX4가 맡는다.
현재 지상 확인 결과는 [10월 11일 기록](../report/local_uwb_fusion_20261011.md)에 있다.
이 절차는 시동·이륙·자동 경유지 실행을 하지 않는다.

## 현재 연결

```text
실내 앵커 A1..A4 -> Tag B 거리·원본 시각
                          |
                     기존 B_TF
                          | 안테나 XY·원본 시각·공분산
                          v
              고정 실내 축 -> ENU -> NED
                          |
           신선도·시계·점프·입력 독점 검사
                          |
                 MAVLink ODOMETRY
                          |
              기존 MAVROS router -> PX4 EKF2
                                      | UWB: 수평만
                                      | 기존 IMU/자기계: 자세
                                      | 기존 range/baro/IMU: 고도
                                      v
                             PX4 local_position
```

A1은 실내 XY 원점이다. 위도·경도는 지정하지 않는다.
PX4가 사용하는 수평 축에 맞추는 회전은 필요하다.
회전과 지리적 원점 설정은 서로 다른 과정이다.
FC 자세는 최초 축 정렬에서만 한 번 읽는다.
상시 UWB 관측에 FC 위치·자세를 복사하지 않는다.

| 정할 값 | 현재 값·근거 |
|---|---|
| 앵커 XYZ | `(0,0,.15)`, `(6.3,0,.15)`, `(0,4.6,.15)`, `(6.3,4.6,.15)` m |
| 입력 기체 | Tag B=6, ROS domain=2 |
| 실내 축 방향 | +X=A1→A2, +Y=A1→A3 |
| 실내 XY 부호 | `map_y_axis_sign=-1`; 기존 현장 배치 기준 |
| 최초 기수 | 사용자 확인 −Y=A3→A1 |
| 지도→ENU 회전 | 이번 부팅 `enu_yaw_deg=-44.3699921468` |
| 수평 원점 이동 | ENU offset=(0,0) m; A1 유지 |
| FC→안테나 | FRD `(-.10,0,-.10)` m; 10월 9일 실측 기록 |
| 관측 나이 | 원본 시각부터 최대150ms |
| 융합 선택 | `EKF2_EV_CTRL=1`; 수평 위치만 |
| 관측 분산 | B_TF XY 공분산 사용; 현재 각0.09m² |
| 지연 | `EKF2_EV_DELAY=0` 유지; 동적 지연 교정 미실시 |

이 회전값은 이번 부팅의 정렬 근거다.
다른 설치·부팅에 자동 복사하지 않는다.
이유: PX4 방향 기준이 달라질 수 있다.

## 재시작·부팅 뒤 순서

1. 기존 MAVROS와 B_TF 수신을 확인한다.
   DISARM·ON_GROUND에서 설정한다.
2. 드론 앞쪽을 확인한 실내 축 방향에 맞춘다.
   설정 중에는 기체를 움직이지 않는다.
3. 새 파일명으로 설정 근거를 수집한다.
   현재 장착·앵커 값이 같을 때만 아래 도구를 쓴다.
4. 적용 ACK 뒤 FC 내부 값을 조회한다.
   저장 여부·실제 융합은 ACK와 구분한다.
5. 관측 서비스를 시작한다.
   현재 FC·host 부팅 근거가 맞아야 발행한다.

연결 호스트의 `bluewak` 사용자에서 실행한다.
아래 `새파일명`을 사용한 적 없는 이름으로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
cd /home/bluewak/ROS2-local-uwb-20261011
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=0
export PYTHONPATH=/home/arialhanho/ROS2-integration-20261007/.test-deps:$PYTHONPATH
python3 src/drone_uwb/tools/configure_local_uwb.py \
  --output /home/bluewak/local-uwb-20261011/새파일명.json \
  --heading=-Y --apply
bash src/drone_uwb/deployment/install_local_uwb_user_service.sh \
  "$PWD" /home/bluewak/local-uwb-20261011/새파일명.json
```

기수가 다른 축이면 `--heading` 값을 바꾼다.
선택값은 `+X`, `-X`, `+Y`, `-Y`다.
비스듬한 방향을 가장 가까운 축으로 임의 승인하지 않는다.
정확한 축 정렬·이동 시험이 필요하다.

설정 도구의 쓰기 범위는 네 파라미터다.
EV POS X/Y/Z와 EV CTRL만 쓴다.
GPS·고도·자기계·flow·range 파라미터는 쓰지 않는다.
EV POS Z는 장착 벡터다. 고도 관측·목표가 아니다.

## 확인·중단·복구

아래 신호로 실제 FC 융합을 확인한다.
토픽 발행만으로 성공을 판정하지 않는다.

| 신호 | 필요한 결과 |
|---|---|
| `/uwb/local_odometry_status` | gate=ready, published 증가, 최근 시각 |
| `vehicle_visual_odometry` | NED XY·원본 시각, q/velocity=NaN |
| `estimator_aid_src_ev_pos` | fused=true, time_last_fuse 증가 |
| `estimator_status_flags` | cs_ev_pos=true; ev_hgt/yaw/vel=false |
| `vehicle_local_position` | xy/z/velocity valid=true |
| 지리적 원점 | ref_timestamp=0, ref_lat/lon=NaN 허용 |

```bash
export XDG_RUNTIME_DIR=/run/user/$(id -u)
systemctl --user status dronestock-local-uwb.service --no-pager
systemctl --user stop dronestock-local-uwb.service
```

서비스 중단은 관측 전송을 멈춘다.
PX4는 새 EV 입력이 끊기면 해당 융합을 중단한다.
기존 센서 경로는 계속 실행한다.
FC 원복은 DISARM·ON_GROUND에서만 한다.
`alignment-active.json`의 `before` 값을 쓴다.

```bash
python3 src/drone_uwb/tools/configure_local_uwb.py \
  --output /home/bluewak/local-uwb-20261011/새원복기록.json \
  --heading=-Y --apply \
  --restore-from /home/bluewak/local-uwb-20261011/alignment-active.json
```

FC·host 재부팅 뒤 기존 서비스는 발행을 차단한다.
새 부팅의 정렬 근거를 수집한 뒤 다시 시작한다.
이번 작업에서는 재부팅 시험을 하지 않았다.

## 비행·모드 범위

현재 PX4 `POSCTL`은 로컬 위치를 사용한다.
표준 `AUTO.LOITER`는 글로벌 위치 조건을 유지한다.
그 조건을 끄거나 가짜 위도·경도를 넣지 않았다.
기존 글로벌 `DO_REPOSITION` 경로도 별도 수정 대상이다.
UWB 융합 성공과 자동 경유지 비행 성공은 다르다.

비행 전 독립 거리·축 방향 검증이 남아 있다.
두 축 이동·회전·기울기·공백 복구를 측정한다.
현재 기수 응답은 정적 축 정렬의 근거다.
동적 정확도나 시간 지연의 교정 완료는 아니다.
