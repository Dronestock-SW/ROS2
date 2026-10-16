# GPS 없는 실내 UWB 수평 융합 절차
GPS·지리적 원점 없이 UWB를 PX4에 넣는 절차다.
10월 11일 관측 서비스를 재시작하거나 복구할 때 읽는다.

2026-10-16 통합본에서는 전달기 변위 거절을 제거했다.
아래 배치·서비스 경로·융합 결과는 10월 11일 기록이다.
최신 단일 태그 시험에서는 실제 배치 미확인으로 위치를 차단했다.
[최신 실행 기록](../report/esp32_single_tag_execution_20261015.md)을 먼저 확인한다.

수평 위치는 UWB를 융합한다. 고도·자세는 PX4가 맡는다.
현재 기체는 전달기 35cm 거절을 OFF로 운용한다.
[18:27 복구·최신 융합 확인](../report/uwb_sender_dependency_recovery_20261011.md)을 먼저 읽는다.
최신 누적 최대 전송 공백3.4초가 남아 있다.
지상 융합 확인을 비행 연속성 검증으로 쓰지 않는다.
지상 확인과 재부팅 복구는 [10월 11일 기록](../report/local_uwb_fusion_20261011.md)에 있다.
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
           신선도·시계·입력 독점 검사
           전달기 변위 점프 거절 OFF
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
| 지도→ENU 회전 | 현재 부팅 18:03 `enu_yaw_deg=-98.4056450711`; 동적 검증 미완료 |
| 전달기 변위 거절 | `jump_guard_enabled=false`; 새 부팅에도 명시해서 수집 |
| 활성 설정 | `/home/bluewak/local-uwb-20261011/alignment-nojump-boot085cec07.json` |
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

1. RC KILL·ARM 값과 실제 FC 상태를 확인한다.
   ON=1·MIDDLE=2·OFF=3이다.
   KILL ON은 정상적으로 시동을 막는다.
2. 기존 MAVROS와 B_TF 수신을 확인한다.
   DISARM·ON_GROUND에서 설정한다.
3. 드론 앞쪽을 확인한 실내 축 방향에 맞춘다.
   설정 중에는 기체를 움직이지 않는다.
4. 새 파일명으로 설정 근거를 수집한다.
   현재 장착·앵커 값이 같을 때만 아래 도구를 쓴다.
   방향 기준이 바뀌면 정지 수집을 재비교한다.
   관찰한 변화의 원인을 임의로 승인하지 않는다.
5. 적용 ACK 뒤 FC 내부 값을 조회한다.
   저장 여부·실제 융합은 ACK와 구분한다.
   값이 이미 맞으면 `--apply` 없이 수집한다.
6. 관측 서비스를 시작한다.
   현재 FC·host 부팅 근거가 맞아야 발행한다.
   설치만으로 실행 중 서비스가 재시작되지는 않는다.
   설정 파일 선택을 바꾼 뒤 지상에서 재시작한다.

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
  --heading=-Y --disable-sender-jump-guard
bash src/drone_uwb/deployment/install_local_uwb_user_service.sh \
  "$PWD" /home/bluewak/local-uwb-20261011/새파일명.json
# 아래 재시작 직전에도 최신 DISARM·ON_GROUND를 확인한다.
systemctl --user restart dronestock-local-uwb.service
```

설치기는 라이브러리 import를 먼저 검사한다.
`pymavlink` 경로가 빠지면 설치를 중단한다.
기존 환경 파일을 덮어쓰지 않는다.
설치 셸의 `PYTHONPATH`를 서비스에 저장한다.
위 export를 생략하면18:06 중단이 재현된다.
`--disable-sender-jump-guard`는 이전 명령과 호환된다.
통합본에서는 옵션 유무와 관계없이 False로 저장한다.
기존 True 파일도 실행 시 False로 정규화한다.
시각·유한값·중복 검사와 PX4 검사는 유지한다.

기수가 다른 축이면 `--heading` 값을 바꾼다.
선택값은 `+X`, `-X`, `+Y`, `-Y`다.
비스듬한 방향을 가장 가까운 축으로 임의 승인하지 않는다.
정확한 축 정렬·이동 시험이 필요하다.

설정 도구의 쓰기 범위는 네 파라미터다.
필요한 값이 다를 때만 `--apply`를 추가한다.
EV POS X/Y/Z와 EV CTRL만 쓴다.
GPS·고도·자기계·flow·range 파라미터는 쓰지 않는다.
EV POS Z는 장착 벡터다. 고도 관측·목표가 아니다.

## 확인·중단·복구

아래 신호로 실제 FC 융합을 확인한다.
토픽 발행만으로 성공을 판정하지 않는다.

| 신호 | 필요한 결과 |
|---|---|
| `/uwb/local_odometry_status` | gate·last_reason=ready, published 증가, 최근 전송 시각 |
| `vehicle_visual_odometry` | NED XY·원본 시각, q/velocity=NaN |
| `estimator_aid_src_ev_pos` | fused=true, time_last_fuse 증가 |
| `estimator_status_flags` | cs_ev_pos=true; ev_hgt/yaw/vel=false |
| `vehicle_local_position` | xy/z/velocity valid=true |
| 지리적 원점 | ref_timestamp=0, ref_lat/lon=NaN 허용 |

`gate=ready`는 전달 전제 조건의 상태다.
점프 거절로 실제 전송이 멈춰도 ready일 수 있다.
`last_send_age_s`와 전송 횟수도 확인한다.
`max_send_gap_s`는 완료된 두 전송 사이 값이다.
진행 중인 중단 길이는 `last_send_age_s`로 본다.
지상 B_TF 수신만으로 FC 융합을 판정하지 않는다.

시동 준비는 해당 모드의 검사 비트를 확인한다.
POSCTL nav_state=2의 비트는4다.
`pre_flight_checks_pass`와 함께 본다.
전체 오류 마스크에는 다른 모드의 조건도 있다.

지상 ToF 하한 미만과 광류 중단은 예상 가능하다.
높이·센서 최소값을 같은 시각에 대조한다.
이 상태만으로 불필요한 복구를 반복하지 않는다.
PX4 고도·자세·로컬 위치 유효성은 별도 확인한다.

ARM·공중에서는 재정렬·원복·재시작하지 않는다.
이유: 전송 중단과 기준 변경을 만들 수 있다.
착륙·DISARM 후 최신 상태를 다시 확인한다.
아래 stop 명령도 같은 지상 조건에서 사용한다.

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
10월11일17:02 새 부팅을 실제 관찰했다.
새 정렬로 지상 전송·융합을 다시 확인했다.
재부팅·시동 명령은 에이전트가 보내지 않았다.

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

10월 11일에는 변위 거절을 옵션으로 우회했다.
이후 통합본은 전달기의 변위 거절 코드를 제거했다.
잘못된 관측과 실제 이동의 구분에는 독립 근거가 필요하다.
그 평가를 전달기 거절의 제거만으로 대신하지 않는다.
[시험 범위·원복 기록](../report/uwb_sender_jump_off_20261011.md)을 따른다.
이전 명령의 `--disable-sender-jump-guard`를 계속 받는다.
옛 이름 `--disable-px4-jump-guard`도 별칭으로 받는다.
FC 파라미터를 바꾸는 `--apply`는 필요하지 않다.
설정 변경·재시작은 최신 지상 DISARM에서만 한다.
PX4 자체 검사와 B_TF 보호는 유지한다.

시험 전 저장공간과 수집 파일 증가를 확인한다.
다음 조각 시작에는 여유1,280MiB가 필요하다.
수집은1GiB를 남기고 쓰기를 중단한다.
3분 조각 교체를 포함한 시험 시간 여유를 확보한다.
비행 기록은 삭제하지 말고 검증 백업한다.
PX4 ULog 기록·회수는 별도로 확인한다.
[왕복 시험의 결측 기록](../report/uwb_position_roundtrip_20261011.md)을 따른다.
