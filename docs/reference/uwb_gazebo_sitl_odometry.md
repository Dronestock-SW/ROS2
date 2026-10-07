# Gazebo UWB 관측의 PX4 입력 계약

이 문서는 UWB 위치를 PX4 외부 관측으로 보낼 때의 좌표·시각 조건이다.
Gazebo SITL 송신기를 연결하거나 EKF2 융합을 검토할 때 읽는다.

## 현재 결론

**직접 MAVLink `ODOMETRY` 경로를 SITL 후보로 선택했다.**
사용자 WSL에는 ROS 2가 없고 PX4에는 MAVLink UDP가 실행 중이다.
기존 ROS 2·MAVROS 브리지는 실물 경로에 그대로 둔다.
같은 PX4에 두 경로를 동시에 송신하지 않는다.

현재 구현은 [좌표·패킷 필드 계약](../../src/drone_uwb/drone_uwb/integration/sitl/sitl_odometry_contract.py)과
검사된 필드를 `pymavlink` 송신 함수에 넘기는 어댑터다.
[PX4 시각 응답부](../../src/drone_uwb/drone_uwb/integration/sitl/sitl_timesync.py)도 준비했다.
이 응답부는 전달받은 MAVLink 송신 객체를 사용한다.
연결을 직접 열거나 관측을 송신하지 않는다.
[Gazebo 시계 보관부](../../src/drone_uwb/drone_uwb/integration/gazebo/gazebo_clock.py)는
받은 시뮬레이션 시각을 보존하고 정지·역행을 검사한다.
2026-10-02에 [실시간 관측 연결기](../runbooks/uwb_gazebo_sitl_observer.md)를 추가했다.
Gazebo 센서 계산·시계·설정 조회·관측 송신을 연결한다.
WSL에서 실행 검증한 상태는 아니다.
Gazebo 토픽의 [실시간 Shadow 계산기](../runbooks/uwb_gazebo_live_shadow.md)는 별도로 준비했다.
이 계산기의 기본 실행은 PX4·MAVLink로 송신하지 않는다.
[읽기 전용 SITL 링크 점검기](../runbooks/uwb_sitl_link_probe.md)는 PX4의 온보드 UDP와
외부 관측 파라미터를 조회한다. 사용자 WSL UDP 실행은 미실시다.
사용자는 10월 2일 가상환경의 `pymavlink` import 성공을 확인했다.
기본 [SITL 설정](../../src/drone_uwb/config/sitl/gazebo_sitl_odometry.json)의
송신 및 다섯 확인 게이트는 모두 `false`다.
숫자 설정은 사용자 장착 보고와 시험 자리값이다.
실행 중 FC 조회나 축별 이동으로 검증한 값이 아니다.

```text
가상 RAW + ToF + IMU → 안테나 위치와 높이 계산
  → A/B 후보·품질 → 장착 기준점 변환
  → UWB 지도↔PX4 좌표 및 공분산 변환
  → 시각·설정·신선도 검사 → MAVLink ODOMETRY
  → PX4 vehicle_visual_odometry → EKF2 수평 융합
```

마지막 두 화살표는 사용자 WSL에서 실행 검증 전이다.
QGroundControl 수신이 0이고 비행 전 점검도 통과하지 못한 사용자 출력이 있다.
해당 상태에서 이륙·융합 성공으로 기록하지 않는다.
첫 정지 UWB 기록의 실제 갱신율은 25.69Hz였다.
[PX4 외부 위치 안내](https://docs.px4.io/main/en/ros/external_position_estimation)는
공분산을 포함한 메시지에 30~50Hz 전송을 권장한다.
실시간 시행에서 입력·계산·전송 갱신율을 따로 측정한다.
첫 기록의 25.69Hz를 융합에 충분한 속도라고 가정하지 않는다.

## PX4 버전에서 확인한 메시지 조건

대상은 WSL에서 보고한 `c4e4ef98e9` 커밋이다.
[해당 버전의 MAVLink 수신 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/mavlink/mavlink_receiver.cpp)는
`ODOMETRY`의 x·y·z가 모두 유한할 때 위치를 채운다.
알 수 없는 quaternion과 속도는 NaN으로 두면 해당 값으로 채우지 않는다.
[같은 버전 EKF2 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/ekf2/EKF2.cpp)는
위치 벡터가 유한하고 좌표계가 유효한지 확인한다.
[외부 관측 파라미터 정의](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/ekf2/params_external_vision.yaml)는
`EKF2_EV_CTRL=1`을 수평 위치 융합으로 정의한다.

| 항목 | 시험 계약 |
|---|---|
| 전송 메시지 | MAVLink `ODOMETRY`, estimator type `VISION` |
| 위치 좌표 | PX4 local NED, m |
| 입력 기준점 | 안테나. 자세와 장착 벡터로 확인된 PX4 기준점으로 한 번 변환 |
| `EKF2_EV_POS_X/Y/Z` | 위 변환을 쓴 경우 모두 0 확인. 중복 장착 보정 방지 |
| 높이 | 유효한 ToF·자세의 계산값. PX4로는 origin 기준 NED z |
| 융합 항목 | `EKF2_EV_CTRL=1`을 조회하고 수평만 허용 |
| 방향·속도 | 이 UWB 측정에는 없음. NaN으로 표시하고 융합 비트도 켜지 않음 |
| 불확실성 | XY 분산을 NED로 회전. 장착 불확실성 시험값을 더함 |
| 패킷 측정 시각 | 설정한 송신기 시계 µs. TIMESYNC 응답과 같은 시계 사용. PX4가 부트 시각으로 변환 |

PX4 수신기는 공분산의 x·y·z 대각 성분을 읽는다.
MAVLink 패킷에는 XY 교차 성분도 넣지만 PX4의 현재 수신 구현은 이를 쓰지 않는다.
`EKF2_EV_NOISE_MD=0`일 때 메시지 분산과 파라미터 하한을 비교한다.
분산 수치가 실제 오차 교정을 증명하지는 않는다.

## 좌표·장착 변환

UWB 지도 XY는 앵커 A1 기준이다.
Gazebo 시험장의 지도와 실제 시험장의 잠정 지도를 혼동하지 않는다.
기체 자세 회전 `R`은 body FLU에서 Gazebo 세계로 향한다.

```text
p_ref_world = p_tag_world + R (l_ref_body - l_tag_body)
p_ref_enu_xy = Rot(yaw) p_ref_world_xy + offset_enu
p_ref_ned_xy = [p_ref_enu_y, p_ref_enu_x]
z_ref_ned = z_px4_origin_world - p_ref_world_z
Sigma_ned_xy = Swap Rot(yaw) Sigma_map_xy Rot(yaw)^T Swap^T + Sigma_mount
```

`l_ref_body`는 PX4가 위치를 뜻하는 기준점의 장착 벡터다.
사용자가 보고한 FC 전방 0.14m는 설정의 시험값이다.
PX4 기준점과 맞는지 확인하기 전에는 `px4_reference_confirmed=false`다.
목표 좌표도 같은 변환으로 NED에 보낸다.
역변환을 시험해 왕복 오차가 없는지 확인한다.
PX4 위치 reset 뒤에는 원점·축 확인을 다시 하고 송신을 닫는다.

## 송신 전 확인과 현재 남은 일

패킷 필드를 만들 때 다음을 모두 요구한다.

1. 좌표·장착·높이·시각·PX4 기준점 확인.
2. FC 연결과 최근 5초 이내 파라미터 조회.
3. 150ms 이내 새 센서 표본.
   새 위치가 없으면 마지막 좌표를 새 시각으로 반복 전송하지 않는다.
4. `EKF2_EV_CTRL=1`, `EKF2_EV_NOISE_MD=0`.
5. `EKF2_EV_DELAY`가 선언값과 일치하고 `EKF2_EV_POS_*`는 0.

실시간 시계 연결의 검증은 아직 끝나지 않았다.
실시간 Shadow 기록에는 Gazebo 표본 시각과 WSL 콜백 시각을
같은 행에 남긴다. 두 값만으로 PX4 부트 시각을 얻을 수 없다.
WSL에서 PX4 시계의 왕복 조회와 시뮬레이션 진행률을 따로 측정해야 한다.
[읽기 전용 부트 시계 점검기](../runbooks/uwb_sitl_clock_probe.md)는
`TIMESYNC` 응답과 왕복 지연을 기록한다.
사용자 WSL의 실제 응답 및 Gazebo 시각 결합은 미실시다.
[PX4 시각 변환 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/lib/timesync/Timesync.cpp)는
동기화 전에는 수신 시각을 사용한다.
동기화 후에는 패킷 시각에 PX4의 추정 시계 차이를 더한다.
따라서 패킷에 PX4 부트 시각을 미리 넣으면 이중 변환이 생긴다.
2026-10-02 재검토에서 앞선 입력 계약의 이 오류를 수정했다.

추가로 [Gazebo 연결 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/simulation/gz_bridge/GZBridge.cpp)를 확인했다.
`clockCallback`은 Gazebo 시각으로 PX4 단조 시계를 갱신한다.
[POSIX 시간 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/platforms/posix/src/px4/common/drv_hrt.cpp)의
lockstep 빌드는 이 값을 `hrt_absolute_time()`으로 사용한다.
WSL 벽시계와 시뮬레이션 시계의 진행률이 같다고 가정하지 않는다.

따라서 SITL 설정의 `sender_clock_domain`은 `gazebo_sim_us`다.
모든 송신 게이트는 계속 `false`다.
사용자 WSL의 lockstep 빌드와 실제 시각 대조는 미검증이다.

```text
동일 Gazebo 월드의 /clock ─→ TIMESYNC 응답 시각(ns)
동일 월드의 센서 측정시각 ─→ ODOMETRY.time_usec(µs)
                                    │
                      PX4 sync_stamp가 추정 시계 차이를 적용
                                    ↓
                     vehicle_visual_odometry.timestamp_sample
```

`odometry_fields`는 관측 시계와 `sender_clock_domain`의 일치를 요구한다.
지원값은 `gazebo_sim_us`와 `wsl_monotonic_us`다.
후자를 쓸 때는 Gazebo→WSL 측정시각 변환을 별도 검증한다.
`px4_boot_us`를 송신기 시계로 선언하면 거부한다.
`time_mapping_confirmed`는 시계 출처와 PX4 수신 변환의 검증을 뜻한다.
WSL 콜백 시각을 측정시각으로 대입하는 허용값이 아니다.

`GazeboSimulationClock`은 WSL 경과 시간으로 Gazebo 시각을 늘리지 않는다.
같은 시각을 반복 수신해도 입력의 유효기간을 갱신하지 않는다.
새 시각이 WSL 시간으로 0.25초 넘게 없으면 응답을 보류한다.
이 0.25초는 미검증 SITL 초기 설정이다.
새 tick이 오면 재개하되, 시각 역행은 명시적 초기화를 요구한다.
콜백 지연은 남으므로 두 시계의 차이를 0으로 단정하지 않는다.

읽기 전용 점검기의 조회만으로 PX4 동기화는 완료되지 않는다.
[PX4 MAVLink 시각 코드](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/mavlink/mavlink_timesync.cpp)는
PX4가 시작한 조회의 응답으로 내부 시계 차이를 갱신한다.
실시간 송신기는 같은 송신기 시계로 이 응답을 제공해야 한다.
수신된 `timestamp_sample`이 기대값으로 변환됐는지 확인한다.
초기 수신만으로 측정 지연이 맞는다고 판정하지 않는다.

준비한 `SITLTimesyncResponder`의 동작은 다음과 같다.

| 입력·상태 | 동작 |
|---|---|
| PX4 `1/1`의 새 시각 조회 | 설정한 송신기 시계의 ns 시각과 원래 조회 시각을 응답 |
| 다른 출처·다른 대상·조회가 아닌 응답 | 무시 |
| 중복 조회 시각 | 재응답하지 않음 |
| PX4 조회 시각 또는 송신기 시각 역행 | 응답 중단, 관측 시각 매핑 무효화 요구 |
| 전송 오류 | 오류를 호출자에 전달하고 응답 중단 |
| Gazebo 시계 미수신·정지 | 새 시각을 받을 때까지 응답 보류 |
| 호출자가 매핑을 초기화한 뒤 `reset_epoch()` 호출 | 새 세션으로 응답 재개 |

실제 연결 시 이 응답은 PX4의 시각 추정 상태를 갱신한다.
따라서 읽기 전용 시계 조회 도구와 구분한다.
`responded`는 송신 함수 호출을 뜻한다.
PX4 동기화 수렴이나 관측 융합 완료를 뜻하지 않는다.
구형 dialect에서 대상 ID를 넣지 못하면 기록에 표시한다.
실시간 연결기는 전용 loopback SITL 연결을 사용해야 한다.
이 응답부는 현재 실시간 연결기에 포함한다.
앞서 배포한 진단 ZIP에는 포함하지 않았다.

실시간 Gazebo UWB·ToF·IMU 계산기 연결을 구현했다.
다음은 WSL에서 PX4 포트·시계를 확인하는 단계다.
SITL 연결기는 한 관측원을 선택하고 중복 전송을 막는다.
`vehicle_visual_odometry` 수신에 이어 EKF 사용 플래그·innovation·reset을 확인한다.
그다음 짧은 목표 이동·정지·복귀의 별도 시행을 기록한다.

순수 좌표·계약·중복 표본·시계 도메인 시험 5개가 통과했다.
`pymavlink 2.4.50`의 실제 MAVLink 2 직렬화·역직렬화 시험도 통과했다.
송신기 시각, NED 좌표, 공분산, 미관측 방향·속도 보존을 확인했다.
이 시험은 메모리 버퍼를 사용했다. UDP 송신 시험이 아니다.
검증한 Linux CPython 3.10 wheel의 SHA-256은
`866143c7a74d8a03e3aff4111aff6b913f8a08f843f5ad08f22b9a58dfbcc21f`다.
사용자 WSL의 설치 버전과 UDP 수신은 미검증이다.
WSL MAVLink 송신, EKF2 융합, 비행 시험은 미실시다.

시각 응답부 시험 5개도 통과했다.
출처·대상 거부, 중복, 시각 역행, 오류 후 중단을 확인했다.
`pymavlink 2.4.50` ARM64 wheel에서 TIMESYNC·ODOMETRY
실제 메시지 직렬화 시험 2개가 통과했다.
이 두 시험 역시 메모리 버퍼를 사용했다.
해당 wheel SHA-256은
`d9a472b6e29a1bd43fc086e9f88452a4cf53a52cb0f58d2b44f962b61f27d75b`다.
5개 패키지 빌드와 설치 환경의 응답부 가져오기도 성공했다.

Gazebo 시계 경로 추가 후 관련 단위 시험 20개가 통과했다.
시간 정지·중복·역행·재시작과 패킷/응답의 시계 일치를 포함한다.
새 시계 보관부를 Gazebo 토픽 콜백에 연결했다.
실제 Gazebo에서의 실행 검증은 미실시다.

## 2026-10-02 관측 연결 확인

실시간 연결기를 추가한 뒤 전체 시험 208개가 통과했다.
선택 의존성 모듈 1개는 기본 환경에서 건너뛰었다.
`pymavlink 2.4.50`을 임시로 제공한 추가 시험 3개도 통과했다.
그중 UDP loopback 시험은 가짜 PX4 응답기를 사용했다.
RAW·ToF·자세→계산→실제 MAVLink 패킷 수신을 확인했다.
PX4 초기화 카운터가 바뀌면 후속 관측을 차단했다.
실제 PX4·EKF 시험으로 집계하지 않는다.
5개 패키지 빌드와 설치 환경 import도 성공했다.

27일 기본 ZIP과 관측 갱신본을 합친 임시 폴더에서
새 모듈 import와 CLI 도움말 실행을 확인했다.
당시 갱신본은 `/home/pgyxn/uwb_gazebo_sitl_observer_20261002.zip`이다.
SHA-256은 `3f0a10d2538e4332b287053dd6b956baa6427965eef9e024b013963bbfd716df`다.
이 파일을 사용자 WSL에 적용한 결과는 아직 없다.
현재 반영은 [누적본 적용 절차](../runbooks/uwb_gazebo_navigation_runbook.md#갱신본-반영)를 따른다.
