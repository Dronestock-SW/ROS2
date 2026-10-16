# 실내 UWB 수평 융합 확인 — 2026-10-11
연결된 Tag B 기체의 변경·재부팅 복구 기록이다.
GPS 없는 경로의 실제 적용값을 확인할 때 읽는다.

## 최초 지상 결과 — 16:40~16:56 KST

PX4에서 UWB 수평 위치의 실제 융합을 확인했다.
GPS와 위도·경도 원점은 사용하지 않았다.
기체는 DISARM·POSCTL·지상 상태였다.
시동·이륙·모드 변경 명령은 보내지 않았다.

| 직접 FC 조회 | 결과 |
|---|---|
| 펌웨어 | FMUv6C, PX4 1.17.0, `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| EKF instance 0·1 | cs_ev_pos=True, fused=True, innovation_rejected=False |
| EV 고도·yaw·속도 | cs_ev_hgt/yaw/vel=False |
| 기존 센서 융합 | cs_opt_flow/rng_hgt/baro_hgt=True |
| 위치·속도 유효성 | xy/z/v_xy/v_z_valid=True |
| 고도·자세 유효성 | local_altitude_invalid=False, attitude_invalid=False |
| 글로벌 원점 | ref_timestamp=0, ref_lat/lon/alt=NaN, xy_global=False |
| 지상 방향 플래그 | heading_good_for_control=False 유지 |

[실행·복구 절차](../runbooks/local_uwb_fusion.md)를 따른다.
지상 정적 확인이다. 비행 정확도 검증은 아니다.

## 막힌 위치와 원인

현재 주된 단절은 B_TF 뒤의 FC 전달 경로였다.
16:12 KST에 B_TF는 이미 신선한 XY를 발행했다.
25초에894개, 평균35.76Hz였다.
같은 구간에35개는 높이·거리 후보 검사를 거절했다.
최대 pose 공백은 약0.842초였다.

| 위치 | 적용 전 확인 | 이번 처리 |
|---|---|---|
| B_TF XY | 정상 발행과 후보 거절이 공존 | 기존 필터·임계값 유지 |
| MAVROS 전달 | sensor-only allowlist에 외부 위치 입력 플러그인 없음 | 기존 router에 관측 전용 전달기 연결 |
| PX4 EV 선택 | EKF2_EV_CTRL=0 | 수평만 선택하는1 적용 |
| FC 외부 위치 수신 | vehicle_visual_odometry 미발행 | NED 안테나 XY 수신 확인 |
| 실제 융합 | estimator_aid_src_ev_pos 미발행 | 두 EKF fused=True 확인 |

B_TF는 지리적 원점 조건으로 막힌 상태가 아니었다.
`external_output_allowed=False`는 유지했다.
B_TF는 관측을 생성한다. 별도 전달기가 FC에 넣는다.

과거 긴 공백에는 ToF 하한·신선도 경계가 있었다.
앵커 누락과 만료된 승인 XY도 있었다.
[공백 분석](uwb_gap_followup_20261011.md)과
[재획득 수정](uwb_reacquisition_20261011.md)을 보존했다.
이번 작업에서 그 임계값을 풀지 않았다.
15:33 한 표본만으로 과거 중단 전환을 확정하지 않는다.

```text
기존 RAW -> B_TF -> XY -- 단절: FC 전달기 없음 / EV_CTRL=0

현재 RAW -> B_TF -> 고정 축 변환 -> 시각·점프·독점 검사
                                             |
                                   MAVLink ODOMETRY
                                             |
                                   기존 MAVROS router
                                             |
                                   PX4 EKF2 수평 융합
```

## 적용값

사용자가 기존 앵커 배치와 기수 −Y를 확인했다.
−Y는 A3→A1이다. 앵커 z는0.15m다.
6.3×4.6m 설정을 사용했다.
16:29 KST에 정적 IMU yaw425개를 수집했다.
ENU 기수 평균45.63000785°, 범위0.055039°였다.
기존 실내 XY 부호−1과 결합했다.
지도→ENU 회전은−44.36999215°다.
원점 이동은(0,0)m다. A1을 수평 원점으로 유지했다.
동적 정렬·시간 지연은 확인 완료로 표시하지 않았다.

| FC 파라미터 | 전 → 후 | 의미 |
|---|---|---|
| EKF2_EV_CTRL | 0 → 1 | 외부 수평 위치만 융합 |
| EKF2_EV_POS_X/Y/Z | (0,0,0) → (−.10,0,−.10)m | FC→안테나 FRD 장착 벡터 |
| EKF2_EV_DELAY | 0 유지 | 동적 지연 교정 미실시 |
| EKF2_EV_NOISE_MD | 0 유지 | 메시지 공분산 사용 |
| EKF2_EV_QMIN | 0 유지 | 미계량 품질 점수 생성 안 함 |
| EKF2_GPS_CTRL | 0 유지 | GPS 융합 안 함 |
| EKF2_HGT_REF | 0 유지 | 기존 barometric reference 유지 |
| EKF2_OF/RNG/BARO_CTRL | 각각1 유지 | 기존 센서 융합 유지 |

FC→태그는10월9일 실측 기록을 사용했다.
ToF→태그 FLU(0,0,0.15)m와 다른 벡터다.
FC가 레버암을 한 번 보정한다.
상시 관측에 FC 위치·자세를 복사하지 않는다.

## 메시지·서비스 경계

ODOMETRY의 LOCAL_NED 프레임을 사용했다.
기존 MAVROS·B_TF 서비스는 재시작하지 않았다.
장치 포트를 추가로 열지 않았다.

| 전송 성분 | 처리 |
|---|---|
| XY | 안테나 측정과 전체2×2 공분산 변환 |
| 시각 | 원본 ROS stamp의 μs; PX4 TIMESYNC가 변환 |
| Z | 필수 자리값0, 분산1,000,000m², EV Z 선택 꺼짐 |
| 자세·속도·각속도 | NaN; 관측하지 않음 |
| 시동·비행 목표 | 메시지·서비스에 없음 |
| FC 쓰기 | 별도 설정 도구의 네 EV 파라미터만 |

FC 수신 표본 지연은 약19.46ms였다.
EV 관측 XY 분산은 각0.09m²였다.
FC 안테나 XY는 NED 약(−4.26988,0.44198)m였다.
보정된 EKF 기체 XY는 약(−4.18968,0.50843)m였다.
FC 고도는 약−0.39029m였다. 자리값0으로 덮이지 않았다.
관측과 추정 차이 약1–5mm는 실물 정확도 증거가 아니다.

관측 나이150ms·TIMESYNC·중복·역순을 검사한다.
입력 독점·다른 전달기·좌표 점프도 검사한다.
관측 세션과 host/FC 시계 초기화를 추적한다.
재부팅 뒤에는 새 정렬 근거까지 발행을 막는다.

서비스는 `bluewak`의 별도 복사본에서 실행한다.
기존 `arialhanho` 센서 서비스를 보존했다.
서비스 이름은 `dronestock-local-uwb.service`다.
설정: `/home/bluewak/local-uwb-20261011/alignment-active.json`.
작업 복사본: `/home/bluewak/ROS2-local-uwb-20261011`.
실행·enable 상태다. cold boot는 미실시다.
기존 부팅의 정렬 파일을 다음 부팅에 사용하지 않는다.

## 확인과 남은 범위

| 확인 | 결과 |
|---|---|
| Windows 관련 시험 | 72통과·ROS 필요4개 미실시 |
| Jetson ROS 환경 관련 시험 | 45통과; 위4개 포함 |
| 추가 MAVLink wire 포함 로컬 관측 시험 | Jetson17통과, Windows16통과·pymavlink 필요1개 미실시 |
| 별도 colcon symlink 빌드 | drone_uwb 성공 |
| 첫25초 관측 | B_TF841개, TDMA 거절65개; 최대 공백108ms |
| 수정 후25초 관측 | B_TF937개; 거절 이유 없이 정상 후보 |
| 마지막 전달기 상태 | ready, published=18,602, 마지막 전송 약10ms 전 |
| 시작 후 최대 전송 공백 | 약174ms |
| FC 실제 수신·융합 | 두 EKF instance에서 수평 융합 확인 |
| 16:56 KST 종료 시점 | 두 EKF fused=True 유지, time_last_fuse 증가, 기존 고도·자세 유지 |
| 시동·이륙·모드 변경·cold boot | 미실시 |
| 독립 XY·두 축 이동·회전·기울기·지연 교정 | 미실시 |

실제 MAVLink NaN·원본 시각 보존 시험도 추가했다.
패킷 시험에서 NaN 자세·속도와 원본 시각을 확인했다.
PX4 eph 약0.042m는 필터 추정 불확실도다.
실제 UWB 오차4.2cm를 뜻하지 않는다.
B_TF는 중첩 창을 쓴다. 독립 실측이 필요하다.

heading_good_for_control을 강제로 바꾸지 않았다.
지상에서는 mag_aligned_in_flight=False였다.
cs_yaw_align=True, cs_mag_fault=False였다.
수평 EV 융합은 정상 동작했다.
이 결과로 이륙 뒤 방향 정상까지 판정하지 않는다.

표준 AUTO.LOITER의 글로벌 위치 조건은 유지한다.
GPS 없는 로컬 운용은 POSCTL·로컬 제어 경로를 쓴다.
기존 글로벌 DO_REPOSITION·자동 미션은 별도 수정 대상이다.
이번 작업은 실제 UWB 수평 융합 경로를 연결한 범위다.

원본 근거는 `.review/local-uwb-20261011/`에 보존했다.
before-live/px4, alignment-active, after-live-1/2가 있다.
final-px4는16:56 KST의 완료 조회다.
핵심 계약·전달기·설정 도구의 Git blob은 로컬·설치본이 같다.
로컬 코드 커밋은b3322b7, 설치 코드 커밋은0b512c3다.
after-px4-1은 진단 구독 충돌로 실패한 기록이다.
전달기 수정 후 after-px4-2는 전체 조회가 완료됐다.
실패 기록을 성공 근거로 사용하지 않았다.

펌웨어 근거:
[ODOMETRY 수신](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_receiver.cpp),
[수평 EV 융합](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp),
[모드 조건](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/ModeUtil/mode_requirements.cpp).

## 새 부팅 복구 — 17:02~17:29 KST

재부팅 후 새 정렬로 UWB 전송을 복구했다.
17:23에는 두 EKF의 수평 융합을 확인했다.
17:27에는 Position 시동 사전 검사를 통과했다.
이 결과는 비행 전 지상 확인이다.
뒤의 비행 내내 융합됐다는 뜻은 아니다.

| 시각 KST | 근거·처리 |
|---|---|
| 약17:02 | host·FC 새 부팅을 관찰. 에이전트의 재부팅 명령 없음 |
| 부팅 직후 | `host_reboot_requires_new_alignment_evidence`로 기존 설정 차단 |
| 17:12 | 사용자 확인 기수 −Y에서 IMU 485개 수집 |
| 17:12 정렬 | body ENU −28.029669°, 범위1.737517° |
| 현재 설정 선택 | `alignment-reboot-1712.json`, 지도→ENU −118.029669° |
| 17:23 | EKF 0·1의 cs_ev_pos=True·fused=True |
| 17:23 시동 실패 | kill_switch=1; Kill engaged·Flight termination 오류 |
| 17:27 | kill_switch=3; pre_flight_checks_pass=True |
| 17:29 | ARM·POSCTL 확인. 정렬 갱신과 내부 조회가 보호 조건으로 중단 |

새 host boot ID는 다음과 같다.
`5cfeefce-460c-4f0c-b9d9-9c31719c64e3`.
설정 선택은 사용자 서비스의 EnvironmentFile에 있다.
`/home/bluewak/.config/dronestock-local-uwb/environment`.
원복 근거 `alignment-active.json`은 보존했다.
재부팅 전 정렬은 새 부팅에 적용하지 않았다.

이번 복구에서는 FC 파라미터를 쓰지 않았다.
EKF2_GPS_CTRL=0·EKF2_EV_CTRL=1을 재확인했다.
장착 벡터 FRD(−.10,0,−.10)m도 유지됐다.
EV 고도·yaw·속도는 계속 꺼져 있었다.
range·baro 융합과 PX4 고도·자세 책임을 유지했다.
위도·경도 원점은 계속 미설정이었다.

## 시동 원인과 지상 예상 상태

시동 차단은 실제 KILL ON을 먼저 확인한다.
지상 센서 하한만으로 실패를 판정하지 않는다.

| 항목 | 이번 근거 | 다음 확인 |
|---|---|---|
| KILL | ON=1일 때 실제 차단; OFF=3 뒤 검사 통과 | RC 상태와 오류 문구 대조 |
| ARM 스위치 | 3은 OFF | 1=ON·2=MIDDLE·3=OFF로 판독 |
| ToF | 지상 약2~3cm, 센서 최소10cm | 지상 예상 상태. PX4 전체 유효성 별도 확인 |
| 광류 | 지상 fused=False, 최소 바닥 거리8cm | 최소 높이 미만은 예상 상태 |
| heading_good_for_control | 지상 False, yaw 정렬 True·mag_fault=False | 공중 방향 정상까지 추정하지 않음 |
| USB | CBRK_USB_CHK=197848 유지 | usb_connected=True만으로 차단 판정 안 함 |
| 글로벌 위치 | 원점 미설정·global invalid | 설치 펌웨어의 POSCTL 로컬 조건 확인 |

17:27의 can_arm_mode_flags는8372167이었다.
POSCTL은 nav_state=2다. 비트값4가 포함됐다.
전체 arming 오류 마스크가0인지로 판정하지 않는다.
다른 모드의 글로벌 조건도 마스크에 남는다.
현재 모드 비트와 사전 검사 결과를 함께 본다.
앞선 ARM=3을 ON으로 읽은 설명은 정정했다.

근거:
[RC 스위치 값](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/ManualControlSwitches.msg),
[KILL 시동 검사](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/manualControlCheck.cpp).

## 정렬·진단의 남은 확인

두 지상 yaw 수집 간 차이는 약7.17°였다.
17:21 수집은701개, 범위0.022513°였다.
body ENU 평균은−20.863225°였다.
센서 기준 변화와 실제 움직임의 원인은 미판정이다.
후속 `alignment-reboot-1722.json`은 미적용이다.
17:29 최종 갱신도 ARM 보호로 미적용이다.
전송기는 계속17:12 설정을 사용했다.

반복 읽기 도구에 새 DDS 식별자를 적용했다.
이후 완료 조회를 확보했다.
모든 빈 응답을 DDS 충돌로 확정하지 않는다.
[진단 변경 기록](uwb_diagnostic_identity_20261011.md)을 따른다.

임시 서비스 출력은 파일에 보존했다.
`/home/bluewak/local-uwb-20261011/service-recovery-diagnostic.log`.
임시 drop-in은 다음 경로였다.
`~/.config/systemd/user/dronestock-local-uwb.service.d/diagnostic.conf`.
시작 후 최대 전송 공백은 시점에 따라 달랐다.
지상1.15초 확인 뒤 비행 인접 구간에서 더 커졌다.
이를 앞선16:56의174ms 결과와 섞지 않는다.
비행 뒤 상태는 [왕복 시험 기록](uwb_position_roundtrip_20261011.md)을 따른다.

원본은 `.review/local-uwb-20261011/`에 보존했다.
reboot-position-ready-2는17:23 완료 조회다.
reboot-arming-causes는17:27 완료 조회다.
reboot-position-final·alignment-final-1729는 보호 중단 기록이다.
성공·중단·미실시 근거를 구분한다.
