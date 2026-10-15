# UWB 복구 개선 1단계: 실제 운영 기준 대조
구조·설정을 수정하기 전에 확인할 운영값과 미확인 항목을 모은 기준표다.
2026-10-15 18:00~18:04 KST의 조회 결과다. 이후 변경 시 다시 대조한다.

**현재 FC 수평 속도 상한은 1.0m/s다.** 10/12 기록의 2.0m/s와 다르며, 사용자가 정한 수평 운용속도 0.25m/s보다 크다. 상승 최대 0.5m/s와 하강 0.25m/s는 현재 FC 값과 일치한다. 이번 작업은 조회만 수행했다. 파라미터 변경·서비스 재기동·로그 재생·비행은 수행하지 않았다.

[필수 설계 검토](../architecture/uwb_gap_recovery_20261015.md)의 후속 자료다. 원본 응답은 [운영 조회 JSON](../report/evidence/uwb_architecture_review_20261015/baseline_readback.json)과 [FC 조회 JSON](../report/evidence/uwb_architecture_review_20261015/fc_baseline_readback.json)에 보존했다. 비밀번호와 인증정보는 기록하지 않았다.

## 1. 실행 중인 조합

운영 센서 서비스는 `0110da8 + A-cache` 조합이다. 최신 통합 전체를 배포한 상태로 읽으면 안 된다.

| 확인 대상 | 현재 조회 결과 | 확인 범위 |
|---|---|---|
| 서비스 | `dronestock-manual-observe.service`, active/running | 18:00:57 조회 시점 |
| 최근 기동 | 17:47:06 KST, `NRestarts=0` | 과거 16:04 기동 기록 이후 다시 시작됨. 누가 어떤 이유로 기동했는지는 이번 조회로 확인하지 않음 |
| 운영 슬롯 | `/home/arialhanho/btf-acache-release-20261015` | 실행 명령·프로세스 환경에서 확인 |
| 기준 소스 | `0110da8d3022df95ceb5b77178ef63d998af895b` | base-release 기록과 source-lock 대조 |
| 소스 무결성 | 잠금 목록 811개 전부 `after_sha256` 일치 | 목록 밖 파일이나 모든 프로세스의 실행 파일까지 증명하지는 않음 |
| C++ 라이브러리 | `e33e76d62e85ca707c7632a993825f9b359661e15924a18a7f17091edc3b266f` | 현재 설정이 지정한 파일 해시 확인 |
| B_TF 설정 | `5842a0cb3e8e48d3746e9fc18713be7847c8836ba12d49363e0e2a49a3f9f3fc` | 저장된 배포 설정 해시와 일치 |
| 기체 격리 | Tag B, tag ID 6, `ROS_DOMAIN_ID=2`, localhost only | Domain 1은 조회하지 않음 |
| ROS 노드 | MAVROS·B_TF·UWB·YDLIDAR 관측 노드 확인 | 이 조회의 목록에는 임무 실행기 없음. 디스크의 실행기 소스가 현재 비행 중이라는 뜻이 아님 |
| FC | FMUv6C, PX4 1.17.0, `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, `lidar2d` | `ver all` 응답. 빌드 시각 10/14 17:29:23. 커밋 문자열만으로 전체 패치 해시가 확정되지는 않음 |

FC는 조회 시작 시 disarmed였다. 기존 MAVROS 라우터를 통해 고정된 진단 명령 24개를 실행했다. 각 명령 전에 연결·disarmed 상태를 확인했다. FC 직렬 포트를 별도로 열거나 제어 명령을 보내지 않았다.

## 2. 속도·융합 설정

| 항목 | 18:01 현재 값 | 설계에 미치는 영향 |
|---|---:|---|
| `MPC_XY_VEL_MAX` | 1.0m/s | 경로 0.25m/s와 보정속도 제한을 별도로 다뤄야 함 |
| `MPC_XY_P` | 0.95/s | 50cm 위치오차의 P항 예시는 여전히 0.475m/s. 실측속도가 아님 |
| `MPC_ACC_HOR` / `MPC_JERK_AUTO` | 1.0m/s² / 4.0m/s³ | 현재 위치·yaw 외부 목표를 이 값만으로 감속 경로로 바꿀 수 있다고 가정하지 않음 |
| `MPC_TKO_SPEED` / `MPC_Z_VEL_MAX_UP` | 0.5 / 0.5m/s | 사용자 상승 기준과 일치 |
| `MPC_Z_VEL_MAX_DN` / `MPC_LAND_SPEED` | 0.25 / 0.25m/s | 사용자 하강 기준과 일치 |
| `EKF2_EV_CTRL` / `EKF2_OF_CTRL` / `EKF2_RNG_CTRL` | 1 / 1 / 1 | 기능 설정과 현재 융합 여부는 따로 확인 |
| `EKF2_EV_DELAY` | 0ms | 실측 지연이 0이라는 뜻이 아님 |
| `EKF2_EV_NOISE_MD` / `EKF2_EVP_NOISE` | 0 / 0.10m | 관측 공분산과 실제 오차를 함께 검토. 숫자를 낮춰 정확도를 만드는 설정이 아님 |
| `EKF2_EV_POS_X/Y/Z` | −0.10 / 0 / −0.10m | 해당 FC 좌표 기준의 설정. ToF→태그 FLU 높이와 직접 비교하지 않음 |
| `EKF2_LID2_VER` / `EKF2_LID2_EN` | 2 / 1 | R3 계약 설정 확인. 지금 LiDAR가 유효하게 융합된다는 증거는 별도 |

수평 상한 0.25m/s는 다음 설정 비교의 후보로 둔다. 이 값도 실제 속도의 순간 초과나 정지거리를 물리적으로 보장하지는 않는다. 제동 성능과 위치 오차를 확인하기 전에 적용 완료로 표시하지 않는다.

## 3. Optical flow는 확인됐지만 10cm 위치 유지가 확인된 것은 아니다

선택된 추정기는 instance 0이고, 가용 추정기는 2개였다. 같은 조회에서 instance 0의 optical flow는 `fused=True`, `innovation_rejected=False`였다. `time_last_fuse=13981166684us`이며 메시지 timestamp보다 약 188.6ms 앞선 값이다. 이는 FC 지연 융합 경로의 스냅샷이다. 이 차이를 곧바로 통신 지연이나 GC 정지 시간으로 분류하지 않는다.

`vehicle_local_position`은 `xy_valid=True`, `v_xy_valid=True`였지만 `eph=2.71492m`, `heading_good_for_control=False`였다. `eph`는 추정기가 보고한 수평 불확실성 지표이며 실측 위치오차가 아니다. 그래도 이 상태만으로 절대 위치 오차 10cm를 입증할 수는 없다. 속도·상대 이동 보조가 살아 있다는 판단과 원래 목적지로 재출발해도 된다는 판단을 분리해야 한다.

이번 EV 위치 listener에는 instance 이름만 있었고 관측 필드가 없었다. 현재 UWB 융합 성공을 판정할 자료로 사용하지 않는다. 18:04 후속 `sensor_optical_flow` 조회는 응답 완료를 확인하지 못해 중단했다. 따라서 실제 optical 센서 모델과 융합 시각의 연속 증가 확인은 남았다. 이 실패를 센서 고장이나 flow 부재로 해석하지 않는다. 현재 reset counter와 delta도 한 시점의 값이므로, 이번 조회 중 리셋이 발생했다고 주장하지 않는다.

## 4. 0.8초와 관측 공분산의 실제 의미

| 현재 B_TF 설정 | 값 | 해석 |
|---|---:|---|
| `B.window_s` | 0.8s | 계산에 보관하는 관측 이력 길이. 0.8초마다 한 좌표를 발행한다는 뜻이 아님 |
| `B.reset_gap_s` | 0.15s | 관측 단절에 따른 이력 초기화 기준 |
| `B.max_extrapolation_s` | 0.04s | 관측 외삽 상한 |
| `max_output_age_s` | 0.15s | 출력 관측 나이 상한 |
| `tof_subset.max_speed_m_s` | 1.0m/s | 부분 관측 판단의 이동 가정. 임무 속도 설정이 아님 |
| `xy_stddev_m` | 0.30m | 설정 표준편차. 실제 측위 오차나 10cm 목표 달성 여부와 구분 |
| `bias_m` / `bias_calibrated` | 4개 모두 0 / false | 물리적 거리 편향이 0이라는 뜻이 아님 |
| 앵커 | 6.3×4.6m, 네 높이 0.15m | 설정과 이전 사용자 확인. 이번 작업에서 재측량하지 않음 |
| ToF→태그 FLU | (0, 0, +0.15)m | `mount_confirmed=true`는 저장된 확인 상태이며 이번 실측 결과가 아님 |

[B 계산 코드](../../src/drone_uwb/drone_uwb/processing/experiments/h80_b.py)는 현재 시각에서 `window_s`만큼 지난 관측을 제거한다. [임무 코드](../../src/sangwon_AI/include/sangwon_ai/field_mission.hpp)의 별도 0.8초는 LiDAR 보조를 허용하는 UWB·브리지 관측 나이 조건이다. 두 값과 GC 지연을 하나의 발행 주기로 묶지 않는다.

## 5. 통합 전에 보존할 소스 차이

줄바꿈을 정규화한 뒤 `field_mission.hpp`와 `field_executor_node.cpp`는 운영 슬롯과 통합본 `92ee698`의 내용이 같았다. `field_executor.hpp`는 다음 동작이 달랐다. 원문 차이는 FC 조회 JSON의 `followup.operating_to_integration_executor_diff`에 있다.

| POSITION 안정 대기 후 출발점 | 운영 슬롯 | 통합본 |
|---|---|---|
| LiDAR가 필수인 임무 | 당시 FC 현재 XY를 캡처 | 이륙 기준 `home_` 유지 |
| LiDAR가 필수가 아닌 임무 | 당시 FC 현재 XY를 캡처 | 당시 FC 현재 XY를 캡처 |

이는 A-cache 효과와 별개의 임무 기준점 차이다. 복구 구현에서 출발점 정책을 우연히 되돌리지 않는다. 후속 소스 수정은 통합본을 기준으로 하고 운영 배포 여부를 별도로 기록한다.

## 6. 하드웨어에서 남은 확인

전체 부품·명목 사양은 [기존 상세 근거](uwb_gap_review_basis_20261015.md)에 보존한다. 이번 원격 시스템의 device tree는 `NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super`, 아키텍처는 aarch64였다. FC도 FMUv6C 계열 응답을 확인했다. 따라서 PC에서 얻은 처리시간을 기체 시간으로 잘못 쓰는 혼동은 줄였지만, 전력 모드·실제 클록·부하별 최악 지연까지 측정한 것은 아니다.

| 남은 값 | 다음 판단에 필요한 이유 |
|---|---|
| 배터리 포함 실제 질량, 프로펠러 모델, 무게중심, 기체 외곽 | 제동 성능과 외곽 30cm 여유 계산 |
| Optical flow 실제 모델·축·장착·품질·바닥별 동작 | 유효한 보조 관측의 범위 결정 |
| FC·UWB·ToF·수평 LiDAR 기준점의 같은 시점 실측 | lever arm과 지도 변환의 일관성 확보 |
| 앵커 좌표 및 알려진 태그 위치의 거리 편향 | 반복 관측 거부의 입력 원인 개선 |
| 실제 제동감속 하한과 지연·위치·추종 오차 상한 | 30cm 여유와 10cm 위치 목표의 달성 판단 |

소프트웨어·설정 대조는 여기까지 완료했다. 실측 항목은 미완료로 유지한다. 다음 작업은 [복구 구현 단계](../runbooks/uwb_recovery_stages_20261015.md)의 2단계다.
