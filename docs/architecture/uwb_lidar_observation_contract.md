# UWB·수평 LiDAR 관측 계약
FINAL 기반 관측·제어 연결 구조의 설명이다.
센서 융합 위치와 후속 실험 범위를 확인할 때 읽는다.

## 현재 구조

비행 상태의 최종 추정기는 PX4 EKF2다.
companion은 UWB 위치 관측을 계산한다.
수평 LiDAR 추가 경로는 격리 실험이다.

```text
UWB 앵커별 거리·측정 시각
  -> MeasuredBtf / H80: 안테나 XY 관측
  -> /uwb/btf_pose
  -> local_odometry: map -> ENU -> NED
  -> MAVLink ODOMETRY: EV 수평 위치
                                  |
IMU / 광류 / TFmini Plus / 기압계 -> PX4 EKF2
                                  |
                      /mavros/local_position/odom
                                  |
                     LocalXYMission: 목표 XY 판단
                                  |
                         PX4 위치·속도·자세 제어

수평 /scan -> keyframe 정합 -> 상대이동 진단
           -> 방향별 장애물 -> 실험용 XY 명령 제한

합성 LiDAR XY 속도 -> 별도 MAVLink 분기
                    -> lidar_velocity2d
                    -> EKF 보조 수평 속도 융합
```

H80은 IMU 가속도를 적분하는 EKF가 아니다.
최근 거리 이력에 이동 모델을 적합한다.
0.8초 창의 끝 시각에서 XY를 계산한다.
따라서 0.8초를 고정 지연으로 대입하지 않는다.
이유: 움직임 주파수에 따라 응답이 달라진다.

## 값의 책임과 시각

| 값 | 생성 주체·기준점 | 좌표·시각 | 불확실성·만료 |
|---|---|---|---|
| B_TF XY | companion·UWB 안테나 | 앵커 map·원시 측정 시각 | 현재 설정 표준편차 0.30m |
| ODOMETRY 위치 | 전달기·안테나 | ENU→NED·원 시각 보존 | 전체 XY 공분산 변환, 최대 나이 150ms |
| FC XY·속도 | PX4 EKF2 | FC 로컬 좌표·FC 측정 시각 | EKF 상태·유효성 사용 |
| 미션 목표 | LocalXYMission | 앵커 map→ENU | 20Hz 목표, UWB 만료 250ms |
| 수평 스캔 | YDLIDAR 드라이버 | scan frame·header | 약 10Hz, 광선 순서 미검증 |
| 정합 변위 | keyframe 실험 | 이전 센서 좌표·두 시각 | 공분산 미보정, FC 출력 없음 |
| 구간 평균 속도 | 관측 계약 함수 | 구간 중간 시각 | 변위 공분산/시간² + 별도 모델 오차 |
| 장애물 거리 | 별도 장애물 함수 | 공통 수평 좌표 | 무효·만료·미관측은 정지 요청 |
| 실험용 LiDAR 속도 | 합성 관측기 | NED·IMU 기준점 | 2×2 공분산·150ms 나이 제한 |

근거 코드:

- `src/drone_uwb/drone_uwb/processing/measured_btf.py`
- `src/drone_uwb/drone_uwb/processing/solvers/h80.py`
- `src/drone_uwb/drone_uwb/integration/local_odometry.py`
- `src/drone_mission/drone_mission/local_xy.py`
- `src/drone_mission/drone_mission/node.py`

설정 주기와 로그 저장률은 구분한다.
실행 8의 EV aid 로그 2Hz는 입력 주기가 아니다.
10Hz 스캔을 반복해서 40Hz 관측으로 만들지 않는다.
이유: 중복 정보의 신뢰도를 부풀릴 수 있다.

## 스캔·IMU 계약

기하·신선도·광선 시각을 따로 확인한다.
세 조건을 모두 만족해야 deskew가 가능하다.

| 항목 | 현재 근거 | 처리 |
|---|---|---|
| LiDAR 윗면 | 사용자: 천장 방향 | 장착 방향 진술로 기록 |
| 태그→LiDAR | 사용자: 약 10cm 아래 | 정밀 FC 변환으로 사용하지 않음 |
| 각도 배열 | 드라이버가 각도별 칸에 배치 | 배열 순서=획득 순서로 가정하지 않음 |
| 광선 시간 합 | 원본에서 scan_time의 약 1.0725배 | 기존 1.01 검사를 완화하지 않음 |
| 설치 SDK 커밋 | 미확정 | 조사한 SDK와 구분 |
| IMU 보조 | 보간한 scan-Z 회전 초기값·일관성 검사 | 독립 LiDAR yaw 측정으로 재융합하지 않음 |
| 구간 단절 | scan/IMU gap 검사 | 변위를 이어 붙이지 않고 새 구간 시작 |
| 공분산 | 정합 잔차·기하 진단만 존재 | `covariance=null`, `px4_aiding_ready=false` |

수평 운동 근사와 완전한 회전 보상은 다르다.
현재 replay에는 광선별 deskew가 없다.
정확한 장착 회전·clock 증명 후에 추가한다.

## 레버암과 공분산

동일 시각·기체축에서 기준점을 옮긴다.
태그에서 LiDAR로 향하는 벡터를 r로 정의한다.
그러면 `v_tag = v_lidar - omega × r`다.
대략 10cm 수직 간격도 roll/pitch 때 영향을 준다.
10deg/s 회전이면 약 0.01745m/s다.

`horizontal_lever_arm`은 XY만 반환한다.
측정하지 않은 vz를 0으로 생성하지 않는다.
이유: 수직 속도 관측으로 오인될 수 있다.
속도·각속도·장착의 결합 공분산을 사용한다.
교차 공분산을 임의로 0으로 확정하지 않는다.

PX4 EV 내부의 레버암 보정과 중복하지 않는다.
2D 실험 경로는 이미 IMU 원점으로 보정한 값이다.
따라서 그 경로에서 안테나 위치를 다시 빼지 않는다.

## PX4 입력 후보 비교

현재는 A안을 격리 실험 대상으로 선택한다.
실제 LiDAR 연결과 실기 배포는 보류한다.

| 기준 | A: 독립 2D 입력 | B: companion 통합 외부 관측 |
|---|---|---|
| 구현 | 전용 수신 topic·admission·보조 XY 융합 | 아직 설계 단계 |
| 측정하지 않은 Z | 패킷 vz=NaN, XY만 추출 | 표준 EV 속도는 유한 XYZ 요구 |
| UWB 시각 | 기존 EV 시각을 유지 | 단일 공통 관측 시각 설계 필요 |
| reset | 입력 버퍼만 비움 | 공통 EV reset의 영향 검증 필요 |
| 상관관계 | 스캔·IMU 상관 보정은 여전히 필요 | UWB·LiDAR·IMU 결합 상관 추가 필요 |
| UWB 중복 | 기존 UWB 위치 1회 | 원래 UWB 별도 송신 중지 필요 |
| Z 영향 | 직접 Z 관측·reset 없음 | 큰 Z 분산의 3D 속도 reset 위험 |
| 유지보수 | PX4 패치와 회귀 시험 필요 | companion 추정기·covariance 관리 필요 |
| 현재 한계 | 다른 수평 aiding이 있어야 융합 | 유효한 통합 공분산·시각 미확정 |

A안도 EKF 상태 간 상관을 통해 Z에 영향을 줄 수 있다.
직접 수직 관측이 없다는 사실만으로 Z 무영향을 주장하지 않는다.
SITL에서 Z·vz·reset을 함께 기록한다.

기존 사용자 작업의 직접 EKF 호출 시험을 재사용했다.
`vz=0`과 큰 분산은 수직 reset을 막지 못했다.
원본 파일의 해시를 확인해 근거로 보존했다.
이는 B의 모든 설계를 배제하는 결과가 아니다.
검증되지 않은 3D 속도 편법을 배제하는 근거다.

## 모드별 장애물 경로

기본 Collision Prevention의 범위를 자동 확대하지 않는다.

| 모드·경로 | 이번 연결 | 검증 상태 |
|---|---|---|
| 수동 Position | 현재 운영 경로 유지 | 실제 기체 미실시 |
| 기존 LocalXYMission Offboard | 기존 위치 목표 생성 유지 | Python 회귀 시험 |
| GuardedLocalXYMission | XY 속도 제한·위치 hold·기존 LAND | 격리 SITL 실험 |
| 기본 PX4 CP | 이번 후보에 적용하지 않음 | Offboard 보호로 가정하지 않음 |
| 조종기·PX4 모드 전환 | 기존 우선권 유지 | 상태 기계 시험 |

정지는 실제 정지를 확인하는 폐루프 조건이다.
0m/s 명령이나 hold 발행만으로 정지 성공을 선언하지 않는다.
이유: 추정 속도와 실제 속도가 다를 수 있다.
2026-10-12 사용자 요구는 외곽 여유 50cm다.
프로펠러 최외곽과 벽 사이에서 측정한다.
이전 10cm는 폐기한 개발 시험 조건이다.
시험 반경 20cm이면 중심 기준 최소 70cm다.
감속은 지연·정지 거리·오차 여유만큼 더 앞서 시작한다.
현재 감속도·속도 오차 한계는 합성 시험 가정이다.
실제 기체에 적용하기 전 측정해야 한다.
관측하는 한 평면 밖의 장애물도 보호하지 못한다.

## 호버 후속 실험 경계

고정 목표 유지는 PX4 위치 제어에 맡긴다.
실험 실행기에서 명시적으로 선택한다.
운영 `FlightNode`는 이 클래스를 생성하지 않는다.

```text
이동: 목표 오차 → 수평 속도 제한 → PX4
도착: 15cm·저속 확인 → 고정 목표 → PX4 위치 유지
매 tick: 장애물 검사 → 제한 속도 또는 위치 hold
지속 차단: 기존 LAND
```

PX4 위치 게인의 상한을 명시해야 한다.
실행기는 `MPC_XY_P <= 1`을 readback한다.
고정 위치 복구의 예상 속도도 검사한다.
속도 제한이 필요하면 제한 속도로 되돌린다.
관측 만료·차단 중에는 호버 시간을 누적하지 않는다.
기존 RC·UWB 공백·좌표 점프 우선권을 유지한다.

실험의 지연·출력 보정값은 기체 설정이 아니다.
최대 지연 50ms와 속도 보정 응답 0.1초를 시험했다.
실제 관측 지연의 보정 완료를 뜻하지 않는다.
10Hz 주기와 관측의 전달 지연은 서로 다르다.
실기에서는 둘을 따로 측정해야 한다.
이유: 늦게 온 측정은 짧은 버퍼와 맞지 않는다.
[후속 결과](../report/uwb_lidar_hover_followup_20261012.md)를 따른다.

## 근거 문헌

| 근거 | 이번 적용 | 한계 |
|---|---|---|
| [PX4 EKF 지연 융합](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) | 센서별 시각·지연 분리 | 실제 커밋 소스와 대조 |
| [PX4 외부 위치](https://docs.px4.io/main/en/ros/external_position_estimation) | 주기 지침과 입력 계약 | 지침을 10Hz 불가로 해석하지 않음 |
| [PX4 충돌 방지](https://docs.px4.io/main/en/computer_vision/collision_prevention) | 모드별 적용 범위 확인 | 현재 Offboard를 자동 보호하지 않음 |
| [Censi PLICP](https://authors.library.caltech.edu/records/vcxzb-61e65) | 점-선 정합의 선택 근거 | 우리 센서 성능의 증거가 아님 |
| [Zhang의 퇴화 분석](https://www.cs.cmu.edu/~kaess/pub/Zhang16icra.html) | 기하 취약 방향 검사 | 잔차만으로 신뢰도를 판단하지 않음 |
| [2D LiDAR UAV 연구](https://mrs.fel.cvut.cz/icuas2021lidar) | 항법에 사용 가능한 구성 참고 | 장착·환경·검증 조건이 다름 |

[검증 결과](../report/uwb_lidar_integration_20261012.md)를 따른다.
