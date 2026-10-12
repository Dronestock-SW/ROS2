# 실측 LiDAR 수평 속도 입력
수동·OFFBOARD 공통 입력 구조를 설명한다.
실측 로직과 FC 적용 범위를 확인할 때 읽는다.

모드와 무관한 관측 노드를 추가했다.
실제 FC 설치와 비행 검증은 아직 남아 있다.

```text
T-mini Pro /point_cloud + MAVROS IMU
  -> 광선별 회전 보정 + 장착 위치 변환
  -> 연속 점군 정합 -> FC IMU 기준 NE 속도
  -> lidar_velocity_bridge (SYS 255 / COMP 198)
  -> MAVROS raw MAVLink -> 전용 PX4 LiDAR2D
  -> EKF2 auxiliary horizontal velocity

UWB B_TF -> 기존 XY 위치 입력 -> 같은 EKF2
TFmini Plus -> 기존 PX4 높이 경로
```

| 항목 | 계약 |
|---|---|
| 입력 시각 | SDK 광선 시각, 실제 IMU 및 timesync |
| 관측 성분 | N/E 속도와 2×2 공분산 |
| 미관측 성분 | 위치·자세·수직 속도는 NaN |
| 위치 기준 | FC IMU 원점, LiDAR 레버암 보정 |
| MAVLink | ODOMETRY, estimator LIDAR, component 198 |
| FC 보호 | 전용 protocol 1·enable·age·delay 확인 |
| 시작 | DISARM/착륙 확인 후 시작 |
| 지속 | 수동·OFFBOARD 모드 전환과 독립 |
| 손실 | 새 관측 중지, 오래된 속도 재전송 없음 |
| 공분산 | 공학적 임시 상한, 정확도 보정 미실시 |

현재 PX4 auxiliary velocity 경로는 다른 수평 보조를 요구한다.
UWB 또는 optical flow 활성 조건이 필요하다.
LiDAR만으로 비행을 유지한다고 해석하면 안 된다.
해당 독립 비행 조건을 검증하지 않았기 때문이다.

노드는 ARM·모드·파라미터를 쓰지 않는다.
비행 명령은 기존 임무 경로가 담당한다.
`published`는 송신 여부다.
실제 융합은 FC `lidar_velocity2d_status`로 확인한다.

실행은 [절차](../runbooks/lidar_physical_velocity.md)를 따른다.
측정 근거는 [시험 기록](../report/lidar_physical_20261012.md)에 있다.
