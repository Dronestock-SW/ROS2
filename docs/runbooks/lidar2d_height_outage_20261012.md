# 2D LiDAR 높이 센서 공백 재현
고정한 2D 입력 구현을 격리 SIH로 검증하는 절차다.
센서 발행 중단·복구 결과를 다시 확인할 때 읽는다.

## 실행 범위

실기와 기존 작업은 그대로 둔다.
별도 runtime 복사본과 새 출력 폴더를 쓴다.
실험은 정지·DISARM 상태다.
ARM·이륙·목표 고도 명령을 보내지 않는다.
이유: 이번 검증 대상은 입력 경로의 수직 영향이다.

결과와 한계는 [보고서](../report/lidar2d_height_outage_20261012.md)에 있다.

| 입력 | 처리 |
|---|---|
| `lidar2d` | UWB XY 위치 + 별도 LOCAL_NED XY 속도 |
| `position_only` | UWB XY 위치만 융합 |
| `legacy3d` | EV BODY_FRD XYZ 속도, vz=0·분산 10,000 |

| 상황 | 40–90초의 상태 |
|---|---|
| `nominal` | 거리계·기압계 발행 유지 |
| `range_gap` | 거리계 발행 중단, 기압계 유지 |
| `all_height_gap` | 거리계·기압계 발행 중단 |

## 1. 고정한 runtime을 복사

아래는 WSL Ubuntu-22.04 Bash 명령이다.
새 목적지를 선택한다. 기존 결과를 덮어쓰지 않는다.
이유: 이전 실행과 새 실행을 구분해야 한다.

```bash
set -e
test ! -e /home/dronestock/uwb_sim/lidar2d-height-outage-repeat
mkdir /home/dronestock/uwb_sim/lidar2d-height-outage-repeat
cp -a /home/dronestock/uwb_sim/lidar2d-height-outage-20261012/runtime \
  /home/dronestock/uwb_sim/lidar2d-height-outage-repeat/runtime
sha256sum /home/dronestock/uwb_sim/lidar2d-height-outage-repeat/runtime/bin/px4
```

PX4 바이너리 SHA-256은 다음과 같다.

```text
fb2223618291204051402a940e7fddf30768b7d62d02bf8092493194f0618988
```

실행기가 바이너리·패치·보드 해시를 확인한다.
새 펌웨어 빌드나 실기 업로드는 하지 않는다.

## 2. 45회 비교 실행

실행기는 각각 새 네트워크 공간을 만든다.
loopback 외 인터페이스가 있으면 거부한다.
세 입력 경로를 별도 PX4 인스턴스로 실행한다.
각 경로에서 세 상황과 다섯 seed를 실행한다.

```bash
/home/dronestock/.venvs/px4/bin/python \
  /mnt/c/Users/A/Documents/ChatGPT/DroneStock/ROS2/src/drone_uwb/tools/run_lidar2d_height_matrix.py \
  --runtime /home/dronestock/uwb_sim/lidar2d-height-outage-repeat/runtime \
  --output /home/dronestock/uwb_sim/lidar2d-height-outage-repeat/matrix
```

| 설정 | 값 |
|---|---|
| seed | 11, 22, 33, 44, 55 |
| 시간 | 각 120초, 요청 가속 10배 |
| 합성 UWB·LiDAR 시작 | 20초, 각각 10Hz |
| 정상 비교 | 25초 이상 40초 미만 |
| 중단 비교 | 55초 이상 85초 미만 |
| 복구 비교 | 105초 이상 120초 미만 |
| 센서 중단 | 40초 |
| 센서 복구 | 90초 |

거리계는 `SIH_DISTSNSR_OVR=10000`으로 멈춘다.
이 값은 해당 SIH의 발행 조건을 끈다.
복구 시 원래 합성값 `0.2`로 되돌린다.
기압계는 `sensor_baro_sim stop/start`로 바꾼다.
EKF2 높이 융합 파라미터는 계속 켜 둔다.
명령과 실제 발행 공백을 따로 대조한다.

## 3. ULog 분석

```bash
/home/dronestock/.venvs/px4/bin/python \
  /mnt/c/Users/A/Documents/ChatGPT/DroneStock/ROS2/src/drone_uwb/tools/analyze_lidar2d_height_outage.py \
  --matrix /home/dronestock/uwb_sim/lidar2d-height-outage-repeat/matrix \
  --output /home/dronestock/uwb_sim/lidar2d-height-outage-repeat/comparison.json \
  --figure /home/dronestock/uwb_sim/lidar2d-height-outage-repeat/comparison.png
```

`checks.failures`를 먼저 확인한다.
ULog 누락과 센서 중단·복구를 검사한다.
2D 수용 횟수와 실제 보조 속도 융합을 구분한다.
높이 플래그와 리셋 횟수도 확인한다.
기압계 재시작은 새 uORB 인스턴스를 만들 수 있다.
분석은 기록된 모든 기압계 인스턴스를 합친다.
기압계 원시 기록은 약 1Hz로 제한된다.
기록 표본 수를 실제 센서 발행률로 해석하지 않는다.

원래 실행 한 건에는 ULog 누락이 있었다.
최종 결과는 같은 조건의 별도 재실행을 사용했다.
원본은 삭제하지 않고 `excluded_runs`에 남겼다.
누락 실행을 재시험한 경우 다음 옵션을 추가한다.

```text
--retry-matrix /home/dronestock/uwb_sim/lidar2d-height-outage-20261012/matrix-retry
```

재실행 폴더의 하위 실행명은 원래 이름과 같아야 한다.
분석기는 원래 dropout과 재실행 품질을 검사한다.
같은 결과를 재생하려면 기존 보존 경로를 사용한다.
새 시험에는 새 출력 경로를 사용한다.

## 4. 결과 해석

높이 변화량은 중단 전 평균에 맞춰 계산한다.
이를 절대 고도 정확도로 쓰지 않는다.
이유: 전역 XY 원점이 없으면 `ref_alt`가 NaN이다.
그래도 EKF 내부 높이는 해발 기준일 수 있다.
NaN 원점을 0으로 바꾸면 기준 차이를 오차로 오인한다.

참값은 SIH가 기록한 정지 상태다.
실제 T-mini Pro 스캔이나 TFmini Plus 광학 모델은 없다.
2D 속도는 이미 장착·기울기를 보정한 NED 입력이다.
UWB는 시험 내내 정상이다.
공중에서 모든 높이 센서가 사라진 경우는 남은 검증이다.
작은 숫자만으로 높이 유지나 비행 안전을 승인하지 않는다.
이유: 정지 조건과 내부 높이 유지 처리만 확인했기 때문이다.
