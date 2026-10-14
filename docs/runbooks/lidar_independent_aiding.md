# LiDAR 독립 융합 재현·실기 확인
R3 후보를 재현하고 실기 상태를 확인하는 절차다.
펌웨어를 설치하거나 융합 근거를 만들 때 읽는다.

후속 사용자 승인으로 [R3·ToF 결합본](../report/r3_tof_deployment_20261014.md)을 실제 FC에 적용했다.
아래 미설치·보류 표기는 해당 기록 당시 상태다.
현재 적용 상태는 위 후속 기록을 우선한다.

## 현재 경계

**현재 실제 FC는 R2이며 R3 설치는 미실시다.**
[결과와 한계](../report/lidar_independent_aiding_20261014.md)를 먼저 읽는다.
아래 빌드·SIH 명령은 기체를 조작하지 않는다.
초기 임무 좌표의 UWB 준비 조건은 유지한다.
[웹 폐루프 결과](../report/web_vertical_takeoff_20261014.md)는 기준 미통과다.
R3 설치·DISARM 점검 준비와 비행용 운영 전환을 구분한다.

## 후보 재현

원본 PX4를 변경하지 않는 별도 경로를 사용한다.
아래는 이번 WSL 환경의 명령이다.
새 재현 경로는 존재하지 않아야 한다.
다른 환경에서는 경로만 바꾼다.

```bash
source /home/dronestock/.venvs/px4/bin/activate
tools=/mnt/c/Users/A/Documents/ChatGPT/DroneStock/lidar-physical-publish/src/drone_uwb/tools
candidate=/home/dronestock/uwb_sim/lidar-independent-rebuild
python "$tools/prepare_lidar_independent.py" \
  --donor /home/dronestock/uwb_sim/uwb-lidar-integration-20261012/PX4-lidar2d-fmu \
  --destination "$candidate" --with-tests
cd "$candidate"
make px4_fmu-v6c_lidar2d -j3
make px4_sitl_lidar2d -j3
make px4_sitl_test unit-test_EKF_lidar_independent -j3
```

도구는 기반 커밋을 검사한다.
필요한 초기화 submodule은 donor에서 공유한다.
추가 submodule이 필요한 빌드는 별도 준비가 필요하다.
기존 기체 펌웨어나 파라미터는 건드리지 않는다.
`prepare`는 전체 R3 패치를 적용한다.
`upgrade_lidar_independent.py`는 R2 패치의 개발용 변환기다.
`prepare` 뒤에 다시 실행하지 않는다.
이유: R3 변경을 중복 적용하게 된다.

6개 EKF 실행 파일을 빌드하고 CTest로 검사한다.

```bash
cmake --build build/px4_sitl_test --target \
  unit-test_EKF_basics unit-test_EKF_externalVision unit-test_EKF_fake_pos \
  unit-test_EKF_fusionLogic unit-test_EKF_height_fusion -- -j3
ctest --test-dir build/px4_sitl_test --output-on-failure \
  -R '^unit-test_EKF_(basics|externalVision|fake_pos|fusionLogic|height_fusion|lidar_independent)$'
```

## 격리 SIH

동일 펌웨어·패치·보드 해시를 묶어 실행한다.

```bash
python "$tools/package_lidar_independent.py" \
  --source "$candidate" --output /home/dronestock/uwb_sim/lidar-independent-runtime
python "$tools/run_lidar_independent_matrix.py" \
  --runtime /home/dronestock/uwb_sim/lidar-independent-runtime \
  --output /home/dronestock/uwb_sim/lidar-independent-matrix \
  --speed-factor 1 --instance-base 84
python "$tools/analyze_lidar_independent_sih.py" \
  /home/dronestock/uwb_sim/lidar-independent-matrix/*-seed*/ \
  --output /home/dronestock/uwb_sim/lidar-independent-matrix-summary.json
```

분석 입력 glob 끝의 `/`를 유지한다.
이유: 같은 이름의 `.log` 파일은 분석 입력이 아니다.
실행 instance 번호는 다른 시험과 겹치면 안 된다.
네트워크 격리와 PX4 instance 잠금은 별개다.

분석은 완전한 기록 창을 요구한다.
LiDAR 단독·UWB 공백에서 다른 수평 보조를 검사한다.
융합 시각 증가 7Hz 이상과 유효율 98% 이상을 요구한다.
두 수평 입력 소실에서는 수평 유효성이 철회돼야 한다.
높이 센서 소실은 수직 유효성 철회를 요구한다.
합성 관측은 시뮬레이터 시계를 사용한다.
실제 Jetson↔FC 시계 동기 검증과 구분한다.

## 웹 이륙 폐루프 재현

다음 명령은 v16의 시험 조건을 재현한다.
POSITION 연결은 확인했지만 목표 기준은 실패한 조건이다.
분석 종료 코드 1을 통과로 바꾸지 않는다.
`numpy`, `pymavlink`, `pyulog`, `websockets`가 필요하다.

```bash
repo=/mnt/c/Users/A/Documents/ChatGPT/DroneStock/lidar-physical-publish
core=/home/dronestock/uwb_sim/lidar2d-independent-20261014/companion-core
cmake -S "$repo/src/sangwon_AI" -B "$core"
cmake --build "$core" --target sangwon_field_executor_replay -j2
python "$tools/field_takeoff_lidar_px4.py" \
  --runtime /home/dronestock/uwb_sim/lidar2d-independent-20261014/runtime-v3 \
  --core "$core/sangwon_field_executor_replay" \
  --output /home/dronestock/uwb_sim/web-takeoff-new-run \
  --instance 86 --scenario uwb_gap --seed 11 \
  --lidar-reported-std .015 --height-reference 2 --mag-type 0 \
  --imu-profile sih_noisy --range-source mavlink \
  --range-noise-model synthetic_quantized --output-tau .25
python "$tools/analyze_field_takeoff_lidar_px4.py" \
  /home/dronestock/uwb_sim/web-takeoff-new-run \
  --output /home/dronestock/uwb_sim/web-takeoff-new-summary.json
```

합성 관측의 불확실성과 SIH의 IMU 잡음 가정을 함께 기록한다.
실기 파라미터 권장값이 아니다.
실제 높이·XY·속도는 ULog 진실값으로 다시 검사한다.
HTTP 응답·모드 전환만으로 통과 처리하지 않는다.

## 실제 FC 확인 순서

설치와 지상 융합 확인을 끝내야 이륙 검증으로 넘어간다.

| 순서 | 수행·기록 |
|---|---|
| 1 | 현장 담당자가 프로펠러 제거·전원 안정·DISARM 확인 |
| 2 | 현재 펌웨어·파라미터·보정값 백업 및 해시 기록 |
| 3 | board 56과 후보 SHA256 대조 후 현장 설치 |
| 4 | `ver all`, `EKF2_LID2_VER=2` 읽기 확인 |
| 5 | CRSF·센서·기존 파라미터 보존 대조 |
| 6 | 검증된 `ground_static`으로 제한된 DISARM 송신 |
| 7 | 수신과 별개로 `time_last_fuse`의 반복 증가 확인 |
| 8 | LiDAR 중단 뒤 활성 철회·다시 시작 시 복구 확인 |
| 9 | UWB 복구 전후 XY 리셋·목표 유지·좌표 일치 확인 |
| 10 | 새 회전 시각·장착·상승 장면 검증 |
| 11 | 실측 근거로 비행용 검증 참조를 채운 뒤 별도 이륙 시험 |

단계 1의 현장 확인은 이번 작업에서 받지 않았다.
사용자는 현재 기체 옆 수동 작업이 불가능하다고 했다.
따라서 이번 작업은 설치 파일·소스 준비까지 수행한다.
원격 무인 펌웨어 설치·재부팅은 실행하지 않았다.

융합 확인은 두 시점 이상의 증가량으로 판정한다.
`accepted>0`, 버전 표식, 단일 `fused` 값은 부족하다.
관측 중단 뒤 시각이 멈추는지도 함께 확인한다.
실기 중단·복구에서 위치 리셋을 숨기지 않는다.
수직 속도는 계속 미관측으로 남긴다.

Jetson 기록 저장 공간도 확인한다.
공간 부족으로 기록기가 대기하면 비행 근거를 수집할 수 없다.
기존 RAW를 지워 공간을 확보하지 않는다.
보존·외부 복사 확인 뒤 별도 저장 계획을 정한다.
