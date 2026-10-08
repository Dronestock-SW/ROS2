# RAW UWB 전체 미션 모사 시험
실제 PX4와 가상 센서로 END를 검사하는 절차다.
실물 비행 전에 연결·고장 처리를 재현할 때 읽는다.

시험은 localhost와 ROS domain 99만 사용한다.
실물 FC·모터·앵커·기존 서비스를 사용하지 않는다.
새 검토 브랜치와 새 출력 폴더에서 실행한다.
구조와 결과 의미는 [연결 범위](../architecture/mission_chain.md)를 따른다.

## 1. 코드와 의존성을 준비한다

PX4 v1.17의 고정 commit을 사용한다.
기준은 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`다.
기존 검토 폴더의 `.review/hover-20261008/PX4-Autopilot`에 있다.
새 환경은 PX4 공식 빌드 의존성도 준비해야 한다.
Python에는 pymavlink·numpy·websockets와 ROS2 Humble이 필요하다.

```bash
cd /home/arialhanho/ROS2-review-20261008-codex
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb drone_demo drone_platform_link drone_mission
cmake -S src/sangwon_AI -B build/sangwon_ai_replay
cmake --build build/sangwon_ai_replay -j2
source install/setup.bash
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

## 2. ROS 실제 시각과 맞는 SIH를 빌드한다

SIH는 lockstep을 끈 가상 보드로 빌드한다.
이유: 별도 ROS 센서는 실제 시각으로 전송한다.
가상 시계의 속도 변화는 시간 동기화 검사를 깨뜨린다.
PX4의 EKF·기체 제어 코드는 바꾸지 않는다.
검토 환경은 Gazebo·XRCE 의존성만 뺀 hover 보드를 사용한다.

```bash
cd .review/hover-20261008/PX4-Autopilot
cp boards/px4/sitl/hover.px4board boards/px4/sitl/missionchain.px4board
printf '\nCONFIG_BOARD_NOLOCKSTEP=y\n' >> boards/px4/sitl/missionchain.px4board
git apply ../../../src/drone_mission/test/px4_sih_sensor_noise.patch
make px4_sitl_missionchain -j2
cd ../../..
```

새 보드 이름을 확인하고 빌드한다.
기존 hover 보드·실물 firmware를 덮지 않는다.
새 rootfs가 시행별로 만들어져 파라미터·ULog가 섞이지 않는다.

## 3. 정상과 고장 시나리오를 순서대로 실행한다

PX4/MAVROS는 시행당 한 개씩 실행한다.
6개 CPU가 있으면 시험 프로세스만 CPU를 나눠 쓴다.
기존 운영 서비스의 affinity는 바꾸지 않는다.
같은 UDP 포트를 쓰므로 동시 시행은 피한다.
이유: 관측·명령의 기체 출처가 섞일 수 있다.

```bash
python3 src/drone_mission/test/mission_chain_px4.py \
  --px4-root .review/hover-20261008/PX4-Autopilot \
  --output /dev/shm/dronestock-new-nominal
```

기본 transport는 HTTP+ROS2+MAVROS+WebSocket이다.
웹의 새 START 요청부터 최종 텔레메트리까지 검사한다.
`--transport ros`는 HTTP/WS 경계를 제외하는 진단 옵션이다.
RAW 센서는 두 태그의 TDMA 계약 중 Tag B 슬롯을 사용한다.
각 거리의 개별 시각과 원본 순번을 보존한다.

| `--scenario` | 주입 | 기대 종료 |
|---|---|---|
| nominal | 잡음 3mm·앵커별 알려진 편향 | END·작업 성공 |
| spike | 상승 중 A2 거리 +1.2m 한 번 | END·작업 성공 |
| nlos | A2 거리 +0.6m·0.8초 | END·작업 성공 |
| coherent_step | 네 거리를 가상 XY +0.8m로 생성 | 격리·회복 후 END |
| phase_delay | 거리 잔차 ±0.035m·시각 차이 | END·작업 성공 |
| short_gap | UWB 0.65초 단절 | HOLD·회복·END |
| long_gap | UWB 5.2초 단절 | LAND·FAILED |
| tof_short_gap | ToF 0.65초 단절 | HOLD·회복·END |
| tof_long_gap | ToF 5.2초 단절 | LAND·FAILED |
| scan_missing | 마커 worker 응답 없음 | 복귀·END·작업 미완료 |
| manual | PX4 POSCTL 실제 전환 | PILOT_OVERRIDE |
| scan_partial | S1 마커 없음·이후 P2/S2/P3 | 남은 작업 후 END·미완료 |
| scanner_missing_partial | S1 판독 응답 없음·S2 정상 | 남은 작업 후 END·미완료 |
| scan_failed_partial | S1 FAILED 응답·S2 정상 | 남은 작업 후 END·미완료 |

스캔 실패 후 후속 작업은 다음 명령으로 재현한다.
각 미션은 P1·S1·P2·S2·P3를 포함한다.
S1 실패를 기록하고 S2 성공·P3 도착까지 확인한다.

```bash
python3 src/drone_mission/test/mission_chain_matrix.py \
  --px4-root .review/hover-20261008/PX4-Autopilot \
  --scenarios scan_partial scanner_missing_partial scan_failed_partial \
  --output /dev/shm/dronestock-new-scan-continuation
```

반복은 새 `--output` 경로를 사용한다.
스캐너 성공 응답은 `SIMULATED-SCAN` 식별자다.
실물 판독 데이터와 혼용하지 않는다.

## 4. 결과를 확인한다

PASS 문자열과 summary의 사실을 함께 확인한다.
정상은 home_verified·landing_verified가 true여야 한다.
실제 가상 truth의 착륙 XY도 출발점 0.3m 안이어야 한다.
END만으로 스캔 성공을 추정하지 않는다.

| 파일 | 근거 |
|---|---|
| `summary.json` | 종료·확인 조건·센서 모델·PX4 commit |
| `web_command.json`, `web_status.json` | HTTP 요청·최종 웹 표시 |
| `mission/events.jsonl` | 명령 요청·응답·단계·스캔 결과 |
| `sensor_inputs.jsonl`, `truth.jsonl` | 독립 센서 생성·실제 가상 궤적 |
| `btf/decisions.jsonl` | RAW 처리·거부·ToF 선택 |
| `fc_pose.jsonl`, `observations.jsonl` | FC 추정·나이·상태 |
| `px4.ulg`, `px4.log` | 실제 EKF 융합·FC 실행 |

가상 native takeoff는 1.7m다. 실물 1.3m 설정은 보존한다.
PX4 v1.17 자력계 최종 정렬은 HAGL 1.5m를 넘겨야 한다.
1.3m 시행에서 yaw 목표가 적용되지 않아 시간 제한으로 착륙했다.
실물 높이·heading 정책은 공간·센서 확인 뒤 결정한다.
가상 scan timeout은 30초다. 3cm·3도 정렬 조건은 유지한다.

가상 SIH IMU 잡음 배율은 기본 0.1이다.
`--imu-noise-scale 1`은 upstream 잡음을 그대로 쓴다.
패치는 센서 모사만 바꾼다. EKF·제어기는 그대로다.
upstream 잡음 조건에서는 스캔 정렬 시간 제한을 관측했다.
0.1 조건의 성공을 실물 잡음 보정 결과로 해석하지 않는다.
잡음 배율은 summary에 저장한다.
실물로 simulator 패치를 배포하지 않는다.

측정 출력은 RAM에 저장한 뒤 영구 저장소로 복사한다.
이유: 저장 지연과 센서 생성 지연을 분리한다.
RAM의 fsync는 전원 단절 내구성 증거가 아니다.
SD 출력에서 freshness 실패를 관측했다.
현장 저장장치의 지속 기록 지연은 별도로 측정한다.

1.3m 대안의 모사 시험은 다음 옵션을 쓴다.
`--takeoff-alt 1.3 --mag-type 6`이다.
Init 정책은 지상 자력계로 yaw를 초기화한다.
비행 중 yaw는 자이로 오차에 영향을 받는다.
실물 적용은 보정·yaw 드리프트 확인 뒤 결정한다.

virtual covariance와 FC noise floor는 별도로 명시한다.
가상 BTF 표준편차는 0.01m다.
가상 FC EVP noise floor는 0.01m다.
가상 flow 최저 잡음은 0.01rad/s다.
가상 H80의 관측 창은 0.2초다.
실물 BTF 관측 창 0.8초는 보존한다.
가상 pose timeout은 0.3초다.
실물의 미보정 BTF 0.30m와 timeout 0.2초는 보존한다.
시뮬레이터 통과값을 실물 보정값으로 복사하지 않는다.

## 5. 실물 준비로 이어간다

가상 확인값으로 실물 gate를 열지 않는다.
실물 profile의 확인값·execute 기본값은 false다.
배치·ToF 장착·flow 축·지연·RC 인계를 현장에서 확인한다.
[센서 점검 절차](position_sensor_check.md)를 따른다.
기존 AI의 정적 지도 검사는 실행 흐름에 연결했다.
카메라·ArUco 품질·QR 장치의 실물 adapter는 후속이다.
