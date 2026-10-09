# 수동 비행 관측 기록

RC 수동 비행의 관측을 모으는 절차다.
좌표·시간 보정 자료를 만들 때 읽는다.
수집기는 비행 명령이나 파라미터를 보내지 않는다.

UWB 원본과 계산 좌표를 같은 실행에서 저장한다.
PX4 상태·자세·위치·시각도 함께 저장한다.
수집 성공은 비행 가능 판정이 아니다.
보정 후보는 기록 분석 후 별도로 적용한다.

## 1. 기록 범위

원본 측정 시각과 수신 시각을 분리해 보존한다.

| 입력 | 남기는 내용 |
|---|---|
| `/uwb/received` | 원시 거리·순서·태그 시각·호스트 시각 |
| `/uwb/btf_decision` | 계산 XY·높이 근거·후보·거부·처리 시간 |
| `/uwb/btf_pose`, `btf_xyz` | 발행 좌표·시각·공분산 |
| `/uwb/btf_status`, `bridge_status` | 결측·발행 조건·거부 상태 |
| MAVROS IMU·odom·ToF | 자세·가속도·추정 위치·속도·거리 |
| MAVROS RC·state·extended_state | 채널·모드·ARM·착륙 상태 |
| MAVROS TIMESYNC·estimator_status | 시계 상태·위치 유효성 |
| `/mavros/vision_pose/pose_cov` | FC 전달 전 EV 관측 |
| MAVLink source | FC 수신 패킷 원본 전체 |
| 광류 토픽 | 존재하면 원시 적분값·품질 |

MAVLink 명령 sink는 구독하지 않는다.
기존 진단 도구의 단일 라우터 조건을 보존한다.
수집기는 FC 포트·서비스·명령 발행기를 만들지 않는다.
관측이 없으면 누락으로 기록한다.
ToF 결측을 가상 높이로 바꾸지 않는다.
수신 공백을 복제 좌표로 채우지 않는다.

## 2. 실행

기존 ROS 관측 프로세스를 유지한 채 실행한다.
Tag B는 domain 2, Tag A는 domain 1이다.

```bash
cd ~/ROS2-review-20261008-codex
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
# 이 현장 설치에서 ROS Python 의존성을 제공하는 기존 경로.
export PYTHONPATH="$HOME/ROS2-integration-20261007/.test-deps:$PYTHONPATH"
df -h /dev/shm
CAPTURE="/dev/shm/manual-$(date -u +%Y%m%dT%H%M%SZ)"
ros2 run drone_uwb uwb_manual_capture record \
  --tag B --output "$CAPTURE" --seconds 900 --max-mib 1024 \
  --config .review/field-20261009/field-btf.json \
  --config .review/field-20261009/field-mission.json \
  --source-revision "$(git rev-parse HEAD)"
```

실제 설정 파일은 `--config /path/to/field.json`로 추가한다.
이 인자를 반복해 B_TF·mission 설정을 함께 저장한다.
지정 파일의 원문과 SHA256을 manifest에 보존한다.
설정·실측 원본은 Git에 올리지 않는다.

최대 실행은 기본 15분이다.
ARM을 관측한 뒤 최신 지상·DISARM이 20초 유지되면 끝난다.
부팅 직후 DISARM만으로 종료하지 않는다.
SIGINT·SIGTERM 또는 기록 폴더의 `STOP` 파일로 끝낸다.
SSH가 끊겨도 유지하려면 현장 실행 관리자가 분리 실행한다.
수집 시작 시 PID와 경로를 표시한다.

```bash
ros2 run drone_uwb uwb_manual_capture status "$CAPTURE"
ros2 run drone_uwb uwb_manual_capture mark "$CAPTURE" 'warehouse +X outward'
touch "$CAPTURE/STOP"
```

마커는 같은 Jetson 부팅에서 기록한다.
조종자 설명 시각이며 실제 운동의 기준 시각이 아니다.

## 3. 수신 확인

`summary.json`은 2초마다 갱신한다.
필수 토픽 누락과 마지막 수신 경과를 확인한다.
`status`의 `recording_active`도 확인한다.
다른 부팅·5초 넘은 요약·종료·오류면 false다.
상태 명령도 이때 종료 코드 1을 반환한다.
이는 토픽 완전성이나 비행 허가 판정이 아니다.
ARM·모드 변화와 토픽별 최대 공백도 남긴다.
오래된 summary는 수집기 생존 근거가 아니다.

기본 사건 파일 한도는 192MiB다.
위 현장 명령은 1GiB를 명시해 사용한다.
오늘 기록 속도는 약 0.76MB/s였다.
192MiB는 이 속도에서 약 4분이면 소진된다.
`estimated_capacity_remaining_s`로 남은 시간을 확인한다.
이 값은 지금까지 평균 속도에 따른 추정이다.
저장공간 512MiB를 남기고 기록을 중단한다.
큐·디스크 오류를 숨기지 않고 요약에 남긴다.
센서 콜백은 디스크 쓰기를 기다리지 않는다.
한도·오류로 중단된 기록을 완전한 기록으로 취급하지 않는다.
비정상 전원 차단은 마지막 기록 일부를 잃을 수 있다.
종료 후 파일을 PC에 복사하고 해시를 대조한다.

저장장치가 부족하면 `/dev/shm`에 제한해 기록한다.
이 위치는 전원 차단 때 사라진다.
PC에서 아래 도구를 함께 실행해 지속 복사한다.
`PID`는 수집기가 표시한 실제 Python PID다.

```bash
python src/drone_uwb/tools/mirror_manual_capture.py \
  --host arialhanho@100.110.163.94 \
  --remote-dir /dev/shm/SESSION --pid PID --output ./SESSION
```

복사가 수집 속도를 따라가지 못하면 지연될 수 있다.
`mirror-status.json`의 바이트 수·갱신 시각을 확인한다.
완료 후 SHA256 일치 전에는 전원 차단 손실 가능성이 있다.
SSH 재접속 시 마지막 복사 바이트부터 이어받는다.
재개할 때 처음·경계 64KiB의 해시를 확인한다.
원격 부팅·PID 시작 시각·요약 갱신도 확인한다.
원본 소실·PID 재사용·파일 축소면 오류로 끝난다.
무한 `tail` 대기를 사용하지 않는다.
전송 중에도 요약 파일과 복사 잔량을 갱신한다.
SSH 전송만 압축하며 저장 원본은 바꾸지 않는다.
같은 LAN의 확인된 IP는 `--connect-address`로 지정한다.
원래 SSH 호스트 키 검증은 유지한다.
재부팅 후에는 새 세션과 새 파일을 만든다.

## 4. 보정 분석

같은 시행의 PX4 ULog를 함께 확보한다.
ROS 위치 유효성만으로 EV·광류 융합을 확정하지 않는다.
ULog에는 실제 기록된 토픽을 먼저 확인한다.
필요 입력이 없으면 미수집으로 남긴다.
수집기가 PX4 로그 설정을 바꾸지는 않는다.

DISARM 상태의 시험 전후에는 고정 읽기를 저장한다.
logger 동작과 실제 파일 경로도 이 결과에 남긴다.
지상 읽기를 공중 융합 근거로 소급하지 않는다.

```bash
python3 src/sangwon_AI/ops/px4_sensor_readback.py \
  --ros-domain 2 --profile manual-flight \
  --output /dev/shm/manual-fc-before.json
```

종료 후 `manual-fc-after.json`에도 같은 읽기를 저장한다.
실제 ULog는 같은 FC 부팅·시행인지 확인해 PC에 복사한다.
과거 날짜 폴더명만으로 연결하지 않는다.
아래 분석기는 로컬 파일만 읽는다.

```bash
PYTHONPATH=src/drone_uwb python3 -m drone_uwb.integration.manual_analysis \
  /path/to/PC-CAPTURE --output /path/to/manual-analysis.json
```

Windows에서는 저장소 위치에서 다음과 같이 실행한다.

```powershell
$env:PYTHONPATH='src/drone_uwb'
python -m drone_uwb.integration.manual_analysis `
  C:/path/to/CAPTURE --output C:/path/to/manual-analysis.json
```

거부 사유·토픽 공백·RC 모드·시계 비율을 요약한다.
시계 단계만 재생하며 B_TF 전체 재현은 아니다.
새 원시 단조 시계 기록은 호스트 시계 변화를 구분한다.
원래 측정·ROS 시각을 고치거나 지연을 확정하지 않는다.

1. 지상·ARM·공중·착륙·수동 모드 구간을 나눈다.
2. UWB 거리와 B_TF의 거부·공백을 확인한다.
3. 알려진 창고 축과 실제 운동을 대조한다.
4. 회전·원점·장착 보정 후보를 계산한다.
5. 원본 측정 시각과 FC 시각의 오차를 계산한다.
6. 다른 이동 구간에서도 같은 후보가 맞는지 본다.

수동 조종 자체가 독립 위치 기준은 아니다.
PX4에 같은 UWB가 융합돼 있으면 비교가 순환한다.
ULog의 광류 융합 상태와 독립 기준을 구분한다.
시각을 맞춘 외부 영상·바닥 눈금도 사용할 수 있다.
후보가 유일하지 않으면 보정을 확정하지 않는다.
이 수집기는 확인 플래그나 FC 값을 자동 변경하지 않는다.

현재 지상 EV 시험기는 킬 해제·ARM 때 종료된다.
이 상태에서 Position 위치 입력 유지가 보장되지 않는다.
실제 수동 비행 모드와 추정기 조건은 별도 확인 사항이다.
수집기를 켰다는 이유로 지상 시험 설정을 비행 승인하지 않는다.

근거: [PX4 로그](https://docs.px4.io/main/en/dev_log/logging),
[외부 위치의 시간 보정](https://docs.px4.io/main/en/ros/external_position_estimation#tuning-ekf2-ev-delay).
구현 확인은 [수집기 시험 기록](../report/manual_flight_capture_20261009.md)에 남긴다.
수동 시험 뒤 수정은 [관측·기록 수정](../report/manual_capture_repairs_20261009.md)을 읽는다.
