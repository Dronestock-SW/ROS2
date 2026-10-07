# 실물 B_TF 지상 수신 절차

실물 센서로 B_TF 관측을 계산하는 절차다.
FC에 위치를 전달하기 전 지상 확인에 사용한다.

## 실행 범위

이 실행기는 관측만 계산·기록한다.
모터 시동·모드 변경·목표 명령은 보내지 않는다.
FC의 위치 입력 브리지를 실행하지 않는다.
이유: 실측 정확도와 전달 좌표 정렬은 미검증이다.

```text
실물 UWB → /uwb/received ────────────┐
PX4 → MAVROS → 하방 거리·자세 ────────┤
                                   B_TF
                                    ↓
                         /uwb/btf_pose + 판정 기록
```

## 준비

프로펠러를 분리하고 시동을 해제한다.
ToF 광축 아래에 평평한 바닥을 확보한다.
다른 수신기가 포트를 사용하면 중복 실행하지 않는다.
기존 실행을 유지하거나 정상 종료를 기다린다.

Jetson Bash에서 실행한다.

```bash
cd /home/pgyxn/github/ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1
export PYTHONIOENCODING=utf-8 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
fuser -v /dev/uwb /dev/pixhawk
```

장착·앵커 설정은 `config/runtime/uwb_btf_real.json`이다.
파일 경로는 `src/drone_uwb`를 기준으로 한다.
기체 장착이나 앵커 위치를 바꾸면 값을 재확인한다.
변경 전 확인 값을 새 기체에 자동 적용하지 않는다.

## 60초 실행

새 기록 폴더만 사용한다.

```bash
record_path=/home/pgyxn/github/ROS2/data/raw/btf_real_$(date +%Y%m%d_%H%M%S)
ros2 launch drone_uwb uwb_btf_bench.launch.py \
  record_directory:="$record_path" stop_after_s:=60.0
```

수신기 종료에 맞춰 세 노드가 함께 종료된다.
어느 구성 요소가 먼저 종료되어도 전체가 종료된다.
로그의 MAVROS 버전 조회 실패는 별도로 해석한다.
이 설정은 command 플러그인을 제외한다.
따라서 버전 요청 서비스가 없다는 경고가 발생한다.
센서 수신 여부는 실제 메시지와 시각으로 확인한다.

## 결과 확인

| 파일·토픽 | 확인 내용 |
|---|---|
| `uwb/raw/serial.raw` | 변경하지 않은 UWB 직렬 바이트 |
| `uwb/raw/received.jsonl` | 원래 앵커별 거리·시각 |
| `btf/config.json` | 해당 실행의 고정 설정 |
| `btf/inputs.jsonl` | UWB·ToF·자세·시각 동기·FC 상태의 수신 순서 |
| `btf/decisions.jsonl` | 계산 좌표·채택/보류 사유·처리시간 |
| `/uwb/btf_pose` | 새로 계산한 UWB 안테나 기준 XY |
| `/uwb/btf_status` | 수신·계산 횟수와 상태 |

`unchanged_B4`는 네 앵커 계산이 일관된 경우다.
`ToF_validated_B3`는 높이로 검증한 세 앵커 계산이다.
보류 사유가 나오면 직전 좌표를 새 관측으로 재발행하지 않는다.
누락 높이를 고정값이나 FC 위치로 대체하지 않는다.
XY 메시지의 z=0은 미관측 자리값이다.

```bash
PYTHONPATH=src/drone_uwb python3 \
  src/drone_uwb/tools/replay_measured_btf.py "$record_path/btf"
```

재생 일치는 같은 기록의 계산 재현 여부다.
실제 위치 정확도나 비행 검증을 대신하지 않는다.
[실물 연결 결과](../report/uwb_btf_real_20261005.md)에 남은 작업을 적었다.
