# 4앵커 기본 실행 복원
이 문서는 네 앵커로 관측을 기록하는 절차다.
3앵커 진단을 끝내고 기본 실행으로 돌아갈 때 읽는다.

## 적용 기준

기본 수신은 A1·A2·A3·A4를 모두 요구한다.
`min_anchors=4`, `active_anchor_mask=15`다.
네 앵커 모두 정상이어야 기본 좌표를 발행한다.
A4가 계속 결측이면 좌표를 보류한다.
이유: 3앵커 진단을 기본 운영으로 사용하지 않는다.

앵커 지도는 6.3×4.6m, 높이 2.2m다.
B_TF도 `required_anchor_count=4`를 유지한다.
세 앵커 후보 비교는 네 앵커의 입력 이상 진단에 쓴다.
물리적으로 A4가 없는 상태로 비행을 승인하지 않는다.

`uwb_a123_bench.yaml`과 A123 도구는 진단용으로 보존한다.
기본 launch는 이를 선택하지 않는다.
기존 원시 기록과 과거 결과는 수정하지 않는다.

## 장치에서 실행

Jetson 연결 후 현재 프로세스와 직렬 점유를 먼저 확인한다.
사용 중인 기록을 임의로 종료하지 않는다.
이유: 다른 기록이나 센서 연결을 끊을 수 있다.
기존 A123 실행이 있다면 해당 실행의 종료를 확인한다.
기체 준비 상태와 A4 응답을 확인한 뒤 기록한다.

아래는 Jetson Bash 명령이다.
저장소의 로컬 변경은 보존·대조한 뒤 갱신한다.
강제 reset이나 전체 파일 덮어쓰기를 사용하지 않는다.

```bash
cd /home/pgyxn/github/ROS2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb
source install/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1 PYTHONIOENCODING=utf-8
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
fuser /dev/uwb /dev/pixhawk
ros2 launch drone_uwb uwb_btf_bench.launch.py \
  record_directory:=data/raw/btf_four_anchor_$(date +%Y%m%d_%H%M%S) \
  stop_after_s:=60.0
```

장치가 점유돼 있으면 실행을 진행하지 않는다.
`fuser`의 출력 없음은 새 기록의 시작 조건 중 하나다.
ToF·자세·시각 대응과 네 앵커 원시 마스크도 확인한다.
FC 위치 전달은 비활성 상태다.
실물 위치 정확도와 비행 검증은 별도로 수행한다.

## 2026-10-07 적용 범위

이번 변경은 Git 저장소의 기본 실행 설정을 명시했다.
기본 launch는 `config/runtime/uwb.yaml`을 직접 사용한다.
기존 호환 설정 경로도 같은 파일을 가리킨다.
Jetson은 확인 시 Tailscale 오프라인·SSH 시간 초과였다.
현재 장치의 실행 복원과 재빌드는 미실시다.
연결이 회복되면 위 절차로 장치 적용을 확인한다.

관련 처리·알고리즘 검사 134개를 통과했다.
A4 결측 차단과 네 앵커 복구 후 발행을 검사했다.
ROS 연결부·launch·기록 도구의 구문 검사도 통과했다.
이번 환경에서 ROS 실행·실물 수신 검사는 미실시다.
