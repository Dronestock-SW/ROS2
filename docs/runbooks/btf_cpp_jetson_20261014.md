# 14일 C++ B_TF 웹 시험 경로
Jetson의 C++ 계산 연결본을 여는 절차다.
저녁 현장 입력 확인과 웹 시험 전에 읽는다.

센서 서비스는 C++ B_TF 경로로 전환했다.
웹·미션·FC 명령 송신기는 자동으로 시작하지 않았다.
기존 웹 주소만 다시 열면 새 실행기가 선택되지 않는다.
아래 `ai-build` 바이너리를 지정한 세션을 실행한다.
현재 관측 입력이 정상이어야 비행 시험을 진행한다.
UWB XY와 ToF가 없는 상태를 FREEZE로 통과시키지 않는다.
새 실측 이력 복구는 센서 설정에서 활성화했다.
`btf-recovery-build/libbtf_native.so`를 사용한다.
ToF 전달 요청은 50Hz다.
FC 20ms 읽기 패치는 아직 업로드하지 않았다.
현재 FC 수집 주기는 별도로 확인해야 한다.

## 1. SSH와 환경

노트북에서 SSH 터널을 연다.

```powershell
ssh -L 8001:127.0.0.1:8001 -L 8002:127.0.0.1:8002 arialhanho@100.110.163.94
```

이후 명령은 Jetson 셸에서 실행한다.

```bash
FLIGHT_SLOT=/home/arialhanho/ROS2-review-20261008-codex/.review/fc-owned-runtime-20261013
BTF_NATIVE_SLOT=/home/arialhanho/ROS2-review-20261008-codex/.review/btf-native-20261014
FLIGHT_SOURCE="$BTF_NATIVE_SLOT/source"
source /opt/ros/humble/setup.bash
source "$FLIGHT_SLOT/source/install/setup.bash"
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
export PYTHONPATH="$FLIGHT_SLOT/field-deps:$FLIGHT_SOURCE/src/drone_mission:$FLIGHT_SOURCE/src/drone_uwb:$FLIGHT_SOURCE/src/drone_platform_link${PYTHONPATH:+:$PYTHONPATH}"
FLIGHT_RUN="$BTF_NATIVE_SLOT/sessions/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$FLIGHT_RUN"
cat "$BTF_NATIVE_SLOT/release.json"
python3 "$FLIGHT_SOURCE/src/drone_uwb/tools/native_btf_readiness.py" \
  --require-idle --require-backend cpp --output "$FLIGHT_RUN/ground-readback.json"
```

위 gate는 교체·중복 실행 검사용이다.
`true`만으로 비행 준비 완료를 뜻하지 않는다.
기체는 DISARM·지상이어야 한다.
조종기·앵커·현장 조종자를 확인한다.
현재 기수 방향과 실제 UWB XY를 확인한다.
ToF 유효값·광류와 EV 실제 융합도 확인한다.
FC·센서 서비스를 중복 실행하지 않는다.
이유: 같은 장비의 입력·명령 소유자가 겹칠 수 있다.

## 2. 기록과 현장 정렬

[기존 절차의 3·4단계](evening_web_flight_20261013.md)를 따른다.
위 셸의 `FLIGHT_SOURCE`와 `FLIGHT_RUN`을 유지한다.
원본 수집기를 재개하고 파일 증가를 확인한다.
ULog·원시 UWB·B_TF 판단·C++ 기록·영상을 함께 남긴다.
내부 trace는 원본 수집과 ULog를 대신하지 않는다.

내부 기록은 다음 경로에 순환 저장한다.
`~/.local/share/dronestock/btf-trace/domain-2/`
최대 128MiB이며 오류와 누락은 상태에 표시한다.

기수의 +Y/-Y 방향을 현장에서 확인한다.
기존 부팅의 정렬 파일을 그대로 재사용하지 않는다.
`final_ground_alignment.py`로 현재 정렬을 생성한다.
미션은 1.7m·상승 제한 0.5m/s를 유지한다.
`takeoff_lidar_required`는 이번 경로에서 활성화하지 않는다.
LiDAR 후보의 융합 검증 참조를 임의로 채우지 않는다.
`EKF2_EV_DELAY`도 이번 작업에서 조정하지 않았다.

## 3. 새 실행기 선택

정렬이 통과한 뒤 아래 검사를 실행한다.

```bash
FLIGHT_PRIVATE=/home/arialhanho/ROS2-review-20261008-codex/.review/field-20261009
FLIGHT_ARGS=(
  --config "$FLIGHT_RUN/aligned/mission.json"
  --alignment "$FLIGHT_RUN/aligned/alignment.json"
  --site-config "$FLIGHT_PRIVATE/site.json"
  --mission-store "$FLIGHT_PRIVATE/mission-library.sqlite3"
  --ledger /home/arialhanho/.local/state/dronestock-flight/operator-requests.json
  --state-dir "$FLIGHT_RUN/runtime"
  --cpp-ai-binary "$BTF_NATIVE_SLOT/ai-build/sangwon_field_mission_ai"
  --cpp-executor-binary "$BTF_NATIVE_SLOT/ai-build/sangwon_field_executor"
)
python3 "$FLIGHT_SOURCE/src/sangwon_AI/ops/final_field_session.py" "${FLIGHT_ARGS[@]}" --check-only
```

검사가 성공하면 같은 인자로 웹 세션을 연다.

```bash
python3 "$FLIGHT_SOURCE/src/sangwon_AI/ops/final_field_session.py" "${FLIGHT_ARGS[@]}"
```

이 명령 자체는 START·ARM을 제출하지 않는다.
관측 송신 후 DISARM에서 실제 FC 융합을 확인한다.
첫 시험은 TAKEOFF → POSITION 5초 → 별도 START다.
미션 시작 XY는 안정된 POSITION의 관측값이다.
제자리 호버 10초 뒤 원래 이륙점으로 복귀·착륙한다.
이 흐름 확인 후 좌표 이동을 시험한다.

거절·공백 때 마지막 승인 목표를 유지한다.
새 UWB 관측이 발행되기 전에는 미션을 진행하지 않는다.
PX4 추정 불가·좌표 리셋·조종자 개입은 별도로 처리한다.
FREEZE가 실제 위치 정확도나 무한 호버를 보장하지 않는다.

## 4. 센서 연결 복구

문제가 있으면 먼저 웹 실행을 종료한다.
아래 검사를 통과한 DISARM·지상에서만 되돌린다.
원래 소스와 서비스는 그대로 보존돼 있다.

```bash
python3 "$FLIGHT_SOURCE/src/drone_uwb/tools/native_btf_readiness.py" \
  --require-idle --output "$FLIGHT_RUN/before-rollback.json" && \
  rm -- ~/.config/systemd/user/dronestock-manual-observe.service.d/native-btf-20261014.conf && \
  systemctl --user daemon-reload && \
  systemctl --user restart dronestock-manual-observe.service
```

다른 drop-in·LiDAR 서비스·FC 파라미터는 변경하지 않는다.
