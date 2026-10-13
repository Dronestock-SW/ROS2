# C++ 실행기 지상 시험

웹 명령을 C++ 실행기로 연결하는 절차다.
지상 시험과 후속 현장 인계에서 읽는다.

**실비행 성공과 초기 밀림 해결은 아직 미확인이다.**
설정·보정·RC 기준은 기존 FINAL과 같다.
시험용 승인 값은 실물 설정으로 복사하지 않는다.
모사 센서의 합격을 실측으로 바꿀 수 없기 때문이다.

## 격리 빌드와 자동 시험

운영 프로세스와 다른 검토 디렉터리에서 빌드한다.
아래 자동 시험은 실제 FC에 연결하지 않는다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to drone_mission drone_platform_link
source install/setup.bash
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
cmake -S src/sangwon_AI -B .review/cpp-executor-build
cmake --build .review/cpp-executor-build --target \
  sangwon_field_executor sangwon_field_executor_replay \
  sangwon_field_mission_ai sangwon_lidar_relative_replay -j2
ctest --test-dir .review/cpp-executor-build \
  -R 'field_executor_contracts|field_mission_ai_contracts|lidar_relative_contracts' \
  --output-on-failure
export PYTHONPATH="$PWD/src/drone_mission:$PWD/src/drone_uwb:$PWD/src/drone_platform_link:$PWD/src/drone_demo:$PWD/src/sangwon_AI/tests:$PYTHONPATH"
python3 src/sangwon_AI/tests/field_executor_ros.py \
  "$PWD/.review/cpp-executor-build/sangwon_field_executor" \
  "$PWD/.review/cpp-executor-evidence-$(date +%s)"
python3 src/sangwon_AI/tests/field_executor_ros.py \
  "$PWD/.review/cpp-executor-build/sangwon_field_executor" \
  "$PWD/.review/cpp-platform-evidence-$(date +%s)" platform_link
```

기존 requirements-field.txt의 라이브러리를 준비한다.
현장 launch에는 새 부팅 보정·요청 원장이 필요하다.
[FINAL 재개 절차](final_resume.md)의 인자를 유지한다.
C++ 경로를 고를 때 인자 하나를 추가한다.

```text
--cpp-executor-binary /절대경로/sangwon_field_executor
```

이 인자는 Python FlightNode를 실행하지 않는다.
Python CppObserver와 C++ 송신기를 실행한다.
기존 `--cpp-ai-binary`는 호환 인자로 유지했다.
C++ 실행기는 공통 AI 코드를 직접 사용한다.
별도의 Python↔AI stdio 왕복은 이 경로에 없다.
새 실행기 프로세스의 nonce가 바뀌면 자동 재개하지 않는다.

## 현장 전 필요한 대조

1. RC·앵커를 켠 지상에서 START 검사 결과를 확인한다.
2. LiDAR X·Y·yaw와 정확한 기준점 Z를 측정한다.
3. 통제된 이동으로 LiDAR 방향·크기·시각을 대조한다.
4. FC 속도와 실측 이동을 같은 시각에 기록한다.
5. 장시간 실제 부하에서 송신 간격을 기록한다.

실물 이동 대조는 [LiDAR 지상 절차](lidar_ground_qualification.md)를 따른다.
원본 UWB/B_TF·ULog·영상도 같은 시험에 보존한다.
지상 실패 원인을 숨기고 START 기준을 완화하지 않는다.
좌표·시각 오류가 비행 중 자세 보정으로 이어지기 때문이다.
