# 바닥 XY 초기화·연속 지상 EV 기록

바닥에서 첫 UWB XY가 막힐 때 읽는 검증 기록이다.
2026-10-09 수정과 실제 배포 결과를 구분한다.

## 수정

유효3앵커 후보에도 이전 XY를 요구하던 순환을 해소했다.
기본 경로는 유지한다. 현장 설정으로만 선택한다.
`ground_subset_initialization=true`를 사용한다.
지상 높이 모델이 활성인 순간에만 초기화한다.
같은 제외 앵커·유일 후보를2초 이상 확인한다.
최소30표본·최대 공백0.1초·반경0.05m를 요구한다.
기존 거리 RMS·후보 차이 검사를 유지한다.
재획득은 마지막 유효 XY의0.1m 안에서만 한다.
공중·유효 ToF 경로의 초기화 정책은 바꾸지 않았다.
FC 위치를 UWB 해로 복사하지 않는다.

`ground_ev_trial.py --continuous-ground`를 추가했다.
45초 종료 대신 이번 부팅의 지상 관측을 계속한다.
DISARM·지상·RC kill ON·명령 출력 비활성을 요구한다.
시각 동기화·원본 시각·공분산 검사를 유지한다.
짧은 입력 공백에는 새 관측을 발행하지 않는다.
명령 권한 조건이 바뀌면 종료한다.
출력 잠금을 소유해 비행 실행기와 동시에 쓰지 않는다.
새 부팅에는 이전 후보 좌표를 사용하지 않는다.
요약 기록은 교체하며 원본 표본은 최근1000개만 보관한다.
동적 정렬·고정 지연 확인 플래그는 false다.
이 모드는 비행용 상시 bridge의 승인과 별개다.

## 검증

Windows 관련 시험67개를 통과했다.
ROS 전용3개는 Jetson 검증 대상으로 남았다.
이전 바닥 거리442개를 지상 높이 모델로 재생했다.
수정 전은 초기화되지 않았다.
수정 후300개를 수락하고 마지막 XY를 반환했다.
마지막 후보는 약(5.266,2.134)m였다.
이는 재생 결과다. 절대 위치 정확도 실측은 아니다.

```bash
python3 -m pytest src/drone_uwb/test/processing/test_ground_initialization.py src/drone_uwb/test/processing/test_measured_btf.py src/drone_uwb/test/processing/test_ground_ev_trial.py src/drone_uwb/test/integration/test_btf_px4_bridge.py -q
colcon build --symlink-install --packages-select drone_uwb
python3 src/drone_uwb/tools/ground_ev_trial.py --candidate FIELD_CANDIDATE --output NEW_SESSION --continuous-ground
```

실제 배포·융합 결과는 후속 기록으로 추가한다.
ARM·이륙·동적 정렬 검증은 이 수정의 증거가 아니다.
