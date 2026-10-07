# UWB 패키지 안내

수신·후보정·관측 연결 코드의 입구다. 기능을 수정하거나 해당 시험을 찾을 때 읽는다.

[멀티태그·웹 연결 인수인계](../../docs/runbooks/multitag_jetson_web_handoff.md)에
TDMA 대응 검사와 A/B 공통 실행기를 반영했다.
실제 ToF·장착 확인과 최종 XYZ 검증은 남아 있다.
현재 태그와 연결할 때 먼저 확인한다.

[단계별 위치표](../../docs/reference/repository_layout.md)의 A~H를 따른다.
[API 사전](../../docs/reference/uwb_module_api.md)에 단위·실패 계약이 있다.

```text
drone_uwb/
|-- contracts/                 공통 계약
|-- acquisition/               UART·입력 검사
|-- processing/
|   |-- timing/                시간·센서 표본
|   |-- geometry/              Z·좌표·장착 위치
|   |-- solvers/               XY·H80·Q_S10·비교 풀이
|   |-- ranges.py              거리 교정·게이트
|   |-- pipeline.py            닫힌 파일 파이프라인
|   |-- settings.py            계산 설정 검사
|   |-- runner.py, simulator.py 파일 실행·합성 입력
|   `-- experiments/           독립 비교 실행기
|-- integration/
|   |-- ros/                   ROS 관측 노드·브리지
|   |-- gazebo/                가상 센서·기록·실행 연결
|   |-- sitl/                  PX4 관측·목표·시계 계약
|   `-- recording.py, replay.py 기록·기존 관측 재생
`-- preimu/                    이전 import 호환

config/  anchors/ runtime/ pipeline/ replay/ gazebo/ sitl/
test/    processing/ experiments/ integration/ + 경로 회귀 시험
launch/  uwb.launch.py
tools/   분석·그림·점검 도구
```

`processing/`와 `integration/`에 남은 옛 파일명도 호환 모듈이다.
구현은 위 하위 폴더에 한 번만 둔다.
기존 CLI 이름은 유지한다.
설정의 옛 파일명은 실제 설정으로 가는 상대 링크다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb
source install/setup.bash
PYTHONPATH=src/drone_uwb:src/drone_demo PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q src/drone_uwb/test
```

명령은 저장소 루트에서 실행한다.
환경별 미설치 의존성과 기존 실패는 [확인 기록](../../docs/report/repository_modularization_20261004.md)을 읽는다.
수신·재생 명령은 [실행 절차](../../docs/runbooks/uwb_data_runbook.md)에 있다.
