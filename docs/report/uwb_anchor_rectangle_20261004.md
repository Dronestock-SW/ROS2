# 앵커 직사각형 고정 기록

2026-10-04 배치 변경과 검증 기록이다. 새 좌표 적용 범위를 확인할 때 읽는다.

## 변경 내용

현재 기본 배치를 x=6.3m, y=4.6m로 바꿨다.
사용자가 A1-A2·A3-A4의 가로폭을 지정했다.
A1-A3·A2-A4의 세로폭도 함께 지정했다.
원점은 A1, +x는 A2 방향, +y는 A3 방향이다.
앵커 높이는 기존 2.2m를 유지했다.

| 앵커 | 이전 x, y (m) | 현재 x, y (m) |
|---|---|---|
| A1 | 0, 0 | 0, 0 |
| A2 | 약 5.810, -0.430 | 6.3, 0 |
| A3 | 0, 4.63 | 0, 4.6 |
| A4 | 약 6.110, 4.357 | 6.3, 4.6 |

기준은 [현재 앵커 좌표](../reference/uwb_anchor_layout.md)다.
새 파일은 `config/anchors/anchors_20261004.json`이다.
ROS 수신·데모 로더·Gazebo 생성의 기본 파일을 바꿨다.
Gazebo 비교 설정에도 같은 좌표를 반영했다.
이전 Gazebo 비교 기본값은 별도 가상 배치였다.
A1=(-3,-2.5), A2=(3,-2.5), A3=(-3,2.5), A4=(3.1,2.3)이었다.
현재는 ROS 기준 파일과 같은 원점·XY를 사용한다.

## 유지한 항목

과거 실측 재생은 당시 좌표를 계속 사용한다.

- `anchors_20260906.json`의 내용·해시를 보존했다.
- 정지 A/B/C/D 재생 설정의 좌표 파일 해시를 확인했다.
- 기존 `data/raw/`, `data/processed/`는 수정하지 않았다.
- 거리 bias·게이트·필터 계수와 비행 출력 조건은 유지했다.
- 기존 목표 허용 구역 x=0.5~5.3m, y=0.5~3.9m는 유지했다.

로드맵·현재 실행 절차·앵커 사전을 갱신했다.
이전 실측 사전과 태그 XY 갱신 문서는 이력으로 표시했다.

## 확인 결과

소스 설정과 계산 경로를 로컬에서 확인했다.

| 확인 항목 | 결과 |
|---|---|
| A1-A2, A3-A4 | 각각 6.3m |
| A1-A3, A2-A4 | 각각 4.6m |
| 대각선 | 약 7.80064m |
| Gazebo 좌표 일치 | 새 기준 JSON과 일치 |
| 무잡음 거리→XY 검산 | 서로 다른 3점. 오차 2.0e-15m 이하 |
| UWB·demo 기존 테스트 | 374 통과, 3개 모듈 skip |
| 과거 재생 설정의 좌표 해시 | A/B/C/D 설정 3개 모두 일치 |
| 패키지 빌드 | UWB·demo 2개 성공 |
| 설치 설정·데모 기본 로딩 | /tmp에서 새 배치 ID·바이트 일치 확인 |

무잡음 검산은 수식·설정 확인이다.
실제 위치 정확도를 측정한 결과는 아니다.
skip 사유는 기존 `pymavlink` 미설치다.

시험 명령은 저장소 루트에서 실행했다.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/drone_uwb:$PWD/src/drone_demo${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest -q -p no:cacheprovider src/drone_uwb/test src/drone_demo/test
colcon build --symlink-install --packages-select drone_uwb drone_demo
```

[변경값](evidence/uwb_anchor_rectangle_20261004/change.json),
[기하·과거 해시 검사](evidence/uwb_anchor_rectangle_20261004/geometry_checks.json),
[pytest 출력](evidence/uwb_anchor_rectangle_20261004/pytest.log),
[빌드 출력](evidence/uwb_anchor_rectangle_20261004/build.log),
[설치 확인](evidence/uwb_anchor_rectangle_20261004/install_checks.json)을 보관했다.

## 남은 확인

실제 장치 배포·실측 검증은 미실시다.
독립 측량, 새 배치의 거리 교정, 기체 시험은 남아 있다.
Z 높이는 이번 사용자 지시에서 변경하지 않았다.
실행 중인 노드와 기존 Gazebo 월드는 재시작·재생성하지 않았다.
새 배치로 가상 시험을 시작할 때 새 월드와 trial을 함께 만든다.
과거 RAW를 새 배치로 덮어 해석하지 않도록 구분한다.
