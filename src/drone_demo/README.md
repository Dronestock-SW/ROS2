# 데모·임무 패키지 안내

표시용 데모와 별도 SITL 임무·평가의 입구다. 입력 생성, 목표 전달, 기록 평가를 수정할 때 읽는다.

| 역할 | 코드 | 설정·시험 |
|---|---|---|
| 표시용 입력·기록 | `core.py`, `node.py`, `export.py`, `cycle.py` | `config/demo.json`, `test/test_demo.py` |
| 임무 관측 | `mission.py`, `mission_node.py` | `config/mission.json`, `test/test_mission.py` |
| SITL 목표·도착 | `sitl_navigation.py`, `sitl_mission.py` | `config/gazebo_navigation_plan.json`, `test/test_sitl_*.py` |
| 독립 비행 평가 | `flight_evaluation.py`, `flight_metrics.py`, `flight_fault_evaluation.py` | `config/gazebo_*evaluation*.json`, `test/test_flight_*.py` |

코드는 `drone_demo/` 아래에 있다.
실행 진입점은 `setup.py`와 `launch/`에서 찾는다.
기존 규모가 작아 파일 구조는 유지했다.

[임무 설계](../../docs/architecture/demo_mission_design.md),
[SITL 실행](../../docs/runbooks/uwb_gazebo_navigation_runbook.md),
[독립 평가](../../docs/architecture/uwb_gazebo_flight_evaluation.md)를 따른다.
