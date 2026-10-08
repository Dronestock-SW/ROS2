# 웹 비행 시험 패키지

로컬 웹의 이륙·이동·착륙 요청을 PX4에 연결한다. 비행 시험을 준비할 때 읽는다.

실행은 [현장 절차](../../docs/runbooks/web_test_flight.md)를 따른다.
[구조 설명](../../docs/architecture/web_test_flight.md)에 관측·제어 책임을 적었다.
로컬 세션은 [인계 절차](../../docs/runbooks/local_jetson_handoff_20261008.md)부터 읽는다.
어제 Tag B·C++ AI 통합 브랜치와의 대조·통합이 먼저다.

| 역할 | 구현 |
|---|---|
| 좌표·요청·시험 범위 검사 | `drone_mission/contracts.py` |
| PX4 비행 순서·중단 판단 | `drone_mission/session.py` |
| MAVROS 상태·서비스 연결 | `drone_mission/node.py` |
| 로컬 웹·HTTP·WebSocket | `drone_mission/local_web.py` |
| 실물 센서·MAVROS 통합 실행 | `launch/test_flight.launch.py` |
| 정렬·확인값·시험 한도 | `config/flight.json` |
| 전체 순서·단절·재요청 검사 | `test/` |
| 기존 실물 ROS 읽기 전용 수집 | `tools/observe_ground.py` |

기본값은 `execute=false`, `bridge_enabled=false`다.
확인 항목도 모두 `false`다.
높이 계산은 UWB 관측 후보정에만 쓴다.
비행 고도와 자세는 PX4가 제어한다.

```bash
# localhost 시험 페이지. 비행 명령 노드는 시작하지 않는다.
ros2 run drone_mission local_flight_web
# 브라우저: http://127.0.0.1:8001
```

웹 입력은 `waypoint`와 `hover` 경유지를 받는다.
QR 스캔·선반 정렬은 이번 실행기에 연결하지 않았다.
운영 웹의 `start` 지원은 별도 반영이 필요하다.
