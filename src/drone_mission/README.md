# 웹 비행 시험 패키지

로컬 웹의 이륙·이동·착륙 요청을 PX4에 연결한다. 비행 시험을 준비할 때 읽는다.

실행은 [현장 절차](../../docs/runbooks/web_test_flight.md)를 따른다.
[구조 설명](../../docs/architecture/web_test_flight.md)에 관측·제어 책임을 적었다.
로컬 세션은 [인계 절차](../../docs/runbooks/local_jetson_handoff_20261008.md)부터 읽는다.
현재 상태는 [로컬 통합 기록](../../docs/report/local_jetson_integration_20261008.md)에 있다.
Tag A/B·TDMA·장착값 검사와 웹 요청을 함께 보존한다.

| 역할 | 구현 |
|---|---|
| 좌표·요청·시험 범위 검사 | `drone_mission/contracts.py` |
| PX4 비행 순서·중단 판단 | `drone_mission/session.py` |
| 전체 작업·정렬·출발점 복귀·END | `drone_mission/mission_chain.py` |
| 기존 C++ 지도 검사·실측 지도 hash 고정 | `drone_mission/native_ai.py`, `sangwon_native_plan` |
| MAVROS 상태·서비스 연결 | `drone_mission/node.py` |
| 로컬 웹·HTTP·WebSocket | `drone_mission/local_web.py` |
| 실물 센서·MAVROS 통합 실행 | `launch/test_flight.launch.py` |
| 정렬·확인값·시험 한도 | `config/flight.json` |
| Tag B·임시 0.15m·이륙 기대값 1.3m | `config/flight_tag_b.json` |
| Tag B 전체 미션·확인값 false | `config/full_mission_tag_b.json` |
| 기체·배치 일치 검사 | `drone_mission/profiles.py` |
| C++ hover와 공통 명령 잠금 | `drone_mission/writer_lock.py` |
| 전체 순서·단절·재요청 검사 | `test/` |
| 기존 실물 ROS 읽기 전용 수집 | `tools/observe_ground.py` |

기본값은 `execute=false`, `bridge_enabled=false`다.
`start_mavros=false`이므로 기존 MAVROS를 먼저 확인한다.
`tag:=B`는 domain 2·ID 6을 함께 선택한다.
웹도 같은 `--config`를 사용해야 배치가 일치한다.
확인 항목도 모두 `false`다.
높이 계산은 UWB 관측 후보정에만 쓴다.
비행 고도와 자세는 PX4가 제어한다.

```bash
# localhost 시험 페이지. 비행 명령 노드는 시작하지 않는다.
ros2 run drone_mission local_flight_web
# 브라우저: http://127.0.0.1:8001
```

웹 입력은 `waypoint`와 `hover` 경유지를 받는다.
`full_mission=true`는 스캔 계약·대기점 복귀·END를 추가한다.
scan x/y는 라벨이며 별도 `staging_xy_m`이 필요하다.
실물 카메라·스캐너 adapter는 후속이다.
전체 미션 START는 C++ 지도 검사를 요구한다.
지도 hash와 실제 출발점 XY를 묶는다.
가상 지도는 실물 프로필에서 거부한다.
[현장 전체 미션 절차](../../docs/runbooks/mission_chain_field.md)를 따른다.
실제 PX4와 모사 센서 시험은 [전체 미션 절차](../../docs/runbooks/mission_chain_virtual.md)를 따른다.
운영 웹의 `start` 지원은 별도 반영이 필요하다.
