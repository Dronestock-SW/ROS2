# 플랫폼 연결 패키지 안내

플랫폼 임무 수신과 상태 전달 서비스의 입구다. 외부 API 연결이나 서비스 설정을 수정할 때 읽는다.

[멀티태그·웹 연결 인수인계](../../docs/runbooks/multitag_jetson_web_handoff.md)를 먼저 확인한다.
웹 좌표는 Wi-Fi/WebSocket으로 보낸다. LoRa 역송신은 필요 없다.
기본 입력은 `/uwb/btf_pose`다. `uwb_xy`·`btf_xy`·`px4_local`을 선택한다.
서명된 관측 WS와 수락 ACK를 사용한다. 기본 임무 조회는 꺼져 있다.
XY의 Z는 null이며 PX4 로컬 XYZ의 창고 정렬은 남아 있다.

| 역할 | 위치 |
|---|---|
| 서비스 실행·수신 | `drone_platform_link/runtime.py`, `setup.py`의 `platform_link` |
| ROS 상태 읽기 | `drone_platform_link/ros_monitor.py` |
| 상태 메시지 구성 | `drone_platform_link/telemetry.py` |
| 서비스·환경 설정 | `deploy/` |
| 로컬 검증 | `test/test_runtime.py` |

기존 모듈 규모와 배포 경로를 유지했다.
실행 의존성은 `websockets==13.1`이다.

기본값은 기존 통신 전용 모드다.
`DRONESTOCK_MISSION_FORWARDING=true`일 때만
GET 응답을 `/mission/assignment`로 전달한다.
비행 명령은 `drone_mission`이 담당한다.
`DRONESTOCK_UWB_TOPIC=/uwb/btf_pose`로 실물 후보를 읽는다.
`DRONESTOCK_WS_URL`로 로컬의 별도 WS 포트를 지정한다.
실제 미션 상태가 들어올 때 ACK·단계 보고를 전송한다.
WebSocket 수신 프레임은 명령으로 해석하지 않는다.

[웹 비행 시험 절차](../../docs/runbooks/web_test_flight.md)와
[연결 구조](../../docs/architecture/web_test_flight.md)를 따른다.
[서비스 절차](../../docs/runbooks/companion_platform_service.md)와
[API 사전](../../docs/reference/companion_platform_api_dictionary.md)을 따른다.
