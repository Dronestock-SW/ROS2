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
[서비스 절차](../../docs/runbooks/companion_platform_service.md)와
[API 사전](../../docs/reference/companion_platform_api_dictionary.md)을 따른다.
