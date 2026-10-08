# 센서 실행 패키지 안내

센서 launch, 표시용 마커, 비행 로그 점검의 입구다. 센서 실행 설정이나 장착·표시 자료를 찾을 때 읽는다.

| 역할 | 위치 |
|---|---|
| Tag별 UWB·PX4·AI 관측 | `launch/companion_observe.launch.py` |
| LiDAR·카메라 실행 | `launch/`, `params/` |
| QR 처리 노드 | `drone_bringup/qr_*_node.py`, `setup.py` |
| 인쇄 마커 원본 | `assets/markers/aruco_id0.png`, `aruco_id1.png` |
| 위치 유지 로그 점검 | `tools/analyze_position_hold.py`, `test/test_position_hold_audit.py` |

마커 0의 159mm 인쇄 기록은 `params/aruco_tracker.yaml`에 있다.
마커 1의 사용 여부는 코드 참조로 확인하지 못했다.
두 원본의 바이트와 루트 호환 링크를 보존했다.
제조사 드라이버의 고정 버전을 유지한다.
통합 관측은 [지상 절차](../../docs/runbooks/flight_uwb_ai_ground.md)를 따른다.
2026-10-07에는 package.xml 주석 문법을 수정했다.
ROS 패키지 검색과 launch 설치를 복구하기 위해서다.

[장비 목록](../../docs/equipment_inventory.md)과
[로그 점검 절차](../../docs/runbooks/position_hold_audit_runbook.md)를 따른다.
