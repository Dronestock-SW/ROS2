# 센서 실행 패키지 안내

센서 launch, 표시용 마커, 비행 로그 점검의 입구다. 센서 실행 설정이나 장착·표시 자료를 찾을 때 읽는다.

| 역할 | 위치 |
|---|---|
| LiDAR·카메라 실행 | `launch/`, `params/` |
| QR 재고 판독 | `launch/qr_inventory.launch.py`, `drone_bringup/qr_*` |
| 인쇄 마커 원본 | `assets/markers/aruco_id0.png`, `aruco_id1.png` |
| 위치 유지 로그 점검 | `tools/analyze_position_hold.py`, `test/test_position_hold_audit.py` |

마커 0의 159mm 인쇄 기록은 `params/aruco_tracker.yaml`에 있다.
마커 1의 사용 여부는 코드 참조로 확인하지 못했다.
두 원본의 바이트와 루트 호환 링크를 보존했다.
기존 코드·파라미터와 제조사 드라이버는 변경하지 않았다.

[장비 목록](../../docs/equipment_inventory.md)과
[로그 점검 절차](../../docs/runbooks/position_hold_audit_runbook.md)를 따른다.

## QR 재고 판독 실행

**선반 정렬 뒤 스캔 세션을 시작해야 실패 횟수를 센다.**

```bash
ros2 launch drone_bringup qr_inventory.launch.py
ros2 service call /qr_fallback_node/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

리더기 입력이 3초씩 3회 연속 없으면 카메라 판독을 켠다.
리더기나 카메라가 한 건을 읽으면 `/qr/data`를 발행하고 세션을 끝낸다.
다음 선반에서는 서비스를 다시 호출한다.

이미 `camera.launch.py`가 실행 중이면 CSI 카메라를 중복으로 열지 않는다.

```bash
ros2 launch drone_bringup qr_inventory.launch.py start_camera:=false
```

기본 DB는 `~/.local/share/dronestock/qr_inventory.sqlite3`에 저장한다.
[QR 파이프라인 설계](../../docs/architecture/qr_inventory_pipeline.md)를 따른다.
