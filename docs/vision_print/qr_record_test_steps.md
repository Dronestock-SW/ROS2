# QR 로컬 저장 시험 절차
이 문서는 출력물 없이 QR 저장을 시험하는 절차다.
`/qr/item` 처리 확인 또는 카메라 시험 때 읽는다.

## 1. 저장 노드 실행
저장 경로를 지정하면 로컬 관측을 기록한다.
workspace 루트에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/drone_bringup:${AMENT_PREFIX_PATH:-}"
ros2 run drone_bringup qr_recorder_node --ros-args \
  -p output_path:=/tmp/dronestock_qr_test.sqlite3
```

`/tmp`는 임시 시험 경로다. 장기 보관에는 쓰지 않는다.
시스템 정리나 재부팅으로 파일이 사라질 수 있다.
장기 보관 시 별도 데이터 폴더 경로를 지정한다.
DB를 build/install/log 폴더에 넣지 않는다.
자동 생성물 정리 때 기록도 삭제될 수 있다.

## 2. 예시 관측 보내기
다른 터미널에서 같은 ROS 환경을 불러온다.
아래 명령은 카메라 없이 저장 경로를 확인한다.
파서와 디코더 시험을 대신하지 않는다.

```bash
ros2 topic pub --once /qr/item std_msgs/msg/String \
  "data: '{\"schema\":\"drone-stock-item/v1\",\"code\":\"TEST\",\"name\":\"TEST1\"}'"
```

로그의 `Saved QR observation`을 확인한다.
저장 실패 시 `QR record not saved`를 출력한다.

## 3. 저장 규칙
동일 내용의 연속 판독만 억제한다.
재고 수량 계산에는 이 건수를 쓰지 않는다.
같은 QR을 가진 제품 여러 개를 구별할 수 없다.

| 항목 | 규칙 |
|---|---|
| 비교 대상 | schema/code/name/zone/shelf/slot/location/date |
| 중복 시간 | 같은 내용의 마지막 수신 후 3초 미만 |
| 반복 수신 | 중복 시간을 계속 갱신 |
| 재등장 | 3초 이상 공백이면 새 관측 |
| 설정 | `duplicate_window_sec`, 양수 |
| 노드 재시작 | 새 세션. 기존 파일은 보존 |
| 저장 실패 | 저장 완료로 처리하지 않음. 다음 수신 때 재시도 |
| ID | 저장마다 UUID. 파일 재조회 시 그대로 유지 |
| 시각 | UTC. 로컬 저장 처리 시각 |
| 데이터 | SQLite `qr_observations` 테이블 |

## 4. 적용 범위
현재는 로컬 시험 기록만 저장한다.
자동 업로드와 미전송 버퍼는 구현하지 않았다.
노드는 명시적으로 실행한 동안 모든 `/qr/item`을 기록한다.

`item_json`은 파서가 정리한 JSON이다.
QR 원문 `raw_qr_data`로 취급하지 않는다.
기록에는 실제 임무·라벨 ID가 아직 없다.
임무별 중복 기준과 스캔 시작·종료 신호도 미합의다.
서버 연동 전에 이 정보의 공급 경로를 정한다.
