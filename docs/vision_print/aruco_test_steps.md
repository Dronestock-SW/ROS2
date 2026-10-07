# ArUco 연구용 라벨 시험 절차
이 문서는 100mm 라벨의 시험 순서다.
출력물을 받은 뒤 카메라 시험 전에 읽는다.

## 1. 출력 크기 설정
검은 정사각형을 재고 설정에 반영한다.

| 항목 | 값 |
|---|---|
| 시험 설정 | `src/drone_bringup/params/aruco_tracker_100mm_test.yaml` |
| `marker_size` | 실측 한 변 길이. 단위 m |
| 예시 | 100mm이면 `0.100` |
| 기존 기본 설정 | 159mm. 변경하지 않음 |

## 2. 실행
workspace 루트에서 실행한다.
카메라가 이미 실행 중이면 아래 명령을 쓴다.
이 환경은 패키지 검색 경로를 명시해야 한다.

```bash
source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/drone_bringup:${AMENT_PREFIX_PATH:-}"
ros2 launch drone_bringup aruco_servoing.launch.py \
  aruco_params_file:="$PWD/src/drone_bringup/params/aruco_tracker_100mm_test.yaml"
```

카메라도 실행하려면 `start_camera:=true`를 덧붙인다.
카메라를 중복 실행하지 않는다. 장치를 동시에 열 수 없다.
QR 디코더는 별도 터미널에서 실행한다.

```bash
source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install/drone_bringup:${AMENT_PREFIX_PATH:-}"
ros2 run drone_bringup qr_decoder_node
```

## 3. 확인
25cm가 목표거리다 (`target_distance_m` 기본값). 허용 범위는 23~27cm다.

하한은 21.1cm다. ArUco를 화면 중앙에 정렬하면 그 아래부터 QR 우측이 잘린다.
2026-10-04 실측 21.2cm로 확인했다(817표본). 계산과 0.1cm 차이다.
17.8cm 아래는 어떤 조건에서도 0%다. 과접근 안전 한계로 쓴다.
상한은 실측 47.4cm다. 25cm에서 2배 가까이 떨어져 있어 제약이 안 된다.
산출 근거는 `aruco_alignment_node.py`의 "목표거리 재검토" 참조.

라벨은 **거치해서** 카메라와 나란히 세운다. 손에 들면 측정이 안 된다.
기울기가 거리보다 판독을 더 좌우하기 때문이다:

| 조건 | 27~47cm 판독률 |
|---|---|
| 거치, 기울기 9~10도 | 85~100% |
| 손에 듦, 기울기 혼재 | 37cm 9% / 45cm 95% (역전) |

기울기 13도면 39~41cm 판독률이 55%로 떨어진다.
책상에 눕히면 ArUco가 화면 아래로 잘린다.

비행 거리는 미확정이다 — 하한 21.1cm는 기하 계산이고 비행 실측 전이다.

| 조작 | 확인할 결과 |
|---|---|
| 두 표식을 카메라에 보여줌 | `/aruco_detections`, `/qr_code/data` 확인 |
| 마커를 좌우·앞뒤로 이동 | `/aruco_alignment/error` 값 변화 |
| 마커를 가림 | `valid=false`, 속도 제안값 0 |
| 검출 노드를 종료 | 마지막 유효 오차 수신 후 0.5초에 만료 |
| 마커를 다시 보여줌 | 유효 오차 수신 후 속도 제안 재개 |

만료 검사는 0.05초 주기로 실행한다.
실제 발행 시점에는 실행 지연이 더해질 수 있다.
`error_timeout_sec` 기본값은 0.5초다.
`valid`가 없는 구형 오차 메시지는 무효로 처리한다.

## 4. 결과의 범위
속도 제안값은 PX4에 연결하지 않았다.
0속도 제안은 실제 기체의 정지를 보장하지 않는다.

좌표축과 부호는 실기 검증 전이다.
현재 목표는 ArUco 중심이다.
옆의 QR이 함께 보이는지는 출력물로 확인한다.
제안 노드 자체가 종료되는 경우도 있다.
실제 제어 수신부에도 입력 만료 처리가 필요하다.
