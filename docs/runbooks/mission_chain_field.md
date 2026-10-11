# 전체 미션 현장 시험 절차
완성된 native 실행 흐름을 현장에 연결하는 절차다.
2026-10-09 첫 시험 비행과 미션 준비 때 읽는다.

코드는 `codex/mission-flight-flow-20261008`을 사용한다.
main과 어제 통합 폴더의 수정은 보존한다.
가상 시험 성공과 실물 비행 성공은 다르다.
10월 9일 실물 파라미터 저장·재부팅을 수행했다.
실물 ARM·이륙·이동 명령은 보내지 않았다.
현재 값은 [현장 준비 기록](../report/field_readiness_20261009.md)을 따른다.

## 1. 실행 코드와 단일 명령 주체를 확인한다

Jetson 검토 폴더에서 빌드한다.
기존 관측 서비스와 센서의 실행 주체를 먼저 확인한다.
USB·MAVROS·UWB 프로세스를 중복으로 시작하지 않는다.
이유: 출처가 섞이면 관측과 명령을 검증할 수 없다.

```bash
cd /home/arialhanho/ROS2-review-20261008-codex
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb drone_demo drone_platform_link drone_mission
cmake -S src/sangwon_AI -B build/sangwon_ai_replay
cmake --build build/sangwon_ai_replay -j2
source install/setup.bash
export ROS_DOMAIN_ID=2
ros2 node list
```

Python의 simulator 의존성 경로를 빌드에 섞지 않는다.
새 setuptools가 Humble symlink 빌드 옵션과 충돌했다.
빌드는 ROS 기본 환경에서 수행한다.
관측 서비스의 단순 활성 상태는 비행 준비 증거가 아니다.

명령 주체는 `flight_mission` 하나다.
C++ AI는 `sangwon_native_plan`으로 지도만 검사한다.
C++ native hover는 같은 writer 잠금을 공유한다.
동시에 실행하면 두 번째 명령 주체는 잠금에서 실패한다.
legacy Offboard·ArUco servo 출력을 함께 연결하지 않는다.
이유: 고도·명령 책임이 PX4와 단일 실행기에서 분리된다.

## 2. 실측값을 별도 설정에 기록한다

`full_mission_tag_b.json`을 별도 현장 파일로 복사한다.
`uwb_btf_tag_b.json`도 별도 현장 파일로 복사한다.
배포 기본 확인값은 모두 false다.
실측·ULog 근거가 있는 항목만 true로 바꾼다.
가상 1.7m·잡음 0.1·covariance 0.01m를 복사하지 않는다.
이유: 현장 잡음과 센서 장착을 아직 보정하지 않았다.

| 항목 | 필요한 근거 |
|---|---|
| 배치·Tag·domain | Tag B=6/domain 2, 앵커 실측 위치·0.15m 높이 |
| UWB | 네 거리·개별 시각·TDMA epoch·정지/이동 원본 |
| 좌표 정렬 | A1/+X/+Y와 FC ENU 원점·yaw 변환 실측 |
| 장착 | ToF 축·바닥 높이·UWB 안테나 body FRD |
| 시간 | BTF 관측 지연·EV_DELAY·bridge 동기화 연속 정상 |
| 융합 | ULog에서 EV XY·flow·range 활성, innovation·속도 일관성 |
| 이륙 | MIS_TAKEOFF_ALT·COM_TAKEOFF_ACT=0 실제 읽기 |
| 방향 | EKF2_MAG_TYPE 실제 읽기, yaw 방향 전환·드리프트 |
| 수동 인계 | 조종기 수신·POSCTL 인계·PX4 failsafe |
| 기록 | 지속 기록 중 pose/관측 나이·저장 지연 |

BTF의 `height.mount_confirmed`를 실측 뒤 설정한다.
`height.flat_floor_confirmed`도 바닥 확인 뒤 설정한다.
`tof_to_tag_body_flu_m`에는 실제 장착 벡터를 쓴다.
앵커·편향·layout_id는 측량 파일과 맞춘다.
확인 플래그만 바꾸고 0 벡터를 실측값으로 쓰지 않는다.
이유: 기울 때 안테나 높이 보정이 달라진다.
launch에는 별도 `btf_config`·`anchor_file`을 지정한다.
기본 BTF의 미확인 장착은 관측을 발행하지 않는다.

시동 전에 실제 RC 채널 4개 이상·1초 이내 입력을 요구한다.
COM_RC_IN_MODE=0과 COM_RC_OVERRIDE의 AUTO 비트도 읽는다.
가상 domain 99는 별도 조이스틱 입력을 사용한다.
가상 수동 모드 시험이 실제 조종기 인계 검증을 대신하지 않는다.

10월 9일 실물은 1.3m·MAG_TYPE=6으로 읽혔다.
RC_IN_MODE=0·OVERRIDE=3도 재부팅 뒤 읽었다.
방위 드리프트와 실제 RC 인계는 여전히 미확인이다.
고정 PX4 v1.17은 자력계 최종 정렬에 HAGL 1.5m를 쓴다.
전체 미션은 MAG_TYPE 0/1과 1.6m 미만 조합을 거부한다.
이유: 가상 1.3m에서 목표 yaw가 적용되지 않았다.

| 선택 | 확인 뒤 설정할 내용 | 한계 |
|---|---|---|
| 자력계 계속 융합 | 공간에 맞는 native 높이·HAGL 정렬 확인 | 천장·자기장 확인 필요 |
| Init 정책 | EKF2_MAG_TYPE=6, expected_ekf2_mag_type=6 | 비행 중 yaw drift 실측 필요 |

Init은 자력계로 초기 yaw만 잡는다.
[PX4 공식 파라미터](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#EKF2_MAG_TYPE)를 따른다.
소프트웨어의 예상값 변경은 FC 파라미터를 쓰지 않는다.
높이·heading 설정은 현장 공간과 센서 근거로 결정한다.

## 3. 지도와 미션을 준비한다

`native_map.template.json`은 실행 가능한 실측 지도가 아니다.
측량 결과로 boundary·altitude_zones·yaw_zones를 채운다.
선반·기둥·금지 구역은 forbidden에 넣는다.
기체 여유 공간·바닥에서 기체 원점 높이도 기록한다.
측량 완료 파일만 `survey_status=SURVEYED`로 표시한다.
파일 SHA-256을 런타임에 고정한다.

```bash
sha256sum /absolute/path/surveyed-map.json
```

`native_body_floor_height_m`은 native 호버의 실측 높이다.
`launch_body_floor_height_m`도 별도로 실측한다.
MIS_TAKEOFF_ALT과 바닥 높이의 관계는 FC 기준으로 확인한다.
이는 충돌 검사 값이다. companion z 명령을 만들지 않는다.
미션은 한 높이·구간당 1m 이내·최대 20개 작업을 쓴다.
AI는 장애물과 경계가 맞지 않는 구간을 거부한다.
새 장애물이 생기면 지도를 갱신한다.
이유: 이 경로는 동적 장애물 회피를 구현하지 않는다.

처음은 자동 출발점에서 호버·착륙을 확인한다.
다음은 가까운 waypoint 하나로 복귀·착륙까지 확인한다.
웹 천장·시험 경로는 [현장 설정 절차](field_web_settings.md)를 따른다.
그다음 yaw·hover·복수 경유점으로 늘린다.
scan의 x/y는 라벨 위치다.
기체 목표는 별도 `staging_xy_m`이다.
마커 보정은 그 점의 0.1m·10도 안에서만 받는다.

실물 스캔 adapter는 아직 미완성이다.
기존 QR String을 성공 결과로 위장하지 않는다.
이유: 실제 획득 시각·대상·창·저장 증거가 없다.
응답이 없으면 스캔 실패를 저장한다.
대기점 복귀 뒤 남은 작업을 계속한다.
스캔은 다음 계약을 완성한 뒤 실물 미션에 포함한다.

| 경계 | 요구 |
|---|---|
| `/mission/scan_request` | 실행·작업·창, OPEN/SCAN/CANCEL, 0.2초 lease |
| `/mission/marker_observation` | 원본 촬영 stamp, 지도 XY/yaw, 대상·보정·장착 참조 |
| `/mission/scan_result` | 원본 획득 stamp, label/marker/context, stored/result_id |

마커·QR·저장 결과는 창을 연 뒤 실제 소스에서 만든다.
촬영·스캐너 adapter의 보정·재투영 품질 시험은 미실시다.

## 4. 지상 관측을 먼저 실행한다

통합 launch는 기존 실행 주체와 겹치지 않을 때 사용한다.
MAVROS가 없으면 지상에서 `start_mavros:=true`를 추가한다.
이 단계는 execute=false·bridge_enabled=false다.

```bash
ros2 launch drone_mission test_flight.launch.py tag:=B \
  config:=/absolute/path/field-full-mission.json \
  btf_config:=/absolute/path/field-btf.json \
  anchor_file:=/absolute/path/surveyed-anchors.json \
  native_plan_binary:=/home/arialhanho/ROS2-review-20261008-codex/build/sangwon_ai_replay/sangwon_native_plan \
  native_map_file:=/absolute/path/surveyed-map.json \
  native_map_sha256:=ACTUAL_SHA256 \
  record_directory:=/absolute/path/new-field-trial
```

웹도 동일한 설정을 사용한다.
운영 웹 1.1 snapshot은 아직 이 실행기와 직접 연결되지 않았다.
이번 시험은 로컬 HTTP/WS 경계를 쓴다.

```bash
ros2 run drone_mission local_flight_web --config /absolute/path/field-full-mission.json
```

같은 Jetson에서 `http://127.0.0.1:8001`을 연다.
다른 PC의 접속은 서버 `--host`와 방화벽 설정을 확인한다.
웹 route JSON에 실측 좌표를 넣고 START를 누른다.
첫 시험은 출발점의 hover 작업 한 개를 사용한다.
아래 좌표는 형식 예시다. 실제 출발점으로 바꾼다.

```json
[{"id":"H1","type":"hover","x":2.0,"y":2.0,"dwell_s":2.0}]
```

다음 시험은 가까운 waypoint를 추가한다.
scan은 실물 adapter가 준비된 뒤 추가한다.
계획 거부 이유·현재 FC 상태·완료 결과를 웹에서 확인한다.

## 5. 조종자가 준비된 시험에서 실행한다

센서·지도·RC·기록 근거를 확인한 설정만 사용한다.
조종기·앵커·비행 배터리·현장 조종자가 준비돼야 한다.
실행 launch에 execute:=true·bridge_enabled:=true를 추가한다.
웹 START는 AUTO.TAKEOFF 확인 뒤 ARM을 요청한다.
소프트웨어는 지상 정지 3초와 각 관측 조건을 다시 검사한다.

상승 완료·수직 속도 정지·XY 안정 2초 후 이동한다.
수동 모드가 우선한다. 자동 재획득·재시동은 하지 않는다.
END는 출발점 복귀·새 착륙 상태·disarm을 확인한 결과다.
mission_complete=true는 모든 작업 성공까지 요구한다.
UNCONFIRMED는 실물 상태를 확인해야 하는 종료다.

스캔 실패·응답 없음은 해당 작업 실패로 저장한다.
대기점 복귀를 확인한 뒤 남은 작업을 계속한다.
마지막 작업을 시도한 후 출발점 복귀·착륙을 수행한다.
조종자 중단·배터리·비행 상태 이상은 기존 중단 정책을 따른다.
스캔 실패가 비행 이상을 무시하게 만들지 않는다.
이유: 작업 실패와 기체 제어권 상실은 다른 결과다.

| 웹 표시 | 의미 |
|---|---|
| 시도한작업 | 대기점 복귀까지 끝낸 작업 ID |
| 남은작업 | 현재 작업부터 아직 시도 중인 작업 ID |
| 스캔실패작업 | 실패 기록이 있는 scan ID |
| 전체경로수행 | 전체 작업 시도·복귀·착륙 확인 |
| 임무완료 | 모든 스캔 성공까지 확인 |

일부 스캔 실패 뒤 END는 정상 비행 종료다.
이때 전체경로수행=true·임무완료=false로 표시한다.
실패 작업을 자동으로 다시 무한 시도하지 않는다.
이유: 남은 작업과 비행 시간을 확보해야 한다.

시험 후 ULog·events·BTF 원본·웹 결과를 함께 보존한다.
첫 실물 비행에 고장 주입을 합치지 않는다.
가상 성공은 추진계·바닥 무늬·조도·자기장 증거가 아니다.
