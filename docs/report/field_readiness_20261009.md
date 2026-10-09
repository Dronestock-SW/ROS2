# 2026-10-09 현장 준비 기록
자동 출발점·천장 명령·RC 인계의 변경 기록이다.
Tag B 현장 시험을 재개할 때 읽는다.

소프트웨어 시험과 실물 준비를 구분한다.
실물은 아직 비행 준비 완료가 아니다.
최초 확인 때 앵커·RC·추진 배터리는 준비 중이었다.
후속 RC 지상 확인은 아래 별도 기록을 따른다.
실물 ARM·TAKEOFF·이동·LAND 명령은 보내지 않았다.

## 기준과 보존

`codex/mission-flight-flow-20261008`에서 작업했다.
시작 commit은 `2c19cad7fb407756d67708bda6128507628fbd80`다.
fetch 뒤에도 원격 시작점은 같았다.
요청한 Windows 인계 파일은 이 PC에 없었다.
대신 브랜치의 인계·후속 스캔·현장 절차를 읽었다.

| 위치 | 처리 |
|---|---|
| Jetson `ROS2-review-20261008-codex` | 같은 브랜치에서 빌드·시험 |
| Jetson `ROS2-integration-20261007` | 기존 브랜치·미커밋 9개 파일 보존 |
| Jetson `Desktop/ROS2` | main·udev 수정·AI 폴더 보존 |
| 기존 sangwon 서비스 5개 | 중단·재설정하지 않음 |
| 이번 관측·가상 시험 | 생성한 PID와 기록만 관리 |

이전 main이나 현장 폴더를 reset하지 않았다.
FC 전체 파라미터 백업은 비공개 기록에 남겼다.
시각·장착·보정값·비행 기록은 Git에 넣지 않는다.

## 구현

| 코드 | 변경 |
|---|---|
| `mavros_test_flight.yaml` | 누락된 `rc_io` plugin 연결 |
| `preflight.py`·`node.py` | 미확인 입력과 최신 상태를 웹에 표시 |
| `local_web.py`·`site.py` | 천장 저장 명령·지상 편집·미션 고정 |
| `native_ai.py` | 천장으로 지도 제한·실측 높이와 여유 검사 |
| `field_presets.py` | 자동 출발점 기준 호버·X/Y/XY 시험 경로 |
| `mission_chain.py` | 매핑된 RC 스위치·예상 밖 모드 인계·재개 금지 |
| `px4_sensor_readback.py` | 고도·RC·전원 정책의 고정 읽기 확대 |
| `full_mission_tag_b.json` | 예상 MAG_TYPE=6·1.3m·확인값 false 유지 |

START 때 검증된 PX4 위치를 다시 읽는다.
기체 기준 창고 XY로 경로를 만든다.
변환 미확인·오래된 위치는 거부한다.
계획과 실행 시작점의 차이는 0.1m 이내다.
실행기의 실제 시작 위치를 복귀점으로 고정한다.
안테나 위치나 임의 원점을 복귀점으로 쓰지 않는다.
1m 시험은 직선 위 0.5m 경유점을 사용한다.
기존 최대 명령 거리와 도착 오차 사이에 여유를 둔다.

천장 입력은 지도 검사에만 사용한다.
더 낮은 실측 지도 상한을 넓히지 않는다.
실측 기체 높이·여유가 없으면 검사를 통과하지 못한다.
START 뒤에는 천장 변경을 거부한다.
PX4 이륙 높이와 명령 altitude=NaN은 유지한다.

## 실물 읽기와 변경

FC는 USB 전원으로 연결되고 disarmed 상태였다.
수정 전 전체 파라미터를 백업했다.
사용자 요청의 낮은 높이·RC 인계를 반영했다.

| 파라미터 | 변경 전 | 저장·재부팅 뒤 읽기 |
|---|---:|---:|
| MIS_TAKEOFF_ALT | 1.3 | 1.3·유지 |
| COM_TAKEOFF_ACT | 0 | 0·유지 |
| EKF2_MAG_TYPE | 0 | 6 |
| COM_RC_IN_MODE | 3 | 0 |
| COM_RC_OVERRIDE | 2 | 3 |
| COM_RC_STICK_OV | 30 | 30·유지 |
| EKF2_EV_CTRL | 1 | 1·유지 |
| EKF2_EV_DELAY | 0 | 0·미보정 |
| EKF2_EV_POS_X/Y/Z | 0/0/0 | 0/0/0·미보정 |

재부팅 직후 MAVROS 전체 pull이 실패했다.
이를 성공으로 기록하지 않았다.
새 고정 shell 조회에서 저장값을 다시 확인했다.
이후 지상 MAVROS mirror에서도 같은 값을 읽었다.
원시 결과는 `policy-after-reboot.json`에 있다.

Init 자력계 정책은 초기 방위만 사용한다.
방위 드리프트와 방향 제어는 별도 실측이 필요하다.
[PX4 v1.17 파라미터 기준](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#EKF2_MAG_TYPE)을 따른다.

| 실물 입력 | 확인 결과 | 남은 확인 |
|---|---|---|
| Tag B | ID6·domain2·0.15m firmware heartbeat | 앵커 동기화·RAW 거리·이동 방향 |
| 앵커 설정 | 6.3×4.6m·높이0.15m | 실제 배치·거리 편향 |
| flow | 원시 품질84~89 | 실제 이동·PX4 융합 |
| ToF | 바닥0~0.003m·최소0.1m | 손으로 높인 상태의 유효 거리 |
| EKF2 | fake position, EV/flow 융합 false | EV XY·flow·range ULog |
| RC | 16채널 모두0·신호 상실 | 수신·보정·모드 스위치·스틱 인계 |
| RC 설정 | 모드CH5·시동CH8·킬CH7 | 스위치 실제 위치·작동 |
| RC 상실 | 0.5초·NAV_RCL_ACT=3(Land) | 실기 failsafe 시험 |
| 추진 배터리 | disconnected | 연결·잔량·센서 노이즈 |
| 전역 기준점 | local xy_global=false·lat/lon 없음 | native XY 명령의 실제 PX4 원점 |

range 융합 true만으로 바닥 ToF를 유효하다고 보지 않았다.
`/uwb/btf_pose.z=0`도 고도로 사용하지 않았다.
실물 브리지는 비활성·지상 전용을 유지했다.

## 장착 방향과 현장 예상값

사용자가 FC는 태그 앞 10cm라고 정정했다.
아래 값은 대략 설명이며 보정 완료값이 아니다.

| 벡터 | 대략값 | 미확인 |
|---|---|---|
| 태그→FC | 앞0.10m·아래0.10m | 좌우 |
| FC→태그, body FRD | x=-0.10·z=-0.10m | y |
| ToF→태그, body FLU | z=+0.15m | x/y·광축 |
| 천장 | 약3.0m | 실제 최저 높이·장애물 |
| 이륙 요청 | 1.3~1.5m | 현재 FC1.3m·실측 바닥 높이 |

미확인 축을 측정된 0으로 기록하지 않았다.
비공개 메모에는 null로 보존했다.
실행 설정의 확인값은 모두 false다.
앵커0.15m 시험 후2.2m 복원 요청도 유지한다.
펌웨어·layout·앵커 파일을 함께 바꿔야 한다.

## 검증 수준

| 구분 | 결과 |
|---|---|
| ROS2 빌드 | drone_uwb/demo/mission/platform_link 성공 |
| Python 단위·계약 | 164개 전체 통과·고정 복귀점 추가 회귀 1개 통과 |
| C++/서비스 CTest | 30개 통과·bench_plan 1개 분리 재시험 포함 |
| 합성 ROS/HTTP/WS | 4개 시나리오 통과·web_gap 재시험 포함 |
| 웹 스크립트 | node --check 통과 |
| 실제 PX4 SIH 스틱 | 가상 입력→POSCTL·PILOT_OVERRIDE·재개 없음 |
| 실제 PX4 SIH 자동 출발점 호버 | 1.3m·MAG6·END·복귀·disarm 확인 |
| 실제 PX4 SIH 자동 XY 경로 | X+1m·Y+1m·역순 복귀·END 확인 |
| 실물 지상 | Tag heartbeat·RC/ToF/FC topic·파라미터 읽기 |
| 실물 비행·실물 인계 | 미실시 |

첫 SIH 스틱 시행은 시각 안정성 상실로 중단했다.
ToF/IMU 시각 문턱을 완화하지 않았다.
이번 물리 관측 프로세스를 중단하고 분리 시험했다.
그 다음 시행에서 PX4의 stick takeover를 확인했다.
첫 실패를 포함해 모든 기록을 보존했다.
합성 web_gap 최초 시행도 입력 대기에서 시간 초과했다.
같은 조건의 분리 재시험을 통과했다.
두 실패 모두 물리 비행 근거로 사용하지 않는다.
XY 최초 시행도 상승 중 관측 공백으로 중단됐다.
메모리 파일시스템에 기록한 분리 시행은 END를 통과했다.
이때 원본 시각·유효성 기준은 바꾸지 않았다.
저장 지연이 원인이라는 확정은 하지 않았다.
CTest 최초 시행은29/30개가 통과했다.
bench_plan은 prepare IPC의2초 제한에서 실패했다.
TMPDIR=/dev/shm의 분리 재시험은32.56초에 통과했다.
IPC·관측 시간 제한은 바꾸지 않았다.

가상 센서는 알려진 편향·잡음·장착을 사용한다.
모든 가상 실행은 domain99·localhost다.
가상 RC 입력은 MAVLink이고 실제 RC 수신기는 아니다.
스캔 응답도 EXPLICIT_VIRTUAL_WORKER다.

## 재현과 이어가기

현장 순서는 [웹 설정 절차](../runbooks/field_web_settings.md)를 따른다.
가상 재현은 [SIH 절차](../runbooks/mission_chain_virtual.md)를 따른다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb drone_demo drone_mission drone_platform_link
source install/setup.bash
# websockets·pymavlink가 있는 기존 비공개 의존성 경로를 사용한다.
export PYTHONPATH=/home/arialhanho/ROS2-integration-20261007/.test-deps:$PYTHONPATH
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m pytest src/drone_mission/test src/drone_platform_link/test -q
python3 src/drone_mission/test/mission_chain_px4.py \
  --px4-root .review/hover-20261008/PX4-Autopilot \
  --takeoff-alt 1.3 --mag-type 6 --trial-case hover \
  --output /absolute/path/new-sitl-hover
```

원시 기록은 검토 폴더 `.review/field-20261009`다.
`observation-processes.json`에 생성한 PID를 기록했다.
웹은 Jetson localhost8001, WS8002를 쓴다.
PC8351은 SSH tunnel이다. 기존8347은 보존했다.
코드 commit `67e23e8`을 GitHub에 push했다.
Jetson 검토 폴더의 origin은 이전 로컬 폴더다.
이를 바꾸지 않고 Git bundle로 같은 commit을 전달했다.
검토 폴더의 시험 중 수정은 stash에도 보존했다.
이 폴더의 단순 `git pull origin`은 GitHub 갱신이 아니다.
다음 갱신도 GitHub commit을 대조해야 한다.
최종 웹은 IDLE·disarmed·천장3m로 읽혔다.
자동 출발점은 null로 표시된다.
이유: 실물 위치·좌표 변환 확인이 아직 없기 때문이다.
`execute=false`·`bridge_enabled=false`로 관측을 재개했다.
재부팅 자동 실행으로 ARM을 시작하지 않는다.
Jetson 저장소의 남은 용량은 약1.5GB였다.
장시간 원본 수집 전에 저장 공간을 확보해야 한다.
기존 기록은 임의로 삭제하지 않았다.

## 후속 RC 지상 확인

사용자가 RC 연결을 알린 뒤 다시 확인했다.
프로펠러 제거·추진 배터리 분리를 확인했다.
Jetson은 재부팅된 상태였다.
기존 자동 실행 서비스 5개는 정상 동작했다.
시험 웹·관측 노드만 다시 시작했다.
명령 출력과 UWB 브리지는 계속 비활성이다.

| 항목 | 실제 확인 |
|---|---|
| RC 링크 | 16채널·link quality 100·lost/failsafe false |
| 스틱 | CH1 롤·CH2 피치·CH3 스로틀·CH4 요의 변화 수신 |
| 관측 가동범위 | CH1 1086~1999·CH2 1000~1999·CH3/4 1002~1999μs |
| 모드 스위치 | CH5의 1000/1500/1999 수신 |
| 기존 모드 반영 | OFFBOARD·ALTCTL·AUTO.LAND 전환 관측 |
| 시동·킬 | CH8·CH7 매핑, 조작 시험 미실시 |
| 추진 전원 | battery connected=false |
| ARM | 수집 중 false 유지 |
| UWB | Tag B heartbeat만 수신·앵커 미연결 |
| ToF 바닥 상태 | 최소 0.1m보다 작은 거리·유효 높이 아님 |

중앙 모드에 Position이 없음을 발견했다.
사용자가 중앙 Position 변경을 명시 승인했다.
`COM_FLTMODE4`만 7에서 2로 변경했다.
새 배치는 Altitude / Position / Land다.
기존 RC 채널·이륙 높이·override 값은 보존했다.
저장 명령 ACK=0과 강제 파라미터 읽기를 확인했다.
1096개 읽기 뒤 변경값 2를 확인했다.
이 변경에서는 FC를 재부팅하지 않았다.
스위치를 움직이지 않으면 이전 모드가 남을 수 있다.
변경 후 ALTCTL→POSCTL 전환을 실제 확인했다.
FC manual_input=true·armed=false를 확인했다.
CH1 최저값은 보정값 1002μs에 못 미쳤다.
전체 가동범위·방향 보정 완료로 기록하지 않았다.

웹 START는 PX4 native 자동 모드를 요청한다.
RC를 OFFBOARD에 둘 필요가 없다.
실제 펌웨어의 stick override는 ARM 조건을 요구한다.
따라서 지상 입력 시험을 공중 인계 성공으로 기록하지 않는다.
RC 수신과 파라미터 저장도 비행 준비 완료를 뜻하지 않는다.
사용자는 송신기 전원 차단 후 재연결을 우려했다.
전원 차단·재연결 시험은 시행하지 않았다.
신호 상실·복구와 실제 공중 인계는 미검증이다.

사용자가 ToF의 바닥 가림을 해소했다.
맞춘 높이는 대략값이라고 명시했다.
ROS 거리는 15초 동안 약 0.38m로 수신됐다.
직접 FC 조회에서는 0.386m였다.
이 비교를 거리 교정 완료로 기록하지 않았다.

| 높이 확보 후 FC 입력 | 읽기 결과 |
|---|---|
| 원시 flow | quality 155·적분 10526μs |
| PX4 처리 flow | quality 157·distance 0.387m |
| EKF 인스턴스 0·1 | cs_opt_flow=true·cs_rng_hgt=true |
| flow aid 인스턴스 0·1 | fused=true·innovation_rejected=false |
| range aid 인스턴스 0·1 | fused=true·innovation_rejected=false |
| UWB 융합 | cs_ev_pos=false |
| 전역 기준점 | xy_global=false·ref_lat/lon=NaN |

이는 지상 정지 상태의 flow·거리 융합 증거다.
동적 축·장착·실측 높이·UWB 공동 융합은 남았다.
설정의 fusion_confirmed는 false를 유지한다.
고정 FC 조회 중 이번 관측 launch만 잠시 중단했다.
포트 해제를 확인하고 읽은 뒤 즉시 재시작했다.
이 관측 공백을 RC 전파 단절로 해석하지 않는다.
기존 자동 실행 서비스와 RC 전원은 유지했다.

원시 수집은 비공개 `rc-ground-030634`에 있다.
`rc-input.jsonl`은 ROS 구독으로만 수집했다.
`set_rc_position.json`은 변경 전후와 ACK를 보존한다.
`tof-raised-readback.json`은 FC 원문을 보존한다.
앵커 연결 후 좌표·장착·시각·융합 검증이 남았다.
[PX4 모드 값](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#COM_FLTMODE1)을 대조했다.

## 시험 웹 미션 관리 검증

사용자가 현장 이동 전에 미션 관리를 요청했다.
저장 미션과 실제 실행 기록을 분리했다.
SQLite에 미션·버전·실행·요청 접수를 보존한다.
같은 내용은 기존 미션을 반환한다.
같은 이름의 다른 미션은 생성하지 않는다.
실행 중인 미션 원본은 수정·삭제할 수 없다.

열린 실행 기록은 하나만 허용한다.
동시 START와 응답 유실 재시도를 검사했다.
저장 후에만 companion에 명령을 노출한다.
저장소 쓰기 실패 시 START는 노출되지 않는다.
웹 재시작 후 경로는 복원한다.
이전 START·복귀·착륙 명령은 재전송하지 않는다.
RC 인계 뒤 새 제어 명령도 거부한다.
다른 실행의 텔레메트리는 결과로 채택하지 않는다.
복귀·착륙은 현재 실행 식별자와 연결을 대조한다.

실행 기록 닫기는 companion 초기화가 아니다.
최신 착륙·disarm과 종료 상태를 확인한다.
만료 미수락 요청은 IDLE에서 정리할 수 있다.
그 기록을 성공·END로 바꾸지 않는다.
새 비행은 새 지상 IDLE 세션을 준비해야 한다.

| 검증 구분 | 결과 |
|---|---|
| Windows 단위·HTTP | 42 passed |
| Jetson 단위·HTTP·ROS 합성 | 46 passed·49.50초 |
| ROS 합성 사례 | 정상 복귀·취소·웹 단절·실행기 재시작 |
| 브라우저 | 별도 8352에서 저장·중복·수정·삭제 확인 |
| JavaScript 구문·diff | node --check·git diff --check 통과 |
| 이번 변경의 실제 SITL | 미실시·ROS 합성 시험과 구분 |
| 이번 변경의 실제 비행 | 미실시·실물 명령 출력 비활성 |

최초 ROS 시험은 3건 실패했다.
2건은 시험 코드가 최신 지상 상태 전에 START를 보냈다.
서버 준비 조건을 기다리도록 시험을 고쳤다.
1건은 합성 높이 관측 공백으로 중단됐다.
높이 유효성 기준은 완화하지 않았다.
동일 관측 기준의 재시험 4건을 통과했다.
최종 전체 46건도 통과했다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m pytest src/drone_mission/test/test_mission_store.py \
  src/drone_mission/test/test_local_web.py \
  src/drone_mission/test/test_site.py \
  src/drone_mission/test/test_local_flight_ros.py -q
```

Jetson 검증은 `/dev/shm`의 별도 소스로 실행했다.
시험 파일은 실물 현장 저장 파일과 분리했다.
합성 ROS는 domain 99·localhost만 사용했다.
실물 domain 2·FC 파라미터·관측 실행기는 보존했다.
시험 로그는 비공개 `.review/field-20261009`에 있다.
`mission-web-final-tests.log`가 최종 결과다.
[미션 관리 절차](../runbooks/field_web_settings.md)를 따른다.

Jetson의 `drone_mission` 빌드도 통과했다.
`colcon build --symlink-install --packages-select drone_mission`을 사용했다.
리뷰 checkout만 정확한 Git bundle로 전진시켰다.
기존 integration 수정과 원격 저장소 설정은 보존했다.
최신 IDLE·착륙·disarm 상태에서 시험 웹만 교체했다.
기존 관측 실행기와 MAVROS는 유지했다.
교체 전 웹 상태는 비공개 파일로 보존했다.
미션 ID가 없던 오래된 착륙 요청은 재전송하지 않았다.

실물 웹에 네 시험 종류를 저장했다.
호버 2초·X 왕복·Y 왕복·XY 역순 복귀다.
저장 미션 4개·실행 기록 0개를 확인했다.
실제 화면에서 호버 미션 불러오기를 확인했다.
웹 주소는 PC의 `http://127.0.0.1:8351/`이다.
끊겼던 SSH 터널을 localhost 범위로 복구했다.
저장 파일은 `.review/field-20261009/mission-library.sqlite3`다.
명령 출력·UWB 브리지는 계속 비활성이다.
기체는 POSCTL·disarm이며 START 조건은 미충족이다.
현장 배치·장착·시각·융합·전원 검증이 남았다.

## 카메라·스캐너·LiDAR 잔여 작업

장치가 보이는 것과 미션 연결 완료를 구분한다.

| 구성 | 현재 확인 | 아직 필요한 것 |
|---|---|---|
| 카메라 | `/dev/video0` 존재·기존 ArUco 코드 | 현재 촬영·원본 시각·장착·지도 변환 |
| USB 스캐너 | SM-2D HID·`/dev/input/qr_reader` 읽기 권한 | 실제 라벨·판독 시각·작업 창 연결 |
| SQLite 저장 | 기존 qr_record_store 코드 | mission result_id·durable 저장 확인 |
| LiDAR | `/dev/lidar`·제조사 launch 존재 | 현재 scan·장착·지도/융합 검증 |
| scan adapter | 미션 요청/결과 계약 존재 | 실물 마커·판독·저장 end-to-end |

기존 QR String을 성공 결과로 감싸지 않았다.
실제 획득 시각·대상·창·저장 근거가 없기 때문이다.
실물 adapter가 없으면 scan timeout을 실패로 기록한다.
대기점으로 복귀하고 남은 작업을 계속한다.
전체 시도 후 출발점 복귀·착륙·END를 요구한다.
새 장애물 회피와 실물 완전 자율 미션은 미완료다.
