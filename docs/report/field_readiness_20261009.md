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

## 1층 이동·재부팅 후 지상 관측

재부팅 뒤 iPad 핫스팟 자동 연결을 확인했다.
Tailscale 최초 SSH는 응답하지 않았다.
같은 핫스팟의 직접 주소로 접속했다.
기존 Jetson host key를 엄격하게 대조했다.
원래 부팅 서비스 5개는 유지됐다.
수동 실행했던 시험 웹·관측 launch는 종료돼 있었다.
포트 해제를 확인하고 관측 모드로만 복구했다.
기존 미션 저장 파일과 네 시험 미션은 보존했다.

사용자는 프로펠러 장착·RC Position을 알렸다.
모터 명령 없이 구독으로 20초 수집했다.
앵커 배치는 기존 6.3×4.6×0.15m라고 확인했다.
기체는 -X 방향이라고 알렸다.
대략 위치는 독립 측량 완료로 처리하지 않았다.

| 항목 | 새 지상 기록 |
|---|---|
| FC·RC | connected·POSCTL·disarmed·RC 16채널 |
| 배터리 | present=true·약25.56V |
| 이륙·RC 정책 | 1.3m·MAG_TYPE6·RC_MODE0·OVERRIDE3 |
| ToF | 약0.01m·최소 유효거리0.10m 미달 |
| B_TF XY·XYZ | 발행 없음·measured_height_unavailable |
| PX4 수평 추정 | const_pos_mode=true·실행 조건 미충족 |
| 좌표 정렬 | 실제 지도 -X와 ENU yaw를 대조해야 함 |
| 브리지 | disabled·published=0 |

별도 20초 원시 거리 창에는 800주기가 있었다.
네 거리 유효·TDMA deadline 통과는 448회였다.
A2 거리 중앙값은 2.54m, A4는 3.49m였다.
사용자 대략 위치의 수평 최소 거리는 4.16m·5.66m다.
태그 높이와 무관하게 사선거리는 이보다 짧을 수 없다.
따라서 앵커 ID·배치·위치 기준의 불일치를 먼저 확인한다.
이 차이를 임의 편향·변환값으로 상쇄하지 않았다.
프로펠러 제거·전원 분리 후 ToF 높이 확보를 요청했다.
장착·바닥·시각·융합 확인값은 false를 유지했다.

이후 사용자가 위치 측정 오류를 정정했다.
정정 위치로 계산한 최소 거리는 다음과 같다.
초기 불일치 판단은 이전 입력에만 적용된다.
이 기록으로 앵커 ID가 바뀌었다고 확정하지 않는다.

| 앵커 | 정정 위치의 수평 거리 | 새 창 원시 거리 중앙값 |
|---|---:|---:|
| A1 | 4.541m | 4.418m |
| A2 | 2.600m | 2.544m |
| A3 | 5.202m | 5.346m |
| A4 | 3.634m | 3.489m |

대략 배치·위치와 거리 규모는 이제 부합한다.
수cm~15cm 차이는 실측 정확도·편향 검증이 남았다.
이를 보정값으로 자동 채택하지 않았다.
새 창은 800회 중 484회가 네 거리·deadline을 통과했다.
원문은 `range-geometry-corrected.json`에 보존했다.

사용자는 ToF가 이륙하면 작동한다고 설명했다.
현재 실행기는 이륙 전에 유효 높이와 B_TF를 요구한다.
따라서 ToF가 최소 거리 미만인 바닥 출발은 차단된다.
이는 센서 자체의 고장 판정이 아니다.
PX4 landed 상태와 ToF 유효 측정은 서로 다르다.
상대 출발 고도 0은 안테나의 바닥 높이 0이 아니다.
미측정 ToF를 0으로 채우거나 조건을 우회하지 않았다.
지상 안테나·렌즈 높이 실측을 추가 요청했다.
해당 바닥 출발 정책의 별도 설계·검증이 남았다.

후속 답변은 안테나 약15cm·ToF 렌즈 약1cm였다.
약1cm 수신은 설치 높이 설명과 부합한다.
센서 최소 거리보다 낮으므로 유효 높이로 쓰지 않는다.
지상에서 안테나와 앵커 높이가 같다는 가정으로만
20초 거리 중앙값의 수평 최소제곱 진단을 수행했다.
진단 XY는 사용자 정정 위치와 약5cm 차이였다.
네 거리의 적합 RMS는 약0.115m였다.
이 값은 실시간 B_TF·PX4 위치·독립 정확도 검증이 아니다.
편향·장착·초기 고도 확인값은 바꾸지 않았다.
`field-ground-geometry-candidate.json`에 가정과 결과를 보존했다.

재접속 중 wall clock이 보정됐다.
부팅 ID는 유지됐으므로 추가 재부팅으로 세지 않았다.
이후 MAVROS time jump 경고도 한 번 관측했다.
NTP 동기화 표시와 관측 시각 보정 완료를 구분한다.
비공개 `field-ground-043144-b132c28e`에 기록했다.
`ground-observer.json`과 `range-geometry-check.json`을 보존했다.

웹 START를 하단 고정 영역으로 옮겼다.
ARM 포함 순서·대기 항목 수를 명시했다.
추정기 체크 이름을 실제 의미에 맞게 수정했다.
PX4 수평 위치 유효와 외부 관측 융합은 별개다.
서버 시작 조건·RC 인계·높이 기준은 바꾸지 않았다.
관련 단위·HTTP·preflight 검사 30개를 통과했다.
JavaScript 구문 검사도 통과했다.
Jetson 패키지 빌드와 웹 상태 수신을 확인했다.
기존 MAVROS·센서 수집은 유지하고 웹만 교체했다.
화면은 이전 실행기의 체크 이름도 의미에 맞게 표시한다.
브라우저 자동 검증은 오류 페이지 URL 정책으로 차단됐다.
따라서 새 고정 버튼의 실화면 검증 완료로 기록하지 않는다.

## 받침대 상태의 후속 실물 수신·융합 확인

기체를 높인 뒤 flow·거리 융합을 확인했다.
UWB 연결과 비행 준비 완료는 아직 아니다.
사용자는 XY를 유지했다고 알렸다.
프로펠러 장착 상태라 이동 시험은 하지 않았다.
실물 ARM·모드 변경·이륙 명령도 보내지 않았다.

| 항목 | 13:49~13:56 KST 확인 결과 |
|---|---|
| ROS ToF | 약0.40m·센서 min 0.10m 이상 |
| FC 거리계 | 약0.39m·하방 orientation 25 |
| flow | 품질105·적분 시간10526us |
| 두 EKF의 flow | `cs_opt_flow=True`, `fused=True` |
| 두 EKF의 거리 높이 | `cs_rng_hgt=True`, `fused=True` |
| innovation | 해당 flow·거리 관측 rejected=False |
| FC 위치 | `xy_valid=True`, `v_xy_valid=True` |
| EV 수평 관측 | `cs_ev_pos=False`, aid source 관측 없음 |
| 외부 odometry | `vehicle_visual_odometry` never published |
| 전역 원점 | ref_timestamp=0, ref_lat/lon/alt=NaN |
| B_TF | ToF 입력은 유효. 장착·바닥 확인이 남음 |
| 웹 | IDLE·disarm·POSCTL, can_start=false |

현재 위치는 flow 기반의 로컬 위치다.
창고 좌표나 UWB 융합 완료로 해석하지 않는다.
`CONST_POS_MODE`만으로 수평 위치를 무효 판정하지 않는다.
이 펌웨어는 at-rest 때도 그 비트를 설정한다.
예측 상대위치 유효와 실제 aid source를 같이 읽었다.
앞선 바닥 상태와 이번 받침대 결과를 구분한다.

장착 수평 차이에 대한 사용자 설명도 받았다.
FC·태그 좌우 차이는 없다고 설명했다.
ToF·태그 앞뒤·좌우는 최대 약1~2cm다.
앞서 받은 수직·앞뒤 설명과 함께 후보로 보존했다.
독립 측량·방향 시험·시각 보정 완료는 아니다.
실행 설정의 장착 확인값을 true로 바꾸지 않았다.

### 실행기와 직렬 포트를 유지한 FC 읽기

기존 도구에 `--ros-domain` 경로를 추가했다.
MAVROS router로 고정 읽기 36건을 완료했다.
현재 MAVROS·센서 수집·웹을 중단하지 않았다.
FC 파라미터 쓰기와 비행 명령은 허용하지 않는다.
매 요청 직전 최신 connected·disarm을 확인한다.
첫 연결·셸 응답 누락 때는 중단하고 기록했다.
고속 원시 메시지 큐를 1024개로 늘렸다.
후속 36건은 모두 query echo·종료 prompt를 확인했다.
부분 기록을 다음 쿼리 성공으로 세지 않았다.

비공개 원본은 다음 파일에 보존했다.
`ground-preflight-1350.json`은 20초 ROS 관측이다.
`fc-raised-readback.json`은 중단된 부분 기록이다.
`fc-raised-readback-final.json`은 완료된 FC 조회다.
원본 파라미터·실측 후보는 Git에 넣지 않았다.

| 검증 구분 | 이번 결과 |
|---|---|
| 기존 미션·웹·RC·저장 회귀 | 143 passed |
| 고정 읽기·쓰기 거부·상태 보호 | 20 passed |
| 실물 지상 관측 | ROS 20초·FC 읽기36건 완료 |
| SITL | 이번 도구 변경에서는 미실시 |
| 실물 비행 | 미실시·명령 출력 비활성 |

```powershell
$env:PYTHONPATH="$PWD/src/drone_mission;$PWD/src/drone_uwb;$PWD/src/drone_demo"
python -m pytest src/drone_mission/test/test_session.py `
  src/drone_mission/test/test_mission_chain.py `
  src/drone_mission/test/test_preflight.py `
  src/drone_mission/test/test_field_presets.py `
  src/drone_mission/test/test_mission_store.py `
  src/drone_mission/test/test_local_web.py `
  src/drone_mission/test/test_site.py -q
python -m pytest src/sangwon_AI/tests/test_mavros_readback.py -q
```

FC 실행 명령은 [센서 점검 절차](../runbooks/position_sensor_check.md)를 따른다.
다음 실물 단계는 프로펠러 제거 후 방향·거리 시험이다.
이후 시각·장착 보정과 지상 EV 융합을 검증한다.
실측 지도와 전역 원점 연결도 별도로 필요하다.
현재 native 자동 모드는 전역 기준을 사용한다.
PX4의 [외부 위치 안내](https://docs.px4.io/main/en/ros/external_position_estimation#enabling-auto-modes-with-a-local-position)를 참고한다.
원점 누락을 임의 GPS 좌표로 대체하지 않았다.
받침대 출발은 바닥 출발 정책 검증을 대신하지 않는다.

## 비상정지와 두 축 지상 이동

FC의 kill 적용과 도표 방향 차이를 확인했다.
사용자는 비상정지를 올렸다고 설명했다.
고정 읽기를 41개로 확대했다.
`RC_MAP_KILL_SW=7`, `kill_switch=1`을 읽었다.
`actuator_armed.kill=True`, `armed=False`였다.
`ready_to_arm=False`도 확인했다.
`safety` uORB 토픽은 이 펌웨어에 없었다.
그 항목을 확인 완료로 기록하지 않는다.
원문은 비공개 `fc-kill-readback.json`에 있다.

사용자는 받침대째 +X 0.5m 이동을 알렸다.
이후 +Y 0.5m 이동도 알렸다.
프로펠러는 장착 상태였다.
비상정지 유지·DISARM의 지상 이동 기록이다.
ARM·비행·고장 주입 시험으로 세지 않는다.

| 단계 | 원시 거리의 진단 XY | PX4 ENU XY |
|---|---|---|
| 이동 전 | 약(4.305, 1.630)m | 약(-0.052, -0.028)m |
| +X 뒤 | 약(4.860, 1.666)m | 약(-0.471, -0.029)m |
| +Y 뒤 | 약(4.819, 2.195)m | 약(-0.535, 0.367)m |

UWB 수치는 네 거리 중앙값의 진단 해다.
태그 높이 후보 0.54m를 가정했다.
장착 검증된 B_TF 출력이나 독립 정확도 증거가 아니다.
거리 적합 RMS는 각각 약0.088·0.141·0.101m였다.
X 이동의 FC 값은 전후 정지 기록 비교다.
X 이동 과정 전체를 새 trace가 담지는 못했다.
Y 이동은 180초 원본 trace에 담았다.
`motion-ranges-diagnostic.json`과 `motion-x-1403.jsonl`을 보존했다.
보고한 0.5m와 FC의 약0.42·0.40m 차이는 남았다.
이 값으로 flow 배율이나 UWB 편향을 바꾸지 않았다.

사용자는 A1→A2를 볼 때 A3가 오른쪽이라고 확인했다.
앵커 XY 도표와 ENU의 축 방향이 다르다.
기체가 -X를 보고 후진한 사실과 별개다.
도표 +X는 대략 ENU -X로 관측됐다.
도표 +Y는 대략 ENU +Y로 관측됐다.
회전각 하나만 바꾸면 두 축을 동시에 맞출 수 없다.

`map_y_axis_sign`을 명시한 평면 변환을 추가했다.
기본값 +1은 기존 배치·시험을 유지한다.
A3가 오른쪽인 도표는 -1을 사용한다.
관측·공분산·명령·출발점·속도·방위를 함께 변환한다.
브리지와 명령의 값이 다르면 정렬을 거부한다.
FC 레버암·body FRD·Z 제어는 그대로다.
3차원 자세 변환에 평면 반사를 쓰는 경로는 거부한다.
원시 UWB와 보정된 관측을 같은 값으로 대체하지 않는다.

| 축 변환 검증 | 결과 |
|---|---|
| Windows 관련 단위 | 178 passed·ROS 의존3건 skip |
| 추가 ROS 어댑터 | 기체 속도·ENU 속도·heading·부호 불일치 거부 시험 |
| Jetson 관련 단위·ROS 합성 | 186 passed·정상 흐름1건 실패 |
| 실패 원인 | 합성 이륙 중 bridge 관측 공백으로 착륙 처리 |
| 실패건 단독 재시험 | 1 passed·12.46초·기준 완화 없음 |
| 빌드 | Jetson 별도 tmpfs에서 UWB·mission 통과·8.87초 |
| 이번 축 변경의 실제 PX4 SITL | 미실시·ROS 합성과 구분 |
| 실물 EV 전달·비행 | 미실시·기존 출력 비활성 유지 |

실패 기록과 재시험 기록을 모두 보존했다.
`reflection-tests.log`, `reflection-ros-recheck.log`를 따른다.
합성 ROS는 domain99·localhost에서만 실행했다.
고정 실물 원점·시각·장착·지도 확인값은 바꾸지 않았다.
FC 수신 공간에서 역변환한 heading도 별도 시험했다.
이는 실물 동적 방위 정렬 완료를 뜻하지 않는다.

종료된 이번 작업의 지상·SITL 기록도 압축 보존했다.
약450MB 원문을 약82MB 압축본으로 만들었다.
파일별 SHA256과 심볼릭 링크를 대조했다.
PC 복사본의 압축 파일 SHA256도 일치했다.
현재 실행 파일 소유자가 없는 폴더만 정리했다.
기존 부팅 서비스와 현재 관측·MAVROS는 유지했다.
복구 파일은 비공개 `closed-field-evidence-20261009-1412.tgz`다.
Jetson과 PC에 각각 보존했다.

## 자 측정 확인과 별도 B_TF 지상 관측

실측 장착값으로 별도 관측 XYZ 924건을 얻었다.
운영 경로의 FC 전달·비행 완료를 뜻하지 않는다.
사용자가 자로 잰 장착값임을 확인했다.
평평한 바닥과 공과대학 시험 장소도 확인했다.
정확한 지도 핀·전역 원점은 아직 없었다.

| 항목 | 기록 |
|---|---|
| FC→태그 FRD | 뒤 0.10m·오른쪽 0m·위 0.10m |
| ToF→태그 FLU | 앞·왼쪽 0m, 위 0.14~0.15m |
| 수평 장착 오차 | 사용자가 최대 0.01~0.02m 가능성을 알림 |
| 이번 관측 명목 높이 | 보고 범위 상단인 0.15m 사용 |
| 원본 | 비공개 `measured-mount-052945/measurements.json` |

FC 기준 벡터는 `(-0.10, 0, -0.10)m`다.
FC 파라미터는 이 단계에서 바꾸지 않았다.
ToF 장착값을 mm 정확도로 주장하지 않는다.
장착·배치 확인은 준비 설정에 반영했다.
시각·좌표 정렬·융합 확인은 여전히 false다.

`observe_measured_btf.py`로 30초 관측했다.
기존 RAW·ToF·IMU·TIMESYNC를 구독했다.
네 출력은 `/diagnostic/uwb/btf_*`로 격리했다.
기존 MAVROS·관측·웹 PID를 유지했다.
연결·DISARM·POSCTL 상태였다.

| 결과 | 값 |
|---|---|
| 원시 계산 cycle | 1,028건 |
| 높이를 갖춘 XYZ 발행 | 924건 |
| XYZ 중앙값 | (4.80984, 2.19586, 0.53957)m |
| X 5~95백분위 | 4.80312~4.81636m |
| Y 5~95백분위 | 2.18332~2.20716m |
| 원본 매핑 시각 기준 발행 age 95백분위 | 0.02159초 |
| TDMA deadline 거부 | 63건 |
| 높이 누락으로 pose 보류 | 73건 |
| 실제 FC 전달·EV 융합·비행 | 미실시 |

분포 폭은 정지 시 변동이며 절대 정확도가 아니다.
발행 age는 물리적 전달 지연의 실측값이 아니다.
고정 transport delay는 여전히 미보정이다.
초기 동기화·만료·TDMA 거부도 그대로 기록했다.
무효 입력을 성공 샘플로 채우지 않았다.
원본 입력·판정·요약은 비공개 `observation/`에 있다.

축 패치 `3711d6f`는 Jetson 소스·빌드에 적용했다.
실제 review checkout 빌드는 8.51초에 통과했다.
이 관측 시점의 운영 프로세스는 재시작 전이다.
새 부호·장착 준비 파일을 운영 적용으로 세지 않는다.
별도 진단 관측이 기존 웹·FC 입력을 덮지 않았다.

## 실측 설정의 운영 관측 적용

승인한 세션 재시작 뒤 웹 XYZ 수신을 확인했다.
사용자가 MAVROS 재연결을 명시적으로 승인했다.
DISARM·지상·IDLE·실행 없음부터 확인했다.
이 작업이 띄운 웹·관측 process group만 종료했다.
FC는 재부팅하지 않았다.
기존 설정·웹 상태를 비공개 폴더에 백업했다.
실측 설정으로 웹과 관측 launch를 다시 시작했다.

새 세션은 `field-ground-053358-b132c28e`다.
운영 `map_y_axis_sign=-1`을 확인했다.
배치·장착 확인은 사용자 실측 근거로 반영했다.
정렬·시각·융합 확인은 false로 유지했다.
관측·명령 양쪽의 미보정 회전·원점은 그대로다.
변환 설정 일치는 실제 정렬 완료와 별개다.

| 재시작 후 확인 | 결과 |
|---|---|
| 15초 B_TF pose 수신 | 533건 |
| 단일 발행자 | RAW·B_TF·MAVROS·bridge 각각 확인 |
| 웹 XYZ | 약(4.812, 2.188, 0.530)m·유효 관측 |
| FC | connected·DISARM·POSCTL |
| 기본 이륙 | MIS_TAKEOFF_ALT 약1.3m 유지 |
| RC | 수신·채널 매핑·AUTO override 설정 확인 |
| 실제 EV 발행 | 0건·bridge 비활성 |
| 실제 비행 START | false |

웹의 관측 유효는 안테나 위치 수신을 뜻한다.
FC body 출발점이나 EKF 융합을 뜻하지 않는다.
FC 레버암 파라미터는 아직 0이다.
실측 기대값과의 차이를 숨기지 않았다.
물리적 지연·회전·원점·실측 지도도 남았다.
현재 native AUTO의 전역 기준점도 없다.
지상 ToF 최소 거리 미만 출발 정책은 미해결이다.

관측 도구 구문 검사와 높이 처리 6개 시험을 통과했다.
이번 추가의 SITL·실물 비행은 미실시다.
세부 원본은 `post-measured-restart-ground.json`과
`post-measured-web-status.json`에 보존했다.

종료된 이전 관측 세션의 원본 1,323,782,338바이트를 보존했다.
압축본은 163,169,511바이트다.
각 파일과 압축본의 SHA256을 대조했다.
PC 복사본 확인 뒤 닫힌 원본 폴더만 정리했다.
Jetson·PC 양쪽에 압축본과 manifest를 남겼다.
작업 뒤 Jetson의 가용 공간은 약1.3GB였다.
복구 파일은 `closed-field-ground-043144-b132c28e.tgz`다.
진행 중 새 세션과 기존 부팅 서비스는 정리하지 않았다.

왕복 지상 기록 전 FC 비상정지를 다시 읽었다.
`kill=True`, `armed=False`, `ready_to_arm=False`였다.
사용자는 프로펠러 장착·받침대 유지를 알렸다.
동적 지상 기록은 실물 비행과 구분한다.

## 왕복 지상 관측과 수신 공백

왕복 기록은 아직 비행용 보정을 통과하지 못했다.
사용자는 X 이동·복귀와 Y 이동·복귀를 알렸다.
왕복 횟수와 실제 이동 거리의 추가 답은 없었다.
두 번의 180초 FC 원본 trace를 기록했다.
같은 시각의 운영 B_TF 판정을 대조했다.

| 구간 | FC odometry | 대응 B_TF | 최대 발행 공백 |
|---|---|---|---|
| 첫 기록 | 5,132건 | 4,651건 | 14.900초 |
| 둘째 기록 | 5,171건 | 4,807건 | 10.126초 |

첫 기록은 X와 Y 이동 일부를 함께 담았다.
두 번째 기록과 겹친 시각은 독립 반복으로 세지 않는다.
FC 자세로 장착 레버암을 투영해 안테나 기준을 대조했다.
반사 부호 -1의 평면 적합 잔차는 각각 약0.025·0.016m다.
추정 회전각은 약175.50·179.16도였다.
시간 차이의 최소 오차 후보는 0.215·0.155초였다.
그러나 5mm 이내 잔차 차이의 후보 범위가 매우 넓었다.
각각 -0.465~0.5초와 -0.365~0.5초였다.
긴 관측 공백이 이동 구간과 겹쳤다.
따라서 고정 지연·정렬값으로 채택하지 않았다.
이 적합의 작은 잔차는 정확도·시각 보정 증거가 아니다.

13.899초 공백에서 jump 격리 205건을 확인했다.
높이 일관성·subset·연속성 거부도 함께 발생했다.
14.900초 공백에서는 RAW 상태 누락이 주원인이었다.
당시 ToF는 약0.32~0.39m였고 대부분 유효했다.
모든 공백을 ToF 지면 사각구간 탓으로 보지 않는다.

14:40~14:44 원본 전달도 비교했다.
시리얼 기록에는 RAW 8,718·TDMA 8,728건이 있었다.
B_TF 입력에는 RAW 8,634·TDMA 8,657건이 있었다.
상태 메시지도 41건 중 40건만 기록됐다.
한 serial read에 최대10개 메시지가 들어왔다.
5개를 넘긴 묶음은 21회였다.
기존 DDS 수신 깊이는5였다.
별도로 시리얼 기록 자체에도 15~20초 상태 공백이 있었다.
DDS 수정만으로 이 원본 공백까지 해결됐다고 보지 않는다.

RAW 전달자·B_TF 수신자의 깊이를64로 바꿨다.
메시지별 원본 시각·TDMA 짝 검사는 유지했다.
150ms queue 만료·거리 일관성·점프 문턱도 유지했다.
실제 ROS burst 시험에서 깊이5의 앞부분 유실을 재현했다.
깊이64에서는 상태·RAW/TDMA 순서를 모두 보존했다.
domain99·localhost의 ROS·TDMA·높이 검사 23개를 통과했다.
첫 시험의 별도 executor context 오류를 고친 뒤 재시험했다.
실물 비행을 허용하는 변경은 아니다.

원본은 비공개 `alignment-roundtrip-1/2` trace와
대응 `btf-decisions`, `fit` 파일에 보존했다.
`raw-delivery-check.jsonl`은 시리얼/DDS 비교 기록이다.
새 지상 검증 전까지 START는 비활성으로 유지한다.

## 수신 큐 수정 후 X·Y 왕복 재시험

두 축 완료를 기록했으나 비행용 연속성은 실패했다.
사용자는 X와 Y 왕복 완료를 각각 알렸다.
지시 거리는 각 0.5m였다. 실제 거리 추가 실측은 없었다.
`a9bbc17`의 Jetson 빌드는 9.09초에 통과했다.
운영 세션 `field-ground-055441-b132c28e`에 적용했다.
관측 세션만 재시작했다. FC는 재부팅하지 않았다.

| 실물 기록 | FC odometry | 유효 B_TF | 최대 발행 공백 |
|---|---:|---:|---:|
| X 왕복 포함 180초 | 5,143 | 4,401 | 8.575초 |
| Y 왕복 포함 180초 | 5,259 | 4,958 | 15.551초 |

두 trace의 겹친 시간은 독립 시험으로 세지 않는다.
수신 대조 30초에는 RAW 1,034건이 있었다.
TDMA 1,037건과 상태 5건도 모두 B_TF에 도착했다.
최대 9개 burst도 유실되지 않았다.
이 구간의 전달 개선은 발행 연속성 통과와 다르다.

가장 긴 공백은 정지 구간에서 발생했다.
수신 queue 만료 뒤 상태 재수신·시각 재학습이 이어졌다.
이때 ToF는 약0.38~0.39m였다.
이동 구간에는 별도 거리 일관성 거부가 있었다.
X의 5.275초, Y의 3.275초 공백을 확인했다.
정렬 후보의 시간 차이도 확정할 수 없었다.
두 trace 모두 5mm 이내 후보가 -0.5~0.5초였다.
정렬·시각·융합 확인을 true로 바꾸지 않았다.

### 지연 메시지의 초기화 범위 수정

오래된 callback은 버리되 정상 태그 신원은 유지한다.
기존 코드는 150ms 초과 시 태그 상태까지 지웠다.
새 `reject_queued_input()`은 미완성 RAW/TDMA 짝을 버린다.
시각 매핑과 관측 창은 다시 학습한다.
검증된 상태의 원래 수신 시각과 만료는 유지한다.
TDMA boot/session·폐기 세션 목록도 유지한다.
오래된 메시지는 상태나 시각을 갱신하지 않는다.
음수 age·실제 reboot·host clock jump는 전체 초기화한다.
10초 상태 만료·150ms queue 제한은 그대로다.
재부팅 시 이전 상태 사용과 이전 세션 재진입을 시험했다.

관련 순수 Python 시험 27개를 통과했다.
다음은 실제 입력 65초를 재생한 결과다.
실시간 발행·FC 전달 시험으로 세지 않는다.

| 동일 입력 12,063건 재생 | 기존 | 수정 |
|---|---:|---:|
| queue 만료 입력 | 20 | 20 |
| 상태 없음 거부 | 428 | 109 |
| 유효 XYZ | 1,550 | 1,584 |
| 최대 XYZ 공백 | 15.551초 | 12.098초 |

수정으로 전체 수신 문제가 해결되지는 않았다.
시각 매핑 재학습과 동적 거리 일관성 문제는 남았다.
START·bridge 실행은 계속 비활성이다.
추가 지상 보정 전에 수신 지연과 원시 거리부터 진단한다.
이 변경의 실제 SITL·실물 비행은 미실시다.

```bash
PYTHONPATH=src/drone_uwb python3 -m pytest -q \
  src/drone_uwb/test/processing/test_tdma_stream.py \
  src/drone_uwb/test/processing/test_measured_btf.py \
  src/drone_uwb/test/processing/test_observation_guard.py
```

비공개 증거는 `alignment-after-qos*`와 대응 `*-fit.json`이다.
`*-gaps.jsonl`에 센서 상태와 공백 원인을 남겼다.
`queue-replay-reset-clock-comparison.json`이 위 재생 결과다.
원본 입력은 `queue-replay-inputs.jsonl`로 보존했다.

종료된 `field-ground-053358-b132c28e`도 백업했다.
원본 469,777,072바이트를 58,938,624바이트로 압축했다.
PC·Jetson 복사본과 각 파일의 SHA256을 대조했다.
열린 파일이 없는 것을 확인한 뒤 원본만 정리했다.
압축본 SHA256은 다음과 같다.
`edfc29f6cdb0a1cd1c99f1607cbe370ff4696d5fa21543c64b57b8766471e72e`
현재 세션과 기존 부팅 서비스는 보존했다.

### 수정 적용 뒤 정지 수신 확인

`084aaad`를 Jetson 관측 경로에 적용했다.
새 세션은 `field-ground-061245-b132c28e`다.
두 패키지 빌드는 9.06초에 통과했다.
Jetson에서도 위 27개 시험을 통과했다.
기존 관측 설정·미션 DB·부팅 서비스는 유지했다.
FC 재부팅·파라미터 변경·모터 명령은 없었다.

정지 상태 60초에서 FC odometry 1,597건을 기록했다.
대응 구간의 유효 B_TF는 1,777건이었다.
최대 발행 공백 8.874초가 다시 발생했다.
이 구간에서 queue 만료·시각 재학습이 이어졌다.
따라서 실시간 수신 문제는 아직 해결되지 않았다.
정지 적합의 회전·시간 후보는 보정 근거로 쓰지 않았다.
web은 XYZ 약(4.870, 2.109, 0.529)m를 수신했다.
FC는 connected·DISARM·POSCTL이었다.
`execute=false`, `bridge_enabled=false`를 유지했다.
`can_start=false`이며 실제 EV 융합은 미실시다.
원본은 `queue-recovery-static*`에 보존했다.
웹 상태는 `post-queue-recovery-web-status.json`이다.

다음 진단은 시각 매핑의 재학습·수신 정체 원인이다.
이동 중 거리 일관성 실패도 따로 해결해야 한다.
현 상태에서 추가 왕복을 반복하거나 이륙하지 않는다.
이유: 정지 상태에서도 수초 동안 위치 관측이 끊긴다.

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
