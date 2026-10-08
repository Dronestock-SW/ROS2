# 2026-10-07 낮은 앵커·비행 시험 준비 기록
임시 높이와 펌웨어·FC 설정 변경의 검증 기록이다.
오늘의 지상 시험과 이후 실비행 준비를 구분할 때 읽는다.

Tag B의 앵커 높이를 0.15m로 적용했다.
기체의 기본 이륙 높이는 별도로 1.3m로 설정했다.
FC 저장·재부팅 후 값 유지도 확인했다.
웹 네 경로는 합성 REPLAY 시험을 통과했다.
실제 웹 비행 명령 전달·SITL·비행은 미완료다.

## 적용한 설정

사용자가 요청한 직사각형의 세로·가로를 유지했다.
아직 독립 실측으로 확정한 좌표는 아니다.

| 대상 | 값·변경 범위 |
|---|---|
| Tag B | ID 6, ROS_DOMAIN_ID 2 |
| 배치 ID | `warehouse-rectangle-6p3x4p6-z0p15-20261007` |
| A1 / A2 | `(0,0,0.15)` / `(6.3,0,0.15)` m |
| A3 / A4 | `(0,4.6,0.15)` / `(6.3,4.6,0.15)` m |
| 복원 예정 | 네 앵커 높이 2.2m |
| Tag A | ID 5/domain 1, 기존 2.2m 설정 보존 |
| 측량·장착·시각 확인 플래그 | 모두 미확인 상태 보존 |
| 원본 서비스·부팅 경로 | 기존 checkout 유지 |

현재 Tag B는 `uwb_tag_b.yaml`을 쓴다.
수신기와 B_TF는 같은 배치 파일을 선택한다.
배치 ID나 좌표가 다르면 launch가 거부한다.
복원용 B_TF JSON과 수신 YAML도 별도로 남겼다.
앵커 펌웨어에는 이 설치 좌표를 쓰지 않는다.
이번에 앵커 펌웨어를 업로드하지 않았다.
Tag A도 같은 낮은 배치에서 B_TF를 쓸 때는
A의 설정을 실제 배치와 다시 대조해야 한다.

같은 높이의 XY 선형식은 높이 항이 상쇄된다.
0.15m 설정만으로 기존 거리 오차가 고쳐지지는 않는다.
B_TF의 하방거리 보정에는 실제 높이·장착값이 필요하다.
XY 메시지의 z=0은 계속 미관측 자리값이다.

## 펌웨어와 실물 USB 수신

별도 펌웨어 저장소에 재현 가능한 실험을 만들었다.
원본 배포 패키지는 변경하지 않았다.

| 항목 | 결과 |
|---|---|
| 저장소 | `ArialHanho/uwb-multitag-research` |
| 기준 | `origin/codex/lora-ground-multitag`, `80dc0ff` |
| 브랜치 | `codex/tag-b-anchor-z015-20261007`, `df61ebf` |
| 실험 | `experiments/anchor-low015` |
| 버전 | `uwb-tag-tdma40-v0.4.1-lora-sync1hz-z015` |
| 빌드·업로드 | Tag B 성공, 네 이미지 hash 검증 |
| 복구 자료 | 업로드 전 전체 flash 8MB 로컬 보존 |
| 업로드 포트 | Windows COM6, ESP32-S3 |
| 실물 식별 | heartbeat ID 6·새 firmware/layout ID 확인 |

전체 flash 백업 SHA256:
`28f51b038cbca6c17b45abfcc8ad9fbb8af5b4d7a0f7e5e020be52a4392478fa`.
로컬 경로는 `.integration/firmware-private/`다.
장치별 백업·원시 수신 로그는 Git에 넣지 않는다.

PC 60.125초 수신 결과는 다음과 같다.
앵커가 최종 위치에 있지 않은 상태의 결과다.

| 항목 | 측정 |
|---|---:|
| RAW cycle | 2,394 |
| 평균 cycle rate | 약 39.9Hz |
| 온전한 1초 창 최소 | 39 |
| 누락 seq / 손상 JSON | 0 / 0 |
| 최대 호스트 수신 간격 | 약 63ms |
| 유효 raw XY | 140 |
| 거리 불일치 | 1,813 |
| 앵커 부족 | 440 |

PC의 짧은 USB 시험은 위치 정확도 증거가 아니다.
과거 Jetson 10분 수신 실패를 해소한 증거도 아니다.
사용자가 Tag B를 Jetson으로 옮긴 뒤 `/dev/uwb`를 확인했다.
Jetson에서도 새 ID·firmware·배치 ID를 수신했다.
10초 준비 뒤 600개의 1초 창을 검사했다.
평균 39.412Hz, 최소 0Hz였다.
35Hz 미만은 16창, 최대 연속 누락은 58개였다.
UART JSON 파싱 오류도 43회 기록됐다.
`/uwb_pose` 48회, `/uwb/btf_pose` 0회였다.
FC vision 출력은 0회로 유지됐다.
현재 물리 배치가 설정과 달라 위치 정확도를 판정하지 않는다.
다만 UART 파싱·연속성 문제는 배치 정확도와 따로 조사한다.
PC 60초와 Jetson 600초는 길이·호스트가 달라 원인 비교가 아니다.
빌드·단위시험은 이번 10분 수신 종료 후 실행했다.

## 사용자가 요청한 PX4 파라미터

DISARM·착륙 상태에서 세 값만 변경 요청했다.
ARM·비행 모드·setpoint 명령은 보내지 않았다.

| 이름 | 변경 전 | 저장·재부팅 후 |
|---|---:|---:|
| `COM_RC_OVERRIDE` | 1 | 2 |
| `EKF2_EV_CTRL` | 0 | 1 |
| `MIS_TAKEOFF_ALT` | 2.5 | 1.2999999523162842 |

마지막 값은 FLOAT32의 1.3m 표현이다.
MAVROS `ParamSetV2`의 반환값을 확인했다.
강제 pull 뒤 다시 읽고 storage 명령을 보냈다.
저장 `245/param1=1`과 reboot `246/param1=1`은 ACK 0이었다.
reboot는 FC만 대상으로 했다. 강제 reboot 값은 쓰지 않았다.
Jetson과 기존 사용자 서비스는 재시작하지 않았다.

첫 재연결의 전체 pull은 시간 초과됐다.
재부팅 뒤 새 USB 세션에서 세 값을 직접 읽었다.
이후 새 MAVROS의 65초 관측에서도 같은 값을 확인했다.
관측 시각은 `2026-10-07T11:00:40.915026Z`다.
직접 읽기는 `PARAM_REQUEST_READ`를 사용했다.
INT32는 PX4의 bytewise 인코딩으로 복호화했다.

증거는 Jetson `.integration-evidence/requested-params/`에 있다.
변경 전 1,096개 pull과 전체 캐시 사본도 보존했다.
`transaction.json`은 초기 pull 실패까지 그대로 남겼다.
최종 성공은 `after-reboot-direct.json`에 기록했다.
후속 ROS 근거는 `post-param-probe/probe/summary.json`이다.

`COM_RC_OVERRIDE=2`는 Offboard 스틱 인계를 켠다.
자동 모드의 스틱 인계 비트는 끈다.
AUTO.TAKEOFF·AUTO.LAND를 쓰는 경로에 이 차이가 중요하다.
모드 스위치와 실제 스틱 입력을 지상에서 검증해야 한다.
`COM_RC_STICK_OV=30`, `COM_FAIL_ACT_T=5`는 바꾸지 않았다.
RC 채널 16개는 관측 시 모두 0이었다.
RC 메시지 수신 자체를 조종기 정상 입력으로 판정하지 않는다.
근거: [PX4 RC override 정의](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#COM_RC_OVERRIDE).

`EKF2_EV_CTRL=1`은 외부 수평 위치 융합의 허용값이다.
실제로 UWB가 도착하거나 EKF에 융합됐다는 뜻은 아니다.
통합 관측 실행기의 UWB bridge 출력은 계속 비활성이다.
`EKF2_HGT_REF=0`, `EKF2_RNG_CTRL=1`도 변경하지 않았다.
1.3m 목표와 실제 PX4 고도 원점은 아직 대조하지 않았다.

## 웹과 C++ 재생

loopback 전용 `ops/bench_preview.py`를 추가했다.
네 순서를 계획 JSON과 수평 경로로 보여준다.
모든 이동 구간은 입력한 같은 목표 Z를 쓴다.
경로 원점은 합성 출발점이며 실제 A1 위치가 아니다.

| 시험 | 출발점 기준 XY 경로 |
|---|---|
| 호버 | `(0,0)`에서 도착 확인 후 3초, 착륙 |
| X | `(0,0) → (1,0) → (0,0)` |
| Y | `(0,0) → (0,1) → (0,0)` |
| XY | `(0,0) → (1,0) → (1,1) → (1,0) → (0,0)` |

기존 C++ core의 native 이륙·Offboard·착륙 순서를 썼다.
네 경로 모두 FakePx4에서 SUCCEEDED를 확인했다.
방문 개수·AUTO.LAND·물리 출력 false도 검사했다.
재생 높이는 1.0m, UI 확인 높이는 1.3m였다.
실제 PX4 SITL을 실행한 결과가 아니다.
브라우저에서 1.3m XY 왕복 계획 생성도 확인했다.

웹은 GET 조회·계획 생성만 제공한다.
POST는 405이며 ROS·serial·flight IPC 연결이 없다.
실제 기체 상태는 마지막 읽기 기록임을 명시한다.
웹 표시가 정상이어도 비행 권한을 만들지 않는다.

## 검증 구분

| 종류 | 결과 |
|---|---|
| 펌웨어 생성기 | 3개 단위시험 통과 |
| 빌드 | Tag B 펌웨어, drone_uwb, sangwon_ai_replay 성공 |
| UWB 전체 | 342개 통과, 새 배치·launch 포함 |
| bringup | 57개 통과, 1개 skip |
| 경로·모드·소스 경계 CTest | 4개 통과, 네 경로 포함 |
| HTTP | 정상 200, 잘못된 높이·종류 400, 미등록 404, POST 405 |
| 실제 FC | 세 파라미터 저장·reboot·재조회 확인 |
| 실제 Tag B | 새 펌웨어 PC 수신 확인, Jetson 재연결 확인 |
| Jetson 10분 수신 | 39.412Hz 평균, 최소 0Hz, 기준 미달 16창, 최대 연속 누락 58. 전송 판정 실패 |
| 실제 SITL / 실비행 | 미실시 / 미실시 |

[기계 판독용 요약](evidence/low015_20261007.json)도 보존했다.

## 빌드 환경에서 수정한 확인 절차

기존 copy install에서 symlink 방식으로 바꿀 때
새 설정 파일의 설치 경로 오류가 발생했다.
기존 install은 원래 방식으로 다시 빌드해 복구했다.
별도 빈 build/install 경로에서 symlink 빌드가 통과했다.
이후 기본 build/install의 symlink 재빌드도 통과했다.
자동 생성 디렉터리를 수동 수정하지 않았다.

첫 전체 pytest는 installed 모듈을 먼저 읽었다.
소스 경로를 기대하는 기존 시험 1개가 실패했다.
`PYTHONPATH`에 `src/drone_uwb`를 먼저 지정해 재실행했다.
검증 대상 코드나 기존 시험 조건은 완화하지 않았다.
bringup의 새 launch 줄 길이 오류 1개도 수정했다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/src/drone_uwb:$PWD/.test-deps:$PYTHONPATH" \
  python3 -m pytest src/drone_uwb/test -q
(cd src/drone_bringup && python3 -m pytest test -q)
ctest --test-dir build/sangwon_ai_replay \
  -R 'bench_plan|sangwon_mode_tests|source_handoff' --output-on-failure
colcon build --symlink-install \
  --build-base .integration-evidence/symlink-validation/build \
  --install-base .integration-evidence/symlink-validation/install \
  --packages-select drone_uwb drone_bringup
```

## 남은 실측·구현

실비행 시험을 열기 전에 다음 근거가 필요하다.

1. 앵커 네 중심 위치·높이와 실제 XY 이동 방향.
2. FC→UWB FRD, ToF→UWB FLU 장착 거리.
3. 창고↔PX4 원점·yaw와 원본 관측 지연.
4. UWB 연속성, 보정 관측, EKF 실제 융합·innovation.
5. 유효 RC 입력·스틱/스위치 인계·재획득 금지.
6. 실제 PX4 command writer와 SITL 네 경로 검증.
7. 실측 고도 기준, native 이륙·착륙의 실제 동작.

카메라·바코드·LiDAR의 미완료 항목은
[통합 기준](../architecture/flight_uwb_ai_integration.md)에 남겼다.
이번 임시 설정으로 완료 상태를 올리지 않았다.
