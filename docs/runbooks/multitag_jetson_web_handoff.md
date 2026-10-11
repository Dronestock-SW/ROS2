# 멀티태그·웹 적용 절차
각 companion과 별도 웹 PC를 연결하는 절차다.
패치된 코드를 실물에 적용할 때 읽는다.

## 적용 상태

코드와 격리 시험까지 완료했다.
실제 companion 센서 연결은 다음 단계다.
[검증 결과](../report/multitag_patch_validation_20261007.md)와
[연결 기준](../reference/multitag_jetson_web_contract.md)을 먼저 확인한다.

| 위치 | 사용할 버전/설정 |
|---|---|
| Tag A/B | `uwb-tag-tdma40-v0.4.1-lora-sync1hz` |
| 지상국 | `lora-ground-v0.4-multitag-rx` |
| 앵커 A1~A4 | 기존 radiofix3 유지 |
| ROS2 브랜치 | `codex/uwb-web-handoff-20261007` |
| 웹 브랜치 | `codex/uwb-observation-handoff-20261007` |

기존 장치의 수정·로그·프로세스를 먼저 기록한다.
실행 중인 MAVROS나 PX4를 중복 실행하지 않는다.
이유: USB 점유와 입력 경로가 충돌할 수 있다.

## 1. 기체별 설정

Tag A와 Tag B를 각 companion에 연결한다.

| 항목 | 기체 A | 기체 B |
|---|---|---|
| Tag | ID 5 | ID 6 |
| 도메인 | 1 | 2 |
| 수신 파일 | `uwb_tag_a.yaml` | `uwb_tag_b.yaml` |
| B_TF 파일 | `uwb_btf_tag_a.json` | `uwb_btf_tag_b.json` |
| 장착값 | 기존 12cm 설정, 현물 재확인 | 미측정. mount_confirmed=false |

파일은 `src/drone_uwb/config/runtime/`에 있다.
실측 ToF→안테나 벡터와 바닥 확인을 반영한다.
B의 0 벡터를 측정값으로 쓰지 않는다.
이유: 확인되지 않은 높이로 거리 보정을 만들게 된다.
외부 출력 허용과 PX4 브리지는 비활성으로 유지한다.
위치·시각·기준점 검증이 아직 남아 있기 때문이다.

## 2. companion 지상 기록

Linux companion에서 빌드 후 Tag를 선택한다.
예시 장치 경로는 실제 연결에 맞춘다.

```bash
colcon build --symlink-install --packages-select drone_uwb drone_platform_link
source install/setup.bash

# A: 새 MAVROS를 시작하는 경우
ROS_DOMAIN_ID=1 ros2 launch drone_uwb uwb_btf_bench.launch.py \
  tag:=A uwb_port:=/dev/uwb fcu_url:=/dev/pixhawk:921600 \
  record_directory:=/tmp/uwb-a-ground stop_after_s:=610

# B: 해당 기체 도메인에서 MAVROS가 이미 실행 중인 경우
ROS_DOMAIN_ID=2 ros2 launch drone_uwb uwb_btf_bench.launch.py \
  tag:=B uwb_port:=/dev/uwb start_mavros:=false \
  record_directory:=/tmp/uwb-b-ground stop_after_s:=610
```

두 명령은 서로 다른 기체에서 실행한다.
`btf_config:=/절대경로/실측설정.json`으로 별도 설정을 선택한다.
설정의 Tag ID와 `tdma_mode=required`가 일치해야 한다.
실행기는 자식 프로세스의 도메인을 A=1/B=2로 설정한다.
같은 기체의 별도 서비스도 같은 도메인을 사용해야 한다.

원본 RAW·TDMA·ToF·자세와 보류 사유를 보존한다.
준비 후 600초의 완전한 1초 창을 평가한다.
4/4 유효 RAW는 매 창 35Hz 이상을 확인한다.
최대 연속 누락 1회 이하도 확인한다.
B_TF 새 좌표율과 정확도는 별도 기록한다.
독립 기준점 없이 잔차만으로 정확도를 판정하지 않는다.

## 3. 웹 PC 준비

관제 서비스는 웹 저장소의 `dashboard/DroneStock-main`이다.
기업 소개 서비스와 별도다.
웹 [적용 문서](https://github.com/ljh006008-blip/https---github.com-ljh006008-blip-Teamproject1/blob/codex/uwb-observation-handoff-20261007/docs/observation-handoff-20261007.md)를 따른다.

웹 DB에 migration `0031_droneunit_observation_state`를 적용한다.
기존 기체 ID에 각 companion의 인증 ID를 바인딩한다.
프로필은 `HOST_OBSERVE`다.
Tag 주소와 웹 ID가 같다고 가정하지 않는다.
서버와 companion의 UTC를 맞춘다.

## 4. companion 관측 전송

기체별 서비스 환경 파일을 작성한다.
경로는 `~/.config/dronestock-companion/companion.env`다.

```dotenv
DRONESTOCK_SERVER_URL=http://실제-웹-PC-IP:8001
DRONESTOCK_DRONE_ID=서버에-등록한-숫자-ID
DRONESTOCK_COMPANION_ID=기체별-장치명
DRONESTOCK_DEVICE_AUTH_ID=서버에-등록한-인증-ID
DRONESTOCK_DEVICE_AUTH_SECRET=서버와-같은-비공개-키
ROS_DOMAIN_ID=1
DRONESTOCK_POSE_SOURCE=btf_xy
DRONESTOCK_TELEMETRY_HZ=10
DRONESTOCK_MISSION_POLL_ENABLED=false
```

B는 도메인을 2로 지정한다.
ID와 인증키도 B의 등록값을 사용한다.
비공개 키는 저장소에 넣지 않는다.
이유: 장치 서명 권한을 가진 값이기 때문이다.

설치·재시작은 [서비스 절차](companion_platform_service.md)를 따른다.
설치기는 기존 환경 파일을 유지한다.
따라서 새 변수를 기존 환경 파일에도 직접 반영한다.
`websocket_observations_accepted` 증가와 거절 0을 확인한다.
HTTP 성공 여부는 기본 관측 모드의 판정 기준이 아니다.

## 5. 별도 지상국 USB 연결

이 명령은 companion이 아니라 로컬 웹 PC에서 실행한다.
지상국은 USB 115200 baud를 사용한다.

```powershell
# 웹 저장소 dashboard/DroneStock-main에서 실행
python tools/esp32_ground_bridge.py --server http://127.0.0.1:8001 --port COM3 --baud 115200
```

COM3은 이번 Windows 벤치의 포트였다.
운용 PC에서는 실제 포트와 서버 주소로 바꾼다.
기존 지상국 인증 설정도 그 PC에 적용한다.
태그 수신 JSON은 LoRa 상태로 처리한다.
LoRa 정상만으로 웹 XY 유효를 표시하지 않는다.

## 6. 화면과 남은 검증

화면은 원본 나이와 연결 상태를 따로 표시한다.

| 상황 | 기대 결과 |
|---|---|
| B_TF XY 수신 | 안테나 XY, Z 미관측 |
| 원본 나이 상한 500ms 초과 | 좌표 만료, 이전 위치를 현재값으로 미표시 |
| Wi-Fi 중단 2초 초과 | companion 연결 끊김 |
| 지상국만 실행 | LoRa 상태만 갱신 |
| px4_local 선택 | 같은 메시지 XYZ, 창고 지도 정렬 필요 |

실제 PX4 융합·창고 변환·비행 검증은 남아 있다.
이번 실행기는 위치 명령이나 고도 제어를 시작하지 않는다.
