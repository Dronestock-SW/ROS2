# Platform 상시 통신 실행 절차
이 문서는 통신 서비스를 켜고 확인하는 절차다.
관측 전송과 연결 상태를 유지할 때 읽는다.

웹 좌표는 companion의 Wi-Fi/WebSocket 경로를 따른다.
[멀티태그 인수인계](multitag_jetson_web_handoff.md)에
TDMA 호환·B_TF 좌표 소스·최종 XYZ의 남은 작업을 정리했다.

## 1. 실행 범위 확인

서비스는 인증된 관측 상태를 보낸다.
구형 임무 조회는 기본 비활성이다.
현재 모드는 `communication_only`다.

| 항목 | 설정·동작 |
|---|---|
| 서비스 이름 | 사용자 서비스 `dronestock-companion` |
| 서버 | `http://203.247.41.82:8001` |
| 웹 기체 ID | `5` |
| 임무 수신 | 기본 꺼짐. `DRONESTOCK_MISSION_POLL_ENABLED=true`일 때 구형 GET 2Hz |
| 인증 | 장치 ID·비공개 키·서버 HOST_OBSERVE 바인딩 필수 |
| 상태 송신 | WS `/ws/drones/5/`, 목표 10Hz |
| 좌표 소스 | 기본 `btf_xy`: `/uwb/btf_pose`. 원본 나이 500ms 이내 |
| 소스 선택 | `uwb_xy`, `btf_xy`, `px4_local`. XY에서 Z는 null |
| 기체 격리 | 환경 파일 `ROS_DOMAIN_ID`: A=1, B=2 |
| 배터리 | `/mavros/battery`의 잔량·전압. 3초 이내 값만 전송 |
| 미측정 FC 값 | null. 비행 제어는 false |
| 연결 복구 | 실패 시 1~30초 간격으로 재시도 |
| 프로세스 복구 | 종료 시 systemd가 3초 뒤 재시작 |
| 자동 실행 | 사용자 서비스 활성화와 `Linger=yes` |
| 로컬 기록 | 최신 임무·통신 상태를 5초마다 갱신 |
| 명령 실행 | 없음. 임무·앵커·제어 요청을 기록만 함 |

```text
ROS 관측 --> companion -- 서명 WS --> Platform
                         <-- 수락 ACK --
                     통신 상태 --> status.json
구형 임무 API -- 선택적 GET --> mission.json
```

기본 위치는 B_TF 안테나 XY 관측값이다.
PX4 로컬 XYZ는 명시적으로 소스를 바꿀 때만 읽는다.
배터리 데이터가 없거나 신선하지 않으면 null로 보낸다.
FC 상태는 아직 연결하지 않았다.
서버 앵커 값과 기존 설치 기록도 다르다.
이유: [연결 시험](../report/platform_connection_20260913.md)에서 차이를 확인했다.
웹 관측 연동의 격리 시험은 [패치 기록](../report/multitag_patch_validation_20261007.md)에 있다.
실제 센서 정확도와 PX4 제어 연동은 별도 작업이다.
서버 좌표와 UWB 좌표가 일치하는지 검증하기 전에는 웹의 위치 점을 운용 기준으로 사용하지 않는다.
WS 송신 성공만으로 화면 반영을 확정하지 않는다.
이유: 서버 저장과 브라우저 표시는 별도로 확인해야 한다.

## 2. 설치·갱신

설치 명령은 소스를 복사하고 서비스를 시작한다.
프로젝트 루트에서 실행한다.

```bash
python3 src/drone_platform_link/deploy/install_user_service.py
```

설치기는 사용자 디렉터리만 사용한다.
기존 환경 설정 파일은 유지한다.
새 도메인·소스·인증·임무 조회 설정은 기존 파일에도 반영한다.
설정 예시는 [멀티태그 절차](multitag_jetson_web_handoff.md)를 따른다.
코드를 수정한 뒤에도 위 명령으로 다시 배치한다.
실행 환경은 `/opt/ros/humble/setup.bash`와 `~/drone_ws/install/setup.bash`를 읽는다.
UWB 노드와 MAVROS가 실행 중이어야 실제 값이 들어온다.
시스템 Python 패키지는 변경하지 않는다.
`websockets 13.1`을 별도 경로에 둔다.
다운로드 파일은 고정 SHA256으로 검사한다.

| 파일·디렉터리 | 역할 |
|---|---|
| `~/.config/systemd/user/dronestock-companion.service` | 실행·재시작 설정 |
| `~/.config/dronestock-companion/companion.env` | 서버·기체 ID·인증 설정 |
| `~/.local/share/dronestock-companion/app/` | 배치한 통신 코드 |
| `~/.local/share/dronestock-companion/deps/` | 전용 통신 라이브러리 |
| `~/.local/state/dronestock-companion/status.json` | 통신 상태와 송수신 횟수 |
| `~/.local/state/dronestock-companion/mission.json` | 마지막 정상 임무 응답과 수신 시각 |
| `~/.local/state/dronestock-companion/communication.log` | 연결·임무 변경 로그. 파일당 1MiB, 이전 3개 보관 |

## 3. 실행 상태 확인

서비스 상태와 실제 송수신 횟수를 함께 확인한다.

```bash
systemctl --user status dronestock-companion --no-pager
systemctl --user is-enabled dronestock-companion
loginctl show-user "$USER" -p Linger
tail -n 20 ~/.local/state/dronestock-companion/communication.log
cat ~/.local/state/dronestock-companion/status.json
```

`websocket_connected=true`를 확인한다.
`websocket_observations_accepted`가 증가해야 한다.
`websocket_observations_rejected`는 0이어야 한다.
기본 모드의 `http_ok=false`, HTTP 횟수 0은 정상이다.
임무 조회를 명시적으로 켰을 때만 HTTP 상태를 검사한다.
기록의 `updated_at_unix`도 현재 시각과 비교한다.
중단 때 마지막 기록 파일은 그대로 남기 때문이다.

등록 시 확인값은 [실행 검증 기록](../report/evidence/platform_service_20260913.json)에 있다.
프로세스 자동 재시작과 통신 복구를 확인했다.
실제 재부팅 시험은 수행하지 않았다.

## 4. 로그·재시작·중지

현재 사용자 권한으로 서비스를 관리한다.

```bash
# 실시간 로그. Ctrl+C는 로그 보기만 끝낸다.
tail -f ~/.local/state/dronestock-companion/communication.log

# 설정 변경 뒤 재시작
systemctl --user restart dronestock-companion

# 지금 중지. 다음 부팅 때는 자동 실행한다.
systemctl --user stop dronestock-companion

# 지금 중지하고 자동 실행도 해제한다.
systemctl --user disable --now dronestock-companion
```
