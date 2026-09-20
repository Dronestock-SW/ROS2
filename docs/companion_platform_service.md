# Platform 상시 통신 실행 절차
이 문서는 통신 서비스를 켜고 확인하는 절차다.
웹 임무 수신과 연결 상태 송신을 유지할 때 읽는다.

## 1. 실행 범위 확인

서비스는 임무를 수신하고 연결 상태를 보낸다.
현재 모드는 `communication_only`다.

| 항목 | 설정·동작 |
|---|---|
| 서비스 이름 | 사용자 서비스 `dronestock-companion` |
| 서버 | `http://203.247.41.82:8001` |
| 웹 기체 ID | `5` |
| 임무 수신 | GET `/api/drones/5/companion-mission/`, 정상 시 2Hz |
| 상태 송신 | WS `/ws/drones/5/`, 목표 10Hz |
| 미측정 센서·FC 값 | null. 위치 유효성은 false |
| 연결 복구 | 실패 시 1~30초 간격으로 재시도 |
| 프로세스 복구 | 종료 시 systemd가 3초 뒤 재시작 |
| 자동 실행 | 사용자 서비스 활성화와 `Linger=yes` |
| 로컬 기록 | 최신 임무·통신 상태를 5초마다 갱신 |
| 명령 실행 | 없음. 임무·앵커·제어 요청을 기록만 함 |

```text
Platform 임무 API -- GET --> companion -- 최신 임무 --> mission.json
Platform WS       <-- 연결 상태 -- companion -- 통신 상태 --> status.json
```

실제 위치와 FC 상태는 아직 연결하지 않았다.
서버 앵커 값과 기존 설치 기록도 다르다.
이유: [연결 시험](report/platform_connection_20260913.md)에서 차이를 확인했다.
웹 좌표·태그 설정·ROS2/PX4 연동은 별도 작업이다.
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
코드를 수정한 뒤에도 위 명령으로 다시 배치한다.
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

`http_ok=true`와 `websocket_connected=true`를 확인한다.
`http_successes`와 `websocket_messages_sent`가 증가해야 한다.
기록의 `updated_at_unix`도 현재 시각과 비교한다.
중단 때 마지막 기록 파일은 그대로 남기 때문이다.

등록 시 확인값은 [실행 검증 기록](report/evidence/platform_service_20260913.json)에 있다.
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
