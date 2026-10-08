# Jetson 자동 점검 배포·운영 절차
이 문서는 실제 적용한 서비스와 남은 설정 절차를 기록한다.
서비스 설치·상태 확인·재부팅 검수·복구 때 읽는다.

상태: 호스트 점검 + C++ core + 웹/센서/PX4 observer 자동 기동 적용 / 실제 비행 preflight 연동 전 / 2026-10-05.
추가 target은 `sangwon-autonomy.target`이다. core/web/host/perception/PX4 observer 5개 서비스 active,
target enabled와 Linger=yes 및 재시작 후 새 runtime ID·읽기 전용 재연결을 확인했다.
실제 서버 계약 불일치로 `can_start=false`, 물리 출력은 미구현이다.
새 운영 절차는 [WEB_JETSON_RUNBOOK.md](WEB_JETSON_RUNBOOK.md),
증거는 [boot 검증](reports/WEB_JETSON_BOOT_CHECK_2026-10-04.json)·[CTest](reports/WEB_JETSON_CTEST_2026-10-04.txt)를 따른다.
아래 표는 기존 host monitor의 설치 근거다.
요구·상태 계약은 [BOOT_PREFLIGHT_SPEC.md](BOOT_PREFLIGHT_SPEC.md)를 따른다.
실제 기체의 설정·센서·제어권 승인은 별도다.
2026-10-05: `sangwon-px4-observer.service`의 C++ ROS 구독기를 설치했다. runner는 ROS setup과 `config/px4.observe.json`(장치별 `px4.local.json` 우선)을 사용한다. MAVROS를 임의 기동하거나 FC 포트/URL을 추측하지 않는다. 실제 transport는 연결 방식·기체 식별 검증 후 설정한다.
`.runtime/px4/health.json`은 mode0600/단일 writer/원자 교체와 boot/session/순번·monotonic 만료를 사용한다. 보고 TTL5초, header 나이2초(state/extended/battery)/1초(RC)는 표시용 시험값이며 PX4 failsafe/BT 시간을 바꾸지 않는다. `ops/stack_status.py`에서 요약과 재검증 OBS 항목을 확인한다.
관련 CTest8/8(32.89초), 실제 구독기 중단→C++ UNKNOWN→새 세대 복구를 확인했다. 근거는 [HOST 인수](reports/PX4_OBSERVER_HOST_ACCEPTANCE_2026-10-05.json)다. 현재 네 스트림은 발행자0/UNKNOWN이며 실제 BP 검사는 UNKNOWN이다. cold boot와 실기체 연결 시험은 남는다.
`sangwon-perception-monitor.service`는 기존 노드에 구독만 연결한다. 카메라 촬영·판독기 활성화·기체 출력을 요청하지 않는다.
설정은 `config/perception.observe.json` 또는 장치별 `config/perception.local.json`이다. 현재 기본 ROS domain=1에서 센서 미수신 UNKNOWN을 확인했다. domain=0의 토픽 발견만으로 실제 프레임 수신을 확인한 것으로 처리하지 않는다.
실제 관측기 중단 뒤 C++ 호스트 점검에서 5초 보고 만료를 반영하고, 재시작 뒤 프로세스 관측만 복구하는 시험을 통과했다. 콜드 부팅은 별도 검수다.
2026-10-04: [JETSON_ENV.md](JETSON_ENV.md)의 개발 환경을 추가 적용·검증했다.
호스트 서비스 active를 재확인했다. 전체 기체 서비스·권한·실장 검증은 아직 남았다.

## 1. 현재 적용 결과

자동 기동 설정과 감시기 장애 재시작을 확인했다.
실제 전원 재투입 시험은 아직 하지 않았다.

| 항목 | 관측·적용 결과 |
|---|---|
| 대상 | `arialhanho@100.110.163.94` |
| 서비스 | `sangwon-health-monitor.service` |
| 서비스 관리자 | 해당 계정의 systemd user manager |
| 설치 상태 | enabled / active |
| 로그인 없는 기동 설정 | Linger=no → yes 적용 |
| 재시작 | on-failure, 5초 후 재시도 |
| 서비스 watchdog | 30초, 정상 검사·파일 저장 후 통지 |
| 장애 주입 | 해당 모니터만 SIGKILL, 자동 재시작 성공 |
| 세션 | 재시작 후 monitor_session_id 변경 확인 |
| 현재 비행 준비 | BLOCKED, can_start=false |
| 무선 연결 | 기존 Wi-Fi 프로파일 autoconnect=yes 확인 |
| 기존 서비스 | SSH·Tailscale·NetworkManager enabled 확인 |
| 직렬 장치 | 현재 `/dev/pixhawk`, `/dev/lidar`, `/dev/uwb` 없음 |
| 권한 | 현재 계정 dialout 미포함, sudo 비대화형 사용 불가 |
| 콜드 부팅 | 미시험 |

실행 증거는 [host_monitor_deployment.json](reports/host_monitor_deployment.json)에 있다.
이 파일은 수집 시점의 증거이며 실시간 상태가 아니다.
systemd의 active/READY는 **진단 서비스 실행 완료**만 뜻한다.
웹의 비행 READY로 변환하지 않는다.

## 2. 설치 파일과 데이터

원본은 우리 디렉터리에 두고 사용자 서비스가 참조한다.

| 경로 | 역할 |
|---|---|
| [ops/health_monitor.py](ops/health_monitor.py) | 읽기 전용 호스트 검사 |
| [서비스 파일](deployment/sangwon-health-monitor.service) | 실행 환경·재시작·watchdog |
| [설치 스크립트](deployment/install_user_monitor.sh) | 시험·서비스 검증·연결·기동·linger 적용 |
| [시험](tests/test_health_monitor.py) | 권한 미발급·신선도·원자적 저장 6개 사례 |
| `.runtime/host_health.json` | 현재 호스트 진단 스냅샷 |
| `.runtime/host_health.lock` | 단일 기록자 잠금 |
| `~/.config/systemd/user/` | 원본 unit을 가리키는 링크 |
| user journal | 상태 변경·서비스 오류 기록 |

`.runtime`은 Git에서 제외한다.
상태 파일은 약 2초마다 갱신한다.
호스트 화면용 만료는 10초다.
명령 timeout 합계 4.5초와 대기 2초를 고려했다.
이 수치를 PX4 위치·명령 유효시간으로 사용하지 않는다.

### 적용한 원칙

- 장치 포트를 열거나 arm·모드·Land 명령을 보내지 않는다.
- 호스트 검사 전체 PASS여도 비행 준비 권한을 발급하지 않는다.
- PX4 preflight·UWB·웹·실측 프로파일은 미구현이면 UNKNOWN이다.
- monitor_session_id와 C++ companion_session_id를 구분한다.
- 이전 boot_id의 상태와 만료된 파일을 STALE로 표시한다.
- 파일 저장이 실패하면 정상 watchdog 통지를 보내지 않는다.

호스트 진단은 실제 C++ preflight 구현 이후에도 보조 입력이다.
OS 파일·장치 별칭 존재를 실제 센서 데이터 정상으로 간주하지 않는다.

## 3. 설치·확인 명령

아래 명령은 기존 사용자 서비스 하나만 다룬다.

```bash
cd /home/arialhanho/Desktop/ROS2/src/sangwon_AI
bash deployment/install_user_monitor.sh
systemctl --user status sangwon-health-monitor.service --no-pager
loginctl show-user arialhanho -p Linger
python3 ops/health_monitor.py --status
journalctl --user -u sangwon-health-monitor.service -n 30 --no-pager
```

설치 스크립트는 다른 기존 unit을 덮어쓰지 않는다.
현재 경로가 다르면 경로 검토를 요구하며 종료한다.
관리자 비밀번호를 입력받거나 문서에 저장하지 않는다.
미완성 모의 비행 프로그램을 부팅 서비스로 띄우지 않는다.

호스트 점검기 코드를 바꾼 뒤에는 시험 후 재시작한다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_health_monitor.py
systemd-analyze --user verify deployment/sangwon-health-monitor.service
systemctl --user daemon-reload
systemctl --user restart sangwon-health-monitor.service
python3 ops/health_monitor.py --status
```

## 4. 관리자 권한이 필요한 후속 작업

장치 연결 전후 실제 권한을 확인한 뒤 적용한다.
현재 sudo는 비밀번호를 요구하므로 아래 작업은 미적용이다.

```bash
sudo usermod -aG dialout arialhanho
```

기존 udev 규칙은 장치를 dialout에 배정한다.
사용자 그룹 변경만으로 이미 실행 중인 프로세스 권한이 바뀌지는 않는다.
다음 계획된 재부팅 뒤 `id`와 서비스의 실제 그룹을 재확인한다.
진행 중인 비행·측정 중에 사용자 세션을 강제로 종료하지 않는다.

기존 udev 파일을 이번 작업에서 덮어쓰지 않았다.
장치가 연결되면 모델·VID/PID·물리 포트를 대조한다.
UWB 통신 사양은 전체 자료를 받은 뒤 확정한다.
PX4/MAVROS·DDS의 포트·baud·대상 ID도 실제 연결에서 고정한다.

## 5. 콜드 부팅 검수

다음 계획된 전원 재투입 때 무로그인 자동 기동을 검증한다.

| 순서 | 확인 | 합격 기준 |
|---|---|---|
| 1 | 이전 boot_id·monitor_session_id 기록 | 비교 가능한 기준 보관 |
| 2 | 전원 재투입 후 원격 접속 | Wi-Fi·Tailscale/SSH 재접속 |
| 3 | unit 최초 시작 시각 | 사용자 로그인 이전 기동 확인 |
| 4 | `.runtime` 상태 | 새 boot_id·새 세션, 오래된 READY 없음 |
| 5 | 미연결 장치 | 해당 항목 FAIL/UNKNOWN, 비행 READY 없음 |
| 6 | 준비 표시 | 최신 C++ 비행 점검이 없으면 can_start=false |
| 7 | 제어 출력 | 부팅만으로 arm·모터·Offboard·이륙 없음 |

현재 기체 장치가 없으므로 자동 연결 검증을 완료 처리할 수 없다.
포트가 준비되면 읽기 전용 상태 수신부터 단계적으로 연결한다.
비행 노드의 전체 자동 기동은 BOOT 명세의 BP-I 순서로 추가한다.

## 6. 준비 완료 알림

첫 버전은 웹 상태 표시를 우선한다.

| 수단 | 확인 결과·조치 |
|---|---|
| 웹 | READY·차단 원인·갱신 나이·RC 인계 상태 구현 요청 |
| 스캐너 LED | DE2110 제품 자료에서 조명/조준 LED 확인. 현재 USB HID 구성에서 독립 제어 가능한지는 미확인 |
| Jetson LED | 현재 sysfs에 키보드 표시등·mmc 표시등만 노출. 이를 비행 준비 표시로 재사용하지 않음 |
| Jetson 음향 | HDMI/APE 오디오 장치는 조회됨. 실제 스피커 연결·출력은 미확인 |
| 모터 알림 | 준비 알림용 회전 제외. 시동·추력 출력은 알림 기능과 분리 |

스캐너 업체에 모델·펌웨어별 명령 문서를 요청한다.
USB HID에서 LED/부저를 개별 제어할 수 있는지 확인한다.
스캔 성공 표시와 READY 표시가 혼동되지 않는지도 검증한다.
읽기 모드 변경이나 가짜 스캔을 알림 방법으로 사용하지 않는다.
물리 표시가 추가되더라도 웹·C++ readiness_revision과 함께 철회되어야 한다.
근거: [DYscan DE2110 제품 자료](https://www.dyscan.com/sale-10423436-micro-controller-qr-code-scanner-module-2d-cmos-image-barcode-scanner-module.html).

## 7. 되돌리기

점검 서비스만 중단·해제하며 기존 SSH·네트워크·RC는 유지한다.

```bash
systemctl --user disable --now sangwon-health-monitor.service
systemctl --user daemon-reload
```

Linger는 이 작업 전 no였다.
다른 사용자 상주 서비스가 필요 없는 경우에만 이전 값으로 복원한다.

```bash
loginctl --no-ask-password disable-linger arialhanho
```

Linger 변경은 해당 사용자 관리자 전체에 영향을 준다.
원본·시험 결과·상태 파일을 삭제하는 명령은 포함하지 않는다.
근거: [systemd 249 서비스](https://raw.githubusercontent.com/systemd/systemd/v249/man/systemd.service.xml),
[loginctl linger](https://raw.githubusercontent.com/systemd/systemd/v249/man/loginctl.xml),
[sd_notify](https://raw.githubusercontent.com/systemd/systemd/v249/man/sd_notify.xml).
