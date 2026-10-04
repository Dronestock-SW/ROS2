# Windows UWB 시뮬레이션 재개 기록
이 문서는 2026-10-02 Windows의 실제 점검 기록이다.
WSL 시뮬레이션을 이어받거나 재시작을 판단할 때 읽는다.

## 현재 판정

최신 누적본의 WSL 반영을 완료했다.
기존 PX4·Gazebo 프로세스는 유지했다.
WSL 명령 실행은 응답 시간 초과로 실패했다.
새 센서 기록·EKF 융합·이동 시험은 미실시다.

| 항목 | 이번 확인 | 의미 |
|---|---|---|
| 실행 위치 | Windows `DESKTOP-0C8GRSK` | Windows 로컬 PowerShell에서 실행 |
| WSL 배포판 | `Ubuntu-22.04`, Running, 2 | 목록 조회 성공 |
| WSL 사용자 | `dronestock` | 최초 내부 조회와 설정 파일 확인 |
| PX4 | PID 819 | 프로세스 존재. 정상 진행과 별개 |
| Gazebo 서버·화면 | PID 1016·1017 | 화면 프로세스는 D 상태 |
| PX4 소스 | `c4e4ef98e9d75063bf3d53ebb2716221ee7505ae` | 현재 main 참조 파일을 읽음 |
| Gazebo | `8.15.0` | 설치된 pkg-config 파일의 버전 |
| Jetson SSH | `pgyxn@100.110.163.94` | Windows의 인증된 SSH로 문서 읽기 성공 |
| 기존 실제 Position 성과 | 27일 보고서를 기준으로 유지 | 이번 가상 모델의 새 비행 검증과 별개 |
| 최신 코드 | 누적본 54개 파일 해시 일치 | 동일 Python import는 아직 미실시 |
| 7cm·융합·목표 이동 | 미검증 | 새 실행 근거 없음 |

## 실행 장애와 근거

14:03 WSL 명령은 31.164초 후 아래 오류로 끝났다.

```text
연결된 구성원으로부터 응답이 없어 연결하지 못했거나,
호스트로부터 응답이 없어 연결이 끊어졌습니다.
오류 코드: Wsl/Service/0x8007274c
```

`--exec`와 시스템 배포판 조회도 각각 20초 안에 응답하지 않았다.
`wsl --debug-shell`은 관리자 권한을 요구하며 종료 코드 1로 끝났다.
Windows 파일 공유로 `/proc`를 읽는 경로는 정상 동작했다.

| 14:07~14:11 `/proc` 항목 | 관측값 |
|---|---:|
| MemTotal | 16,334,448 kB |
| MemAvailable | 1,720 kB → 0 kB |
| SwapTotal / SwapFree | 4,194,304 / 0 kB |
| Gazebo 서버 VmRSS / VmSwap | 7,095,996 / 1,905,568 kB |
| Gazebo 화면 VmRSS / VmSwap | 8,635,264 / 2,240,292 kB |
| memory full avg10 | 24.61 |
| io full avg10 | 79.66 |

메모리와 스왑 고갈을 직접 확인했다.
이 상태가 명령 실행 지연을 유발했을 가능성이 있다.
메모리 증가의 최초 원인이나 누수 위치는 미확인이다.
프로세스 존재만으로 시뮬레이션 정상 진행을 판정하지 않았다.
최초 Bash 조회의 Gazebo 무출력은 종료 코드가 확정되지 않았다.
PowerShell 인수 처리로 출력된 종료 코드 값을 근거에서 제외했다.

## 보존과 최신 누적본 반영

기존 디렉터리는 `/home/dronestock/uwb_sim/uwb-gazebo-equipment`다.
새 실행 디렉터리는 아래 경로다.

```text
/home/dronestock/uwb_sim/windows_resume_20261002_140858/uwb-gazebo-equipment
```

27일 기본 ZIP 위에 현재 누적 ZIP 하나만 반영했다.
기존 `runs/equipment_02`를 새 실행 폴더로 복사했다.
원래 설정과 설치된 SDF는 Windows에도 별도 보관했다.

| 대조 대상 | SHA-256 또는 결과 |
|---|---|
| 기본 ZIP | `faf4fdc44a659f6733d9d3c314f76638c43097860d8f7b23adbfb312ed0de8ba` |
| 최신 누적 ZIP | `2b5dc76d5a3127491f04437ac0630bb886a6c93a9de899ec44a91669122663ee` |
| 원본·복사 trial.json | `38ab27fac63c8e333e8bc094f7e47ca76b1248f2bdf1f7107a97aac561e2f557` |
| 누적본 54개 파일 | 배포본 바이트와 전부 일치 |
| 설치 월드·원본 생성 월드 | 일치 |
| 설치 기체·원본 생성 기체 | 일치 |

앵커는 기존 불규칙 배치를 유지했다.
시드는 7이다. 기존 B 시간창은 0.8초다.
새 폴더를 만들었으며 과거 시행을 덮어쓰지 않았다.
관측·목표 송신 확인 플래그는 기존 false를 유지했다.

## 기존 실행값

`/proc/819/environ`에서 아래 값을 확인했다.

```text
PX4_SYS_AUTOSTART=4016
PX4_SIM_MODEL=gz_dronestock_x500
PX4_GZ_WORLD=dronestock_uwb
PX4_GZ_MODEL_POSE=2.09,1.68,0.24,0,0,0
```

기존 PX4 실행 경로는 아래와 같다.

```text
/home/dronestock/github/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

기존 Windows PX4 로그의 마지막 수정 시각은 11:32:51 KST다.
그 로그의 IMU 시각 오류와 GCS 경고도 보존했다.
새 시각·센서 상태로 해석하지 않았다.

## 이어갈 순서와 첫 명령

WSL 복구 후 아래 세 단계를 차례로 실행한다.
각 단계의 종료 코드와 원본 출력을 새 시행에 남긴다.

1. PX4·Gazebo 상태와 실제 토픽을 다시 확인한다.
2. 같은 Python에서 Gazebo·NumPy·pymavlink·실행기를 가져온다.
3. RAW 발행기 하나와 navigation monitor 하나를 실행한다.

Windows PowerShell의 첫 상태 확인 명령은 아래와 같다.

```powershell
wsl --list --verbose
wsl -d Ubuntu-22.04 -u dronestock --cd / --exec pgrep -a -x px4
wsl -d Ubuntu-22.04 -u dronestock --cd / --exec timeout 10s gz topic -l
```

실행 중이면 PX4·Gazebo를 유지한다.
중지됐을 때만 위의 기존 실행값으로 기동한다.
UDP 14540에는 통합 monitor 한 프로세스만 연결한다.

가상환경에는 pymavlink 2.4.49와 NumPy 2.2.6이 있다.
Gazebo 모듈은 시스템 dist-packages에 있다.
이 값은 파일 목록이며 import 성공의 증거가 아니다.
동일 Python import에서 누락된 경로만 보완한다.

## 모델 비교와 남은 연결

첫 시행은 현재 0.8초 설정을 보존한 정상 센서 기록이다.
높이 판정과 ToF·자세·장착·바닥면을 먼저 검토한다.
그 뒤 A와 B H80 0.4초 후보를 같은 입력으로 비교한다.
동적 7cm 달성이나 후보 채택을 선행 선언하지 않는다.
M03은 과거 기각 근거가 있어 자동으로 추가하지 않는다.

```text
가상 RAW + ToF + IMU
  → 같은 세션 Shadow 계산과 높이 판정
  → 좌표·기준점·시계·설정 확인
  → A 관측 송신과 실제 EKF 융합 확인
  → 공중 Hold에서 짧은 목표 이동·복귀·착륙
  → 정답·ULog 독립 평가
```

실행기는 시동·이륙을 자동 수행하지 않는다.
실제 FC 파라미터는 이번 작업에서 바꾸지 않았다.
메시지 수신과 EKF 융합 성공을 별도로 판정한다.
독립 다점·동적 7cm, 연결, 이동은 각각 완료 근거가 필요하다.

## 보류 결정

사용자는 이미 실행 중인 프로세스 유지를 요청했다.
그래서 WSL 종료나 기존 프로세스 종료를 실행하지 않았다.
응답 없는 Ubuntu-22.04 재시작 여부를 사용자에게 확인 요청했다.
현재 선택 답변은 아직 받지 못했다.
복구 승인이 오면 기존 설정으로 기동하고 첫 기록을 이어간다.

기계 판독 근거는 `../../data/processed/uwb/windows_resume_20261002_140858/evidence.json`에 보관했다.
Microsoft의 [WSL 진단 안내](https://learn.microsoft.com/en-us/windows/wsl/troubleshooting-guide)도 참고했다.
이번에는 강제 크래시나 메모리 덤프를 수행하지 않았다.
