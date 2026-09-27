# WSL Gazebo·PX4 SITL 재실행 절차

이 문서는 설치를 마친 시험 PC의 실행 절차다. 시뮬레이터를 다시 켤 때 읽는다.

기준일: 2026-09-21. Windows 11 + WSL Ubuntu 22.04 대상이다.
설치 결과와 확인 한계는 [시험 기록](report/gazebo_sitl_20260921.md)에 있다.
UWB 연동은 HW팀 정비가 끝날 때까지 보류한다.

## 1. 명령을 넣을 창 구분

프롬프트를 먼저 확인한다. 표의 프롬프트는 입력하지 않는다.

| 창 | 프롬프트 예시 | 실행할 명령 |
|---|---|---|
| Windows PowerShell | `PS C:\...>` | `wsl`, Windows 경로 조회 |
| WSL Ubuntu | `(px4) dronestock@DESKTOP-0C8GRSK:~$` | `cd`, `source`, `make` |
| PX4 콘솔 | `pxh>` | `listener`, `param`, `commander` |
| companion 터미널 | `pgyxn@user-desktop:...$` | 이번 PC 실행 절차의 대상 아님 |

`~/github/PX4-Autopilot`은 WSL 안의 경로다.
PowerShell의 `~`는 Windows 사용자 폴더다.
두 셸에서 같은 문자열이 같은 폴더를 가리키지 않는다.

## 2. WSL 진입 — PowerShell

Windows PowerShell에서 아래를 실행한다.

```powershell
wsl --list --verbose
wsl -d Ubuntu-22.04
```

프롬프트가 `dronestock@DESKTOP-0C8GRSK`로 바뀌면 진입했다.
이후 3절 명령은 이 Ubuntu 창에서 실행한다.

배포판이 없으면 설치 완료 상태가 아니다.
처음 설치할 때 사용한 명령은 다음과 같다.
기존 배포판에서는 다시 설치할 필요가 없다.

```powershell
# 새 PC에서만 실행. Windows 관리자 PowerShell 사용.
wsl --set-default-version 2
New-Item -ItemType Directory -Force -Path D:\WSL
wsl --install -d Ubuntu-22.04 --location D:\WSL\GazeboUbuntu
```

재부팅 요구가 나오면 Windows를 재시작한다.
그 뒤 배포판 목록을 확인한다.
배포판이 없을 때만 설치 명령을 다시 실행한다.
기능 활성화만 끝난 상태와 Ubuntu 설치를 구분한다.
[WSL 설치 명령](https://learn.microsoft.com/en-us/windows/wsl/basic-commands#install)

## 3. Python 환경과 Gazebo 실행 — WSL Ubuntu

설치된 venv와 PX4 저장소를 사용한다.
아래 블록은 PowerShell이나 `pxh>`에 넣지 않는다.

```bash
cd ~/github/PX4-Autopilot || exit
source ~/.venvs/px4/bin/activate || exit

export PATH="$HOME/.venvs/px4/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/lib/wsl/lib"
unset CMAKE_PREFIX_PATH CMAKE_INCLUDE_PATH CMAKE_LIBRARY_PATH
unset Protobuf_DIR protobuf_DIR Protobuf_ROOT protobuf_ROOT
unset CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH LIBRARY_PATH
hash -r

which python
python -m pip check
gz sim --versions
make -j2 px4_sitl gz_x500_lidar_down
```

Python 경로는 `/home/dronestock/.venvs/px4/bin/python`이다.
경로나 venv가 없으면 이 절차의 설치 전제가 충족되지 않는다.
다른 폴더에서 `make`를 실행해도 해결되지 않는다.

이 PATH는 현재 Ubuntu 셸에만 적용한다.
이전 빌드에서 Windows Anaconda 경로가 섞였기 때문이다.
Python venv는 C++ 의존성 검색까지 격리하지 않는다.

`-j2`는 동시 빌드 작업을 2개로 제한한다.
이번 문서에서 추가한 부하 완화 옵션이다.
기존 성공 실행에서 이 옵션을 썼다는 기록은 없다.
PX4의 [Makefile](https://github.com/PX4/PX4-Autopilot/blob/main/Makefile)은 작업 수를 Ninja에 전달한다.
이 제한은 Gazebo 실행 중 CPU 사용량 제한과 다르다.

`Startup script returned successfully` 뒤 `pxh>`를 확인한다.
`No connection to the GCS`가 보이면 4절에서 연결한다.

## 4. QGroundControl 연결 — PowerShell과 Windows 앱

기존 SITL 창은 유지하고 PowerShell 창을 하나 더 연다.

```powershell
wsl -d Ubuntu-22.04 -- ip -4 -o addr show dev eth0
```

`inet` 뒤 주소에서 `/20` 같은 접미부를 뺀다.
시험 때 주소는 `172.25.0.249`였다.
다음 실행에서도 같은 주소라고 가정하지 않는다.

Windows QGroundControl을 연다.
`Application Settings → Comm Links`로 이동한다.
기존 `PX4-WSL` 링크가 있으면 서버 주소를 갱신한다.
없으면 아래 값으로 링크를 추가한다.

| 항목 | 설정 |
|---|---|
| Name | `PX4-WSL` |
| Type | UDP |
| 로컬 포트 | `14550` |
| 서버 주소 | 방금 조회한 WSL IP 뒤에 `:18570` |

일반 UDP 자동 연결은 끄고 수동 링크를 사용한다.
같은 수신 포트를 중복해서 열지 않기 위해서다.
서버 주소를 추가한 뒤 `Save → Connect`를 누른다.

PX4 콘솔의 `partner IP`와 QGroundControl 기체 표시를 확인한다.
이번 시험에서는 이어서 `Ready for takeoff!`가 나왔다.
[PX4 공식 연결 안내](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl#qgroundcontrol-on-windows)

## 5. 센서와 준비 상태 확인 — PX4 콘솔

Gazebo를 시작했던 `pxh>` 창에서 입력한다.
로그가 끼어들어도 Enter를 눌러 명령을 입력할 수 있다.

```text
listener sensor_combined
listener distance_sensor
listener vehicle_local_position
listener estimator_status_flags
param show EKF2_RNG_CTRL
param show EKF2_HGT_REF
commander check
```

거리 출력, `dist_bottom_valid`, 준비 상태를 확인한다.
지상의 거리 약 0.177m는 이번 모델에서 본 값이다.
모든 모델에 강제하는 합격 기준이 아니다.
실패하면 QGroundControl에 나온 현재 사유를 확인한다.
검사를 강제로 우회하지 않는다. 원인을 구분할 수 없기 때문이다.

## 6. 가상 이착륙과 종료 — PX4 콘솔

이 절차는 위에서 띄운 가상 기체에만 적용한다.
실물 기체의 비행 절차는 별도로 검증한다.

준비 상태가 통과하면 아래를 한 줄씩 입력한다.

```text
commander arm
commander takeoff
```

Gazebo에서 상승을 확인한 뒤 거리를 조회한다.

```text
listener distance_sensor
```

확인을 마치면 착륙한다.

```text
commander land
```

착륙과 모터 정지를 확인한 뒤 다시 조회한다.

```text
listener distance_sensor
```

종료할 때 PX4 창에서 `Ctrl+C`를 누른다.
남아 있는 Gazebo 창과 QGroundControl도 닫는다.

## 7. 재개 전에 버전·경로 기록

PX4의 정확한 커밋은 아직 수집하지 않았다.
기본 시험을 다시 할 때 아래 출력도 보관한다.

WSL Ubuntu 셸에서 실행한다. `pxh>` 명령이 아니다.

```bash
cd ~/github/PX4-Autopilot || exit
git rev-parse HEAD
git describe --tags --always --dirty
gz sim --versions
python --version
```

QGroundControl 버전은 앱 정보 화면에서 기록한다.
배포판의 실제 저장 위치는 PowerShell에서 확인한다.

```powershell
Get-ChildItem HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss |
    ForEach-Object { Get-ItemProperty $_.PSPath } |
    Select-Object DistributionName, BasePath
```

`BasePath`가 D 드라이브인지 확인한다.
WSL의 `~/github`를 `/mnt/d`로 옮기는 절차가 아니다.
파일 이동이나 배포판 삭제는 이 실행 절차에 필요하지 않다.

## 8. 다시 오류가 날 때

실패한 단계의 출력부터 보존한다.

| 증상 | 확인·조치 |
|---|---|
| `source`·`export`를 찾지 못함 | PowerShell인지 확인하고 2절로 이동 |
| PX4 폴더·make 대상 없음 | WSL 계정과 `~/github/PX4-Autopilot` 확인 |
| Protobuf 헤더 버전 오류 | 아래의 버전과 CMake 경로를 함께 확인 |
| `No connection to the GCS` | 현재 WSL IP와 수동 UDP 링크 확인 |
| WSL `0x8007274c` | 실행 작업을 종료한 뒤 PowerShell에서 `wsl --shutdown`, 다시 진입 |

`wsl --shutdown`은 다른 WSL 작업도 종료한다.
응답이 계속 없으면 Windows를 재시작한다.
배포판 제거로 해결하려 하지 않는다. 저장한 환경과 로그를 잃기 때문이다.

Protobuf 진단은 WSL Ubuntu에서 실행한다.

```bash
cd ~/github/PX4-Autopilot || exit
type -a protoc
protoc --version
dpkg-query -W -f='${Package} ${Version}\n' protobuf-compiler libprotobuf-dev libgz-msgs10-dev
grep -i protobuf build/px4_sitl_default/CMakeCache.txt
head -n 25 /usr/include/gz/msgs10/gz/msgs/details/header.pb.h
```

이전에는 `Protobuf_DIR`에 `/mnt/c/Users/A/anaconda3/`가 있었다.
그 기록만 보고 Ubuntu 패키지를 무조건 올리지 않는다.
선택한 헤더·라이브러리·컴파일러 경로를 먼저 맞춘다.
