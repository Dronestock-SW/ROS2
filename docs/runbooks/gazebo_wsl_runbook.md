# WSL Gazebo·PX4 SITL 재실행 절차

이 문서는 설치를 마친 시험 PC의 실행 절차다. 시뮬레이터를 다시 켤 때 읽는다.

갱신일: 2026-09-27. Windows 11 + WSL Ubuntu 22.04 대상이다.
설치 결과와 확인 한계는 [시험 기록](../report/gazebo_sitl_20260921.md)에 있다.
사용자 요청으로 [가상 UWB 비교](uwb_gazebo_shadow_runbook.md)를 준비한다.
9월 21일 실물 연동 보류와 구분한다.
가상 UWB 관측의 PX4 전달·융합은 아직 검증하지 않았다.

## 현재 목표와 진행 순서

목표는 사용자 앵커·센서 배치를 가상 공간에 넣고 UWB 계산 후보를 비교하는 것이다.
그 뒤 선정한 위치 관측을 PX4에 연결해 가상 호버링을 시험한다.
Gazebo는 가상 기체·환경·센서를, PX4 SITL은 가상 비행제어기를 담당한다.
Python 도구는 가상 UWB 거리 생성·기록과 계산 후보 비교를 담당한다.

```text
WSL Ubuntu 진입 → Python 계산·통신 모듈 확인
  → 준비한 앵커·장비 모델 적용 → 가상 센서 데이터 수신
  → 같은 입력으로 A/B/C/D 비교 → 위치 관측의 PX4 융합·호버링 시험
```

현재는 사용자 WSL에서 수정 시험장과 PX4의 시작을 확인한 단계다.
로그에 `Gazebo world is ready`와 `gz_bridge`의 새 모델 이름이 있다.
PX4 시작 스크립트도 완료됐다.
IMU·기압·하방 거리의 최근 표본도 확인했다.
다음은 별도 Ubuntu 창에서 가상 UWB RAW를 생성·기록하는 단계다.
명령은 [장비 절차](uwb_gazebo_equipment_runbook.md) 5절을 따른다.
모델 파일 설치와 실제 센서 수신 성공은 구분한다.
사용자가 명령의 목적을 이해하며 진행하도록 다음 순서로 안내한다.
목적 → 입력할 셸 → 명령 → 예상 출력 → 다음 단계 순서다.
셸이 다르면 먼저 진입만 확인한 뒤 다음 명령을 안내한다.

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

창 제목이 Windows PowerShell이어도 WSL에 들어가면 같은 창에서 Ubuntu를 사용한다.
명령 입력 위치는 창 제목이 아니라 마지막 줄의 프롬프트로 판단한다.
VS Code가 SSH로 연결한 `pgyxn@user-desktop`은 코드 준비 작업공간이다.
사용자 Windows PC의 `dronestock` WSL과 다른 환경이다.

| 명령 | 목적 |
|---|---|
| `wsl -d Ubuntu-22.04` | 설치된 Ubuntu 환경에 진입. 재설치 명령이 아님 |
| `sudo apt update` | Ubuntu 설치 가능 패키지 목록 갱신 |
| `sudo apt install -y python3-numpy` | 시스템 Python의 배열·수치 계산 라이브러리 설치 |
| `/usr/bin/python3 -c "..."` | 시스템 Python으로 짧은 코드 실행. 현재는 모듈 로딩 확인에 사용 |
| `make -j2 px4_sitl gz_x500_lidar_down` | 기존 기본 모델로 PX4 SITL 빌드·실행. Python 모듈 검사와 별도 단계 |

PowerShell에서 `sudo` 비활성화나 `/usr/bin/python3`를 찾지 못했다는 오류가 나면
Windows 설정을 바꾸기 전에 WSL Ubuntu 진입 여부를 확인한다.
이 작업에 필요한 것은 Ubuntu의 `sudo`와 Python이다.

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

`eth0` 조회 결과가 비어 있으면 주소를 추측하지 않는다.
WSL Ubuntu 창에서 다음 두 출력으로 실제 IPv4 인터페이스와 경로를 확인한다.

```bash
ip -br -4 addr
ip -4 route
```

인터페이스 이름이 달라졌을 수 있다.
Microsoft의 [WSL 네트워크 안내](https://learn.microsoft.com/windows/wsl/networking)는
Windows에서 WSL 주소를 조회할 때 `wsl.exe hostname -I`를 제시한다.
미러링 네트워크 모드에서는 양쪽이 `127.0.0.1`로 통신할 수 있다.
현재 PC의 모드와 출력 확인 전에는 QGroundControl 서버 주소를 확정하지 않는다.

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

27일 사용자 출력에서 짧은 커밋을 확인했다.
`git describe`는 `v1.18.0-beta1-700-gc4e4ef98e9`다.
Gazebo 출력은 `8.15.0`이다.
`/opt/ros`는 없으며 다른 설치 경로는 미확인이다.
전체 커밋과 Python 버전은 아래 명령으로 보관한다.

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

### 가상 UWB 비교 연결 준비

별도 WSL 창에서 Gazebo 통신을 조회한다.
실행 중인 `pxh>` 창은 그대로 둔다.
새 PowerShell에서 `wsl -d Ubuntu-22.04`를 실행한다.
이후 Ubuntu 셸에 아래를 입력한다.

```bash
gz topic -l
/usr/bin/python3 -c "from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V; import numpy; print('Gazebo Python OK')"
```

첫 명령은 현재 Gazebo 통신 항목을 나열한다.
두 번째는 위치 기록에 쓸 Python 모듈을 확인한다.
출력 또는 오류 전문을 보관한다.
2026-09-27 사용자 출력에서 `default / x500_lidar_down_0`의 토픽 이름을 확인했다.
Gazebo Python 두 모듈은 로딩됐고 `numpy`에서 실패했다.
후속 사용자 출력에서 Ubuntu의 `python3-numpy` 설치 완료를 확인했다.
이후 같은 검사에서 `Gazebo Python OK` 출력을 받았다.
세 모듈 로딩은 확인됐다. 센서 메시지 수신은 아직 미확인이다.
설치 명령은 [장비 적용 절차](uwb_gazebo_equipment_runbook.md)의 0절을 따른다.
PX4·Gazebo 기본 실행과 이 조회에는 ROS가 필요하지 않다.
프로젝트 ROS 2 노드 실행은 별도 환경이 필요하다.

## 8. 다시 오류가 날 때

실패한 단계의 출력부터 보존한다.

| 증상 | 확인·조치 |
|---|---|
| `source`·`export`를 찾지 못함 | PowerShell인지 확인하고 2절로 이동 |
| PX4 폴더·make 대상 없음 | WSL 계정과 `~/github/PX4-Autopilot` 확인 |
| `PX4 server already running for instance 0` | 같은 번호의 PX4가 실행 중이다. 모듈 확인 중에는 기존 실행을 유지하고 별도 Ubuntu 셸에서 확인 명령만 실행 |
| Protobuf 헤더 버전 오류 | 아래의 버전과 CMake 경로를 함께 확인 |
| `No connection to the GCS` | 현재 WSL IP와 수동 UDP 링크 확인 |
| WSL `0x8007274c` | 실행 작업을 종료한 뒤 PowerShell에서 `wsl --shutdown`, 다시 진입 |

중복 실행 오류 뒤 `ninja`와 `make`가 실패했다고 표시될 수 있다.
2026-09-27 사용자 출력에서는 컴파일이 아니라 PX4 실행 단계가 중단됐다.
이 출력만으로 기존 PX4·Gazebo의 정상 동작까지 판정하지 않는다.
재시작이 필요할 때는 기존 PX4 실행 창에서 `Ctrl+C`로 정상 종료한다.
그 뒤 실행 명령을 한 창에서 한 번만 사용한다.
[PX4 공식 실행 안내](https://github.com/PX4/PX4-user_guide/blob/main/en/dev_setup/building_px4.md)도 이 종료 방법을 설명한다.
새 장비 모델로 전환할 때는 [장비 적용 절차](uwb_gazebo_equipment_runbook.md)의 3절을 따른다.

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
