# Gazebo·PX4 SITL 기본 시험 결과

이 문서는 2026-09-21 가상 시험 결과다. 완료 범위와 재개 지점을 확인할 때 읽는다.

## 완료 범위

Windows PC에서 기본 가상 이착륙을 확인했다.
Gazebo·PX4 SITL·QGroundControl 연결을 마쳤다.
UWB 연동은 HW팀 정비로 보류했다.

| 항목 | 판정 | 근거 |
|---|---|---|
| Gazebo·PX4 SITL 실행 | 완료 | `Startup script returned successfully` |
| 가상 IMU 수신 | 확인 | `sensor_combined` 가속도·각속도 출력 |
| 하방 거리 수신 | 확인 | `distance_sensor`, 방향 25 |
| EKF2 거리 융합 상태 | 활성 상태 확인 | `cs_rng_hgt=True`, `cs_rng_terrain=True` |
| Windows QGroundControl 연결 | 완료 | `partner IP`, `Ready for takeoff!` |
| 기본 가상 이착륙 | 확인 | 지상→공중→지상 거리 변화 |
| UWB→SITL 관측 전달·융합 | 미실행·보류 | HW팀 정비 후 재개 |
| 추정 정확도·반복성 평가 | 미실행 | 기준값과 연속 로그 비교가 남음 |

근거는 사용자가 대화에 제공한 출력과 화면이다.
[출력 발췌](evidence/gazebo_sitl_20260921_user_excerpt.md)에 보존했다.
작성자가 해당 Windows PC에서 직접 실행한 기록은 아니다.
전체 ULog와 Gazebo 기준 궤적은 저장소에 수집하지 않았다.
이번 문서 갱신에서도 시뮬레이터를 재실행하지 않았다.

## 실행 환경

시뮬레이터는 별도 Windows PC에서 실행했다.
companion의 ROS2 작업공간과 구분한다.

| 항목 | 확인값 또는 확인 수준 |
|---|---|
| Windows | Windows 11 Pro, 10.0.26100, 64비트 |
| CPU·RAM | Intel Core i7-9700F, 31.9 GB |
| GPU | NVIDIA GeForce RTX 2070 SUPER |
| Linux 환경 | WSL의 `Ubuntu-22.04` 배포판 |
| Linux 계정 | `dronestock@DESKTOP-0C8GRSK` |
| Python 환경 | `~/.venvs/px4`, Python 3.10 |
| PX4 저장소 | `~/github/PX4-Autopilot`, main 기반 설치 |
| PX4 정확한 커밋 | 미수집 |
| Gazebo | Harmonic, `gz sim --versions` 출력 `8.15.0` |
| 시험 모델 | `gz_x500_lidar_down` |
| QGroundControl | Windows 앱 연결 확인. 정확한 버전 미수집 |
| WSL 설치 위치 | 설치 명령에 `D:\WSL\GazeboUbuntu` 지정 |
| 실제 디스크 저장 위치 | 배포판의 현재 `BasePath` 미수집 |

WSL에 Gazebo와 C++ 의존성을 설치했다.
Python 패키지는 venv에 설치했다.
venv는 Gazebo·C++ 라이브러리를 격리하지 않는다.
설치 위치를 지정한 명령만으로 D 드라이브 상태를 확정하지 않는다.

## 확인한 연결

QGroundControl에 WSL 주소를 직접 등록했다.

```text
Windows PC
├─ QGroundControl: UDP 수신 14550
│       ↕ MAVLink
└─ WSL Ubuntu-22.04: PX4 UDP 18570
   ├─ PX4 SITL: EKF2·비행 제어
   │       ↕ 가상 센서·모터 입력
   └─ Gazebo: x500_lidar_down

companion ROS2 / 실물 UWB: 이번 SITL에 연결하지 않음
```

| QGroundControl 설정 | 시험 때 사용한 값 |
|---|---|
| 링크 이름 | `PX4-WSL` |
| 형식 | UDP |
| 로컬 포트 | `14550` |
| 서버 주소 | `172.25.0.249:18570` |
| PX4가 알린 상대 주소 | `172.25.0.1` |

WSL 주소는 재시작 후 달라질 수 있다.
다시 연결할 때 현재 주소를 조회한다.
[PX4의 Windows QGroundControl 연결 절차](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl#qgroundcontrol-on-windows)를 따른다.

연결 전에는 `No connection to the GCS`가 나왔다.
이때 `commander check`도 실패했다.
연결 후 로그는 `Ready for takeoff!`로 바뀌었다.
이전 실패와 이후 준비 상태를 시각 순서로 구분한다.
연결 후 `commander check`의 재조회 출력은 미수집이다.

## 센서와 EKF2 확인

하방 거리가 기체의 상승·하강에 따라 변했다.

| 시점 | `current_distance` | 단위 |
|---|---:|---|
| 처음 지상 | 0.17701 | m |
| 공중 | 2.52873 | m |
| 착륙 후 | 0.17701 | m |

지상에서 센서와 바닥 사이 간격이 남는다.
따라서 착륙 후 값이 0일 필요는 없다.
세 표본으로 정확도·안정성 통계를 계산하지 않는다.
이유: 연속 궤적과 독립 기준값을 확보하지 않았다.

| 출력·설정 | 확인값 | 해석 |
|---|---|---|
| `orientation` | 25 | 아래 방향 |
| `min_distance`, `max_distance` | 0.1, 100 m | 가상 센서 범위 |
| `signal_quality` | -1 | 품질 정보 미제공 |
| `variance` | 0 | 분산 미상 또는 무효 표시 |
| `EKF2_RNG_CTRL` | 1 | 조건부 거리 융합 |
| `EKF2_HGT_REF` | 1 | GPS 높이 기준 |
| `dist_bottom_valid` | True | 바닥 거리 추정 유효 |
| `cs_rng_hgt`, `cs_rng_terrain` | True | 거리 융합·지형 추정 활성 상태 |
| `cs_rng_fault`, `cs_rng_stuck` | False | 해당 고장 표시 없음 |
| `cs_gnss_pos`, `cs_gnss_vel` | True | 위성항법 관측도 사용 중 |
| `cs_ev_pos`, `cs_ev_hgt` | False | 외부 위치·높이 융합 비활성 |

`variance=0`을 오차가 없다는 뜻으로 읽지 않는다.
[거리 메시지 정의](https://docs.px4.io/main/en/msg_docs/DistanceSensor)를 따른다.
높이 기준과 거리 융합 사용 여부는 별개다.
[거리 융합 설정](https://docs.px4.io/main/en/advanced_config/parameter_reference#EKF2_RNG_CTRL),
[높이 기준 설정](https://docs.px4.io/main/en/advanced_config/parameter_reference#EKF2_HGT_REF)을 함께 본다.

초기 `heading_good_for_control=False`도 기록됐다.
나중에 `cs_yaw_align=True`와 이륙 준비 로그가 나왔다.
같은 heading 필드의 최종 재조회 값은 없다.
모든 상태 필드가 정상으로 바뀌었다고 확장하지 않는다.

이 모델은 하향 1D LiDAR를 사용한다.
TFmini Plus의 제품 오차를 재현한 모델은 아니다.
실물 TFmini Plus 입고·배선·융합 완료로 계산하지 않는다.

## 설치 중 문제와 확인 수준

환경 오류를 거친 뒤 SITL 기동에 성공했다.

| 문제 | 확인한 사실 | 남은 한계 |
|---|---|---|
| 배포판을 찾지 못함 | WSL 기능 설치 뒤 Ubuntu 설치를 진행 | 기능 설치와 배포판 설치를 구분 |
| PowerShell에서 Linux 명령 실행 | `source`, `export`, `make` 인식 실패 | WSL 진입 뒤 실행해야 함 |
| Protobuf 빌드 오류 | CMake 설정에 Windows Anaconda 경로 혼입 | 마지막 성공 빌드의 캐시 원문 미수집 |
| CPU 과부하 호소 | PX4 빌드 도중 발생 | CPU·메모리 사용량 수치 미수집 |
| WSL 연결 시간 초과 | `Wsl/Service/0x8007274c` 출력 | 정확한 원인·복구 명령 결과 미수집 |

Protobuf 설치값은 `3.12.4`였다.
Gazebo 헤더는 `3.12.0` 이상을 요구했다.
버전 숫자보다 혼입된 검색 경로를 우선 조사했다.
Linux PATH 제한과 CMake 캐시 재생성을 안내했다.
그 뒤 기동 성공 출력이 왔다.
어떤 조치가 단독 원인이었는지는 확정하지 않는다.

## 보류와 재개 지점

HW팀 정비가 끝나면 UWB 입력 계약부터 확인한다.
현재 코드의 브리지는 기본 비활성이다.
설정 근거는 [uwb.yaml](../../src/drone_uwb/config/uwb.yaml)이다.

```text
실물 태그 → uwb_node → /uwb_pose
                         ↓
                  uwb_px4_bridge (기본 비활성)
                         ↓
             /mavros/vision_pose/pose_cov → MAVROS → PX4
```

이 경로는 코드에 존재한다. 이번 SITL에서 실행하지 않았다.
좌표축·원점·시각·장착 위치 확인이 필요하다.
코드의 전달 조건은 [frames.py](../../src/drone_uwb/drone_uwb/frames.py)에 있다.
UWB의 z=0은 미관측 자리값이다.
고도와 자세 제어는 [PX4 책임](../altitude_policy.md)으로 유지한다.

실물 태그와 가상 기체는 서로 다른 운동을 한다.
실물 관측을 그대로 넣으면 가상 IMU와 어긋날 수 있다.
동적 가상 시험은 같은 Gazebo 운동에서 관측을 만든다.
실측 로그는 잔여 잡음·편향·지연을 정하는 자료로 쓴다.
이 연결 방안은 조사 결과이며 구현 완료가 아니다.

재개 순서는 다음과 같다.

1. HW팀의 정비 완료와 새 출력 형식을 확인한다.
2. 시험 PC의 PX4 커밋·파라미터·로그를 확보한다.
3. ROS2·MAVROS 실행 환경과 관측 경로를 정한다.
4. 같은 운동의 관측으로 전달·융합을 검증한다.
5. Gazebo 기준값과 EKF2 추정값의 오차를 평가한다.

기본 시험 재실행은 [WSL 실행 절차](../gazebo_wsl_runbook.md)를 따른다.
