# Windows UWB 재시작과 첫 실시간 기록
이 문서는 2026-10-02 Windows에서 수행한 결과다.
WSL의 관측 융합·동적 시험을 이어갈 때 읽는다.

## 현재 결과

사용자 승인으로 Ubuntu-22.04를 재시작했다.
기존 설정으로 PX4·Gazebo가 정상 기동했다.
첫 RAW·ToF·IMU·PX4 상태 기록도 정상 종료했다.
UWB 관측 송신·EKF 융합·목표 이동은 아직 미검증이다.

| 항목 | 실제 확인 |
|---|---|
| PX4 / Gazebo 서버·화면 PID | 430 / 641·642. 14:39 KST 조회 성공 |
| PX4 커밋 / Gazebo | c4e4ef98e9d75063bf3d53ebb2716221ee7505ae / 8.15.0 |
| 기존 설정 | 4016 / gz_dronestock_x500 / dronestock_uwb / 2.09,1.68,0.24,0,0,0 |
| trial.json | 38ab27fac63c8e333e8bc094f7e47ca76b1248f2bdf1f7107a97aac561e2f557. 원본과 동일 |
| Python 결합 import | Gazebo·NumPy·pymavlink·SITLNavigationSession 성공 |
| 실제 Python / NumPy | ~/.venvs/px4/bin/python / 1.21.5 |
| CLI 도움말 | sitl_navigation --help 종료 코드 0 |
| 첫 monitor / RAW 발행기 | 모두 종료 코드 0, duration_complete |

재기동 직후 사용 메모리는 454 MiB였다.
가용 메모리는 15,298 MiB로 회복했다.
PX4·Gazebo를 유지한 14:39 조회에서는 다음과 같았다.
사용 4,119 MiB, 가용 11,561 MiB, 스왑 사용 0 MiB다.
기동 초기의 사용 1,273 MiB와 비교하면 증가가 보인다.
증가 원인은 아직 미확인이다. 장시간 시험 전에 진단한다.
14:48 KST 최종 조회에서도 같은 세 프로세스가 실행 중이었다.
가용 메모리는 8,846,304 KiB로 더 줄었고 스왑 사용은 0이었다.
Gazebo 서버·GUI RSS는 각각 3,264,596·3,883,640 KiB였다.

## 최신 실행 위치와 Python 경로

```text
/home/dronestock/uwb_sim/windows_resume_20261002_140858/uwb-gazebo-equipment
```

실제 기록 폴더는 위 경로의 `runs/restart_20261002_01`이다.
원래 폴더·월드·기체·앵커·장착·잡음 설정은 보존했다.
최신 누적본의 54개 파일은 배포 ZIP과 해시가 일치한다.

가상환경만 사용한 최초 import는 gz 누락으로 실패했다.
실행 프로세스의 PYTHONPATH에 시스템 경로만 추가했다.
설치나 전역 가상환경 설정 변경은 수행하지 않았다.

```text
PYTHONPATH=<최신 실행 폴더>/src/drone_uwb:<최신 실행 폴더>/src/drone_demo:/usr/lib/python3/dist-packages
```

이 구성의 하나의 Python에서 모두 가져왔다.
NumPy 1.21.5는 시스템 경로에서 실제 로딩됐다.
가상환경에 존재하는 NumPy 2.2.6과 구분한다.

## 첫 같은 세션 기록

monitor는 navigation monitor + sitl timesync 모드였다.
RAW 발행기는 정상 입력이며 별도 이상 주입은 없었다.
시드 7과 기존 B 0.8초 창을 유지했다.
관측·목표 확인 플래그는 기존 false를 유지했다.

| 기록 | 결과와 분모 |
|---|---:|
| monitor 시뮬레이션 구간 | 127.732~157.780 s, 30.048 s |
| RAW 수신 / 계산 | 812 / 812 |
| ToF / IMU / clock | 3,036 / 7,589 / 7,590 |
| RAW 발행기 전체 기록·발행 | 939 / 939, 약 35 s |
| RAW 발행 성공률 | 26.8276 Hz, 시뮬레이션 시각 기준 |
| monitor 공통 구간 RAW 누락 | 0 / 812 |
| B 성공 / 계산 RAW | 810 / 812, 99.7537% |
| A/C/D/WLS 보류 | 각 812 / 812 |
| MAVLink HEARTBEAT / ODOMETRY | 30 / 908 |
| ESTIMATOR_STATUS / PARAM_VALUE | 295 / 231 |
| TIMESYNC 응답 | 295. 시각 정렬 완료 판정과 별개 |
| UWB 관측 / 목표 명령 송신 | 0 / 0 |

A 등은 orientation_alignment_unconfirmed 사유로 보류됐다.
B의 초기 두 주기는 거리 이력 준비 부족이었다.
실제 파라미터 응답 11종을 수집했다.
EKF2_EV_CTRL=0, EKF2_GPS_CTRL=7이다.
EKF2_OF_CTRL=1, EKF2_RNG_CTRL=1도 확인했다.
이 설정의 존재만으로 센서의 실제 융합을 판정하지 않았다.
수신한 PX4 ODOMETRY reset_counter는 구간 내 11이었다.
monitor 임무 상태는 WAITING / inputs_missing이다.
COMMAND_ACK는 상태 요청 응답이며 목표 이동 성공이 아니다.

## B 정지 진단 범위

같은 Gazebo 세션 정답을 계산 입력과 분리해 대조했다.
유효 출력 810개를 같은 측정 시각의 안테나 정답과 비교했다.

| 지표 | B H80 0.8초 정지 결과 |
|---|---:|
| 중앙값 | 1.178 cm |
| RMS | 1.398 cm |
| p95 | 2.433 cm |
| 최대 | 3.749 cm |
| 7cm 이상 | 0 / 810 |

한 지점 지상 정지 진단이다.
독립 다점·동적 7cm 최종 달성으로 판정하지 않았다.
A/B/C/D 선정·B 0.4초 비교·실제 비행은 미실시다.

## 로그와 재현 근거

RAW·ToF·자세·시계·정답·상태 원문·설정·명령을 보관했다.
각 실행기의 manifest와 input_index는 입력·소스 해시를 담는다.
현재 실행 중인 ULog 원본은 다음과 같다.

```text
/home/dronestock/github/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/2026-10-02/05_29_43.ulg
```

원본을 닫거나 변경하지 않고 앞부분을 복사했다.
ULog 스냅샷 크기는 120,787,931바이트다.
SHA-256은 `96c37fd4914e65e1132db827738c410cce85e6250568563577c871d4e656e0ab`다.
이는 실행 중인 로그의 스냅샷이다. 최종 종료 로그가 아니다.

전체 기록과 ULog 스냅샷은 별도 ZIP에 보관했다.
파일: `uwb_windows_restart_records_with_ulog_20261002.zip`
크기: 50,904,520바이트.
SHA-256: `6d50ec1686124490f8566c5acd15747f0c729d054c4639117bc4f3a10e3790de`.

원격 보관 위치는 다음과 같다.

```text
/home/pgyxn/github/ROS2/data/raw/uwb/windows_restart_20261002_01
```

원격 ZIP과 SHA-256 파일, 실행 evidence JSON을 함께 보관했다.
원격 ZIP의 SHA-256이 일치했고 40개 항목의 ZIP 무결성 검사가 통과했다.
인계 문서와 docs/README 색인에 새 보고서·Goal 프롬프트를 연결했다.
새 Goal 프롬프트는 원격 `docs/uwb_goal_prompt_20261002.txt`에도 보관했다.

## 다음 작업과 Goal 인계

다음 단계는 메모리 증가와 ToF·자세·기준점 확인이다.
그 뒤 A의 좌표·시각 검증과 실제 EKF2 융합을 확인한다.
B 0.4초와 모델 비교, 독립 다점·동적 평가를 이어간다.
공중 Hold 준비와 짧은 이동·복귀·이상 대응도 남아 있다.
실제 FC 파라미터는 이번 작업에서 바꾸지 않았다.

새 Goal용 전체 프롬프트는 UTF-8 텍스트로 작성했다.
Windows outputs의 `uwb_goal_prompt_20261002.txt`다.
Goal을 이 응답에서 설정하거나 별도 채팅을 만들지는 않았다.
사용자가 `/goal` 입력에 프롬프트를 적용할 수 있다.
명령 사용 근거는 [공식 Goal 안내](https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex)다.
