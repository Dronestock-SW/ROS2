# 부팅 후 수동 비행 기록
전원 재인가 뒤 관측·수집을 자동 시작하는 절차다.
수동 호버 복구와 UWB 보정 자료 수집 때 읽는다.

부팅은 비행 명령을 실행하지 않는다.
기존 C++ 임무 실행기로의 이식은 후속 작업이다.
수동 호버 → UWB 융합 → C++ 이식 → 웹 UI 순서다.

```text
FC flow/ToF/IMU ─ MAVROS ─┬─ 수집기 ─ 영구 저장장치
Tag B ─ UWB 원본 ─ B_TF ─┘
          EV bridge 없음 / mission 실행기 없음
```

## 설치

기존 다섯 sangwon 서비스와 현장 설정을 보존한다.
`src/drone_uwb/deployment/manual-capture.env.example`을
`~/.config/dronestock/manual-capture.env`로 복사한다.
실제 workspace·태그·현장 설정 경로를 넣는다.
환경 파일과 실제 교정값은 Git에 넣지 않는다.

```bash
cd ~/ROS2-review-20261008-codex
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_uwb --symlink-install
mkdir -p ~/.config/systemd/user ~/.local/libexec/dronestock
cp src/drone_uwb/deployment/run_manual_capture.sh ~/.local/libexec/dronestock/
cp src/drone_uwb/deployment/dronestock-manual-*.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now dronestock-manual-capture.service dronestock-manual-observe.service
loginctl show-user "$USER" -p Linger
```

Linger=yes가 로그인 없는 자동 시작 조건이다.
서비스 활성과 실제 전원 재인가 시험은 구분한다.
포트가 없거나 다른 프로세스가 소유하면 대기한다.
기존 포트 소유자를 종료하거나 강제로 열지 않는다.
관측 노드 종료 때 관측 묶음을 다시 시작한다.
수집 서비스는 별도로 유지하며 결측을 기록한다.
MAVROS allocator 오류의 근본 원인 해결은 별도다.

## 기록 확인

기본 저장 경로는 다음과 같다.
`~/.local/share/dronestock/manual-captures/`
RAM 저장소가 아니므로 재부팅 후에도 남는다.
3분 또는 기존 수집기 종료 조건마다 새 조각을 만든다.
조각 교체 때 작은 수신 공백이 생길 수 있다.
여러 조각은 boot_id와 원본 시각으로 함께 분석한다.

```bash
systemctl --user status dronestock-manual-{observe,capture}.service
cat ~/.local/share/dronestock/manual-captures/status.json
ros2 run drone_uwb uwb_manual_capture status /absolute/path/to/capture-SESSION
```

supervisor의 active는 자식 프로세스 상태다.
실제 기록 상태는 수집기의 status로 확인한다.
필수 토픽 누락·공백·큐 오류도 함께 확인한다.
앵커가 꺼져 있으면 거리·B_TF가 누락될 수 있다.
이를 기록 성공이나 융합 성공으로 바꾸지 않는다.

| 보존 항목 | 정책 |
|---|---|
| 조각 크기 | 최대 256MiB, 기본 180초 |
| 저장장치 여유 | 1GiB를 남기고 쓰기 중단 |
| 다음 조각 시작 | 여유 1,280MiB 이상 |
| 정상 지상 대기 | 가장 최근 3개 보존 |
| ARM·상태 불명·오류·비정상 종료 | 자동 삭제하지 않음 |
| 위 보존 구간 직전 지상 조각 | 출발 전 문맥으로 추가 보존 |
| 전원 차단 | 마지막 버퍼·checkpoint 일부 손실 가능 |

정상 지상 대기는 신선한 연결·DISARM·지상이 필요하다.
조각의 시작·끝·수신 간격도 1.5초 안이어야 한다.
오래된 지상 대기만 삭제하며 비행 기록을 덮지 않는다.
디스크가 부족하면 status에 대기 사유를 남긴다.
10초마다 저장 파일의 fsync를 시도한다.
전원 차단에 대한 원자적 전체 기록을 보장하지 않는다.

## FC 경계와 ULog

관측 묶음에는 mission·EV bridge가 없다.
MAVROS vision/setpoint plugin도 로드하지 않는다.
MAVROS 통신과 고정 텔레메트리 요청은 존재한다.
DISARM 연결 때 COMMAND_LONG 511만 요청한다.
IMU 100Hz·ToF 40Hz 등 기존 측정 주기를 쓴다.
전역 원점 조회 반복은 제거했다.
이는 FC 전체 무송신 시험과 다르다.

부팅별 `fc-before-BOOT.json`을 한 번 저장한다.
FC가 DISARM일 때 고정 읽기만 한다.
기록 실패·미완료면 파일 내용을 확인한다.
자동으로 FC 파라미터를 바꾸거나 재부팅하지 않는다.

10월 10일 호버 기준은 MAG_TYPE=0, FLOW_ROT=4다.
MPC_THR_HOVER=0.35, EV_CTRL=1을 유지한다.
MAG_TYPE은 9일 저장·FC 재부팅·재조회까지 마쳤다.
사용자가 확인한 흐름은 그 복원 전 비행에서 발생했다.
복원 후 호버 성공은 아직 미확인이다.

FC ULog는 ROS 수집기와 별개다.
기존 SDLOG_MODE=1을 이번 자동 수집이 바꾸지 않는다.
장시간 지상 기록은 2GB 상한에 도달한 사례가 있다.
전원 재인가 뒤 logger 상태와 파일 증가를 확인한다.
같은 시행의 ULog를 별도로 확보해야 한다.
로그가 멈췄다고 센서 융합 중단으로 단정하지 않는다.

## 중지와 회수

```bash
systemctl --user stop dronestock-manual-capture.service
systemctl --user stop dronestock-manual-observe.service
```

수동 비행 기록을 PC로 복사하고 SHA256을 대조한다.
PC 검증 전 비행 원본을 지우지 않는다.
영구 해제는 두 서비스에 `disable --now`를 사용한다.
기존 C++ 서비스와 웹 서비스는 별도로 보존한다.
배포·정리·지상 확인은 [10월 10일 결과](../report/manual_boot_capture_20261010.md)에 있다.

## C++ 후속 분석

수집기는 변경 없이 같은 schema 1을 기록한다.
종료된 조각을 복사한 뒤 C++로 직접 재생한다.
다른 boot_id의 조각은 한 시계로 합치지 않는다.
실행 중인 조각은 뒤에 쓰기가 계속될 수 있다.
손상·미완성 행은 성공 보고서 없이 오류로 끝난다.
구체적인 명령은 [C++ 준비 결과](../report/cpp_observation_preparation_20261010.md)를 따른다.
