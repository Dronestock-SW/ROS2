# Dronestock-SW / ROS2 — 드론 자율이동 MVP

드론 자율이동 저장소의 설치·실행 안내다. 작업을 시작할 때 읽는다.

## 현재 작업 상태

기술 방향은 [로드맵](docs/roadmap.md)을 따른다.
구 8주 WBS는 현재 기준이 아니다.

- [위치 유지 참고 설정](docs/report/perfect_holdv2_20260927.md):
  `perfect_holdv2.params` 원본과 27일 ULog 비교.
- [27일 Position 위치 유지](docs/report/position_hold_20260927.md):
  두 구간의 추종 오차·속도와 분석 도구.
- [기본 설정 확인](docs/report/setup_status_20260921.md):
  2026-09-21 `pgyxn` 계정·저장소·설치 상태.
- [UWB 참고 자료와 코드 상태](docs/uwb_h80_qs10_reference.md):
  9월 20일 ZIP, 필터 기본값, 남은 연결 작업.
- [문서 색인](docs/README.md): 기준·절차·확인 기록.

기능 완료 때마다 관련 `docs/`를 함께 갱신한다.
[에이전트 규칙](AGENTS.md)에 완료 조건을 명시했다.

## 접속 정보
- Jetson Orin Nano (JetPack 6.2 / Ubuntu 22.04 / ROS2 Humble)
- SSH: ssh user@100.110.163.94 (Tailscale — 팀원은 Tailscale 설치 + 네트워크 초대 필요)

## 받기 (clone)
LiDAR 드라이버는 제조사 저장소를 서브모듈로 참조한다. `--recursive` 없이 받으면 그 폴더가 빈 채로 온다.

```bash
git clone --recursive https://github.com/Dronestock-SW/ROS2.git drone_ws
# 이미 --recursive 없이 받았다면
git submodule update --init
```

## 빌드 방법
cd ~/drone_ws 후 colcon build --symlink-install, 그다음 source install/setup.bash

## LiDAR 실행
```bash
ros2 launch drone_bringup lidar.launch.py
```
파라미터는 `src/drone_bringup/params/lidar_tmini.yaml`에 있다. 제조사 코드
(`src/ydlidar_ros2_driver`)는 수정하지 않는다 — 제조사 새 버전과 충돌하고,
별도 저장소라 우리 수정이 이 저장소에 기록되지 않기 때문이다.

## Gazebo 가상 시험 — 2026-09-21

Windows PC의 WSL에서 기본 가상 이착륙을 확인했다.
Gazebo·PX4 SITL·Windows QGroundControl을 연결했다.
하방 거리는 지상 0.17701m → 공중 2.52873m → 지상 0.17701m였다.
UWB 연동은 HW팀 정비로 보류했다.

| 문서 | 읽는 시점 |
|---|---|
| [WSL 재실행 절차](docs/gazebo_wsl_runbook.md) | 터미널 구분·실행·QGroundControl 재연결 |
| [기본 시험 결과](docs/report/gazebo_sitl_20260921.md) | 완료 범위·남은 검증 확인 |
| [사용자 제공 출력](docs/report/evidence/gazebo_sitl_20260921_user_excerpt.md) | 결과의 근거와 수집 한계 확인 |
| [9월 20일 준비 논의](docs/report/gazebo_preparation_20260920.md) | 센서 역할과 가상 시험 설계 검토 |

이번 결과는 실물 비행이나 UWB 융합 완료를 뜻하지 않는다.
실행 환경은 companion과 별도다.

## 브랜치 규칙
현재는 1인 개발이라 main에 직접 push 한다. 리뷰할 사람이 없는 PR은 절차 비용만 남는다.

- main: 항상 빌드되는 상태 유지
- 커밋 메시지: "Phase/작업ID: 내용" (예: "Phase 0: uwb_node 초안")

**협업자가 합류하면 아래로 전환한다.**

- main 직접 push 금지, PR로만 병합
- 작업 브랜치: feat/작업ID-설명 (예: feat/W1-02-uwb-node)

## 역할
- A: ROS2 / 자율주행 로직
- B: 하드웨어 / 센서 / Pixhawk
- C: 웹 / 알고리즘 / 로그 / 문서

## Topic 구조
| Topic | 내용 |
|---|---|
| /uwb_pose | UWB 수평 위치 관측. z=0은 미관측 자리값 |
| /target_pose | 웹에서 들어온 목표 좌표 |
| /target_valid | 목표 좌표 허용/거부 |
| /cmd_vel | 이동 명령 (x, y 속도) |
| /scan | YDLIDAR 전방 scan |
| /tfmini_range | TFmini 하방 거리 (고도) |
| /safety_stop | 장애물 정지 신호 |
| /flight_state | Pixhawk 상태 |

## UWB 개발 자료

[활용 데이터](data/README.md)와 [코드·자료 분류 기준](docs/uwb_data_layout.md)을 따른다.
수신은 `acquisition/`, 보정은 `processing/`, ROS 연결은 `integration/`에 둔다.
[모듈 입출력](docs/uwb_module_api.md)과 [재실행 절차](docs/uwb_data_runbook.md)를 참고한다.
