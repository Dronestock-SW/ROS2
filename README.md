# Dronestock-SW / ROS2 — 드론 자율이동 MVP

웹 좌표 명령 기반 실드론 자율이동 (8주 WBS)

## 접속 정보
- Jetson Orin Nano (JetPack 6.2 / Ubuntu 22.04 / ROS2 Humble)
- SSH: `ssh user@100.110.163.94` (Tailscale — 팀원은 Tailscale 설치 + 네트워크 초대 필요)

## 빌드 방법
```bash
cd ~/drone_ws
colcon build
source install/setup.bash
```

## 브랜치 규칙
- `main`: 항상 빌드되는 상태 유지. 직접 push 금지, PR로만 병합
- 작업 브랜치: `feat/작업ID-설명` (예: `feat/W1-02-uwb-node`)
- 커밋 메시지: `작업ID: 내용` (예: `W1-02: uwb_node 초안`)

## 역할
- A: ROS2 / 자율주행 로직
- B: 하드웨어 / 센서 / Pixhawk
- C: 웹 / 알고리즘 / 로그 / 문서

## Topic 구조
| Topic | 내용 |
|---|---|
| /uwb_pose | UWB 현재 위치 (x, y만 — z 없음) |
| /target_pose | 웹에서 들어온 목표 좌표 |
| /target_valid | 목표 좌표 허용/거부 |
| /cmd_vel | 이동 명령 (x, y 속도) |
| /scan | YDLIDAR 전방 scan |
| /tfmini_range | TFmini 하방 거리 (고도) |
| /safety_stop | 장애물 정지 신호 |
| /flight_state | Pixhawk 상태 |
