# Pixhawk 펌웨어/연결 방식 결정

> 구 WBS W1-07 산출물. 확정 완료된 결정이며 roadmap.md와 일치한다.

- 펌웨어: PX4 (Offboard 모드 공식 지원, 자료 풍부. 6C Mini 공장 탑재 확인됨)
- 연결: MAVROS2 (ROS2 topic 구조와 일치)
- 제어 흐름: Jetson /cmd_vel -> MAVROS2 -> PX4 Offboard
- 고도: TFmini Plus를 PX4 EKF에 융합 (확정 — roadmap 결정 2)
- 보조: PMW3901 광학흐름을 PX4에 직결 (roadmap 결정 8)
- 결정일: 2026-07-12 / B 동의로 확정 완료