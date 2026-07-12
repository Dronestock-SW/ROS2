# W1-07: Pixhawk 펌웨어/연결 방식 결정

- 펌웨어: PX4 (Offboard 모드 공식 지원, 자료 풍부)
- 연결: MAVROS2 (ROS2 topic 구조와 일치)
- 제어 흐름: Jetson /cmd_vel -> MAVROS2 -> PX4 Offboard
- 고도: TFmini rangefinder는 PX4 EKF 융합 여부를 W3-03에서 결정
- 결정일: 2026-07-12 / 확정: B 동의 후
