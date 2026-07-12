# 고도(z) 처리 원칙 (W1-09)

1. UWB는 x, y 수평 위치만 제공한다. (/uwb_pose에 z 없음)
2. 고도는 TFmini(하방 거리) + Pixhawk IMU/EKF가 유지한다.
3. companion(Jetson)은 z를 직접 제어하지 않는다.
4. z 유효성 검사는 개발하지 않는다.
   - virtual_wall_node에 아래 한 줄만 유지:
   - `# TODO: z 범위 검사 (예외처리, 후순위)`

## 센서 역할
- UWB: x, y 수평 위치
- TFmini: 하방 고도 측정
- YDLIDAR: 전방 장애물 감지 (고도 아님)
- Pixhawk IMU/EKF: 고도 제어 전담
