# 고도(z) 처리 원칙

> 구 WBS W1-09 산출물. 전면 재설계(roadmap.md) 이후에도 유효한 원칙이다.

1. UWB는 x, y 수평 위치만 제공한다. (/uwb_pose에 z 없음)
2. 고도는 TFmini Plus(하방 거리) + Pixhawk IMU/EKF가 유지한다. (EKF 융합 확정 — roadmap 결정 2)
3. companion(Jetson)은 z를 직접 제어하지 않는다.
4. z 유효성 검사는 개발하지 않는다.
   - virtual_wall_node에 아래 한 줄만 유지:
   - `# TODO: z 범위 검사 (예외처리, 후순위)`

## 센서 역할
- UWB (DWM3000EVB 태그 + 자체 제작 앵커 4기): x, y 수평 위치
- TFmini Plus: 하방 고도 측정 (PX4 EKF 융합)
- T-mini Pro (YDLIDAR): 전방 장애물 감지 + Phase 2 스캔매칭 (고도 아님)
- PMW3901 광학흐름: PX4 직결 — 실내 위치 유지 보조 (roadmap 결정 8)
- Pixhawk IMU/EKF: 고도·자세 제어 전담