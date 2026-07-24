# Dronestock 시스템 로드맵 (전면 재설계판)

> 기존 8주 WBS는 초안(러프 스케치)으로 격하한다. 본 문서가 기술 방향의 기준이다.
> 일정 제약 없음 — 판단 기준은 "일정에 맞는가"가 아니라 "아키텍처로서 옳은가".
> 단, 각 Phase 종료 시점에 반드시 "동작하는(나는) 상태"를 유지한다. 이는 일정 관리가 아니라 통합 리스크 관리다.
> 최종 갱신: 2026-07-24

## 핵심 설계 결정 (확정)

1. 위치추정: UWB + LiDAR 스캔매칭 융합을 채택한다. 구 WBS의 "SLAM 아님" 조항은 공식 폐기.
2. 고도(z): TFmini Plus를 PX4 EKF에 융합한다. companion(Jetson)은 z를 직접 제어하지 않는다 (altitude_policy.md 유지).
3. 웹: API 경계(topic)로 추상화하고 개발은 보류. 경계 topic만 지키면 웹 전체 교체가 드론 측에 무영향.
4. 제어: 자세제어는 PX4 내부 루프에 전담시킨다 (자세제어 AI 도입 안 함). companion은 위치제어만 담당하며, 상황별 게인 스케줄링(예: 목표 근접/외란 시 게인 세트 전환)으로 고도화한다.
5. 임무 로직: FSM으로 시작하고, 임무 복잡도가 임계(다중 목표 순회 + 마커 시퀀스)를 넘는 시점에 Behavior Tree로 전환한다.
6. 마커: 접근/정렬용 ArUco(또는 AprilTag) + 데이터용 QR의 하이브리드 라벨. 단, QR 단독 pose 추정을 먼저 실험하고 한계를 문서화한 뒤 전환한다.

## 웹 API 경계 (불변 인터페이스)

| Topic | 방향 | 내용 |
|---|---|---|
| /target_pose | 웹 → 드론 | 목표 좌표 명령 |
| /target_valid | 드론 → 웹 | 목표 허용/거부 + 사유 |
| /flight_state | 드론 → 웹 | 연결/시동/모드/현재 상태 |
| /system_log | 드론 → 웹 | 로그 스트림 |
| /mission_result | 드론 → 웹 | 스캔 결과(QR 데이터) — Phase 4에서 정의 |

웹이 무엇이든(자체 서버, 외부 API, 대시보드) 이 topic 계약만 지키면 드론 측 코드는 불변.

## Phase 구조

### Phase 0 — 기반 (진행 중)
- [x] Jetson 환경 (JetPack 6.2 / ROS2 Humble / Tailscale / VS Code Remote)
- [x] T-mini Pro → /scan 발행 (10Hz 확인)
- [x] Pixhawk 6C Mini heartbeat (PX4 탑재 확인, MAVROS connected)
- [ ] TFmini Plus → UART 수신 노드 + PX4 EKF rangefinder 융합 설정
- [ ] UWB(DWM3000EVB) → /uwb_pose 발행 — 앵커 세트 보유 여부 확인 선행
- [ ] udev rule로 장치 고정 이름 부여 (/dev/lidar, /dev/pixhawk, /dev/tfmini)

### Phase 1 — 안정 비행 베이스라인
- 이륙 시퀀스 자동화: arm → 상승 → Loiter 진입 (tether 환경)
- UWB 필터링을 이동평균이 아닌 EKF급으로 설계
- FSM 임무 골격: IDLE → TAKEOFF → LOITER → GOTO → ARRIVED → LAND
- 위치제어 PID + 게인 스케줄링 1차 (근접 감속 게인 세트)
- 가상벽/keepout 검사 (구 WBS W2 항목 승계)
- 완료 판정: tether 상태에서 웹 좌표 이동 + 안전정지 반복 재현

### Phase 2 — 위치추정 고도화
- LiDAR 스캔매칭: scan-to-scan → scan-to-map 순차 도입
- UWB + 스캔매칭 + IMU 융합 (robot_localization EKF 또는 자작)
- PMW3901 광학흐름 융합 후보 평가 (보유 장비 활용)
- 완료 판정: UWB 단독 대비 위치 추정 오차 감소를 로그로 입증

### Phase 3 — 마커 기반 정밀 작업
- 실험 1: QR 단독 pose 추정 (QRCodeDetector + solvePnP) → 검출 거리/각도 한계 문서화
- 실험 2: ArUco 하이브리드 라벨 (프린터 출력, 10~15cm, 실측 크기 파라미터화)
- 비주얼 서보잉: 도달 → 마커 정렬 → 스캔 위치 유지
- 이 시점에서 FSM → BT 전환 검토
- 완료 판정: 지정 선반 앞 정렬 후 QR 판독 성공 반복 재현

### Phase 4 — 임무 시스템 완성
- BT 기반 다중 목표 순회 (선반 N개 스캔 미션)
- /mission_result 스키마 확정 + 웹 API 재접속
- 야간 자동 재고 파악 시나리오 실증 — "밤사이, 재고가 파악됩니다"

## 폐기/보류 항목

- 자세제어 AI: 폐기. 자세 루프는 PX4 검증 영역. companion의 AI 활용은 게인 스케줄링·상위 판단으로 한정
- 자작 마커: 보류. ArUco/AprilTag가 동일 문제의 검증된 해답. 학습용 사이드 실험으로만 허용
- 웹 개발: 보류. API 경계 topic 계약만 유지