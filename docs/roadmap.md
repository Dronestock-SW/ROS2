# Dronestock 시스템 로드맵 (전면 재설계판 v2 — 군집 반영)

> 본 문서가 기술 방향의 유일한 기준이다.
> 일정 제약 없음 — 판단 기준은 "아키텍처로서 옳은가".
> 단, 각 Phase 종료 시 반드시 "동작하는(나는) 상태"를 유지한다 (통합 리스크 관리).
> 최종 갱신: 2026-07-24

## 시스템 개요 (군집 재고조사)

- 드론 2대가 사전 분할된 구역을 각자 순회하며 야간 재고를 스캔한다
- 드론 간 통신 없음. 각 드론 = 완전 독립 시스템 (Jetson + Pixhawk + 센서 풀세트)
- 지상국 1개가 LoRa로 다중 드론의 상태 텔레메트리를 수신해 웹으로 중계한다
- 1호기 검증(Phase 3) 완료 후 2호기 조립 (부품 추가 주문: 익월 / 조립: 약 2개월 후)

## 핵심 설계 결정 (확정)

1. 위치추정: UWB + LiDAR 스캔매칭 융합. 구 "SLAM 아님" 조항 공식 폐기
2. 고도(z): TFmini Plus → PX4 EKF 융합. companion은 z 직접 제어 금지 (altitude_policy.md)
3. 웹: API 경계(topic)로 추상화, 개발 보류. 경계만 지키면 웹 교체가 드론에 무영향
4. 제어: 자세제어는 PX4 전담. companion은 위치제어 + 게인 스케줄링(상황별 게인 세트 전환)
5. 임무 로직: FSM으로 시작 → 다중 목표+마커 시퀀스 시점에 Behavior Tree 전환
6. 마커: ArUco(정렬) + QR(데이터) 하이브리드 라벨. 단 QR 단독 pose 실험 → 한계 문서화 → 전환 순서
7. 군집 격리: 드론별 ROS_DOMAIN_ID 분리 (1호기=1, 2호기=2). 네임스페이스 개조 불요 — 두 드론의 ROS2는 서로 완전 불가시
8. 광학흐름: PMW3901을 Phase 1부터 PX4 직결 — 실내 Loiter 안정화에 즉시 활용
9. 임무 분배: 사전 구역 분할(정적). 동적 할당·드론 간 회피는 범위 외

## 지상국 / 통신 아키텍처

```
드론 1 ── LoRa ──┐
                  ├── 지상국 (LoRa 수신 → 웹 중계) ── 웹
드론 2 ── LoRa ──┘
```

- LoRa는 저대역폭(수 kbps)이다. 상태 텔레메트리(위치·모드·배터리·임무 진행) 요약본만 송신한다.
  /scan 등 대용량 topic을 LoRa로 보내는 설계는 금지
- 스캔 결과(QR 데이터)는 소용량이므로 LoRa 송신 가능
- 개발/디버깅 중에는 WiFi(Tailscale)가 별도로 살아 있으므로 LoRa는 운용 채널, WiFi는 개발 채널로 역할 분리

## 웹 API 경계 (불변 인터페이스)

| Topic | 방향 | 내용 |
|---|---|---|
| /target_pose | 웹 → 드론 | 목표 좌표 명령 |
| /target_valid | 드론 → 웹 | 목표 허용/거부 + 사유 |
| /flight_state | 드론 → 웹 | 연결/시동/모드/상태 |
| /system_log | 드론 → 웹 | 로그 스트림 |
| /mission_result | 드론 → 웹 | 스캔 결과(QR 데이터) — Phase 4에서 스키마 정의 |

실운용 시 이 topic들의 요약본이 LoRa를 타고 지상국을 경유한다. topic 계약 자체는 불변.

## Phase 구조

### Phase 0 — 기반 (진행 중)
- [x] Jetson 환경 (JetPack 6.2 / ROS2 Humble / Tailscale / VS Code Remote / Claude Code)
- [x] T-mini Pro → /scan 발행 (10Hz)
- [x] Pixhawk 6C Mini heartbeat (PX4 탑재, MAVROS connected)
- [x] UWB 앵커 4기 자체 제작 + 설치 + 좌표계 검증 (멀티 태그 지원 확인)
- [ ] TFmini Plus → PX4 EKF rangefinder 융합 설정
- [ ] PMW3901 → PX4 직결 배선 + 파라미터 설정 (결정 8)
- [ ] UWB 태그 → /uwb_pose 발행 노드
- [ ] udev rule 장치 고정 이름 (/dev/lidar, /dev/pixhawk, /dev/tfmini)
- [ ] ROS_DOMAIN_ID=1 설정을 1호기 .bashrc에 고정 (결정 7)

### Phase 1 — 안정 비행 베이스라인 (1호기, tether)
- 이륙 시퀀스 자동화: arm → 상승 → Loiter 진입 (PMW3901+TFmini 융합 상태에서)
- UWB 필터링 EKF급 설계
- FSM 임무 골격: IDLE → TAKEOFF → LOITER → GOTO → ARRIVED → LAND
- 위치제어 PID + 게인 스케줄링 1차 (근접 감속 게인 세트)
- 가상벽/keepout 검사 (구 WBS W2 승계)
- LoRa 텔레메트리 송신 노드 1차 (드론 측) — 지상국 수신단 기보유, 링크 테스트 즉시 가능
- 완료 판정: tether 상태에서 웹 좌표 이동 + 안전정지 반복 재현

### Phase 2 — 위치추정 고도화
- LiDAR 스캔매칭: scan-to-scan → scan-to-map
- UWB + 스캔매칭 + IMU 융합 (robot_localization EKF 또는 자작)
- 완료 판정: UWB 단독 대비 위치 오차 감소를 로그로 입증

### Phase 3 — 마커 기반 정밀 작업
- QR 단독 pose 실험 → 검출 거리/각도 한계 문서화 → ArUco 하이브리드 전환
- 비주얼 서보잉: 도달 → 마커 정렬 → 스캔 위치 유지
- FSM → BT 전환 검토
- 완료 판정: 지정 선반 정렬 + QR 판독 반복 재현
- ★ 이 Phase 완료 = 2호기 조립 착수 조건

### Phase 4 — 군집 임무 시스템 완성
- 2호기 조립 (ROS_DOMAIN_ID=2, 1호기 설정 복제)
- 구역 분할 임무 정의 (드론별 담당 구역 좌표 세트)
- BT 기반 다중 목표 순회 (선반 N개 스캔)
- 지상국: 다중 드론 LoRa 수신 + /mission_result 스키마 확정 + 웹 재접속
- 야간 자동 재고 파악 실증 — "밤사이, 재고가 파악됩니다"

## 폐기/보류 항목

- 자세제어 AI: 폐기 (자세는 PX4 검증 영역)
- 드론 간 통신/상호 회피: 범위 외 (구역 분리로 원천 차단)
- 자작 마커: 보류 (ArUco/AprilTag가 검증된 해답)
- 웹 개발: 보류 (API 경계 topic 계약만 유지)
- 동적 임무 할당: 범위 외 (사전 구역 분할 확정)