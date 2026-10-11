# C++ 관측 입력 경계
수동 비행 로그와 C++ 행동 계층의 연결 기준이다.
호버 복구 전 입력을 준비하고 재생할 때 읽는다.

관측 해석은 `src/sangwon_AI`의 C++가 담당한다.
현장 수집기는 원본 ROS 메시지를 그대로 기록한다.
이번 연결은 파일 재생이다. 실행 서비스는 바꾸지 않는다.

```text
Tag B → B_TF ──────────┐
PX4 → MAVROS ─────────┴→ 수집기 → manifest + events.jsonl
                                       ↓
                         C++ CaptureObservations
                         ├─ UWB 안테나 XY 관측
                         ├─ PX4 로컬 ENU 위치·상태
                         └─ ToF 거리·추정기 상태·시각 진단
                                       ↓
                         오프라인 보고서 / 후속 adapter 입력
```

## 입력 계약

수집기 schema 1을 직접 읽는다. 별도 변환 파일은 필요 없다.

| 입력 | 보존·해석 | 유효 판정의 한계 |
|---|---|---|
| manifest | boot_id, tag, tag_id, domain, source_revision | 사용자 설정 식별자다. 실물 신원 증명은 아니다 |
| config_evidence | 원본 설정의 내용·SHA256 대조 | 보고서에는 이름·hash만 남긴다. 실측 인증은 아니다 |
| 모든 표본 | 원본 header_ns, 수신 ROS·monotonic ns | 서로 다른 시계 값을 직접 빼지 않는다 |
| B_TF pose | uwb_map, 안테나 XY, XY 공분산 | z=0·단위 quaternion은 미관측 자리값이다 |
| PX4 odom | map ENU, base_link FLU, pose·twist·공분산 | 0 공분산을 오차 0으로 해석하지 않는다 |
| PX4 estimator | 위치·속도·자세 상태와 오류 flag | 메시지 수신과 EKF 준비 완료는 별개다 |
| ToF | 센서 frame, 거리, 최소·최대 범위 | 창고 Z나 FC 고도로 치환하지 않는다 |
| FC 상태 | 연결, ARM, mode, landed | 모드 전환·시동·착륙 명령은 만들지 않는다 |
| timesync | offset, 원격 시각, 왕복 지연 | 수신 성공만으로 고정 지연을 확정하지 않는다 |

원본 시각의 정수 ns 정밀도를 유지한다.
같은 header 재수신은 새 관측으로 인정하지 않는다.
동일 좌표·새 header는 정지 중 새 관측으로 인정한다.
오래된 값·미래 시각·시간 역행·frame 오류를 구분한다.
새 오류 표본이 오면 이전 좌표를 유효하게 유지하지 않는다.
수신 공백 때 마지막 값의 유효성을 자동 만료한다.
기록 순서의 monotonic 역행은 재생 오류로 처리한다.
FC 재부팅·시각 reset 후에는 새 수집 조각으로 조사한다.

진단 만료값은 위치·ToF 250ms, 추정기·시각 500ms다.
FC 상태·지상 판정은 1.5초다.
이는 파일 진단 기준이다. 비행 승인 임계값이 아니다.
원본 age와 수신 후 경과를 합쳐 만료시킨다.
공분산은 유한성·대칭·양의 준정부호를 검사한다.
UWB XY 분산은 양수여야 한다.

## 후속 연결

C++ `CaptureObservations`는 ROS·소켓·FC 포트를 사용하지 않는다.
동일 `ingest` 계약을 후속 구독 adapter에서 재사용한다.
현재 `State`와 BT에 실물 입력을 자동 주입하지 않는다.
이유: 창고↔PX4 변환과 명령 정책이 아직 미확정이다.

2026-10-10 `/scan` 관측 검사를 추가했다.
[C++ LiDAR 준비](cpp_lidar_input.md)의 경계를 따른다.
원본 TF는 선택 기록한다. 장착 적용·스캔매칭은 후속이다.

`can_start`, `flight_authority`, `physical_output_enabled`는 false다.
정렬·장착·시간·융합 검증 flag를 보고서가 승격하지 않는다.
UWB를 PX4 상태로 바꾸거나 별도 EKF를 만들지 않는다.
수동 호버 → 동적 보정 → PX4 융합 → C++ 명령 연결 순서다.
고도 제어는 [고도 기준](../altitude_policy.md)을 따른다.

실행 명령과 결과는 [준비 기록](../report/cpp_observation_preparation_20261010.md)을 따른다.
