# 수신 Companion 비행 코드 검토
이 문서는 9월 11일 ZIP의 비행 코드 검토다.
우리 기체에 적용할 범위를 정할 때 읽는다.

## 1. 결론

수신 비행 코드는 현재 기체에 그대로 적용할 수 없다.
임무 흐름은 참고할 수 있다. 제어 계층은 재설계가 필요하다.

| 구분 | 판단 |
|---|---|
| 재고조사 목적 | 경유지 이동·스캔·복귀 흐름은 부합 |
| 우리 설계 | 고도 책임·상태추정·ROS2 경계와 충돌 |
| PX4 호환성 | 모드 송신 값의 형식이 맞지 않음 |
| 실패 처리 | 출력 제한 우회·낡은 값 도착 판정 등 재현 |
| 비행 중 웹 경로 변경 | v1 계약이 금지. 실행 코드도 교체 안전성을 보장하지 않음 |
| 권장 활용 | HTTP/WS 계약·임무 모델·로그 설계 참고 |
| 현재 실행 서비스 | `drone_platform_link`. ZIP 비행 코드를 사용하지 않음 |

이번 검토는 코드와 모의 입력을 사용했다.
기체·시리얼·비행 출력은 연결하지 않았다.
기존 통신 서비스도 변경하지 않았다.

## 2. 비교 기준과 실행 흐름

판정 기준은 우리 로드맵과 실제 PX4 버전이다.

| 기준 | 근거 |
|---|---|
| 제어·상태추정 책임 | [로드맵](../roadmap.md) 결정 2·4·10 |
| 고도 책임 | [고도 정책](../altitude_policy.md) |
| 장비 | [BOM](../equipment_inventory.md). TFmini Plus·PMW3901은 문서상 미입고 |
| UWB 관측 | [노드 설계](../architecture/uwb_node_design.md). 수평 관측 전용 |
| 웹 계약 | [Platform 계약 v1](../hw_handoff/platform_api_v1/companion_platform_api_contract_v1.md) |
| PX4 버전 | 9월 11일 저장 기록의 `Release 1.16.0` |
| 검토 ZIP | [원본](../hw_handoff/platform_api_v1/DroneStock-Companion-Jetson-20260911.zip) |
| 검토 증거 | [모의 입력 결과 13건](../report/evidence/companion_flight_review_20260913.json) |

ZIP SHA256:
`5dd97e663e450d60b8a88ca00b79d9a950db5094cb07ee6863a65e96d24f528b`

아래 소스 위치는 ZIP의 `tools/companion/` 기준이다.
원본은 수정하지 않았다.

```text
수신 ZIP
웹 GET --+--> 주 루프: UWB 위치 예측·보정 + 고도 선택
센서 ----+                     |
                              v
                     임무 FSM → XY·고도 제어
                                      |
                                      v
                             RC 제한 → 출력 backend
                                      |        |
                                      |        +→ 외부 pose·거리 → FC
                                      +→ RC 채널 덮어쓰기 → FC

우리 기준
웹 → 통신 → /target_pose → 목표 허용 검사 → 임무·수평 제어 → PX4
UWB → 관측 전처리 → 검증한 외부 관측 경로 -----------------> PX4 EKF2
TFmini Plus·PMW3901·FC IMU ------------------------------> PX4 EKF2
PX4 추정 상태 → 임무·수평 제어
PX4 상태·UWB 관측 → 통신 → 웹
```

우리 비행 노드가 모두 구현됐다는 그림은 아니다.
수평 명령 형식과 고도 유지 모드의 조합은 남은 설계다.

## 3. 설계와 충돌하는 부분

변수명을 바꾸는 것만으로 해결되지 않는다.

| 항목 | 우리 목적·기준 | ZIP 동작 | 판단·소스 |
|---|---|---|---|
| 고도 | PX4가 고도 유지 | companion 고도 PID로 throttle 채널 계산 | 명시적 충돌. `control/altitude_controller.py:19`, `controller_manager.py:60` |
| 이륙·착륙 | PX4의 고도·자세 제어 사용 | throttle 증감으로 상승·하강 계산 | 교체 대상. `takeoff_controller.py:69`, `landing_controller.py:17` |
| 수평 상태 | PX4 EKF2가 비행 상태를 추정 | UWB에서 속도·다음 위치를 예측하고 직접 제어에 사용 | 결정 10과 충돌. `estimation/uwb_pose_resolver.py:86` |
| FC 되먹임 | FC 추정값을 독립 관측으로 재입력하지 않음 | FC yaw를 외부 ODOMETRY yaw로 다시 전송 | 융합 설정에 따라 중복 관측 위험. `control/backend.py:122` |
| 장비 | PX4 직결 TFmini Plus·FC IMU | companion 직결 ToF10120·MPU6050 필수 기본값 | 장비·데이터 경로가 다름. `config.py:54`, `app.py:74` |
| UWB | x·y 관측만 제공 | `CONTROL_GRADE`와 UWB z를 요구 | 수평 전용 태그와 충돌. `io/uwb_serial_reader.py:275`, `takeoff_controller.py:121` |
| 내부 통신 | ROS2 API 경계 유지 | 직접 시리얼 소유·MAVLink 송신 | 기존 MAVROS 경로와 통합 설계 필요. `app.py:57`, `io/pixhawk_mavlink.py:243` |
| 좌표 | 검증한 직교 좌표·회전·원점·시각 | 서버 x·y와 FC yaw를 바로 결합 | 배치·방향 검증이 빠짐. `xy_controller.py:21`, `backend.py:122` |
| 정밀 스캔 | ArUco 정렬과 QR 판독 | 경유지 도착 뒤 QR 카메라 판독 | Phase 3 목적의 일부만 포함 |

RC 채널은 조종기 입력을 표현한다.
모터별 PWM 직접 제어와 같은 뜻이 아니다.
ZIP도 PX4의 내부 자세 제어를 대체하지는 않는다.
문제는 고도 제어와 비행용 관측 처리 책임이다.

외부 관측의 좌표·지연·장착 위치는 함께 맞춰야 한다.
PX4도 이를 별도 설정 대상으로 설명한다.
[PX4 v1.16 외부 위치 입력](https://docs.px4.io/v1.16/en/ros/external_position_estimation).
FC yaw 재전송이 현재 기체 EKF를 오염시켰다는 뜻은 아니다.
이번에는 실제 FC 송신과 융합을 수행하지 않았다.

## 4. 우선 수정할 코드 결함

아래 항목은 실기 적용 전에 해소해야 한다.

| ID | 문제와 재현 조건 | 관측 결과·영향 | 근거 |
|---|---|---|---|
| F01 | PX4 모드의 세 값 묶음을 정수 필드로 송신 | 모의 정수 직렬화에서 실패. `set_flight_mode()`는 false 반환 | `io/pixhawk_mavlink.py:103` |
| F02 | 외부 관측 실패 시 최종 RC 출력을 다시 생성 | 제한 후 throttle 1000이 송신 직전 1500으로 바뀜. 낡은 ToF 0.2m도 공중 판정에 사용 | `control/backend.py:98`, `backend.py:197` |
| F03 | `LAND`에서 신뢰하지 않는 고도 0.05m 입력 | `POWER_OFF` 전환과 DISARM 요청. 착지 지속·FC 착지 상태 확인 없음 | `mission/mission_fsm.py:146` |
| F04 | 목표 위치의 마지막 값만 남고 XY 유효성 false | 0.8초 뒤 도착 완료. 도착 함수가 유효성·속도를 검사하지 않음 | `mission/route_manager.py:45` |
| F05 | 이동 중 서버 status가 HOLD 또는 CANCEL | 이동 상태와 기존 목표 유지. status는 주로 이륙 허가에만 사용 | `mission/mission_fsm.py:64`, `launch_guard.py:44` |
| F06 | 진행 번호 1에서 경로 개정값 변경 | 번호 0으로 초기화. 비행 상태·명령 우선순위 확인 없음 | `mission/mission_fsm.py:54`, `route_manager.py:26` |
| F07 | UWB x·y 정상, UWB z 미지원 | 이륙의 `XY_STABILITY_CHECK`가 완료되지 않음 | `control/takeoff_controller.py:121` |
| F08 | MPU6050 roll 18° 입력 | 즉시 `EMERGENCY_CUT`. 시간 지속 검사 없음. 별도로 10초 된 정상 자세도 age를 무시해 OK | `safety/imu_safety_monitor.py:26` |
| F09 | UWB seq=2의 같은 x=0.3을 두 제어 주기에 제공 | 처리 x가 0.084 → 0.185952로 변함. 새 관측 없이 재처리 | `estimation/uwb_pose_resolver.py:74` |
| F10 | `--dry-run-rc`에서 FC 모드가 POSCTL | 모터 출력·arm은 막지만 LOITER 모드 요청은 발생 | `app.py:66`, `control/backend.py:43` |

F01은 PX4 모드 표현과 대조했다.
pymavlink의 PX4 모드는 세 값의 묶음이다.
전용 처리에서는 이를 나누어 사용한다.
[pymavlink 공식 소스](https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py).
실제 FC 모드 변경 시험은 하지 않았다.

F02의 1000·1500은 조종기 throttle 채널 값이다.
값 1500이 실제 부양을 보장하지 않는다.
또한 사전 제한을 통과한 값을 뒤에서 변경한다.

F03의 DISARM은 요청 발생을 뜻한다.
실제 PX4가 공중 DISARM을 수락했는지는 시험하지 않았다.
F08도 독립 IMU 판정이 출력 중단 요청으로 이어진다는 뜻이다.
18°를 우리 기체의 승인된 비상 기준으로 사용하지 않는다.
이유: 기체 시험과 운영 합의의 근거가 없기 때문이다.

## 5. 추가 확인이 필요한 부분

설정 이름이나 송신 성공만으로 기능을 인정할 수 없다.

| 항목 | 코드에서 확인한 사실 | 필요한 검증 |
|---|---|---|
| 제어 주기 | 20Hz 주 루프에서 HTTP 동기 조회. GET 대기 제한 2초 | 통신 지연 때 제어 주기 유지. 50ms 주기에 2초 대기는 40주기 분량 |
| 제한 설정 | `max_xy_speed_mps`, `max_xy_step_m`, `yaw_alignment_ready`는 설정 외 참조 없음 | 제한이 실제 명령에 적용되는지 검사 |
| 제어 방식 선택 | `fc_guided`와 미인식 이름도 `companion_rc`로 변환 | 미지원 모드 거부. 이름만 보고 Offboard로 판단하지 않음 |
| 외부 관측 수락 | `send_*()` 성공을 통신 함수 반환으로 판단 | FC 수신과 EKF2 사용 상태를 따로 확인 |
| 자세·고도 출처 | FC yaw와 상대고도를 다시 외부 관측에 포함 가능 | 독립 관측만 공급하도록 필드·융합 비트 대조 |
| 공분산 | `*_error_m`·`yaw_error_rad`를 공분산 대각에 그대로 대입 | 표준편차인지 분산인지 단위 합의 |
| 관측 초기화 | 큰 UWB 변화에서 추적값 초기화. ODOMETRY reset counter는 고정 0 | 초기화 알림과 급변 거부 정책 검증 |
| FC 메시지 최신성 | 고도·자세·배터리가 공유 수신 시각 사용 | 각 값의 실제 관측 시각 유지 |
| 수동 제어 복귀 | 비행 중 FC 모드 변화에 대한 출력 중단 경로가 뚜렷하지 않음 | 조종자 수동 전환이 확실히 우선하는지 시험 |
| 장애물 | 이 ZIP에서 LiDAR·가상벽 검사 경로를 찾지 못함 | 선반·벽·허용 구역 검사 구현 확인 |

공분산과 reset counter는 MAVLink 계약과 대조했다.
[MAVLink ODOMETRY 정의](https://mavlink.io/en/messages/common.html#ODOMETRY).

Offboard를 선택하는 경우에는 별도 검증이 필요하다.
웹 2Hz 조회를 FC 제어 유지 신호로 사용할 수는 없다.
PX4 v1.16은 2Hz를 넘는 유지 신호를 요구한다.
진입 전 1초 이상 공급도 요구한다.
[PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard).
이는 Offboard 적용을 확정했다는 뜻이 아니다.

## 6. 재사용 범위

임무 모델과 통신 계약을 남기고 제어 연결을 다시 정한다.

| 부분 | 활용 판단 |
|---|---|
| HTTP·WS 필드 계약 | 우리 통신 어댑터의 기준으로 사용 |
| FSM의 단계 분리 | 설계 참고. 실패·취소·복구 전이는 수정 필요 |
| 경유지 모델 | ID·경로 개정·scan 유형은 재사용 후보 |
| QR 요청 ID·오프라인 저장 | 서버 수락·거부 의미를 확인한 뒤 재사용 후보 |
| 로그·진단 | 출처·시각·상태 전환 기록 방식 참고 |
| 고도 PID·throttle 상승·하강 | 우리 비행 코드에 이식하지 않음 |
| UWB 비행 상태 예측기 | 우리 관측 전처리를 대체하지 않음 |
| RC backend·외부 관측 묶음 | 그대로 사용하지 않음. F01~F10과 책임 분리를 먼저 해결 |

제외 이유는 고도·상태추정 책임 충돌과 재현된 결함이다.
센서 송신과 비행의 판정 기준은 [검증 기준 초안](../architecture/companion_link_flight_acceptance.md)을 본다.
