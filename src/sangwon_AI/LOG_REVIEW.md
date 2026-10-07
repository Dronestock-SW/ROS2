# 수동 RC 호버링 ULog 초기 확인
이 문서는 제공된 ULog의 관측 결과를 설명한다.
행동 조건을 설계할 때 자료의 범위를 확인한다.

## 1. 판단

이 로그는 호버링 상태 확인과 재생 설계에 쓸 수 있다.
UWB 좌표 이동·자동 우회 성능의 증거는 아직 없다.

| 항목 | 확인 결과 |
|---|---|
| 파일 | log_23_2026-9-27-13-34-50.ulg |
| 크기 | 2,839,356 bytes |
| 기록 길이 | 70.908608초 |
| PX4 | v1.17.0 / PX4_FMU_V6C |
| 기록된 topic 인스턴스 | 107개 |
| 파서 | pyulog 1.2.4 / NumPy 2.5.3 |
| 로그의 dropout 표식 | 0개 |
| 파라미터 변경 기록 | 0개 |
| 사용자 확인 | 수동 RC 비행, 호버링 포함 |

dropout 표식 0개가 모든 센서의 완전성을 뜻하지 않는다.
기록 주기는 실제 센서·제어 주기와 다를 수 있다.

## 2. 관측과 설계 영향

topic별 timestamp를 기준으로 기초 통계를 확인했다.
선택한 15개 topic의 instance 0에서 시각 역행·중복은 0개다.

| 관측 | 근거 | 설계 영향 |
|---|---|---|
| RC 비행 모드가 바뀜 | ALTCTL ↔ POSCTL, Offboard 기록 없음 | 모드별 구간을 나눠 분석 |
| 위치 기록 있음 | vehicle_local_position 709건, 약 10Hz | 제어용 상태 재생 가능 |
| XY 유효 표식 | 709/709건 true | 유효 표식이 정확도 보장은 아님 |
| 외부 위치 융합 미설정 | EKF2_EV_CTRL=0 | 이 비행의 UWB 융합 증거 없음 |
| 방향 추정 원천 | EKF2_MAG_TYPE=0, cs_mag_hdg=1, cs_ev_yaw=0 | 이 로그의 heading은 자력계 기반 |
| 방향 제어 품질 표식 | heading_good_for_control=false 709/709건 | 의미·원인과 자동 모드 적합성 확인 필요 |
| 방향 reset 계수 | heading_reset_counter=3, 기록 내 변화 없음 | 기록 전 reset 여부·실제 방향 정확도 미확인 |
| 외부 위치 기록 없음 | vehicle_visual_odometry, estimator_aid_src_ev_pos 없음 | UWB 자체의 미설치는 단정 불가 |
| 추력 명령 있음 | vehicle_thrust_setpoint 3,541건 | 부하·구간 비교에 활용 |
| 호버 추력 추정 있음 | 403건 중 valid 382건 | 유효 구간만 따로 비교 |
| POSCTL 중 유효 추정 | 215건, 중앙값 0.32089 | 호버 구간 분석의 기준 후보 |
| XY 목표값 일부 비유한 | x/y 각 292/709건만 유한 | 모드에 따른 미사용 필드 구분 |
| 거리계·광학흐름 기록 | 원시 topic 각각 약 1Hz | 빠른 장애물·센서 반응 검증에는 부족 |
| 광학흐름 융합 표식 | instance 0의 fused=1, 139/141건 | 일부 비행 상태의 관측 근거 |
| RC 입력 있음 | manual_control_setpoint 355건 | 조작 구간과 유지 구간 구분 가능 |
| RC 인계 설정 | COM_RC_OVERRIDE=1, 스틱 기준 30% | Auto 인계만 켜진 기록 |
| RC 입력원 선택 | COM_RC_IN_MODE=3 | 첫 유효 RC 또는 MAVLink 수동 입력원 고정 |
| Offboard 송신 상실 설정 | COM_OF_LOSS_T=1.0초, COM_OBL_RC_ACT=0 | 상실 뒤 Position 모드 후보 |
| RC 연결 상실 설정 | COM_RCL_EXCEPT=0, NAV_RCL_ACT=3 | 해당 펌웨어는 Land 설정 |
| 공통 실패 반응 지연 | COM_FAIL_ACT_T=5.0초 | 실제 Offboard Land 지연은 시험 필요 |
| 배터리 자체 동작 | COM_LOW_BAT_ACT=0, BAT_LOW/CRIT/EMERGEN_THR=15/7/5% | 현재는 경고만 설정, 독립 착륙 실패 동작 필요 |

추력 xyz는 기체 축 기준 정규화 명령이다.
단위 N의 실측 추력으로 해석하지 않는다.
[해당 펌웨어의 메시지 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/VehicleThrustSetpoint.msg)

0.32089는 호버 추력의 PX4 추정치다.
실제 추력·기체 질량 또는 모터별 출력값이 아니다.
POSCTL 전체가 안정 호버였는지도 입증하지 않는다.
기록의 초기 `MPC_THR_HOVER` 설정은 0.35였다.
차이는 설정 변경 사유를 검토할 근거다.

초기 `EKF2_HGT_REF=0`은 기압계 높이 기준이다.
`EKF2_RNG_CTRL=1`은 조건부 거리계 융합이다.
로드맵이 제시한 Range 기준값 2와 다르다.
거리 융합 표식은 116/141건이다.
융합 표식과 고도 기준 선택은 별개로 기록한다.

이 기록에는 자동 이륙·Offboard 비행이 없다.
따라서 RC 스틱 인계 동작은 아직 검증되지 않았다.
`COM_RC_OVERRIDE=1`은 해당 펌웨어에서 Auto만 선택한다.
Offboard 스틱 인계를 쓰려면 bit 1도 켜야 한다.
[해당 펌웨어의 매개변수 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/commander_params.c)

Offboard 상실과 RC 상실은 서로 다른 고장이다.
기록된 설정이 실제 비행에서 작동했는지는 입증되지 않았다.

## 3. 모드 전환

시간은 ULog 헤더 시작 시각을 기준으로 한다.

| 시작(초) | 모드 |
|---|---|
| 0.027 | ALTCTL: 고도 제어 |
| 20.124 | POSCTL: 위치 제어 |
| 27.685 | ALTCTL |
| 31.480 | POSCTL |
| 53.199 | ALTCTL |

POSCTL 구간은 호버링 분석의 후보다.
RC 입력·속도·착륙 상태를 함께 봐야 한다.
구간 전체를 안정 호버링으로 확정하지 않았다.
[모드 값 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleStatus.msg)

## 4. 판단 한계와 다음 자료

한 번의 RC 비행으로 자율비행 임계값을 확정하지 않는다.

| 영향도 | 부족한 근거 | 필요한 자료 |
|---|---|---|
| 높음 | 창고 절대 좌표 정확도 | 동기화된 UWB 기록·기준 위치 |
| 높음 | 지도 기준 yaw·자력계 정확도 | 선반 주변 독립 방향 기준·반복 시험 |
| 높음 | 웹 명령에서 행동까지의 연결 | 임무·지도·행동 전환 기록 |
| 높음 | 우회 성능 | LiDAR·장애물 배치·경로 기록 |
| 중간 | 안정 호버링의 정확한 구간 | RC 조작 설명·반복 비행 |
| 중간 | 비행 전후 상태 | 전체 세션 기록·메타정보 |

위 관측의 신뢰도는 파일 내부 값에 한정한다.
현재 장비 설정은 별도로 확인해야 한다.
시작 부근에는 헤더보다 앞선 표본도 있다.
일부 상태는 기록 시작 전부터 이어졌을 수 있다.
PX4 위치를 독립된 실제 위치 기준으로 간주하지 않는다.

## 5. 재현 근거

원본은 Windows 바탕화면에 보존했다.
이 확인 과정에서 기체나 비행 파라미터를 바꾸지 않았다.

- SHA256: `896a3f021f0b2eb9fd943e7960e2add8dfff67bcfe15541c442fac175146690f`
- 펌웨어 해시: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- 원본: `C:/Users/ASWPC_NTBOOK/Desktop/log_23_2026-9-27-13-34-50.ulg`
- 분석 노트북: [analysis/ulog_inspection.ipynb](analysis/ulog_inspection.ipynb)
- 분석 도구: [PX4/pyulog](https://github.com/PX4/pyulog)

분석 노트북은 로그를 읽는 확인용 자료다.
기체에 배포할 코드 구현은 아직 하지 않았다.
