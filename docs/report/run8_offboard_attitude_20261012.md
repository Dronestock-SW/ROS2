# 실행 8 OFFBOARD 자세 분석
FC의 자세 요구와 실제 자세를 대조한 기록이다.
이륙 밀림 원인을 검토하거나 재시험 전에 읽는다.

PX4는 위치·자세 제어기를 사용하고 있었다.
이륙 초반 자세 추종 차이는 관측됐다.
그 차이만으로 밀림 원인을 확정할 수 없다.
수평 추정 변화가 최종 UWB 공백보다 먼저였다.

## 제어 경로의 사실

companion은 XYZ 위치와 yaw 목표를 보낸다.
PX4가 위치·속도·가속도·자세·각속도를 제어한다.

```text
웹 START -> C++ 판단 -> 단일 실행기
 -> MAVROS setpoint_raw/local (XYZ + yaw)
 -> PX4 위치 오차 -> 속도 요구 -> 추력 벡터
 -> 자세 요구 -> 자세/각속도 제어 -> 모터
```

PositionTarget type_mask는 2552다.
속도·가속도·yaw rate 입력은 무시한다.
XYZ와 yaw는 활성이다.
ENU에서 NED로 MAVROS가 한 번 변환한다.
목표 송신 주기는 20Hz다.
출발 XY·높이 기준은 이전 정책을 유지한다.
companion 자세 제어기를 추가하지 않았다.

FC는 이륙 전 지상 보정 추력을 제한한다.
spoolup을 거친 뒤 rampup을 실행한다.
최신 FC 소스의 그 동작을 로그와 대조했다.
초기 spoolup 구간의 내부 XY setpoint NaN은 그 정책과 일치한다.
companion의 고정 출발 목표가 바뀌었다는 뜻은 아니다.

Altitude 모드는 수평 위치를 유지하지 않는다.
중립 스틱은 기체를 수평으로 하고 고도를 유지한다.
관성·바람에 따른 수평 이동을 조종자가 보정한다.
따라서 Altitude 밀림만으로 자세 제어 고장을 증명하지 못한다.

## 같은 FC 시간축의 결과

외부 ARM 명령을 0초로 놓았다.
실행 8 원본과 이전 명령 정렬 근거를 보존했다.

| ARM 후 | 기록 |
|---|---|
| 0.003초 | OFFBOARD·위치·자세 제어 flag=true |
| 1.010초 | takeoff_state=READY_FOR_TAKEOFF |
| 1.052초 | RAMPUP |
| 2.188초 | land detector가 이륙 판정 |
| 2.368초 | FC 추정 XY 오차 3cm 초과 |
| 3.018초 | 자세 요구와 실제 자세 차이 최대 2.786도 |
| 3.025초 | takeoff_state=FLIGHT |
| 3.188초 | 광류 융합 시작 |
| 7.767초 | 마지막 UWB 원본 시각, 기존 정렬로 추정 |
| 8.051초 | UWB 나이 0.284초로 LAND 요청 |

3.018초의 자세 값은 아래와 같다.

| 축 | PX4 요구 | FC 실제 추정 |
|---|---|---|
| roll | +1.385도 | +0.488도 |
| pitch | +1.277도 | -1.352도 |

3.188초부터 LAND 전 자세 차이 중앙값은 0.499도다.
95백분위는 1.855도, 최대는 2.270도다.
수평 추력 요구 104개 중 102개는 추정 목표 쪽이었다.
수평 보정 자체가 꺼졌다는 가설은 지지되지 않는다.

로그된 자세 요구를 과거 방향으로 맞췄다.
요구 시각 나이는 최대 100ms로 제한했다.
ULog 기록은 약 20Hz여서 제어 지연 인증은 아니다.
기체 위치의 독립 실측값도 아니다.

## 아직 가설인 부분

실제 이동과 추정 오차가 동시에 있을 수 있다.
영상은 이동을 보여주지만 거리를 정밀 측정하지 못한다.
UWB aid 값도 독립 실물 이동 기준은 아니다.
동시 B_TF 원시 거리·decision 기록은 확보되지 않았다.

| 가설 | 현재 근거와 추가 자료 |
|---|---|
| 이륙 초반 외력·지면효과 | 일시적 자세 추종 차이. 기체/모터 대조·동기 영상 필요 |
| 수평 추정 지연·광류 축/배율 | 광류 융합이 3.188초부터. 무동력 이동·회전 대조 필요 |
| UWB 동적 편향·지연 | EV 변화가 FC 추정 변화보다 컸음. 원시 거리와 decision 필요 |
| 기체/센서 기준점 불일치 | 장착 기준과 보정값 대조 필요. 센서 존재만으로 정확도 보장 안 됨 |

상승 속도를 1m/s로 올리지 않았다.
추력·게인·만료 임계값도 바꾸지 않았다.
지면효과 원인이라는 증거가 아직 없기 때문이다.
목표 1.7m·상승 제한 0.5m/s·고정 출발 XY를 유지한다.

## 재현과 원본

run8_attitude_audit.py는 FC에 연결하지 않는다.
ULog를 읽고 CSV·그림·JSON을 생성한다.

```bash
python3 src/sangwon_AI/ops/run8_attitude_audit.py \
  --ulog log-21.ulg --output NEW_ANALYSIS_DIR \
  --arm-boot-s 1525.885285 --land-relative-s 8.050767
```

분석 Python에는 pyulog·numpy·matplotlib가 필요하다.
현장 실행기의 필수 의존성은 아니다.
원본 SHA-256:
`ad5806870a362373a1374d180c04ce949e8ae6f770e136a2a2d87d2e199b9256`.

현재 FC readback도 같은 firmware hash를 확인했다.
spoolup 1초·ramp 1.8초·상승 제한 0.5m/s도 같다.
현재 DISARM 상태에서 OFFBOARD 전환은 시도하지 않았다.

근거: [PX4 OFFBOARD](https://docs.px4.io/v1.17/en/flight_modes/offboard),
[Altitude](https://docs.px4.io/v1.17/en/flight_modes_mc/altitude.html),
[해당 FC 위치 제어 소스](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp).
