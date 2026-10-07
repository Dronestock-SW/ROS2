# 27일 ULog의 UWB 시험 입력 준비

새 ULog를 B/C 개발에 반영한 기록이다. 활용 가능한 센서 자료와 계산 대기 조건을 확인할 때 읽는다.

## 반영 결과

**27일 자료를 센서 입력 준비에 반영했다.**
사용자는 같은 비행의 UWB RAW 없이 ULog만 있다고 확인했다.
따라서 이 파일만으로 A/B/C 위치 비교를 수행하지 않았다.
기존 9월 6일 UWB와 27일 센서를 동시 관측으로 합치지 않았다.
두 파일은 서로 다른 측정 세션이기 때문이다.

| 항목 | 결과 |
|---|---|
| 원본 | `log_23_2026-9-27-13-34-50.ulg` |
| 길이 | 70.908608초 |
| 거리 기록 | 71개. 센서 선언 범위 안의 기록 58개 |
| 거리 기록 간격 | 중앙값 1.01510초 |
| 자세 기록 | 1,417개. 표본 간격 중앙값 0.049571초 |
| Position 모드 | 기록에서 확인. 호버링 오차 평가는 미실시 |
| 외부 위치 융합 | `EKF2_EV_CTRL=0` |
| 앵커별 UWB RAW | 파일 내 검색 일치 없음. 사용자도 별도 파일 없음 확인 |
| 안테나 높이 계산 | 미실시. 장착·좌표 변환 확인 대기 |

거리 기록 간격을 실제 센서 출력 속도로 해석하지 않는다.
이 ULog에 저장된 표본 간격이다.
현재 A의 짧은 센서 나이 조건에 바로 넣을 수는 없다.
오래된 값을 최신 관측으로 바꾸지 않는다.
주기 변경이나 보간은 별도 시험 조건으로 검증한다.

## 추가 구현

원래 시계·좌표계를 보존하는 파일 준비 모듈을 추가했다.

```text
27일 ULog
  ├─ distance_sensor → 원본 거리·범위·품질·장치 정보
  └─ vehicle_attitude → 원본 quaternion·표본 시각
                             ↓
               native_sensor_events.jsonl
                             ↓
             readiness.json + C 입력 준비 프로파일
                             ↓
             안테나 높이·C 위치 계산은 입력 대기
```

| 산출물 | 역할 |
|---|---|
| `processing/experiments/flight_inputs.py` | 원본 센서 자료 추출·준비 상태 기록 |
| [native_sensor_events.jsonl](../../data/processed/flight_inputs_20260927/native_sensor_events.jsonl) | 거리 71개·자세 1,417개. 총 1,488개 |
| [readiness.json](../../data/processed/flight_inputs_20260927/readiness.json) | 개수·간격·차단 사유·확인한 파라미터 |
| [C 준비 프로파일](../../src/drone_uwb/config/c_flight_inputs_20260927.json) | 원본·추출물 해시와 누락 입력 |
| [검증 기록](../../data/processed/flight_inputs_20260927/verification.json) | 테스트·빌드·동일 입력 재생 결과 |

C 준비 프로파일은 C 실행 설정으로 완성된 파일이 아니다.
`executable_model_config=false`로 구분했다.
UWB RAW·시계 대응·장착 정보는 null로 남겼다.
원본 SHA-256은 다음과 같다.
`896a3f021f0b2eb9fd943e7960e2add8dfff67bcfe15541c442fac175146690f`

## 실제 필드와 어댑터 조건

원본 값을 기존 파이프라인 형식으로 잘못 해석하지 않는다.
형식이 같아 보여도 시계·좌표 의미가 다르기 때문이다.

| 원본 필드 | 확인·처리 |
|---|---|
| `distance_sensor.timestamp` | PX4 부팅 기준 us. 별도 측정시각 필드는 없음 |
| `signal_quality=-1` | 품질 미상. 0점 또는 완전 신뢰로 바꾸지 않음 |
| `signal_quality=0` | 무효 신호 |
| `variance=0` | 분산 미상·무효. 출력 `variance_m2=null` |
| `orientation=25` | 하방 설정. 실장 검증 완료라는 뜻은 아님 |
| 자세 `q[w,x,y,z]` | 기체 FRD에서 지구 NED로의 회전 |
| `timestamp_sample` | 자세의 원시 자료 시각. 발행 시각과 함께 보존 |
| `EKF2_RNG_POS_X/Y/Z=0` | 로그 파라미터값. 실제 장착 위치 측정값으로 사용하지 않음 |

거리 필드의 의미는 해당 펌웨어의
[DistanceSensor 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/DistanceSensor.msg)를 확인했다.
자세 방향과 quaternion 순서는
[VehicleAttitude 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleAttitude.msg)를 확인했다.
Position 코드 2는
[VehicleStatus 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleStatus.msg)에 따른다.

창고 기준 `R_WB`로 쓰기 전에 변환이 필요하다.
PX4·ESP32·host 시계도 별도 대응이 필요하다.
거리계 광축·ToF 및 UWB 장착 오프셋도 확인한다.
센서 모델은 아직 물리적으로 확인하지 않았다.
로그는 `SENS_EN_TF02PRO=1`, `SENS_TFMINI_CFG=0`이다.
이 설정만으로 실장 모델을 단정하지 않는다.

품질 미상인 거리 58개를 최종 높이 유효 표본으로 부르지 않는다.
58개는 원본 범위·신호 검사만 통과한 개수다.
모든 안테나 높이는 null로 보존했다.

## 재실행과 검증

파일 준비 모듈은 장치나 ROS에 연결하지 않는다.

```bash
cd /home/pgyxn/github/ROS2
PYTHONPATH=src/drone_uwb:/home/pgyxn/.cache/uwb-audit/pyulog \
python -m drone_uwb.processing.experiments.flight_inputs \
  --ulog data/raw/flight/20260927/log_23_2026-9-27-13-34-50.ulg \
  --output /tmp/uwb_flight_inputs_new_run
```

pyulog는 기존 감사 환경의 설치본을 사용했다.
새 재생 폴더에 기록해 기존 추출물을 보존한다.

| 검사 | 결과 |
|---|---|
| UWB 전체 pytest | 89개 통과. 센서 준비 시험 3개 포함 |
| 두 번 센서 추출 | events·readiness 바이트 일치 |
| B 폴더 정리 후 재확인 | 기존 위치·판정·평가 결과 바이트 일치 |
| 위치·기체 전달 | 미실시. `flight_valid=false` 유지 |

거리 품질 미상·분산 미상·자세 시각 오류를 시험했다.
모드 표본 수와 시간 구간을 구분하는 검사도 통과했다.
관찰된 Position 구간은 약 7.56초와 21.72초다.
상태 메시지 시각의 구간이며 위치 정확도 지표는 아니다.

## B/C 작업의 현재 범위

B는 기존 1분 UWB의 비교를 완료한 상태다.
[B 결과](uwb_h80_b_20260927.md)는 이번 ULog의 비행 성능이 아니다.
[C 명세](../specs/uwb/06_anchor_triplets.md)에 27일 센서 프로파일을 추가했다.
C/D의 첫 위치 비교는 이후 기존 UWB 자료로 완료했다.
[C/D 결과](uwb_subsets_cd_20260927.md)는 27일 비행 성능이 아니다.
27일 비행의 A/B/C 비교에는 해당 비행의 RAW가 필요하다.
새 비행 기록에는 UWB와 센서의 시각 대응도 함께 남긴다.


## 기능별 커밋 전 통합 확인

2026-09-27 최종 작업본으로 UWB 테스트 142개를 통과했다.
전체 5개 패키지 빌드와 새 CLI 두 개도 확인했다.
이전 시험 수치와 당시 검증 기록은 보존했다.
이번 실기·Gazebo 실행 검증은 미실시다.
[통합 확인 기록](uwb_commit_review_20260927.md)을 따른다.
