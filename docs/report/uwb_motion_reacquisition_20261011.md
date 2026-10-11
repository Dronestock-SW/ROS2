# UWB 이동 중 재획득 검증 — 2026-10-11
속도·공백 기반 관측 복구의 구현 결과다.
코드 적용 범위와 수동 비행의 남은 검증을 확인할 때 읽는다.

## 결과

이동 중 복구를 구현했다. 실제 긴 공백은 아직 남는다.
FC 파라미터·고도 제어·비행 명령은 변경하지 않았다.
main과 feature_uwb는 변경하지 않았다.

이전 코드 `1155967`의 근거리 복구를 보존했다.
새 코드는 신뢰할 이동 이력이 있을 때만 범위를 옮긴다.
거부 중에는 기준 위치·속도를 freeze한다.
현재 측정만 발행한다. 오래된 XY는 반복하지 않는다.
[한도와 이륙 정책](../architecture/btf_reacquisition.md)을 따른다.

## 변경 범위

| 코드 | 변경 |
|---|---|
| `processing/motion_reacquisition.py` | 승인 UWB의 속도·공백·복귀 일관성 심사 |
| `processing/observation_guard.py` | 선택형 raw 이동 복구, 연속 점프 잠금 |
| `processing/measured_btf.py` | 실측 높이·거리 일관성 연결, 부분 누락 이력 보존 |
| `processing/experiments/tof_subset.py` | 만료된 B3의 이동 복구 |
| `tools/replay_measured_btf.py` | 지상 초기화 재생도 같은 raw 검사 사용 |
| `tools/analyze_btf_gaps.py` | 이동 재획득·거부 이유 기록 |
| `tools/probe_recorded_motion_gaps.py` | 기록 좌표에 인위적 공백을 넣는 분리 시험 |

기존 H80 fitting·거리 보정값·100ms 높이 만료는 그대로다.
PX4 브리지 기본 점프 검사도 그대로다.
따라서 이번 변경은 UWB 관측 단계의 복구다.
상시 EV 발행이나 융합 완료로 해석하지 않는다.

## 원본 로그 재생

추가 승인 없이 기존 출력과 점프 거부를 보존했다.
기준은 `1155967`의 C1/C2/C3 재생이다.
원본 식별·해시는 [근거리 복구 기록](uwb_reacquisition_20261011.md)을 따른다.

| 기록 | XY 이전→이후 | 삭제 | 공통 XY 차이 | raw 점프 거부 | 최대 공백 |
|---|---:|---:|---:|---:|---:|
| C1 | 880→880 | 0 | 0m | 121→121 | 12.350초* |
| C2 | 372→372 | 0 | 0m | 87→87 | 16.450초* |
| C3 | 1,667→1,667 | 0 | 0m | 283→283 | 8.701초 |

별표는 비행 구간 끝에서 잘린 공백이다.
세 파일의 원시 입력·guard 기준·H80 결과도 동일하다.
이 결과는 거부된 좌표가 모두 잘못됐다는 증거가 아니다.

C3 raw 거부 283개를 다음처럼 분류했다.
- 188개: 현재 실측 높이·거리 일관성 근거가 부족하다.
- 44개: 연속 수신 중 점프가 발생해 freeze했다.
- 51개: 공백 전 승인 속도 이력이 부족하다.

B3 단계도 급변 후 freeze 또는 속도 이력 부족이 남는다.
이 구간에는 PX4 추정 이동도 포함돼 있다.
PX4 위치는 독립 정답이 아니므로 정상 이동을 확정하지 않는다.
실물 경로를 독립 측정하지 않아 오거부율을 계산할 수 없다.

## 기록 기반 인위적 공백

기록된 XY·시각을 보존하고 0.2/0.4/0.6초를 숨겼다.
복구 모델에는 공백 전 승인 이력만 입력했다.
이후 기록은 복귀 검사에만 썼다.
원래 수신 공백을 해결한 시험과 구분한다.
raw·TDMA·H80 전체가 아닌 이동 심사기만 시험했다.

| 기록 | 겹치는 시험 구간 | 재획득 | 기존 10cm 밖 재획득 |
|---|---:|---:|---:|
| C1 | 257 | 35 | 23 |
| C2 | 67 | 12 | 0 |
| C3 | 469 | 82 | 30 |
| 합계 | 793 | 129 | 53 |

53개는 기존 고정 반경의 이동 차단을 줄인 사례다.
독립적인 53회 비행이나 정확도 향상 수치가 아니다.
나머지 664개는 속도 변화·경로 차이·점프로 거부됐다.
그중 정상 가감속·회전의 오거부 가능성도 남는다.
임계값을 낮춰 통과 수를 늘리는 조정은 하지 않았다.

## 검증

| 구분 | 결과 |
|---|---|
| Windows 관련 시험 | 82개 통과 |
| Jetson 격리 시험 | 161개 통과 |
| Jetson 격리 colcon 빌드 | drone_uwb 성공, 설치 import 성공 |
| 합성 이동 | 여러 방향·0.25~1.70m/s·0.3~0.9초 공백 재획득 |
| 실제 거리식 합성 | B3 높이/입력 누락, B4 통신/앵커 누락 복구 |
| freeze | 정지 후 큰 튐·지속 편향·역방향·시각 역행 거부 |
| 시각·후보 무결성 | 후보 교대·간격 초과 시 확인 재시작 |
| 새 SITL·실물·비행 | 미실시 |

격리 ROS_DOMAIN_ID=172, localhost 제한을 사용했다.
운영 도메인·FC에 시험 출력을 보내지 않았다.
최초 격리 시험은 의존 패키지 경로 누락으로 5개를 건너뛰었다.
기존 설치 의존성을 읽고 격리 소스 우선으로 전부 재검사했다.

## 재현 및 증거

원본·재생·상세 결과는 Git 밖에 보존한다.
PC 증거 폴더는 `Drone5-evidence/20261011/uwb-motion-reacquisition`이다.
`final-comparison.json`에 파일 해시와 전후 비교를 남겼다.
`recorded-gap-probe.json`에 각 가상 공백의 근거를 남겼다.

```bash
PYTHONPATH=src/drone_uwb OPENBLAS_NUM_THREADS=1 \
  python3 src/drone_uwb/tools/replay_manual_btf.py /private/C3 \
  --output /private/final-c3.jsonl
python3 src/drone_uwb/tools/analyze_btf_gaps.py /private/C3 \
  --replay /private/final-c3.jsonl \
  --baseline-replay /private/near-prior-final-c3.jsonl \
  --output /private/comparison.json
PYTHONPATH=src/drone_uwb python3 src/drone_uwb/tools/probe_recorded_motion_gaps.py \
  /private/near-prior-final-c3.jsonl --output /private/probe.json
```

## 다음 수동 비행 기록

사용자는 오늘 수동 비행으로 원인 검증을 이어간다.
자동 START·EV 발행을 켜는 변경은 포함하지 않는다.
수집기는 기존 부팅 자동 실행을 유지한다.

1. 비행 전 관측·수집 서비스와 파일 증가를 확인한다.
2. 이륙 시각과 그림자·바닥·조명 조건을 기록한다.
3. 조종 가능한 범위에서 정지 호버와 일정 방향 이동을 구분한다.
4. 이동 축·대략 거리·heading·높이 변화를 착륙 후 기록한다.
5. 동시 UWB·ToF·flow·PX4 및 FC ULog를 보존한다.
6. 공백 전 속도, freeze 이유, 복귀 관측을 시간순으로 대조한다.

정확한 오거부 판정에는 독립 이동 기준이 필요하다.
바닥 표시 간 실제 거리나 고정 카메라 기록이 도움이 된다.
PX4 추정과 UWB가 일치해도 두 값만으로 정답을 만들지 않는다.
