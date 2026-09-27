# M02 ToF·자세·Z 어댑터

추후 제공될 Z 또는 원시 ToF를 연결하는 명세다. 높이 관측을 거리 기하 계산에 연결할 때 읽는다.

## 목적과 현재 상태

태그 안테나 기준의 시각별 Z를 제공한다.
사용자는 9월 26일 GitHub 비행 로그를 자료로 지정했다.
거리·자세 기록은 확인했다. 동시 UWB와 시계 대응은 필요하다.
현재 `height.py`의 수식은 파이프라인에서 호출하지 않는다.
별도 A 실행기는 합성 ToF·자세로 이 수식을 호출한다.
시각 선택·누락 처리를 포함한다. Z 평활은 미적용이다.
[A 실행·검증 기록](../../report/uwb_baseline_a_20260926.md)에 범위를 적었다.
고도 제어와 자이로 RAW 교정은 이 모듈의 역할이 아니다.
[ZSample 규격](00_interfaces.md)을 따른다.

2026-09-27 후속 요청으로 새 ULog를 반영했다.
사용자는 27일 자료가 ULog뿐이라고 확인했다.
거리 71개·자세 1,417개를 원래 시계로 추출했다.
[입력 준비 기록](../../report/uwb_flight_inputs_20260927.md)을 따른다.
거리 기록 간격 중앙값은 1.01510초다.
이를 실제 센서 출력 속도나 연속 높이로 해석하지 않는다.
동시 UWB·시계 대응·장착 확인까지 높이 계산은 대기한다.
이 자료가 기존 9월 6일 UWB와 동시 측정된 것은 아니다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 A | `provided_z` | ZSample. 시각·단위·기준점 필수 |
| 입력 B | `tof_distance_m`, `tof_bias_m` | float, m. 원시 ToF 경로 |
| 입력 B | `attitude`, `beam_axis_B` | AttitudeSample, float[3] 단위벡터 |
| 입력 B | `lever_tof_B_m`, `ground_z_m` | float[3], float. 기준 바닥 평면 |
| 공통 | `lever_uwb_B_m`, `target_time_us` | FC→안테나 오프셋, 정렬 목표 시각 |
| 출력 | `z_antenna` | ZSample, reference_point=uwb_antenna |
| 출력 | `raw_z_m`, `filtered_z_m`, `gate_decision` | 관측·처리값·판정 분리 |
| 출력 | `used_attitude_ids`, `age_s`, `sigma_z_m` | 계보·지연·불확실성 |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `input_kind` | provided_z/raw_tof | 자료 확인 후 선택 |
| `max_z_age_s`, `max_attitude_age_s` | s, 양수 | 미확정 |
| `min_downward_projection` | 0~1 | 미확정. 광축 판정용 |
| `z_filter` | none/median3/alpha_beta | none을 기준선으로 비교 |
| `alpha`, `beta`, `max_vz_m_s` | 무차원·m/s | 시험 후보. 원본 참고값과 구분 |
| `jump_margin_m`, `jump_speed_m_s` | m, m/s | 미확정 |
| `pending_radius_m`, `confirm_count` | m, 양의 정수 | 미확정 |

## 처리 구조와 산식

```text
제공된 Z ── 시각·기준점 확인 ─────────┐
원시 ToF + 자세 + 장착값 → 기하 계산 ─┤
                         FC/안테나 기준점 통일
                                  ↓
                    품질 판정 → 선택적 평활 → ZSample
```

W는 위쪽 Z, R은 `R_WB`, l은 FC에서 센서로의 벡터다.

```text
d = tof_distance_m - tof_bias_m
z_fc = ground_z - (R*l_tof).z - d*(R*beam_axis_B).z
z_ant = z_fc + (R*l_uwb).z
z_pred = z_prev + vz_prev*dt
innovation = z_obs - z_pred
z_filtered = z_pred + alpha*innovation
vz_filtered = clip(vz_prev + beta*innovation/dt, ±max_vz)
z_gate_limit = jump_margin_m + jump_speed_m_s*dt
within_gate = abs(z_obs-z_pred) <= z_gate_limit
```

바닥 평면·광축·자세가 맞을 때의 관측식이다.
제공 Z가 안테나 기준이면 기준점 변환을 다시 하지 않는다.
불확실성은 거리·자세·장착값의 오차 전파로 산출한다.
모르면 null을 유지하고 오차 모델 필요 시험을 차단한다.
위 변화 게이트도 시험안이다. 평활식의 혁신과 판정을 구분한다.
게이트 밖 관측은 pending_radius 안에서 confirm_count회 모일 때 재획득한다.
첫 유효 관측은 이전 예측과 비교하지 않고 다른 품질 검사 후 초기화한다.

## 실패와 상태

Z 미제공은 `blocked:missing_z`다.
광축이 바닥을 향하지 않거나 반사면이 불명확하면 거부한다.
바닥 단차·사람·장애물 반사는 별도 라벨로 보존한다.
보류된 관측은 원래 시각을 유지한다.
장착값 또는 기준점이 바뀌면 평활 상태를 초기화한다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 높이·자세·장착값이 알려진 합성 기하 | 부호·기준점 변환 일치 |
| provided_z 경로 | 거리 보정·장착 변환 중복 없음 |
| 누락·노후·미래 시각·비단위 광축 | 사유와 함께 차단 |
| 경사·바닥 변화·단절 | 오수용과 정상 변화 거부율 기록 |
| none/median3/alpha_beta 비교 | 오차뿐 아니라 지연·복귀 시간 비교 |

센서 원본 자료는 확보했다. 안테나 Z 실측 시험은 정렬·장착 확인 후 수행한다.
원본 함수의 0.40·0.20·0.8은 참고값이며 채택값이 아니다.
