# M13 시간 필터 비교

위치 관측의 시간 평활과 직접 거리 필터의 명세다. 필터 종류별 잡음 감소와 지연을 비교할 때 읽는다.

## 목적과 현재 상태

관측 전처리의 오차·지연·복구 특성을 평가한다.
필터 상태는 시험 내부에서만 사용한다.
PX4 출력의 재융합이나 비행용 EKF 추가가 아니다.
새 시험 모듈은 미구현이다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 A | `position_observation` | M08/M09/M04의 Candidate와 공분산 |
| 입력 B | `frame`, `anchors`, `z_sample`, `C_range_m2` | 직접 거리 UKF용. A와 택일 |
| 입력 | `previous_state`, `dt_s` | 상태·공분산·측정 간격 |
| 출력 | `estimate` | 평활 위치 Candidate |
| 출력 | `state`, `state_covariance` | [x,y,vx,vy]와 4×4 행렬 |
| 출력 | `innovation`, `innovation_covariance` | 관측과 예측의 차이 |
| 출력 | `predicted_only`, `last_observation_time_us` | 새 관측 없는 예측 구분 |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `filter_variant` | none/kf_position/ukf_position/ukf_range | 별도 비교군 |
| `process_spectral_density` | m²/s³, 양수 | 미확정 |
| `initial_covariance` | 상태별 m²·m²/s·m²/s² | 필수 명시 |
| `max_prediction_s`, `reset_gap_s` | s | 미확정 |
| `ukf_alpha`, `ukf_beta`, `ukf_kappa` | 무차원 | UKF군에서 필수 |
| `innovation_gate` | off/configured | M11과 중복 적용 여부 기록 |

## 처리 구조와 산식

```text
새 시각 → 운동 예측 → 위치 관측 또는 거리 관측 갱신
       → 상태·공분산 검사 → 관측 기반 결과/예측 전용 결과
```

```text
state = [x,y,vx,vy]
F = [[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]]
Q = q * [[dt^3/3,0,dt^2/2,0], [0,dt^3/3,0,dt^2/2],
         [dt^2/2,0,dt,0], [0,dt^2/2,0,dt]]
P_pred = F*P*F^T + Q
position measurement h(state) = [x,y]
range measurement h_i(state) = sqrt((x-ax_i)^2+(y-ay_i)^2+(z-az_i)^2)
```

위치 관측과 선형 운동이면 KF가 기본 비교군이다.
같은 선형 조건의 UKF가 더 좋다고 미리 가정하지 않는다.
ukf_range는 비선형 거리식에 직접 관측을 넣는 별도 모델이다.
첫 위치는 같은 입력의 M09 풀이로 초기화하고 초기 공분산을 명시한다.
초기화에 쓴 거리를 같은 시각의 필터 갱신에 다시 넣지 않는다.
동일 시각의 같은 거리를 위치와 거리로 중복 갱신하지 않는다.
M04의 겹치는 시간창은 연속 위치 오차를 상관시킨다.
그 영향을 무시한 공분산은 approximate로 표시한다.
KF 공분산 갱신은 Joseph 형식 등 수치 안정 방식을 사용한다.
UKF는 sigma-point 가중치·평균·공분산 재구성을 검사한다.

## 실패와 상태

초기 관측 전에는 예측 위치를 만들지 않는다.
공백에서는 예측 전용으로 표시하고 fresh=false를 유지한다.
공분산의 비양정치·발산·역행 시간은 초기화 사유다.
측정 시각을 계산 완료 시각으로 바꾸지 않는다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 일정 위치·일정 속도 | 상태 단위와 수렴 |
| 선형 동일 입력의 KF/UKF | 수치 허용오차 안의 일치 |
| 가속·회전·정지 | 잡음 감소와 시간 지연의 교환관계 |
| 단절·재수신 | 예측 오차·복귀 시간·최대 점프 |
| 겹친 H80 관측 | 평활 중복과 공분산 과신 |

참고: [ESA 칼만 필터](https://gssc.esa.int/navipedia/index.php/Kalman_Filter).
논문의 UKF 결합 아이디어와 구현 재현은 구분한다.
