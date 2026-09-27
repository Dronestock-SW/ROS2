# M09 GNSS 원리를 참고한 거리 가중 위치 풀이

여러 UWB 거리를 함께 푸는 비교 기준 모델 명세다. 동일 가중과 품질 가중의 효과를 비교할 때 읽는다.

## 목적과 현재 상태

관측별 불확실성을 위치 계산에 반영한다.
GNSS의 가중 추정 원리를 UWB 거리식에 적용하는 시험안이다.
기존 `rawxy.py`는 동일 가중 기준선의 참고 함수다.
2026-09-27 `processing/weighted_xy.py`를 추가했다.
네 앵커와 명시된 거리 공분산으로 위치를 푼다.
공유 Z·지도 오차의 자동 전파는 미구현이다.
실측 품질 추정과 통합 Candidate 연결은 남아 있다.
[가상 비교 준비 결과](../../report/uwb_gazebo_shadow_20260927.md)를 참고한다.
2026-09-26에는 A 전용 `experiments/uniform_xy.py`를 추가했다.
네 앵커·시각별 Z로 동일 가중 위치를 계산한다.
수렴 확인을 포함하며 공분산은 산출하지 않는다.
[A 실행·검증 기록](../../report/uwb_baseline_a_20260926.md)을 참고한다.
후속 [실측 정지 시험](../../report/uwb_baseline_a_static_20260926.md)도 수행했다.
수기 기준점과 별도 수신 로그로 편향 차감 전후를 비교했다.

## 현재 함수 입출력

독립 풀이 함수와 향후 통합 계약을 구분한다.

| 항목 | `solve_weighted_xy`의 현재 계약 |
|---|---|
| `anchors` | 네 앵커 좌표 `(4,3)`, m |
| `ranges` | 이미 편향을 차감한 사거리 `(4,)`, m |
| `heights` | 해당 시각 높이 scalar 또는 `(4,)`, m |
| `range_covariance_m2` | 대칭 양정치 `(4,4)`, m². 필수 |
| 설정 | 반복 40회, step 1e-7m, condition 상한 1e6 |
| 출력 | `ok/reason`, `xy_m`, 원래 잔차·RMS, 가중 비용 |
| 진단 | 반복 횟수, 가중 Jacobian condition, 조건부 XY 공분산 |

Cholesky로 잔차와 Jacobian을 백색화한다.
SVD 기반 최소제곱으로 갱신량을 구한다.
비용이 감소하는 step을 선택한다.
동일 분산일 때 A와 같은 위치를 확인했다.
가중치가 잘못되면 결과가 나빠질 수 있다.
이 함수는 거리 품질을 자동으로 알아내지 않는다.

## 계획된 통합 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `frame`, `anchors`, `z_sample` | 같은 시점의 보정 사거리·Z |
| 입력 | `C_range_m2` | N×N 공분산/null |
| 입력 | `initial_xy_m` | float[2]/null, 결정적 초기화 |
| 출력 | `estimate` | Candidate와 조건부 위치 공분산 |
| 출력 | `fit` | 원래 거리 잔차·가중 잔차·반복·수렴 |
| 출력 | `geometry` | 사용 행렬의 rank·condition |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `weight_mode` | uniform/calibration/rf_cir | 별도 variant |
| `min_anchors` | 정수 | 기준 비교군 4. 축소 시험군 3 |
| `iterations`, `step_tol_m`, `cost_tol` | 정수·m·무차원 | 실행 전에 명시 |
| `sigma_floor_m`, `sigma_cap_m` | m, 양수 | 교정 자료로 설정 |
| `condition_max` | 무차원 | 미확정 |
| `robust_loss` | none/huber | none 기준선. Huber는 별도 비교 |

## 처리 구조와 산식

```text
보정 거리·Z → 오차 공분산 → 초기 XY
  → 가중 비선형 최소제곱 반복 → 품질 판정 → 후보
```

```text
d_i(p) = sqrt((x-ax_i)^2+(y-ay_i)^2+(z-az_i)^2)
e_i = d_i(p)-r_cal_i
p_hat = argmin e^T * inverse(C_effective) * e
G_i = [(x-ax_i)/d_i, (y-ay_i)/d_i]
P_xy ≈ inverse(G^T * inverse(C_effective) * G)
```

거리 모델이 독립이면 대각 성분은 `sigma_i^2`다.
공유 Z와 앵커 좌표 오차는 오차 전파로 추가한다.
M12의 품질 가중치를 이미 공분산에 넣었다면 중복 곱하지 않는다.
선형계는 QR/SVD 또는 양정치 분해로 푼다.
위치 공분산은 국소 선형·오차 모델 조건부 근사다.
지속 편향까지 자동 포함한 정확도 보장으로 쓰지 않는다.

UWB 입력은 왕복 측정 등으로 얻은 거리 계약이다.
GNSS 수신기 시계 미지수를 임의로 추가하지 않는다.
의사거리 입력은 M16의 별도 모델 정의가 필요하다.

## 실패와 상태

양정치가 아닌 오차 행렬, 0 거리, rank 부족은 실패다.
공분산이 없는 weighted 모드는 차단한다.
uniform 기준선은 위치만 산출하고 공분산을 null로 둘 수 있다.
반복 한도 도달과 수렴 성공을 구분한다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 같은 sigma | uniform과 같은 위치 |
| sigma 비율 변화 | 예상 오차가 큰 관측 영향 감소 |
| 공통 Z 오차 | 대각 근사와 상관 포함 결과 구분 |
| 한 앵커 지속 편향 | 잔차·실제 오차·과신 비율 확인 |
| M04/M06/M07와 비교 | 같은 시간축의 오차·coverage·지연 |

근거: [ESA 가중 최소제곱법](https://gssc.esa.int/navipedia/index.php/Weighted_Least_Square_Solution_%28WLS%29).
위 UWB 목적함수와 어댑터는 프로젝트 시험 설계다.


## 커밋 전 확인

2026-09-27 UWB 통합 테스트 142개를 통과했다.
동일 가중 일치·상관 공분산·잘못된 입력 검사를 포함한다.
실측 거리 품질 추정과 비행 검증은 남아 있다.
[통합 확인 기록](../../report/uwb_commit_review_20260927.md)을 따른다.
