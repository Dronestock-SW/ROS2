# M06 세 앵커 조합별 위치 후보

네 앵커를 세 개씩 묶어 후보를 만드는 명세다. 조합별 풀이와 공통 거리 풀이를 비교할 때 읽는다.

## 목적과 현재 상태

같은 시점의 거리에서 최대 네 위치 후보를 생성한다.
새 시험 모듈이며 현재 구현돼 있지 않다.
기존 `rawxy.py`는 초기 참고 함수다.
논문의 세부 가중치 수식을 재현했다고 표시하지 않는다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `frame`, `anchors`, `z_sample` | RangeFrame·AnchorMap·안테나 Z |
| 입력 | `range_covariance` | 4×4 m²/null. 동일 가중 기준선은 null 허용 |
| 입력 | `initial_xy_m` | float[2]/null. 결정적 초기화 정책 |
| 출력 | `candidate_set` | 최대 네 Candidate와 실패 묶음 |
| 출력 | `subset_ids`, `omitted_anchor_id` | 조합과 남겨둔 앵커 식별 |
| 출력 | `residuals_all_anchors_m` | 남겨둔 앵커를 포함한 진단 |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `triplets` | ID 목록 | ABC, ABD, ACD, BCD |
| `solver` | unweighted/weighted_slant | 각각 별도 variant |
| `iterations`, `step_tol_m` | 정수, m | 시험 전에 명시 |
| `condition_max` | 무차원 | 미확정 |
| `max_frame_skew_s` | s | M01과 동일 |
| `require_four_inputs` | bool | 기본 true |

## 처리 구조와 산식

```text
네 거리 + 같은 시점 Z
      → ABC / ABD / ACD / BCD
      → 묶음별 XY 풀이·M10 품질 검사
      → CandidateSet → M08 결합
```

각 묶음 S에 대해 다음 오차를 최소화한다.

```text
e_i(x,y) = sqrt((x-ax_i)^2+(y-ay_i)^2+(z-az_i)^2)-r_cal_i
p_S = argmin e_S^T * inverse(C_S) * e_S
```

구현은 선형계 풀이를 사용하고 역행렬을 직접 만들지 않는다.
Z가 없으면 묶음 생성까지만 하고 풀이를 차단한다.
각 묶음은 같은 원본 거리와 Z를 사용한다.
기준 앵커 순서만 바꾼 결과는 별도 측정으로 세지 않는다.
후보 간 공통 관측을 `used_obs_ids`로 추적한다.

## 실패와 상태

세 앵커의 배치가 나쁘면 해당 후보만 거부한다.
한 앵커 누락 시 기본 비교군은 blocked다.
가용한 세 앵커만 쓰는 축소 모드는 별도 variant다.
이 모드는 네 후보 결합과 같은 이름으로 집계하지 않는다.
남겨둔 앵커 잔차는 점수용 진단이다.
독립적인 기준 위치 오차로 부르지 않는다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 앵커 입력 순서 변경 | 같은 조합 ID와 결과 |
| 무잡음·알려진 XY | 네 후보가 기준 위치와 일치 |
| 한 앵커의 편향 | 영향을 받는 세 묶음과 제외 묶음 기록 |
| 일직선·외곽 배치 | 기하 품질과 오차의 관계 |
| 네 앵커 전체 풀이와 비교 | 결합 전후 오차·가용률·계산량 |

참고: [네 앵커 조합 가중 결합 논문](https://wxdg.cbpt.cnki.net/portal/journal/portal/client/paper/476eac747bbba892fc7fabbbfce0d7a5).
초록의 구조만 확인했다. 상세 재현은 추가 본문 확인 대상이다.
