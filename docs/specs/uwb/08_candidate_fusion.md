# M08 위치 후보 가중 결합

여러 위치 후보를 한 후보로 결합하는 명세다. 단순 평균·품질 가중·상하위 가중 방식을 비교할 때 읽는다.

## 목적과 현재 상태

M06 또는 M07이 만든 후보를 같은 조건에서 결합한다.
0.7·0.3·0.03은 대화에서 나온 시험 후보값이다.
GNSS 공통 상수나 검증된 계수로 취급하지 않는다.
현재 구현은 없으며 [공통 규격](00_interfaces.md)을 따른다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `candidates` | CandidateSet. 같은 시각·기준점 |
| 입력 | `frame`, `z_sample`, `anchors` | 공통 거리 잔차 평가 자료 |
| 입력 | `candidate_lineage` | 묶음·앵커 쌍·공유 obs_id |
| 출력 | `estimate` | 결합 Candidate |
| 출력 | `weights`, `ranks`, `scores` | 후보별 가중치와 산정 근거 |
| 출력 | `dispersion_m2` | 후보 간 산포. 정확도 공분산과 구분 |
| 출력 | `covariance_kind` | 기본 unknown. 교정 전 신뢰구간 없음 |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `fusion_level` | group/pair_slot | 후보 네 개/중간 슬롯 12개 구분 |
| `policy` | uniform/quality/ranked_groups | 별도 variant |
| `score_kind` | standardized_range_residual | 작은 값이 높은 순위 |
| `score_floor`, `weight_cap` | 양수 | 실행 전 명시 |
| `low_group_mass` | 0~1 | 0.30·0.03 등은 후보. 기본 미채택 |
| `split_policy` | count_half | 유효 후보 수 기준으로 절반 분리 |
| `min_candidate_count` | 정수 | 실험군별 명시 |

## 처리 구조와 산식

```text
후보·계보 → 시간/유효성 검사 → 공통 잔차 점수
          → 가중치 계산 → 정규화 → 결합 후보·가중치 기록
```

```text
score_k = sqrt(mean((e_ki/sigma_i)^2))
quality_weight_k = min(weight_cap, 1/max(score_k^2, score_floor))
normalized_weight_k = weight_k/sum(weight)
p_fused = sum(normalized_weight_k * p_k)
```

품질 점수에 기준 위치를 사용하지 않는다.
sigma가 없으면 quality 정책을 차단하고 uniform을 별도 실행한다.
ranked_groups는 상위 질량 `1-low_group_mass`를 배분한다.
하위 질량은 `low_group_mass`다. 각 그룹 안에서는 균등 배분한다.
예를 들어 12개 모두 유효할 때 6개씩 나뉜다.
일부 실패 시 유효 개수로 다시 나누고 실제 개수를 기록한다.
유효 후보가 K개이면 상위 ceil(K/2)개와 나머지로 나눈다.
ranked_groups는 최소 두 후보가 필요하다. 한 그룹이 비면 실패한다.
uniform·quality의 최소 개수는 실험군 설정을 따른다.
동률 순서는 후보 ID로 결정해 재생을 재현한다.

같은 원시 자료를 여러 번 반영하는 영향도 비교한다.
네 묶음의 균등 평균은 각 세 교점의 평균과 연결된다.
이를 두 가지 독립적인 센서 융합 성공으로 세지 않는다.

## 실패와 상태

모든 가중치가 0이거나 비유한 값이면 실패한다.
후보 산포를 최종 위치 오차 공분산으로 대체하지 않는다.
지속 편향은 여러 후보를 같은 방향으로 이동시킬 수 있다.
후보들이 모였다는 이유만으로 참값이라고 판정하지 않는다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 후보 순열·동률 | 결정적 결과·가중치 합 1 |
| 후보 4개/12개/일부 누락 | 실제 개수와 분할 규칙 일치 |
| 한 후보 복제 | 정보 중복과 결과 변화 기록 |
| 한 앵커 편향·공통 편향 | 단순 다수결 실패 여부 |
| uniform/quality/그룹 질량 비교 | 독립 평가 자료의 오차·가용률 |

0.03을 거리 편향이나 시간 필터 이득과 혼용하지 않는다.
확정 계수는 [선정 절차](90_evaluation.md)를 거쳐 기록한다.
