# M11 잔차 기반 이상 검출·제외

GNSS의 이상 관측 검사를 UWB에 시험하는 명세다. 검출 전용과 조건부 관측 제외를 비교할 때 읽는다.

## 목적과 현재 상태

관측 간 불일치를 검사하고 원인이 명확할 때만 재계산한다.
기존 시험에서 앵커 제거가 나빴다는 기록은 유지한다.
여기서는 독립된 비교군으로만 설계한다.
항공용 RAIM 인증이나 보호 수준 보장 구현이 아니다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `frame`, `anchors`, `z_sample` | M09와 같은 관측 |
| 입력 | `full_fit`, `C_effective_m2` | 전체 거리 풀이·오차 모델 |
| 입력 | `fault_hypotheses` | 검사할 앵커 제외 가설 목록 |
| 출력 | `test_statistic`, `threshold`, `degrees_of_freedom` | 검정 근거 |
| 출력 | `fault_detected`, `suspected_ids`, `excluded_ids` | 검출·추정·실제 제외 구분 |
| 출력 | `estimate`, `hypothesis_results` | 재계산 결과와 모든 가설 기록 |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `mode` | detect_only/exclusion_trial | detect_only 기준선 |
| `threshold_method` | calibrated_quantile/chi_square | 사전 선택 |
| `false_alarm_target` | 0~1 | 미확정 |
| `max_exclusions` | 정수 | 네 앵커 시험은 최대 1 |
| `min_remaining_anchors` | 정수 | 3 |
| `min_score_separation` | 무차원 | 원인 구별 기준. 미확정 |
| `multiple_test_policy` | 명시된 보정 방식 | 여러 가설 검정 시 필수 |

## 처리 구조와 산식

```text
전체 관측 M09 → 잔차 검사
      ├─ 정상 → 그대로 기록
      └─ 이상 → detect_only는 경고
             → exclusion_trial은 제외 가설별 재계산
             → 남은 잔차·M10 기하·원인 구별 검사
             → 유일한 수용 가설 / 원인 불명 차단
```

```text
T = e^T * inverse(C_effective) * e
nominal_dof = observation_count - rank(G)
```

카이제곱 기준은 알려진 Gaussian 오차와 국소 선형화를 가정한다.
적응 가중치·강건 손실·추정 공분산이면 그 분포가 달라질 수 있다.
그 경우 독립 교정 자료로 검출 임계값을 정한다.
XY 두 미지수의 네 거리면 명목 자유도는 2다.
하나 제외 후 세 거리면 1이다. 이것만으로 원인 식별을 보장하지 않는다.
추가 상태를 추정하면 자유도를 다시 계산한다.
공유 Z의 오차는 공분산에 반영한다.
가장 큰 잔차의 앵커를 무조건 버리지 않는다.

## 실패와 상태

입력 중복·공분산 부재·자유도 부족은 검정을 차단한다.
둘 이상의 가설이 비슷하면 원인 불명으로 남긴다.
두 개 이상 편향은 최대 한 개 고장 가정의 한계 시험이다.
검정 통과를 무고장 또는 참 위치의 증명으로 표시하지 않는다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 무고장 자료 | 오검출률·불필요 제외율 |
| 앵커별 단발/지속 편향 | 검출률·미검출률·검출 지연 |
| 두 앵커 편향·기하 불량 | 원인 불명과 잘못된 제외 비율 |
| detect_only/조건부 제외 | 오차·출력률·최대 점프 |
| 통계 가정 불일치 | 목표 오검출률과 실측 비율 차이 |

근거: [ESA RAIM](https://gssc.esa.int/navipedia/index.php/RAIM_Fundamentals),
[Zabalegui 등, UWB 적용 논문](https://www.sciencedirect.com/science/article/pii/S0263224121003456).
논문의 세부 알고리즘 완전 재현은 아직 아니다.
