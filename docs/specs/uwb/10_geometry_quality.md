# M10 앵커 배치·계산 품질 판정

앵커 배치에 따른 위치 풀이의 민감도 명세다. 후보·거리 풀이·재계산의 품질을 평가할 때 읽는다.

## 목적과 현재 상태

관측 개수와 별도로 위치를 구분할 수 있는지 판단한다.
기존 풀이마다 조건수 계산 방식이 달라 공통화가 필요하다.
GNSS 배치 평가 원리를 XY 거리 모델에 적용한다.
GNSS의 시계 상태가 있는 GDOP와 같은 숫자로 비교하지 않는다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `candidate`, `anchors`, `z_sample` | 위치·선택 앵커·높이 |
| 입력 | `jacobian` | N×2, 또는 H80/Q의 해당 설계행렬 |
| 입력 | `C_effective_m2` | N×N/null |
| 출력 | `geometry` | GeometryReport |
| 출력 | `gate_decision` | rank 부족·기하 불량·판정값 미정 구분 |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `matrix_kind` | xy_range/h80_scaled/q_data/q_regularized | 필수 명시 |
| `rank_tolerance` | 상대 기준 | 실행 전에 명시 |
| `condition_max`, `hdop_xy_max` | 무차원 | 미확정 |
| `column_scaling` | none/declared | 사용한 스케일을 결과에 포함 |
| `quality_gate_mode` | report_only/reject | report_only 기준선 |

## 처리 구조와 산식

```text
사용 앵커·후보 → 기하 행렬 → SVD
  → rank·조건수 → 선택적 오차 예측 → 품질 보고/거부
```

```text
G_i = [(x-ax_i)/d_i, (y-ay_i)/d_i]
condition = largest_singular_value/smallest_singular_value
hdop_xy = sqrt(trace(inverse(G^T*G)))
P_xy = inverse(G^T*inverse(C_effective)*G)
```

hdop_xy는 동일 거리 분산을 가정한 무차원 배치 지표다.
P_xy는 공분산이 있을 때만 m² 단위로 산출한다.
두 값을 실측 위치 오차라고 표시하지 않는다.
H80의 스케일된 행렬 조건수와 XY DOP는 별도 필드다.
Q에서는 관측만의 rank와 규제식 포함 rank를 둘 다 기록한다.
규제식이 해를 만들었다고 관측 정보가 충분해진 것은 아니다.

## 실패와 상태

역행렬이 정의되지 않으면 수치 대신 null과 사유를 남긴다.
배치가 좋더라도 NLOS 거리 편향이 없다고 판정하지 않는다.
report_only는 진단만 한다. reject의 임계값은 사전 설정한다.
임계값 미정인 reject 실행은 parameter_unset이다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 같은 위치·앵커의 평행 이동 | 지표 불변 |
| 좌표축 회전 | 등가 배치의 품질 유지 |
| 일직선·밀집·외곽·넓은 배치 | rank와 민감도 비교 |
| 저 DOP·편향 거리 | 작은 DOP를 정확도 보장으로 오인하지 않음 |
| report_only/reject | 오차 감소와 출력 손실을 함께 평가 |

근거: [ESA 위치 오차와 DOP](https://gssc.esa.int/navipedia/index.php/Positioning_Error).
