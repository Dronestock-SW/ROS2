# M14 위치 후보 게이트

위치 튐과 새 위치 재획득 판정 명세다. 위치 모델 뒤의 출력 품질을 비교할 때 읽는다.

## 목적과 현재 상태

잘못된 점프를 줄이면서 실제 움직임을 통과시킨다.
ZIP에 참고 처리가 있으나 파일 파이프라인에는 연결되지 않았다.
좌표 보류와 새 관측 수용을 분리한다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `candidate`, `geometry`, `source_age_s` | Candidate·M10 보고·age |
| 입력 | `residual_report` | geometric_rms_m/native_model_rms_m 구분 |
| 입력 | `previous_state` | 마지막 수용 위치·시각·보류 군집 |
| 출력 | `gate_decision` | GateDecision |
| 출력 | `estimate`, `last_estimate` | 새 수용 후보와 과거 후보 분리 |
| 출력 | `jump_m`, `allowed_jump_m`, `pending_count` | 판정 근거 |

## 설정

| 파라미터 | 단위·범위 | 참고값 / 상태 |
|---|---|---|
| `mode` | off/reference/experiment | 별도 비교군 |
| `margin_m`, `speed_m_s` | m, m/s | 0.08 / 0.80 참고값 |
| `cluster_radius_m`, `confirm_count` | m, 정수 | 0.12 / 3 참고값 |
| `rms_limit_m` | m | 0.10 참고값 |
| `residual_kind` | geometric/native_model | 사전 선택 |
| `dt_min_s`, `dt_max_s` | s | 0.01 / 0.25 참고값 |
| `reacquire_policy` | cluster_and_elapsed_motion | 이동 가능성도 확인 |

## 처리 구조와 산식

```text
후보 유효성·age·잔차 → 이전 위치와 변화 비교
                ├─ 허용 범위 → 수용
                └─ 범위 밖 → 군집 확인 → 이동 가능성 → 재획득/보류
```

```text
dt = clamp(t_candidate-t_last_accepted, dt_min, dt_max)
allowed_jump = margin + speed*dt
jump = norm(p_candidate-p_last_accepted)
```

새 위치 군집이 확인돼도 경과시간상 도달 가능한지 검사한다.
동일 과거 후보를 반복 입력해 확인 횟수를 늘리지 않는다.
M04/M05/M09의 자체 RMS 정의는 서로 다를 수 있다.
공통 기하 잔차와 모델 내부 잔차를 모두 기록한다.
Q의 임시 편향을 포함한 잔차는 native_model이다.
비교군 간 residual_kind를 몰래 바꾸지 않는다.

## 실패와 상태

게이트 off라도 NaN·미래 시각·실패 후보는 통과시키지 않는다.
보류 시 last_estimate만 남기며 시각을 갱신하지 않는다.
오래 정지하는 출력이 정확하다고 평가하지 않는다.
지도 경계는 선언된 다각형/범위를 사용한다.
특정 앵커의 x값을 전체 지도 경계라고 가정하지 않는다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 한 번 튐·정상 복귀 | 오수용·복귀 지연 |
| 실제 이동 시작·급정지 | 정상 움직임 거부율 |
| 새로운 위치의 연속 후보 | 재획득 조건·대기 시간 |
| 같은 과거 후보 반복 | 확인 횟수 증가 없음 |
| off/reference 비교 | 오차·coverage·최대 점프·hold 비율 |

좌표를 내보내는 비행용 정책의 완료를 뜻하지 않는다.
