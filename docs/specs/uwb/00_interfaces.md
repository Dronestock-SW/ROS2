# UWB 시험 모듈 공통 입출력

아이디어별 모듈이 공유할 자료형과 상태 규칙이다. 어댑터나 시험 실행기를 구현할 때 읽는다.

## 기본 규칙

**측정값·시각·출처·사용 가능 여부를 함께 전달한다.**
명세 버전은 `uwb_experiment_v1`이다.
기존 `uwb_pipeline_diagnostic`과 별도 형식이다.
아래 형식을 기존 실행기가 지원한다고 가정하지 않는다.

길이는 m, 시간 간격은 s, 각도는 rad로 통일한다.
시각은 매핑된 `host_monotonic_us`의 정수 us다.
측정 시각과 수신 시각을 따로 보관한다.
배열의 앵커 순서는 `anchor_ids`로 명시한다.
빈 값은 JSON `null`이다. NaN·Infinity는 거부한다.
누락한 오차 분산을 0으로 채우지 않는다.

## 공통 자료형

필수 필드가 없으면 해당 단계에서 차단한다.

| 자료형 | 필드·형식 | 단위·규칙 |
|---|---|---|
| `Envelope` | `schema_version`, `session_id`, `event_id`: string | 세션과 입력의 고유 식별자 |
| | `source`: simulation/measured | 종류가 바뀌면 상태 초기화 |
| | `host_received_us`: int, `raw_payload`: 원문 참조 | 원문 보존 후 해석 |
| `Observation` | `obs_id`, `anchor_id`, `tag_id`: string | 개별 측정 식별 |
| | `device_time_us`: int, `measurement_time_us`: int/null | 매핑 전 원시 시각도 보존 |
| | `raw_slant_m`, `cal_slant_m`: float/null | 기울어진 거리. 보정은 RAW−편향 |
| | `accepted`: bool, `reason`: string | 거부해도 원본 삭제 금지 |
| | `sigma_range_m`: float/null | 양수. 추정 출처 필요 |
| | `rf`: object/null | 장치가 실제 제공한 진단만 저장 |
| `AnchorMap` | `map_id`, `frame_id`, `anchor_ids`: string/list | 입력 자료에 명시적으로 결합 |
| | `positions_m`: N×3 float | 각 앵커 x,y,z. 순서와 ID 일치 |
| | `survey_status`, `covariance_m2`: string/object/null | 실측·참고 배치 구분 |
| `RangeFrame` | `frame_event_id`, `t_ref_us`, `observations[]` | 동시 풀이에 사용할 관측 묶음 |
| | `source_skew_s`: float, `sync_policy`: string | 비동시 거리 허용 기준 기록 |
| `RangeWindow` | `t_ref_us`, `start_us`, `observations[]` | 각 표본의 원래 시각 유지 |
| `AttitudeSample` | `measurement_time_us`, `R_WB`: 3×3 | 몸체 B의 벡터를 창고 W로 회전 |
| | `covariance_rad2`: 3×3/null, `valid`: bool | gyro RAW는 자세 행렬이 아님 |
| `ZSample` | 아래 전용 표 참조 | ToF 기반 높이 관측 |
| `Candidate` | `candidate_id`, `xy_m`: float[2] | 위치 후보. 창고 W의 태그 안테나 기준 |
| | `t_ref_us`, `used_obs_ids[]`, `anchor_ids[]` | 어느 측정으로 계산했는지 추적 |
| | `cov_xy_m2`: 2×2/null, `covariance_kind`: string | unknown/approximate/calibrated |
| | `residuals_m[]`, `score`: float/null | 점수식·방향을 모델 설정에 명시 |
| | `parent_ids[]`, `metadata`: object | 후보 생성 단계와 중복 관계 |
| `CandidateSet` | `candidates[]`, `rejected[]`, `t_ref_us` | 실패한 묶음과 이유도 남김 |
| `GeometryReport` | `rank`, `condition`, `singular_values[]` | 정의한 행렬 기준 |
| | `hdop_xy`, `predicted_cov_xy_m2`: float/object/null | 배치 지표와 오차 예측 구분 |
| `GateDecision` | `accepted`, `reason`, `pending_count` | 위치·거리 게이트 공통 판정 |
| | `observed_value`, `accepted_value`, `last_accepted_time_us` | 보류값에 새 측정 시각 부여 금지 |

## ToF 기반 Z 연결

제공될 자료를 이 형식으로 변환한다.
사용자가 지금 이 양식으로 다시 제출할 필요는 없다.

| 필드 | 형식 | 의미·필수 조건 |
|---|---|---|
| `sample_id` | string | 높이 표본 식별자 |
| `source` | simulation/measured | 합성·실측 구분 |
| `method` | provided_z/tof_geometry | 제공된 Z 또는 원시 ToF에서 계산 |
| `measurement_time_us` | int/null | 호스트 시간축으로 매핑한 측정 시각 |
| `clock_domain` | string | `host_monotonic_us` 또는 미매핑 식별 |
| `z_m` | float/null | 선언한 기준점의 높이 |
| `reference_point` | fc_origin/uwb_antenna | 두 기준점을 혼용하지 않음 |
| `frame_id` | string | AnchorMap과 같은 기준 또는 검증된 변환 필요 |
| `sigma_z_m` | float/null | 높이 오차 표준편차. 미상은 null |
| `valid` | bool | 제공자가 표시한 관측 유효성 |
| `reason` | string | missing_z, clock_unmapped 등 |
| `upstream_obs_ids` | string[] | 생성에 사용한 센서 표본 |

현재의 입력 대기 상태는 다음과 같다.

```json
{
  "type": "z_input_status",
  "schema_version": "uwb_experiment_v1",
  "status": "awaiting_input",
  "z_m": null,
  "sigma_z_m": null,
  "measurement_time_us": null,
  "reference_point": null,
  "frame_id": null,
  "valid": false,
  "reason": "awaiting_user_tof_z"
}
```

이는 유효한 `ZSample`이 아닌 준비 상태 메시지다.
파일 수신만으로 Z를 사용 가능으로 바꾸지 않는다.
시각·단위·기준점·좌표계를 확인한 뒤 변환한다.
제공된 Z에는 ToF 거리 보정을 중복 적용하지 않는다.
원시 ToF이면 M02의 자세·광축·장착값이 필요하다.

## 결과 형식

개별 단계는 `ModuleResult[T]`를 반환한다.
T는 각 명세의 출력 묶음이다. 전처리에 위치 출력을 요구하지 않는다.

| `ModuleResult[T]` 필드 | 형식·규칙 |
|---|---|
| `module_id`, `variant_id`, `event_id` | 단계·설정·입력 식별 |
| `status`, `reason` | 아래 ModelResult와 같은 상태·사유 체계 |
| `payload` | 각 문서의 출력 묶음 T. 처리 불가이면 null |
| `input_ids`, `diagnostics` | 입력 계보·판정 근거 |

실행기는 단계 결과를 모아 모델별 `ModelResult`를 만든다.
모든 실행 틱에 성공 또는 실패 결과 하나를 남긴다.

| `ModelResult` 필드 | 형식 | 규칙 |
|---|---|---|
| `run_id`, `model_id`, `variant_id`, `config_hash` | string | 같은 모델의 설정 차이를 구분 |
| `event_id`, `t_ref_us`, `produced_time_us` | string/int | 입력 시각과 출력 시각 분리 |
| `status` | ok/pending/rejected/blocked/failed | 계산·판정 상태 |
| `reason` | string | 기계적으로 집계할 사유 |
| `estimate` | Candidate/null | 실패 시 새 위치를 만들지 않음 |
| `last_estimate` | Candidate/null | 참고용 과거값. 원래 시각 유지 |
| `fresh`, `held`, `predicted_only` | bool | 새 관측·보류·예측을 구분 |
| `eligible_for_comparison` | bool | 해당 지표의 정상 관측 집계 가능 여부 |
| `external_output_allowed`, `flight_valid` | bool | 이번 시험 규격은 항상 false |
| `input_ids`, `z_sample_ids` | string[] | 입력 계보 |
| `compute_us`, `source_age_s` | float/null | 계산 비용·관측 지연 |
| `diagnostics` | object | 잔차·조건수·가중치·반복·게이트 상태 |

`ok`는 시험 계산의 성공이다. 비행 사용 허가는 아니다.
`held=true`와 `predicted_only=true`는 새 관측으로 세지 않는다.
공분산을 산출하지 못해도 위치 후보는 기록할 수 있다.
그 후보는 공분산을 요구하는 결합·검정에서 차단한다.

## 좌표·시간·상관관계

같은 시점과 기준점의 관측만 결합한다.

W는 앵커 지도에 선언한 오른손 좌표계다. +Z는 위다.
B는 장착 정보에 선언한 몸체 좌표계다.
축 순서와 방향을 `frame_definition`에 저장한다.
GNSS의 ENU와 W의 원점·방향은 별도 변환한다.
자세 행렬은 정규직교이며 행렬식이 +1이어야 한다.

모델은 현재 수신 컷오프 이후 자료를 읽지 않는다.
나중에 도착할 자세를 미리 사용하면 정보 누설이다.
시간 보간은 필요한 두 표본이 이미 수신됐을 때만 허용한다.
그때 생긴 지연과 목표 측정 시각을 결과에 남긴다.
허용 간격을 넘는 보간·외삽은 `blocked` 처리한다.

공통 Z가 거리 여러 개에 영향을 주면 오차도 공유한다.
가능한 경우 `C_range + g_z g_z^T sigma_z^2`를 사용한다.
여기서 `g_z[i]=(z_ant-a_z[i])/predicted_range[i]`다.
Z와 거리의 상관을 무시한 근사에는 표시를 남긴다.
같은 거리에서 만든 후보들을 독립 관측으로 세지 않는다.

## 설정과 함수 경계

실행 설정은 입력 자료와 함께 고정해 보관한다.

| 항목 | 규칙 |
|---|---|
| `enabled`, `mode` | 명세 기본값 disabled/offline_only |
| `parameters` | 각 기능 문서의 필드. 미확정 필수값은 null |
| `parameter_origin` | source_reference/experiment_candidate/measured |
| `dataset_id`, `map_id`, `calibration_id` | 자료·배치·편향 버전 연결 |
| `seed`, `code_revision`, `dirty_patch_hash` | 재현 정보 |
| `reset_policy` | boot·세션·출처·시계 단절 시 상태 초기화 |

제안 공통 함수는 `configure(config)`, `reset(reason)`,
`process(input, context) -> ModuleResult[T]`다.
전체 모델의 `process_tick`은 이를 `ModelResult`로 묶는다.
`context`에는 평가용 기준 위치를 넣지 않는다.
각 모듈의 세부 입력·출력은 개별 문서가 지정한다.
미확정 필수값으로 실행하면 `parameter_unset`을 반환한다.
