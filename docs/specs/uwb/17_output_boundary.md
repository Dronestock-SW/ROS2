# M17 좌표 변환·출력 경계

시험 결과를 같은 기준점으로 기록하는 명세다. 모델 결과를 평가하거나 외부 전달 경계를 확인할 때 읽는다.

## 목적과 현재 상태

태그 안테나 위치와 FC 기준 위치를 구분한다.
기존 `height.py`, `transform.py`에 독립 수식이 있다.
현재 파이프라인은 위치 계산·외부 출력을 호출하지 않는다.
이번 시험 결과도 파일 기록만 허용한다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `model_result`, `z_sample`, `attitude` | XY 결과·안테나 Z·자세 |
| 입력 | `lever_uwb_B_m` | float[3], FC→안테나 벡터 |
| 입력 | `R_PW`, `origin_W_m` | 목표 프레임 회전·원점 |
| 입력 | `transform_covariance` | 장착·원점·방향 오차 정보/null |
| 출력 | `antenna_candidate`, `fc_candidate` | 원래 결과와 변환 결과 분리 |
| 출력 | `covariance_kind`, `source_age_s`, `gate_state` | 품질·지연·경계 상태 |
| 출력 | `external_output_allowed`, `flight_valid` | 항상 false |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `output_frame` | warehouse/declared_target | warehouse 기준선 |
| `output_reference_point` | uwb_antenna/fc_origin | 실행 전에 명시 |
| `rotation_tolerance` | 무차원 | 행렬 검증용. 명시 필요 |
| `external_output_enabled` | bool | false만 허용 |
| `max_output_age_s` | s | 진단 기준. 미확정 |

## 처리 구조와 산식

```text
모델 결과 → 상태·시각 검사 → 센서 기준점 변환
          → 좌표계 변환 → 후보·공분산·사유 기록
          → 외부 전달 게이트 닫힘
```

```text
p_FC_W = p_ant_W - R_WB*lever_uwb_B
p_P = R_PW*(p_FC_W-origin_W)
P_P = R_PW*P_W*R_PW^T + transformed_calibration_uncertainty
```

위치 공분산이 XY만 있으면 완전한 3D 공분산이라고 쓰지 않는다.
Z와 자세·장착 오차를 더해 필요한 3D 전파를 정의한다.
그 정보가 없으면 변환 위치와 unknown 공분산을 기록한다.
R은 정규직교이고 det=+1이어야 한다.
프레임 변환은 한 번만 수행한다.

## 실패와 상태

미정 장착값·원점·방향은 해당 변환을 차단한다.
안테나 위치 평가에는 FC 변환을 필수로 요구하지 않는다.
보류 후보에 새 시각을 부여하지 않는다.
시험 결과를 ROS/PX4/LoRa로 보내는 어댑터는 만들지 않는다.
이유: 이번 범위는 모델 비교용 파일 기록이다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 항등·90도 회전·이동 | 알려진 기준점과 일치 |
| 장착 오프셋과 자세 변화 | 안테나/FC 차이 재현 |
| 축 반사·잘못된 행렬 | 거부 |
| null·held·stale 결과 | 새 유효 위치로 오인하지 않음 |
| 모든 모델 출력 | 외부 허용 false, 원본 계보 보존 |

실제 전달 규격의 완성은 별도 통합 검증 범위다.
