# M15 LiDAR 기반 거리 보조

LiDAR 정보를 UWB 거리와 대조하는 아이디어 명세다. 앵커 대응점과 독립 기준을 확보했을 때 읽는다.

## 목적과 현재 상태

LiDAR가 실제 제공하는 정보부터 구분해 비교한다.
현 장비는 YDLIDAR T-mini Pro다.
스캔 한 번이 곧 앵커까지의 거리라는 가정은 하지 않는다.
앵커 식별·지도·장착 변환·자료는 아직 검증되지 않았다.

2026-09-28에 예측·관측 보정의 시험 순서를 정했다.
[속도별 비교 절차](lidar_speed_tuning.md)를 따른다.
스캔매칭 관측의 보정 강도·시간 응답을 비교한다.
아래 앵커 거리 대조와 구분되는 후속 시험이다.
속도별 운용값과 PX4 연결은 아직 확정하지 않았다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 A | `anchor_detection` | anchor_id·센서 내 좌표/거리·대응점 근거 |
| 입력 B | `independent_lidar_pose` | 독립 지도상의 센서 위치·공분산 |
| 공통 | `scan_time_us`, `sensor_extrinsics`, `anchor_map` | 시각·장착변환·좌표 지도 |
| 공통 | `uwb_observation`, `source_lineage` | 대조할 거리·원시 자료 계보 |
| 출력 | `lidar_derived_range_m`, `sigma_m` | 같은 기준점의 거리와 불확실성 |
| 출력 | `range_difference_m`, `association_status` | 차이·대응점 판정 |
| 출력 | `bias_candidate_m`, `reason` | 교정 후보. 기존 교정값 자동 변경 금지 |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `mode` | direct_target/map_pose | 경로 선택. 미확정 |
| `operation` | diagnostic/calibration_trial | diagnostic 기준선 |
| `max_time_skew_s` | s | 미확정 |
| `association_threshold` | 선언한 점수 단위 | 미확정 |
| `min_calibration_samples`, `calibration_window_s` | 정수·s | 미확정 |
| `max_bias_update_m` | m | 교정 시험군에서 필수 |

## 처리 구조와 산식

```text
LiDAR 자료 → 앵커 대응/독립 지도 위치 확인
 → UWB 안테나 기준점·시각으로 변환 → 거리 차이 기록
 → 별도 교정 자료에서만 편향 후보 생성
```

```text
r_reference_i = norm(p_uwb_antenna_W - anchor_i_W)
delta_calibrated_i = r_cal_i - r_reference_i
absolute_bias_candidate_i = robust_center(r_raw_i-r_reference_i)
```

지도 위치가 UWB로 보정됐다면 독립 기준이 아니다.
그 경우 진단 기록만 하고 독립 교정 시험을 차단한다.
직접 표적 경로는 LiDAR 반사점과 앵커 기준점의 차이를 보정한다.
2D 스캔과 3D 사거리의 관측 가능 차이를 명시한다.
완전한 상대 위치를 알 수 없으면 사거리로 변환하지 않는다.
신뢰도가 높다는 가정 대신 기준 위치로 오차를 확인한다.

## 실패와 상태

대응점 부재·다중 표적·프레임 불일치·시각 지연은 차단한다.
실행 평가 구간에서 정답을 이용해 편향을 계속 조정하지 않는다.
M03의 교정 파일은 별도 버전으로만 교체한다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 알려진 센서·안테나 오프셋 | 기준점 거리 변환 일치 |
| 잘못된 앵커 식별 | 교정 오염 방지 |
| 스캔 가림·유리·이동 물체 | 대응점 실패와 오차 기록 |
| UWB를 사용한 LiDAR 위치 | 계보로 독립 기준 사용 차단 |
| UWB 단독/대조/교정 적용 | 독립 평가 구간의 개선과 비용 |

필요 자료가 없으면 blocked로 남긴다.
GNSS 기준국 보정의 성능을 그대로 적용하지 않는다.
