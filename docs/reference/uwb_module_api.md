# UWB 모듈 입출력 사전

현재 구현의 입력·출력·실패 처리를 정리한 사전이다.
함수 호출이나 후속 API를 설계할 때 읽는다.

실제 파일은 [단계별 위치표](repository_layout.md)에서 찾는다.
2026-10-04 경로 정리에서 함수 계약·수치 설정은 유지했다.
아래 표의 짧은 모듈명은 다음 경로를 기준으로 읽는다.

| 모듈명 | 실제 패키지 |
|---|---|
| `clock`, `sensors` | `drone_uwb.processing.timing` |
| `height`, `transform`, `gazebo_geometry` | `drone_uwb.processing.geometry` |
| `observations`, `rawxy`, `range_solver`, `weighted_xy`, `h80`, `qs10`, `triplets`, `intersections`, `candidate_fusion` | `drone_uwb.processing.solvers` |
| `ranges`, `settings`, `pipeline`, `runner`, `simulator` | `drone_uwb.processing` |
| `node`, `bridge`, `frames`, `bench_probe` | `drone_uwb.integration.ros` |
| `gazebo_*`의 외부 연결 | `drone_uwb.integration.gazebo` |
| `sitl_*`, `px4_clock_tracker` | `drone_uwb.integration.sitl` |

옛 import는 같은 모듈 객체로 연결한다.
전체 대응은 [모듈 경로표](../../data/module_paths.json)에 있다.

## 공통 계약

RAW·보정 거리·좌표를 서로 다른 필드로 전달한다.

| 필드·형식 | 계약 |
|---|---|
| 앵커 배열 | A1, A2, A3, A4 순서. 좌표 `(4,3)` m |
| RAW 거리 | `raw_slant_m[4]`, m. 결측은 원본 null 보존 |
| 보정 거리 | `cal_slant_m[4] = raw - bias`, m. RAW 덮어쓰기 없음 |
| 수신 시각 | `host_received_monotonic_ns`, host 단조시계 ns |
| ROS 시각 | `host_received_ros_ns`, ROS 시계 ns. 단조시계와 구분 |
| 장치 시각 | `cycle_start_us`, `cycle_end_us`, `sample_time_us[4]`. ESP32 부팅 기준 us |
| 센서 공통 시각 | `measurement_time_us`, `clock_domain=host_monotonic_us` |
| 자세 | `quaternion_wxyz[4]`, 단위 quaternion. `rotation_convention=R_WB` |
| 상태 | `valid` 또는 `ok`, `fresh`, `reason`. 실패 좌표를 정상값으로 사용하지 않음 |
| 파일 입력 envelope | `source=measured/simulation`, host 수신 시각, `message` |
| 실측 수신 JSONL | host·ROS 수신 시각과 `message`. 기존 정지 실행기가 직접 소비 |
| 계산 좌표 | `uwb_map`의 안테나 XY(m). FC 좌표 전환은 별도 함수 |
| 비행 출력 | 파일 실행기는 `flight_valid=false`, `external_output_allowed=false` |

실측 JSONL과 파일 envelope는 동일 형식이 아니다.
기존 실측은 `static_a.read_capture`로 읽는다.
ToF 결합 실행기는 시계 대응이 끝난 envelope가 필요하다.
단순 필드 추가로 UWB·ULog의 동기화가 성립하지 않는다.

## Gazebo 경로 비교와 WLS

별도 Python 실행 경로이며 비행 출력은 없다.

| 호출 | 입력 | 출력·실패 |
|---|---|---|
| `integration.gazebo_rig.build` | PX4 모델 경로·앵커·장비·시험 설정·새 출력 폴더 | 원본 모델을 읽어 별도 기체·월드·trial·해시 생성 |
| `integration.gazebo_rig.install_assets` | 생성 폴더·PX4 모델 경로 | 새 이름만 설치. 기존 이름은 거부 |
| `processing.gazebo_geometry.tag_position` | 기체 월드 위치·단위 wxyz quaternion·몸체 FLU 장착 위치 m | `p_tag = p_body + R_WB*l_tag` |
| `processing.gazebo_geometry.VirtualRanges.sample` | 증가하는 시뮬레이션 시각·기체 pose | 가상 RAW와 별도 정답. 중복/역행·자세 오류 거부 |
| `integration.gazebo_ranges` CLI | Gazebo 위치 토픽·기체 이름·trial 설정·출력 경로 | Gazebo StringMsg 안의 `sim_uwb_cycle`와 JSONL. MAVLink 송신 없음 |
| `processing.weighted_xy.solve_weighted_xy` | 앵커 `(4,3)` m, 보정 거리 4개 m, 높이 scalar/4개 m, 거리 공분산 `(4,4)` m² | `ok/reason`, XY·잔차·RMS·가중 비용·조건부 XY 공분산. 행렬·기하·수렴 실패 구분 |
| `integration.gazebo_capture.pose_record` | Gazebo `Pose_V`, 최상위 모델 이름 | `gazebo_sim_us` 시각·XYZ·자세. 이름 미일치/중복은 None |
| `processing.experiments.gazebo_trial.run` | 위치 JSONL 경로, 설정 JSON 경로, 새 출력 경로 | A/B/C/D/WLS 결과·요약·설정·원본 복사·해시 |
| `processing.experiments.gazebo_scenarios.run_scenarios` | 같은 입력·기본 설정·새 출력 경로 | 여섯 조건과 전체 비교 JSON. 잡음·WLS sigma·고장 조건은 시나리오 값 적용 |

시각 중복·역행·출처 혼합은 거부한다.
출력 폴더 재사용도 거부한다.
실측 수신 JSONL을 Gazebo 위치 JSONL로 간주하지 않는다.
자세한 형식·설정은 [실행 절차](../runbooks/uwb_gazebo_shadow_runbook.md)를 따른다.

## 수신·공통 모듈

수신부는 원본 내용을 보정하지 않는다.

| 모듈·호출 | 받는 파라미터 | 반환·출력 | 실패·상태 |
|---|---|---|---|
| `contracts.protocol.decode_line` | JSON bytes/str | dict | `InvalidSample`: JSON 오류·객체 아님 |
| `contracts.protocol.finite/integer` | 값 | bool | bool을 숫자로 허용하지 않음 |
| `acquisition.serial_io.SerialInput` | `path`, `baud=921600` | `read()`: bytes 또는 None | 끊김 `OSError`, 단독 장치 잠금. `close()` 필요 |
| `acquisition.framing.LineFramer` | `limit=8192`, `feed(chunk: bytes)` | 완성 행 bytes 목록 | 초과 행 폐기, `overflows` 증가 |
| `acquisition.validation.InputValidator` | settings, `require_recent_status=True` | 상태 유지 검사기 | 실시간은 상태 timeout 적용 |
| `check_common` | `msg` | None | schema=1·tag 불일치 예외 |
| `on_status` | `msg`, `mono_ns` | None | 준비·앵커 순서·시계·필터 여부 검사 |
| `on_cycle` | `msg`, `mono_ns`, `ros_ns` | `Cycle` | `InvalidInput(reason, reset)` |

검사기가 읽는 설정은 아래 다섯 필드다.
`tag_id`, `status_timeout_s`, `max_cycle_s`,
`max_range_m`, `max_report_span_s`를 사용한다.
`require_recent_status`는 생성자 인자로 따로 받는다.

`Cycle`은 `seq/start_us/end_us/mask`를 가진다.
`raw`는 길이 4 ndarray이며 내부 결측은 NaN이다.
`sample_us/failure/indices/excluded`도 전달한다.
`seq_gap/host_mono_ns/ros_ns/extra`로 출처를 보존한다.
내부 NaN을 그대로 JSON에 직렬화하지 않는다.

## 정제·계산 모듈

각 모듈은 호출자가 준 값과 자체 이력만 사용한다.

| 모듈·호출 | 받는 파라미터 | 반환·출력 | 실패·상태 |
|---|---|---|---|
| `clock.ClockMap.update` | `esp_us`, `host_ns`, 생성 시 settings | None, `state/ready/alpha/residual_p95_s` 갱신 | 준비 전 좌표 시각 사용 보류 |
| `clock.ClockMap.host_s` | ESP32 us | host 단조시각 s | `ready` 확인 후 호출 |
| `sensors.SensorInputs.add` | message, source, host_ns | None, 최대 512개/종류 버퍼 | 미대응 시계·미래시각·잘못된 자세 `ValueError` |
| `sensors.SensorInputs.snapshot` | 측정시각 us | ToF·자세 sample/age_ms/available | 기본 최대 나이 0.10s. 과거 값의 유효성 별도 표시 |
| `ranges.subtract_bias` | raw[4], bias[4] m | 보정 거리 list | 원본 결측은 None. 호출자는 배열 길이를 맞춤 |
| `ranges.RangeGate.update` | source_us, value_m, 생성 시 settings | `RangeDecision` | accepted/value_m/reason/pending_count/allowed_change_m |
| `ranges.RangeHistory` | window_s, max_samples=192; append(anchor, source_us, value_m) | rows, counts()[4] | 승인 관측만 추가. prune(now_us)로 오래된 관측 제거 |
| `height.tof_to_fc_height` | 거리·편향 m, R_WB(3,3), ToF 축[3], 레버암[3] m, 바닥 z m | FC 높이 float m | 회전·광축·거리 오류 `ValueError`. 제어 명령 아님 |
| `height.antenna_to_fc_position` | 안테나 XYZ, R_WB, UWB 레버암 m | FC XYZ ndarray | 벡터·회전 오류 `ValueError` |
| `height.alpha_beta_step` | 이전 z, vz, 관측 z, dt_s | 새 z(m), vz(m/s) | dt>0. alpha=.40, beta=.20, 속도 ±.8 |
| `height.median3` | 유한값 3개 | 중앙값 float | 개수·값 오류 `ValueError` |
| `observations.solve_xy` | anchors(4,3), ranges[4], indices | XY[2], 최대 쌍 잔차 m | 동일 앵커 높이 전제. 퇴화 배치 `InvalidSample` |
| `rawxy.solve_raw_xy` | anchors, ranges, indices, z_ant m, settings | `RawXY` | valid/x/y/mask/rms_m/max_m/condition/reason |
| `range_solver.solve_slant_xy` | anchors(N,3), ranges[N], 높이 스칼라 또는 [N], required_count=4, 반복 설정 | `RangeFit`: 위치·잔차·조건수·반복 수 | required_count는 3 또는 4. A는 반드시 4 |
| `triplets.make_triplets` | 앵커 ID 네 개 | 정렬된 네 조합 | A1~A4 이외·중복 거부 |
| `triplets.solve_triplet_xy` | 세 앵커·거리·높이·settings dict | fit dict | rank·수렴·수치 실패 보존 |
| `triplets.make_candidates` | 지도(4,3), 보정 거리[4], 높이, t_ref_us, anchor_ids, obs_ids, settings dict | C의 네 후보·균등 결합·남은 앵커 잔차 | 하나라도 실패하면 결합 차단 |
| `intersections.circle_intersections` | 중심 XY 두 개, 반지름 두 개 m, DSettings | 교점 0/1/2개·분기·수치 정리 여부 | 분리·포함·중심 중복 구분 |
| `intersections.select_intersection` | 교점, 세 번째 앵커·거리·높이, tie_margin_m | 선택 위치·세 번째 거리 잔차 | 동률이면 ambiguous_intersection |
| `intersections.make_candidates` | C와 같은 지도·보정 거리·높이·시각·ID, DSettings | D의 12슬롯·네 묶음·결합·투영 진단 | strict만 구현. 실패 슬롯도 보존 |
| `candidate_fusion.uniform_fuse` | candidates, required_ids, t_ref_us | xy_m·ID별 weights·dispersion_m2·관측 ID | 중복·필수 ID 누락·무효·과거 후보 거부 |
| `h80.fit_h80` | anchors, idx[N], s[N] 초, r[N] m, t_ref_s, settings | `H80Fit` | ok/reason, x0/y0/x1/y1, 잔차·조건수·표본수 |
| `qs10.solve_q_s10` | H80과 같은 관측, z_ant m, init[4], settings | `QFit` | ok/reason, 위치 계수·bias[4]·잔차·cost |
| `transform.validate_rotation` | matrix(3,3), tolerance=1e-9 | 회전 ndarray | 직교·det 검사 실패 `ValueError` |
| `transform.warehouse_to_px4` | point_w[3], rotation_pw, origin_w[3] | PX4 기준 XYZ | 좌표 변환만 수행 |
| `transform.body_to_px4` | rotation_pw, rotation_wb | 기체→PX4 회전행렬 | 두 회전 검증 |
| `settings.settings_from_mapping` | 설정 dict | `PreimuSettings` | 미등록 키·잘못된 값 `ValueError` |
| `pipeline.Pipeline.process` | 입력 envelope, 생성 시 PipelineConfig | 진단 dict | input/range/clock/sensor 상태와 reason. 계산·전달 닫힘 |
| `observations.Processor.process` | msg, mono_ns, ros_ns; 생성 시 layout/settings | `Decision` | reason/details와 선택적 Observation |

`Observation`의 필드는 다음과 같다.
`seq`, `stamp_ns`, `x`, `y`, `variance`, `anchor_mask`,
`pair_residual_m`, `report_span_s`, `source_mode`다.
x·y·잔차는 m, variance는 m²다.
`stamp_ns`는 ROS 시계로 대응한 측정시각이다.

H80/Q_S10의 x1·y1은 창으로 정규화한 위치 계수다.
곧바로 m/s 속도로 해석하지 않는다.
호출 전 관측 배열의 길이·단위·유한값을 맞춘다.
모든 독립 수식이 모든 잘못된 입력을 잡지는 않는다.
새 외부 API는 먼저 입력 검사 계층을 통과시킨다.

## 실행·통합 모듈

실행기는 입출력을 소유하고 수식은 계산부에 둔다.

| 모듈·호출 | 입력 | 출력·오류 |
|---|---|---|
| `processing.runner.run_lines` | bytes 행 iterable, output, PipelineConfig, metadata | input.jsonl·diagnostics.jsonl·summary.json. 기존 output 거부 |
| `processing.simulator.simulated_events` | layout, settings, duration_s=10, seed=7 | 합성 envelope iterator. 실측 표시 금지 |
| `experiments.uniform_xy.solve_uniform_xy` | anchors, ranges, z_m, max_iterations=40, step_tol_m=1e-7, condition_max=1e6 | `UniformFit`: ok/reason/xy_m/residuals_m/rms_m/condition/iterations |
| `experiments.baseline_a.BaselineA.process` | envelope, input_line | status/reason/estimate/diagnostics. 파일 전용 |
| `experiments.baseline_a.run` | input_path, output, config, layout, reference=None | summary + 결과 JSONL·CSV·manifest. 입력 해시 불일치 거부 |
| `experiments.static_a.read_capture` | path, tag_id, firmware | 원본 검사된 행 목록 |
| `experiments.static_a.estimate_bias` | rows, anchors, reference_xyz_m, minimum_samples=100 | bias_m과 산출 근거 |
| `experiments.static_a.calculate` | rows, anchors, height_m, bias_m, solver_settings | 차감 전후 fits. 평가 XY는 받지 않음 |
| `experiments.static_a.run` | config_path, root, output | summary와 결과 파일. 교정·평가 같은 입력 거부 |
| `experiments.h80_b.read_events` | path, tag_id, firmware | 수신 순서의 cycle·실패·boot 이벤트. 파일 세션 메타데이터 사전 확인 |
| `experiments.h80_b.H80Window` | anchors(4,3), bias_m[4], BSettings | 독립 0.8초 거리 창. 높이가 다른 앵커 배치는 거부 |
| `H80Window.process` | Cycle, input_line | ok/reason, xy_m, velocity_m_s, fit, 표본수·나이·출처 |
| `H80Window.reset` | reason 문자열 | 과거 표본·시간 상태 폐기 |
| `experiments.h80_b.run` | config_path, root, output | A/B 결과·가용률·paired 오차·별도 시간 기록·manifest |
| `experiments.subset_comparison.calculate` | 수신 events, anchors, height_m, bias_m, config | 같은 주기의 A/B/C 또는 A/B/C/D 결과·시간·건수 |
| `experiments.subset_comparison.evaluate` | results, reference_xy, excursion_interval, models | 모델별 가용률·쌍별/전체 공통 시각 오차 |
| `experiments.subset_comparison.run` | config_path, root, 새 output | 결과·후보·CSV·평가·manifest. 입력 해시·교정 분리·이전 A/B 검사 |
| `experiments.flight_inputs.run` | ulog_path, 새 output 경로 | 원래 PX4 시계·자세 형식의 센서 JSONL·준비 상태. pyulog 필요 |
| `integration.recording.open_record_files` | 새 directory, metadata dict | raw/received/decisions/status 스트림 dict. 호출자가 close |
| `integration.node.UwbNode` | ROS 설정, UART, 앵커 JSON | `/uwb/raw`, `/uwb/status`, `/uwb_pose`. 기록은 별도 하위 폴더 |
| `integration.frames.gate` | BridgeSettings, connected, state_age_s, params, param_age_s | `ready` 또는 차단 사유 str |
| `integration.frames.rotate_xy_covariance` | x, y, covariance(2,2), settings | 변환 XY와 공분산. 잘못된 공분산 거부 |
| `integration.bridge.UwbPx4Bridge` | `/uwb_pose`, FC 상태·파라미터 | 게이트 통과 시 MAVROS 수평 위치 관측 |
| `integration.replay` | 수신 파일·설정·평가 기준 CLI | 기존 실시간 Processor의 파일 재생 결과 |
| `integration.bench_probe` | ROS 센서·상태·파라미터 CLI | 지상 연결 점검 기록 |

ROS 파라미터와 MAVROS 상세는
[기존 API 문서](uwb_mavros_parameter_api.md)를 따른다.
웹 API를 이번에 추가하지 않았다.

B의 CLI는 `uwb_h80_b`다.
`--config`, `--root`, `--output`을 받는다.
실제 적용값과 완료 범위는 [B 기록](../report/uwb_h80_b_20260927.md)에 있다.
공통 H80 함수는 기본 세 앵커도 허용한다.
BSettings는 네 앵커를 요구한다.
`fit.coefficients`의 순서는 `[x0,y0,x1,y1,q0,q1,q2]`다.
`rank`, `condition`, `converged`를 함께 반환한다.
같은 높이의 앵커·0.8초 이내 과거 표본을 요구한다.
`current_frame_rms_m`은 별도 기하 진단이다.
그 값으로 B의 결과를 거부하지 않는다.

27일 ULog 준비 모듈은 표준 ZSample을 만들지 않는다.
원본 `px4_boot_us`, `FRD_body_to_NED_earth`를 유지한다.
거리 표본의 측정시각·분산이 미상이면 null을 남긴다.
변환 전 자료를 기존 `SensorInputs`에 바로 넣지 않는다.
그 클래스가 요구하는 host 시계·창고 좌표와 다르기 때문이다.
[센서 준비 기록](../report/uwb_flight_inputs_20260927.md)을 따른다.

C/D의 CLI는 `uwb_subset_compare`다.
`--config`, `--root`, `--output`을 받는다.
설정의 `models`는 `["C"]` 또는 `["C","D"]`다.
A/B도 같은 실행에서 항상 함께 계산한다.
첫 프로파일은 기존 정지 자료를 읽는 실행기다.
27일 ULog와 자동으로 결합하는 실행기는 아니다.
[C/D 시험 기록](../report/uwb_subsets_cd_20260927.md)에 적용값이 있다.

C/D의 `heights`는 m 단위다.
스칼라 또는 관측별 길이 4 배열을 받는다.
관측 시각과 높이 정렬은 호출자가 수행한다.
`t_ref_us` 기본값 0은 함수 단위 시험용이다.
파일 실행기는 주기 종료 시각과 원본 행별 관측 ID를 준다.
실측 연결에서도 생략하지 않아야 계보를 보존할 수 있다.
후보의 `dispersion_m2`는 평균 주위 산포다.
이를 위치 정확도 공분산으로 취급하지 않는다.

| C/D 기본 설정 | 값 |
|---|---|
| C 반복·수렴·조건수 | 40회 / 1e-7m / 1e6 |
| 공통 거리 범위 | 0 초과, 80m 이하 |
| D 교점 정책 | strict |
| D 동률 폭 | 1e-6m |
| D 최소 수평 거리 | 1e-4m |
| D 수치 허용오차 | 1e-10m² |
| D 최소 중심 간격 | 1e-6m |
| 결합 | C 네 후보, D 묶음별 세 슬롯·최종 네 후보 모두 요구 |

## 설정 기본값

다음 표는 2026-09-27 구현 기본값이다.
JSON/YAML 적용값은 실행 manifest에 따로 기록한다.
후보 기본값을 검증된 운용값으로 취급하지 않는다.

### 파일 보정 설정: PreimuSettings

| 파라미터 | 기본값 |
|---|---|
| `tag_id` | `'5'` |
| `range_bias_m` | `[0.0, 0.0, 0.0, 0.0]` |
| `range_bias_calibrated` | `False` |
| `fixed_z_m` | `1.2` |
| `uwb_lever_arm_body_m` | `[0.0, 0.0, 0.0]` |
| `lever_arm_confirmed` | `False` |
| `max_range_m` | `80.0` |
| `max_cycle_s` | `0.1` |
| `max_report_span_s` | `0.04` |
| `max_queue_s` | `0.15` |
| `status_timeout_s` | `10.0` |
| `min_anchors` | `3` |
| `clock_window_s` | `10.0` |
| `clock_min_samples` | `30` |
| `clock_alpha_min_span_s` | `5.0` |
| `clock_scale_tolerance` | `0.001` |
| `clock_residual_p95_max_s` | `0.05` |
| `raw_xy_max_condition` | `30.0` |
| `raw_xy_max_rms_m` | `0.15` |
| `raw_xy_region_margin_m` | `1.0` |
| `window_s` | `0.8` |
| `sigma_r_m` | `0.08` |
| `huber_m` | `0.12` |
| `min_samples_per_anchor` | `3` |
| `min_samples_total` | `12` |
| `max_scaled_condition` | `1000000.0` |
| `fit_period_s` | `0.05` |
| `max_extrapolation_s` | `0.04` |
| `h80_irls_iterations` | `8` |
| `recovery_gap_s` | `0.15` |
| `q_min_span_s` | `0.4` |
| `q_max_source_age_s` | `0.1` |
| `q_iterations` | `40` |
| `q_step_halvings` | `14` |
| `q_bias_max_m` | `1.0` |
| `q_bias_prior_m` | `0.1` |
| `q_velocity_weight` | `1.0` |
| `q_init_last_valid_max_age_s` | `2.0` |
| `warmup_s` | `0.1` |
| `bias_detect_m` | `0.08` |
| `bias_hold_s` | `0.8` |
| `range_gate_margin_m` | `0.2` |
| `range_gate_speed_m_s` | `1.2` |
| `range_reacquire_cluster_m` | `0.15` |
| `range_reacquire_confirm` | `3` |
| `position_gate_margin_m` | `0.08` |
| `position_gate_speed_m_s` | `0.8` |
| `position_reacquire_cluster_m` | `0.12` |
| `position_reacquire_confirm` | `3` |
| `residual_rms_max_m` | `0.1` |
| `residual_max_max_m` | `0.3` |
| `failed_after_s` | `0.5` |

### 기존 실시간 설정: Settings

| 파라미터 | 기본값 |
|---|---|
| `tag_id` | `'5'` |
| `source_mode` | `'raw_ranges'` |
| `min_anchors` | `4` |
| `max_range_m` | `1000.0` |
| `status_timeout_s` | `10.0` |
| `max_cycle_s` | `0.1` |
| `max_report_span_s` | `0.04` |
| `max_queue_s` | `0.15` |
| `max_pair_residual_m` | `0.15` |
| `range_step_margin_m` | `0.2` |
| `max_speed_m_s` | `1.2` |
| `history_timeout_s` | `0.5` |
| `recovery_samples` | `3` |
| `clock_warmup_samples` | `30` |
| `clock_window_s` | `5.0` |
| `xy_stddev_m` | `0.3` |
| `geometry_tolerance_m` | `0.03` |
| `report_delay_s` | `0.0` |

### PX4 연결 설정: BridgeSettings

| 파라미터 | 기본값 |
|---|---|
| `enabled` | `False` |
| `alignment_confirmed` | `False` |
| `timing_confirmed` | `False` |
| `sensor_mount_confirmed` | `False` |
| `enu_yaw_deg` | `0.0` |
| `enu_offset_x_m` | `0.0` |
| `enu_offset_y_m` | `0.0` |
| `expected_ev_delay_ms` | `0.0` |
| `source_frame` | `'uwb_map'` |
| `pose_topic` | `'/uwb_pose'`. 실물 시험은 `'/uwb/btf_pose'` |
| `verify_ev_sensor_position` | `False`. 실물 시험은 `True` |
| `expected_ev_pos_x_m/y_m/z_m` | 각 `0.0`. PX4 FRD 실측값 필요 |
| `max_age_s` | `0.2` |
| `state_timeout_s` | `2.5` |

추가 ROS 설정: `port=/dev/uwb`, `baudrate=921600`,
`anchor_file=""`(패키지 기본 앵커 JSON),
`watchdog_timeout_s=0.5`, `record_directory=""`(기록 끔),
`stop_after_s=0.0`(자동 종료 없음)이다.

`PipelineConfig`는 `preimu`, `range_bias_source="unconfigured"`,
`calculation_enabled=false`, `external_output_enabled=false`를 받는다.
두 게이트를 true로 설정하면 생성 시 거부한다.

## 실물 웹 시험의 ROS 출력

2026-10-08 추가한 실행 경로다.
기존 파일 파이프라인의 게이트와 구분한다.

| 항목 | 계약 |
|---|---|
| `/uwb/btf_pose` | PoseWithCovarianceStamped. 안테나 XY, z 자리값 0 |
| `/uwb/btf_xyz` | PoseStamped. 같은 시각의 XY·바닥 기준 안테나 높이 |
| `require_height_for_pose` | 기본 false. 비행 launch는 true; 높이 누락 때 XY 중지 |
| `/uwb/bridge_status` | gate·settings·FC mirror·published·최신 관측 stamp |
| 브리지 상태 발행 | 10Hz. FC mirror 조회는 1Hz |
| 장착값 검증 | EV_POS 기대값과 FC mirror 차이 ≤0.01m |

XYZ는 고도 제어 입력이 아니다.
같은 창의 네 높이 입력이 없으면 `xyz_m=null`이다.
실제 FC 전달은 브리지 enabled와 정렬·시각·장착 확인에 따른다.
설정·출력 규격은 [연결 구조](../architecture/web_test_flight.md)를 따른다.
