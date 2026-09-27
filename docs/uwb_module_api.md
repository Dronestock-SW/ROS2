# UWB 모듈 입출력 사전

현재 구현의 입력·출력·실패 처리를 정리한 사전이다.
함수 호출이나 후속 API를 설계할 때 읽는다.

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
| `max_age_s` | `0.2` |
| `state_timeout_s` | `2.5` |

추가 ROS 설정: `port=/dev/uwb`, `baudrate=921600`,
`anchor_file=""`(패키지 기본 앵커 JSON),
`watchdog_timeout_s=0.5`, `record_directory=""`(기록 끔),
`stop_after_s=0.0`(자동 종료 없음)이다.

`PipelineConfig`는 `preimu`, `range_bias_source="unconfigured"`,
`calculation_enabled=false`, `external_output_enabled=false`를 받는다.
두 게이트를 true로 설정하면 생성 시 거부한다.
