# UWB 선행 필터 — 인수인계 v1.1 ↔ 현재 코드 매핑

> 이 문서는 인수인계서 v1.1(명세 v1.7)의 필드가 `src/drone_uwb` 어디에 있는지 찾아보는 사전이다.
> 구현 단계 A~H를 시작하기 전과, 필드 하나를 옮길 때마다 읽는다.
> 기준: uwb-f 0257c32 + 작업트리(2026-09-20). 인수인계서의 "Jetson"은 이 저장소에서 companion이다.

현재 XY 배치는 [6.3×4.6m 직사각형](uwb_anchor_layout.md)이다.
아래 `현재`와 `조치` 열은 최초 비교 시점 기준이다.
지금의 파일·함수 계약은 [API 사전](uwb_module_api.md)을 따른다.

## 2026-09-21 상태 보충

같은 날 후속 작업으로 파일 파이프라인을 연결했다.
ToF 실측 자료는 없으며 시뮬레이션 입력부터 축적한다.
Z·좌표 계산 게이트는 닫혀 있다.
최신 연결은 [파이프라인 기준](../architecture/uwb_pipeline_design.md)을 따른다.
아래 고정 Z·미연결 표시는 이 작업 전 비교 기록이다.

아래 매핑 표는 구현 전 비교 기록이다.
현재 `feature_uwb`에는 `preimu/` 모듈이 있다.
설정·입력·시각·raw XY·H80·Q_S10 소스를 확인했다.
게이트 기본값은 `1ef3f03`에서 갱신됐다.
수신 노드에서 이 모듈을 쓰는 연결은 찾지 못했다.
전체 처리·출력·로그 검증 완료를 뜻하지 않는다.

현재 값과 ZIP 차이는 [참고 자료](uwb_h80_qs10_reference.md)를 본다.
계정·빌드 상태는 [기본 설정 확인](../report/setup_status_20260921.md)을 본다.
아래의 `없음` 표시는 최초 비교 시점의 상태다.

## 1. 최초 비교 결론

입력 검증과 시간 매핑은 절반쯤 있다. H80·Q_S10·출력 스키마는 새로 만든다.

| 영역 | 상태 | 이유 |
|---|---|---|
| A 입력 parser·원본 저장 | 있음, 필드 3종 미수용 | `rf_valid_mask`, `twr_seq`, RF 진단을 읽지 않는다 |
| B clock mapping | 부분 | 오프셋(β)만 있다. 스케일(α)과 FC·ToF 시각이 없다 |
| C 설정 loader | 앵커 좌표만 | `range_bias_m`, 고정 z, 레버암, `R_PW`가 없다 |
| D ToF Z | 생략 | 임의 고정 z로 대체한다(2026-09-20 결정) |
| E raw XY | 2D 선형만 | 3D 최소제곱, RMS/max 잔차, 영역 검사가 없다 |
| F H80, G Q_S10 | 없음 | 신규 |
| H 출력 스키마·shadow | 없음 | `/uwb_pose`가 x,y만 낸다 |

## 2. 현재 코드 구조

수신 노드 1개와 소비 노드 1개로 이어진다. 인수인계서 §0이 요구한 5항목을 아래 표에 채웠다.

```
ESP32 태그 ─ USB UART 921600 ─▶ uwb_node ─ /uwb_pose(x,y) ─▶ uwb_px4_bridge ─▶ /mavros/vision_pose/pose_cov ─▶ PX4 EKF2
   (JSONL)      /dev/uwb          │  ├─ /uwb/raw(JSON)        (기본 disabled)
                                  │  └─ /uwb/status(1Hz)
                                  └─ record_directory ─▶ serial.raw / received.jsonl / decisions.jsonl (기본 꺼짐)
```

| 항목 | 현재 구현 | 파일 | 인수인계서와 차이 |
|---|---|---|---|
| 입력 포트 | `/dev/uwb`, 921600, `O_RDONLY`+flock, DTR/RTS 없음 | `serial_io.py`, `node.py` | 동일 |
| 프레임 parser | `LineFramer`(8192B 한계) + `decode_line`(NaN 거부) | `core.py` | 동일 |
| 시간원 | 호스트 monotonic_ns + ESP32 `cycle_end_us`, 5초 창 최소지연 오프셋 | `core.py` `Processor` | α·β 회귀 아님 |
| ToF 입력 | 없음 | — | 고정 z로 대체 |
| 자세 입력 | 없음. `/mavros/imu/data`는 `bench_probe`가 읽기 전용으로만 구독 | `bench_probe.py` | SLERP 불필요(고정 z, 자세 없음) |
| 출력 consumer | `uwb_px4_bridge`(x,y+공분산), 데모 `mission_node` | `bridge.py`, `frames.py` | z·valid·reason 없음 |
| 로그 | `record_directory` 지정 시에만 저장. 기본값은 빈 문자열(꺼짐) | `node.py`, `uwb.launch.py` | "항상 보존"과 다름 |
| 테스트 | 함수 14개 | `test/test_observations.py` | 새 단계마다 추가 |

## 3. 입력 필드 (§4 ↔ 코드)

필수 필드는 대부분 읽는다. 진단 필드 3종은 읽지 않고, 명세에 없는 태그 XY 필드가 들어온다.

"캡처"는 2026-09-06 `uwb_raw.txt`다. 오늘 스트림은 확인하지 않았다.

| 필드 | 캡처 | 코드가 하는 일 | 조치 |
|---|---|---|---|
| type, schema, tag_id | 있음 | 분기, `schema==1`, `tag_id` 일치 검사 | 없음 |
| seq | 있음 | 중복·역행·재시작 검사(32bit wrap) | 누락(delta>1)은 통과시키고 세지 않음 → 카운트 추가 |
| cycle_start_us / end_us | 있음 | 범위·역행 검사, 사이클 0.10s 초과 폐기 | 없음 |
| cycle_duration_us | 없음 | `end-start`로 직접 계산 | 태그가 보내면 교차검사 |
| valid_mask | 있음 | 0~15 검사, 비트별 앵커 선택 | 없음 |
| rf_valid_mask | 없음 | 읽지 않음 | 오늘 스트림 확인 후 수용 |
| raw_slant_m[4] | 있음 | 유한·0<r≤max_range 검사 | `range_bias_m` 차감 → `cal_slant_m` 추가 |
| sample_time_us[4] | 있음 | 사이클 범위 검사, 앵커 간 40ms 초과 폐기 | 앵커별 `tau_i`로 확장(현재는 평균 하나) |
| twr_seq[4] | 없음 | 읽지 않음 | 오늘 스트림 확인 후 수용 |
| failure[4] | 있음 | mask=1인데 `ok`가 아니면 사이클 전체 폐기 | 앵커 단위 제외로 바꿀지 결정 |
| RF diagnostics[4] | 없음 | 읽지 않음 | 로그 보존만 |
| attempt_count | 있음 | 읽지 않음 | 명세에 없음, 로그 보존 |
| raw_xy_valid, raw_x_m, raw_y_m, raw_xy_anchor_mask | 있음 | `source_mode=tag_xy`일 때만 사용 | 명세는 "태그 raw XY 추가 금지" → 펌웨어 확인 |

상태 메시지 `uwb_raw_status`는 `uwb_ready`, `anchor_order`, `anchor_count`, `clock_domain`, `temporal_filter_applied`를 검증에 쓴다. `firmware`, `anchor_layout_id`, `raw_xy_method`는 로그에만 남고 비교하지 않는다.

## 4. 출력 필드 (§12 ↔ 현재)

현재 출력은 x, y, 공분산뿐이다. `jetson_uwb_preimu`는 새 스키마다.

| 명세 필드 | 현재 대응 | 상태 |
|---|---|---|
| type, schema, seq | `Observation.seq` | 부분 |
| measurement_time_us | `Observation.stamp_ns`(ns, 사용 앵커 report 시각의 평균) | 부분. 단위·의미가 다르다 |
| host_rx_time_us | `received.jsonl`의 `host_received_monotonic_ns` | 로그에만 있음 |
| raw_slant_m[4] | `received.jsonl`의 원본 메시지 | 로그에만 있음 |
| cal_slant_m[4] | 없음 | 없음 |
| raw_xy_valid, raw_xy_mask, raw_x_m, raw_y_m | `details.candidate_xy_m`(2D 선형) | 부분 |
| z_obs, z_m3, z_ab, z_mode, z_valid | 없음 | 생략(고정 z) |
| x_m, y_m | `Observation.x`, `y` | 있음 |
| z_m | 없음. `/uwb_pose`의 z는 0으로 둔 미관측 자리값 | 신규(`z_source=fixed_assumed`) |
| valid, fresh | `reason=='accepted'`로 암시. fresh 없음 | 부분 |
| solver | 없음(선형 풀이 하나) | 없음 |
| solver_anchor_mask | `Observation.anchor_mask` | 있음 |
| source_age_ms | `details.queue_s`(수신 시점 지연) | 부분. 정의가 다르다 |
| residual_rms_m / max_m | `details.pair_residual_m`(쌍 잔차 최대) | 부분. 정의가 다르다 |
| q_bias_hat_m[4], recovery_hold_ms | 없음 | 신규 |
| reason | `Decision.reason` | 부분. 이름 체계가 다르다 |
| processing_ms, quality | 없음. `Observation.variance`만 있다 | 신규 |

## 5. reason (§13 ↔ 현재)

같은 이름은 `insufficient_anchors` 하나뿐이다. 나머지는 기준이 달라서 이름을 정리해야 한다.

| 명세 reason | 현재 reason | 차이 |
|---|---|---|
| ok_h80 | accepted | H80이 없다. 선형 풀이 통과를 뜻한다 |
| ok_q_recovery | 없음 | 신규 |
| recovery_warmup | warming_up | 사이클 수 기준(clock 30회, 연속 통과 5회). 0.10s 시간 기준이 아니다 |
| recovery_bias_hold | 없음 | 신규 |
| insufficient_anchors | insufficient_anchors | 동일. 설정 `min_anchors`는 4 (명세는 최소 3, 4개 우선) |
| source_stale | queued_sample, invalid_host_stamp | 수신 지연 기준이다 |
| clock_unsynced | 없음 | `source_restart_wait_status`가 일부 겹친다 |
| tof_invalid, z_untrusted | 해당 없음 | 고정 z라서 발생하지 않는다 |
| ill_conditioned | degenerate_geometry | 조건비 30 초과 기준. 명세의 열 스케일링 1e6과 다르다 |
| residual_high | inconsistent_ranges | 쌍 잔차 0.15m 초과 기준이다 |
| solver_failed | invalid_schema | `LinAlgError`를 `invalid_schema`로 뭉뚱그린다 |

명세에 없는 현재 reason은 정리 대상이다: `schema_or_tag_mismatch`, `unsupported_status`, `unsupported_type`, `status_unavailable`, `invalid_cycle`, `duplicate_or_out_of_order`, `cycle_too_long`, `invalid_anchor_arrays`, `valid_mask_range_conflict`, `invalid_report_time`, `report_span_too_long`, `range_jump`, `non_increasing_stamp`, `tag_layout_mismatch`, `invalid_tag_xy`.

## 6. 설정 (§3 ↔ 현재)

앵커 좌표는 있지만 값이 인수인계서와 다르다. 나머지 설정은 없다.

| 설정 | 현재 | 조치 |
|---|---|---|
| 앵커 A1~A4 좌표 | `config/anchors_20260906.json`(비정형) | 인수인계서는 (0,0), (5,0), (0,4.5), (5,4.5). §8-1 참조 |
| anchor_z_m[4] | 2.2 (전부 동일, 검사 있음) | 없음 |
| range_bias_m[4] | 없음 | 신규. 0=미교정으로 표시 |
| 고정 z | 없음 | 신규 `fixed_z_m`. 태그 실제 높이를 잰다 |
| ground_z_m, tof_axis_body, tof_lever_arm | 없음 | 고정 z 결정으로 미사용. "미사용"으로 기록 |
| uwb_lever_arm_body_m, R_WB | 없음 | 0, 단위행렬 가정을 설정에 명시 |
| warehouse_to_px4_R | `frames.py`의 `enu_yaw_deg`(2D 회전) | 3D proper rotation이 필요한지 결정 |
| warehouse_px4_origin_m | `enu_offset_x_m`, `enu_offset_y_m` | 대응됨 |
| antenna delay 16385 | 태그 펌웨어 쪽 | companion 코드 없음 |

## 7. 시간 (§6 ↔ 현재)

현재는 호스트 오프셋 하나로 시각을 맞춘다. 앵커별 시각과 다중 clock은 없다.

| 명세 | 현재 |
|---|---|
| t_host | `time.monotonic_ns()`(`node.poll`) |
| t_ESP32 | `cycle_end_us`, `sample_time_us` |
| t_FC | 없음 |
| α(clock 스케일) | 없음 |
| β(clock 오프셋) | 5초 창의 최소 지연 오프셋 |
| tau_i(앵커별 source 시각) | 없음. 사용 앵커의 평균 시각 하나만 쓴다 |
| 자세 SLERP, ToF latest valid | 없음(고정 z라서 불필요) |
| clock mapping residual, end-to-end latency P95 | 기록 없음 |

## 8. 누락 목록

코딩 전에 확인할 것이 3건이다. 그 뒤에 신규 구현과 결정 항목이 이어진다.

### 8-1. 코딩 전 확인

| # | 항목 | 왜 확인하는가 |
|---|---|---|
| 1 | 앵커 좌표 확정 | 좌표가 다르면 XY가 평균 0.71m 어긋난다(§9) |
| 2 | 오늘 태그 스트림의 필드 | 캡처에는 `rf_valid_mask`, `twr_seq`, RF 진단이 없고 `raw_xy_*`가 있다 |
| 3 | 태그 펌웨어 이름 | 캡처는 `uwb-tag-for-jetson-v1.8-raw-xy`. 명세는 `tag_raw_only` 빌드를 전제한다 |

### 8-2. 신규 구현

순서: C → A 보강 → B → E → F → G → H. D는 생략한다.

1. 설정 loader: `range_bias_m`, `fixed_z_m`, 레버암 0, 좌표 변환
2. 원본 로그 기본 켜기. 이유: 원본을 남기지 않는 최적화는 금지다(인수인계서 §1)
3. clock α·β 회귀, `tau_i`, `source_age`
4. 3D raw XY, RMS/max 잔차, 조건수
5. H80, Q_S10, valid·reason 계약, `jetson_uwb_preimu` 스키마, replay 확장

### 8-3. 결정 필요

| 항목 | 충돌 | 결정할 사람 |
|---|---|---|
| z를 PX4로 보낼지 | 인수인계서는 ODOMETRY XYZ. `altitude_policy.md`는 UWB가 x,y만 제공한다 | 사용자 |
| H80·Q_S10이 관측치 전처리인지 | roadmap 결정 10은 이상치 제거·평활까지만 허용한다 | 사용자·팀 |
| 500ms failed에서 hold 제외 | 인수인계서 §13이 FC 안전 담당 승인을 요구한다 | FC 안전 담당 |

금지: 고정 z를 PX4로 보내지 않는다. 임의값은 실제 높이를 측정하지 않아 EKF2 고도 추정을 오염시킨다.

## 9. 근거

앵커 좌표는 9/6 캡처로 확인했다. 단일 캡처이므로 오늘 다시 측정해야 한다.

| 확인 | 결과 |
|---|---|
| 캡처 사이클 수 | 125개(4앵커 전부 유효 124개) |
| 태그의 `raw_x/y` ↔ 직사각형 좌표로 재계산한 값 | 최대 차이 0.000m |
| 태그의 `raw_x/y` ↔ `anchors_20260906.json`으로 재계산한 값 | 평균 0.71m, 최대 0.74m |
| 쌍 잔차(평균/최대) 직사각형 | 0.10m / 0.20m |
| 쌍 잔차(평균/최대) 저장소 좌표 | 0.09m / 0.17m |
| 앵커별 거리 표준편차(A1/A2/A3/A4) | 0.334 / 0.024 / 0.012 / 0.018 m |

잔차만으로는 어느 좌표가 맞는지 판정할 수 없다. 줄자로 재측정해야 한다. 태그 상태 메시지의 `anchor_layout_id`는 `warehouse-5x4p5-z2p2-v2`이고, 저장소 좌표의 `layout_id`는 `warehouse-irregular-z2p2-20260906`이다.

## 2026-09-27 구현 경로 갱신

위 비교표는 당시 구조의 기록이다.
현재 수신·보정·ROS 연결은 별도 폴더에 있다.
새 수신 파일은 `record_directory/raw/`에 쓴다.
판정·상태는 `record_directory/processed/`에 쓴다.
[현재 모듈 계약](uwb_module_api.md)을 구현 기준으로 쓴다.
