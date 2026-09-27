# M01 입력·시간 품질 게이트

UWB·보조 센서의 시각과 입력 품질 판정 명세다. 원본 재생과 센서 시계 어댑터를 구현할 때 읽는다.

## 목적과 현재 상태

다른 시점의 관측이 같은 위치 풀이에 섞이지 않게 한다.
기존 `inputs.py`, `clock.py`, `sensors.py`가 일부 담당한다.
장치별 시계 어댑터와 공통 시험 형식은 추가 구현 대상이다.
[공통 규격](00_interfaces.md)을 사용한다.

## 입출력

| 방향 | 파라미터 | 형식·단위·조건 |
|---|---|---|
| 입력 | `event` | Envelope. 원본과 수신 시각 필수 |
| 입력 | `clock_config` | 장치 ID·시계 단위·wrap 규칙 |
| 입력 | `status`, `seq`, `sample_times` | 장치 상태·순번·표본별 시각 |
| 출력 | `observations` | 매핑 시각을 포함한 Observation[] |
| 출력 | `clock_state` | warming/ready/reset/unmapped |
| 출력 | `gate_decision` | GateDecision과 시각 오차 진단 |
| 출력 | `range_frame` | 동시 풀이 조건을 만족할 때 RangeFrame |

## 설정

| 파라미터 | 단위·범위 | 초기 상태 |
|---|---|---|
| `max_source_age_s` | s, 양수 | 시험 전에 명시 |
| `max_frame_skew_s` | s, 0 이상 | 시험 전에 명시 |
| `max_clock_residual_s` | s, 양수 | 장치별 미확정 |
| `reorder_policy` | reject/bounded_buffer | 기본 reject |
| `reorder_buffer_s` | s | bounded_buffer일 때 필수 |
| `clock_map_id` | string | 미지정 시 unmapped |

## 처리 구조

```text
원문 보존 → 형식·중복 검사 → 장치 시계 매핑
        → 측정 age·묶음 skew 검사 → 관측/실패 기록
```

시간 매핑은 `t_host = alpha*t_device + beta`다.
`age = cutoff_host - t_measurement`를 기록한다.
왕복 통신이나 하드웨어 동기화 여부를 별도 표시한다.
수신 시각을 측정 시각이라고 바꾸어 쓰지 않는다.
묶음은 실제 관측 ID를 보존한다. 평균 거리로 대체하지 않는다.
H80에는 비동시 표본을 각자의 시각으로 전달한다.
동시 풀이 모델에는 skew 검사 후 전달한다.

## 실패와 상태

부팅·시계 역행·출처 변경은 하위 상태 초기화 이벤트다.
시계 미매핑이면 원본을 기록하고 위치 계산만 차단한다.
중복 순번을 새 관측으로 세지 않는다.
유효하지 않은 최신 센서를 과거 정상 센서로 숨기지 않는다.

## 테스트와 평가

| 시험 | 확인할 결과 |
|---|---|
| 정상 시각·순번 | 원본 시각과 매핑값 모두 보존 |
| 중복·역순·wrap·boot | 정책별 거부·초기화 사유 재현 |
| 지연·버퍼 적체 | source age 증가. 최신으로 오인하지 않음 |
| 미래 표본 | 아직 도착하지 않은 관측 미사용 |
| 비동시 앵커 | frame 차단과 window 보존 구분 |

측정값은 수용률·시각 오차·지연 p95·재시작 복구 시간이다.
합성 시계로 단위 시험 가능하다. 실측 동기 성능은 별도다.
이번 작성에서는 시험하지 않았다.
