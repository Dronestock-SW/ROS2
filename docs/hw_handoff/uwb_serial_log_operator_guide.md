# UWB 시리얼 로그 사용 해설서 (HW 파트 수신본)

> **수신 자료 전사본이다.** 2026-08-12 HW 파트에서 받은 파일을 UTF-8로 다시 인코딩해 옮겼다.
> 원본은 인코딩이 깨진 상태(UTF-8을 CP1252로 읽은 mojibake)로 도착했다. 바이트 단위 원본이 필요하면 HW 파트에 재요청한다.
> 이 문서는 **기준 문서가 아니다.** 우리 쪽 판단 기준은 docs/roadmap.md 다.

**대상**: UWB 펌웨어를 개발하지 않은 다른 부서·상위·통합·Jetson 담당자
**목적**: 로그의 숫자와 `true/false`가 왜 나왔고 제어에 어떤 영향을 주는지 한 줄씩 판단할 수 있게 한다.
**검증 기준**: 2026-08-09 COM3 실측 로그 `data/ai_logs/pilot_20260809_115530_x070_y180_z120.jsonl` (568 프레임, 15초)

---

## 0. 이것만 기억하면 되는 3가지

```text
1) fix=false 이면 x,y 숫자가 보여도 제어에 쓰면 안 된다.
2) 로그 줄에는 두 종류가 있다. 대부분의 줄에는 거리·품질 필드가 아예 없다. (2장)
3) 필드가 없는 것과 false인 것은 완전히 다르다. 없는 것은 "미지원/미출력"이다.
```

이 문서를 처음 읽는다면 **1장(용어) → 2장(프레임 종류) → 3장(필드 요약)** 만 먼저 읽어도 로그 한 줄을 해석할 수 있다. 4장 이후는 필요할 때 찾아보는 사전이다.

---

## 1. 먼저 알아야 할 용어 (비전공자용)

| 용어 | 한 문장 설명 | 비유 |
|---|---|---|
| **UWB** | 아주 짧은 전파 펄스를 쏘아 거리를 재는 무선 방식 | 초음파 대신 전파 |
| **태그 / 앵커** | 태그 = 위치를 알고 싶은 이동체. 앵커 = 벽에 고정된 기준점 4개 | 태그=사람, 앵커=벽에 박힌 못 |
| **DS-TWR** | 태그와 앵커가 신호를 두 번 주고받아 왕복시간을 재는 방식. 시계를 맞출 필요가 없는 게 장점 | 공을 두 번 주고받으며 거리 재기 |
| **ToF** | 전파가 날아간 시간(Time of Flight). 여기에 빛의 속도를 곱하면 거리가 나온다 | 소리가 되돌아오는 시간 |
| **경사거리 / 수평거리** | 경사거리 = 높이차를 포함한 직선거리. 수평거리 = 바닥에 그림자로 내린 거리 | 사다리 길이 vs 바닥 그림자 길이 |
| **bias** | 항상 일정하게 길거나 짧게 나오는 **고정 오차** | 눈금이 밀린 자 |
| **sigma (σ)** | 값이 얼마나 **흔들리는지** 나타내는 항. 정확도가 아니라 안정도다 | 손떨림 정도 |
| **Median / Trimmed mean** | 최근 여러 개 중 가운데 값(또는 최소·최대 뺀 평균)을 쓰는 필터. 튄 값 한 개를 죽인다 | 심사위원 최고·최저점 제외 |
| **EMA** | 이전 값에 새 값을 조금씩 섞는 필터. 부드러워지지만 조금 늦어진다 | 서서히 눈에 적응하기 |
| **gate** | "이건 너무 갑자기 변했다"고 판단해 값을 거부하는 문턱 | 문지기 |
| **NLS / Gauss-Newton** | 앵커 4개 거리로 그린 원들이 가장 잘 겹치는 한 점을 반복 계산으로 찾는 방법 | 네 원의 교차점 더듬어 찾기 |
| **residual** | 찾은 위치에서 다시 계산한 앵커 거리와 실제 측정 거리의 **차이** | 답을 대입해 본 오차 |
| **RMS** | 여러 residual을 하나의 대표값으로 묶은 평균적 크기 | 전체적으로 얼마나 안 맞나 |
| **Huber** | residual이 큰 측정의 영향력을 줄이는 방법. 이상치가 결과를 끌고 가지 못하게 한다 | 튀는 의견 가중치 낮추기 |
| **Kalman filter** | 이전 위치·속도로 다음 위치를 **예측**하고, 새 측정과 섞어 보정하는 필터 | 예측과 관측의 절충 |
| **innovation** | 새 측정치가 Kalman 예측과 얼마나 다른가 | 예상과 실제의 차이 |
| **Mahalanobis d²** | 그 차이를 불확실성으로 나눠 정규화한 값. 이 문서 기준 9.21 초과면 이상치로 거부 | 오차를 σ 단위로 환산 |
| **공분산** | Kalman이 스스로 "지금 내 위치 추정이 얼마나 불확실한가"를 담아둔 값 | 자신감 수치 |
| **NLOS** | 태그와 앵커 사이에 장애물이 있어 신호가 돌아오는 상태. 거리가 실제보다 길게 나온다 | 벽 뒤로 돌아온 소리 |

> **가장 자주 생기는 오해**: sigma가 작으면 정확한 것이라고 생각하는 것. sigma는 **흔들림**만 본다. 항상 81 cm 길게 재는 앵커도 흔들림이 없으면 sigma는 작다. 실제로 아래 12장의 A2가 정확히 그런 경우다.

---

## 2. 로그 줄에는 두 종류가 있다 — 가장 많이 헷갈리는 부분

이것을 모르면 "필드가 사라졌다", "펌웨어가 이상하다"는 잘못된 보고가 나온다.

같은 `type: "uwb_pose"`이지만 **줄마다 필드 개수가 다르다.**

| 프레임 | 개수 (실측 568개 중) | 주기 | 필드 수 | 용도 |
|---|---:|---|---:|---|
| **압축 프레임** | 511개 (90%) | 매 사이클 (약 23 ms) | **13개** | 위치와 제어 상태만 빠르게 전송 |
| **상세 프레임** | 57개 (10%) | **10 seq마다 1회** | **94개** | 거리·필터·품질 진단용 |

### 압축 프레임에 있는 13개 필드 전부

```text
type, seq, t, fix, x, y,
xy_control_fresh, xy_control_held, hybrid_source,
pose_interval_ms, range_cycle_ms, reason, host_time_utc
```

실측 압축 프레임 예 (seq=119):

```json
{"type":"uwb_pose","seq":119,"t":5060,"fix":true,"x":0.267,"y":1.833,
 "xy_control_fresh":false,"xy_control_held":true,"hybrid_source":"none",
 "pose_interval_ms":24,"range_cycle_ms":20,"reason":"hold_bridge",
 "host_time_utc":"2026-08-09T02:55:35.457345+00:00"}
```

### 상세 프레임에만 있는 81개 필드 (분류)

| 분류 | 필드 |
|---|---|
| 거리 4단계 | `raw_r1~4`, `corrected_r1~4`, `horizontal_r1~4`, `solve_r1~4` |
| 보정 참고값 | `cal_r*_raw`, `cal_r*_corrected`, `range_bias_a*_m`, `tag_height_m`, `anchor_height_a*_m` |
| 거리 품질 | `range_sigma_a*_m`, `range_weight_a*`, `range_geo_residual_a*_m`, `range_fail_a1~a4`, `range_ms_a*` |
| 위치 계산 | `raw_pose_x/y`, `pose_residual_max_m`, `pose_residual_rms_m`, `pose_residual_rejected`, `pose_jump_rejected`, `pose_speed_mps`, `pose_speed_rejected`, `raw_guard_healthy`, `solve_anchor_mask`, `solve_mode`, `range_input` |
| 적응형 제외 | `adaptive_excluded_mask`, `adaptive_exclusion_state_mask`, `adaptive_variance` |
| 상태·점수 | `xy_control_usable`, `qscore`, `quality_score`, `tag_layout_synced`, `layout_id`, `id`, `drone_id` |
| 타이밍 | `cycle_ms`, `cycle_total_ms`, `serial_emit_ms`, `lora_send_ms`, `lora_service_ms`, `range_cycle_budget_ok` |

> **판별 규칙**: `raw_r1`이 있으면 상세 프레임, 없으면 압축 프레임이다.
> 압축 프레임에서 `range_fail_a1`을 찾지 못했다고 해서 앵커 통신이 실패한 것이 아니다. **애초에 그 줄에 출력되지 않는다.**
>
> 패널은 압축 프레임에서 이 필드들을 "OK"나 "false"로 채우지 말고 **항상 '해당 없음'** 으로 표시해야 한다.

---

## 3. 로그 한 줄 30초 판독법

```text
1. fix                 → 지금 이 XY를 제어에 써도 되는가?
2. reason              → 왜 이런 결과가 되었는가?
3. fresh / held        → 새 값인가, 직전 값 잠시 유지인가?
4. hybrid_source       → 어느 경로로 만든 위치인가?
--- 여기까지는 압축 프레임에서도 가능 ---
5. range_fail_a1~a4    → 앵커 통신·거리 계산이 성공했는가?
6. raw_r* vs solve_r*  → 필터가 거리를 얼마나 바꿨는가?
7. pose_residual_rms_m → 네 거리가 한 점에서 만나는가?
8. solve_anchor_mask   → 실제로 몇 개 앵커로 풀었는가?
```

가장 중요한 규칙:

```text
fix=true  → 현재 XY를 제어/전송에 사용 가능
fix=false → x,y 숫자가 화면에 보여도 제어값으로 사용 금지
```

실측 로그에서 `xy_control_usable`은 `fix`와 항상 같이 갔다 (57/57 일치). 둘 중 하나만 봐도 된다.

---

## 4. 전체 처리 흐름

```text
[거리 단계]  raw_r  →  corrected_r  →  horizontal_r  →  solve_r
             DS-TWR    bias 제거       높이차 제거      Median+EMA+gate

[위치 단계]  solve_r + 앵커좌표  →  가중 NLS  →  후보 검사  →  (Kalman)
                                    raw/filtered   residual·jump·speed

[제어 단계]  fresh x,y  →  최대 500 ms hold  →  fix=false
```

`x`, `y`만 보고는 원인을 알 수 없다. 반드시 같은 줄의 거리, residual, `reason`을 함께 봐야 한다.

---

## 5. 거리 필드: RAW에서 solver 입력까지

### 5.1 네 단계 요약

| 단계 | 필드 | 계산 | 실측(seq=120, A2) |
|---|---|---|---|
| 1. RAW | `raw_r1~4` | DS-TWR 왕복시간 × 빛의 속도 | 5.441 |
| 2. bias 보정 | `corrected_r1~4` | `raw_r - range_bias_a*_m` | 5.441 |
| 3. 높이 보정 | `horizontal_r1~4` | `sqrt(corrected² - (tag_h - anchor_h)²)` | 5.441 |
| 4. 시간 필터 | `solve_r1~4` | Median(3) → EMA(α=0.32) → jump gate | **5.513** |

### 5.2 왜 지금은 1~3단계 값이 전부 같은가

실측에서 `raw = corrected = horizontal`로 완전히 동일하다. 단계가 무의미해서가 아니라 **현재 설정값이 전부 0이거나 같기 때문**이다.

```text
range_bias_a1~a4_m = 0.0     → 2단계에서 뺀 값이 없음
tag_height_m       = 1.20 m
anchor_height_a1~4 = 1.20 m  → 높이차 0 → 3단계에서 줄일 값이 없음
```

**이 단계들이 값을 바꾸기 시작하는 시점**:

- 앵커별 antenna delay를 교정해 `range_bias_a*_m`에 0이 아닌 값을 넣는 순간 → 2단계가 갈라진다.
- 태그를 드론에 실어 높이가 앵커와 달라지는 순간 → 3단계가 갈라진다. 예를 들어 태그 0.30 m, 앵커 1.20 m면 높이차 0.90 m이므로 경사거리 3.115 m는 수평거리 2.982 m가 된다 (약 13 cm 감소).

따라서 **1~3단계가 다 같다는 것은 "아직 교정을 안 했다"는 신호**이지 정상 상태가 아니다. 12장의 체계적 오차와 직결된다.

### 5.3 `cal_r*_raw` / `cal_r*_corrected`

교정 작업용 참고 필드다. 각각 `raw_r*`, `corrected_r*`와 같은 값을 담고 있으며, 다지점 bias 측정 스크립트가 이 필드를 읽는다. 일반 판독에서는 무시해도 된다.

### 5.4 4단계(시간 필터)의 세부 동작

| 순서 | 이름 | 로직 | 효과 |
|---|---|---|---|
| a | Median(3) 또는 trimmed5 | 최근 3개 중앙값 / 5개 중 최소·최대 제외 평균 | 한 번 튄 순간 outlier 제거 |
| b | EMA | `filtered = previous + 0.32 × (window - previous)` | 부드러워지나 약간 지연 |
| c | jump gate | 이전 필터 값과 0.12 m 초과 차이면 거부. 같은 새 값이 **4회** 모이면 실제 이동으로 인정해 재추종 | 튄 값은 막고 진짜 이동은 영구 대결하지 않음 |

gate가 거부하면 그 앵커는 `range_fail_a*=gate`로 표시되고, 이전 필터값이 유지되거나 그 사이클 solver에서 빠진다.

---

## 6. 거리 품질 필드

### `range_sigma_a*_m` — 흔들림

- 최근 거리의 2차 차분으로 추정한 앵커별 잡음 표준편차
- 작을수록 시계열이 안정적. **정확도가 아니라 안정도**다.
- 실측 평균: A1 0.015, A2 0.021, A3 0.024, A4 0.033 m → 네 앵커 모두 매우 안정적이다.

### `range_weight_a*` — solver 영향력

- 범위 0.08~1.0. 1.0이 정상, 작을수록 위치 계산에서 무시된다. 0.08이 바닥값이다.
- noise weight과 geometry weight 중 **더 낮은 값**을 쓴다.
- 0.30 이하가 3회 지속되면 적응형 제외 후보가 된다.
- 실측 최신값: A1 1.00, A2 0.584, **A3 0.08**, A4 0.08

### `range_geo_residual_a*_m` — 다른 앵커들과의 불일치

```text
abs(이전 위치에서 예상한 앵커 거리 - 이번 horizontal 거리)
```

- 한 앵커만 따로 노는지 찾는 값. 0에 가까울수록 일치.
- 0.18 m 초과 + 다른 앵커보다 1.5배 이상 + 0.12 m 이상 차이 → 해당 앵커 가중치 하향
- 실측 평균: A1 0.066, A2 0.248, **A3 0.532**, A4 0.219 m

> **주의**: 이 값은 **진짜 좌표와의 오차가 아니다.** "이전에 추정한 위치와 얼마나 모순되는가"다. 그래서 12장에서 보듯 bias가 가장 큰 A2가 아니라 A3가 제외되는 역전이 일어난다.

### `range_fail_a1`~`range_fail_a4` — 앵커별 결과 코드

| 코드 | 쉬운 뜻 | 확인할 곳 |
|---|---|---|
| `ok` | 정상 | 없음 |
| `budget` | 한 사이클 제한시간 40 ms 초과 | 통신 지연·로그 부하 |
| `recover` | UWB 재초기화 요청됨 | SPI/DW3000 상태 |
| `ptx`, `pto` | Poll 송신 자체 실패/시간초과 | 태그 송신·SPI |
| `prx` | Poll 후 수신 전환 실패 | DW3000 상태 |
| `rto`, `rrd`, `rmm` | Response 시간초과/읽기 실패/주소·종류 불일치 | 해당 앵커 전원·ID |
| `ftx`, `fto`, `frx` | Final 송수신 단계 실패 | RF 상태·타이밍 |
| `qto`, `qrd`, `qmm`, `qiv` | Report 시간초과/읽기/불일치/앵커 invalid 보고 | 앵커 응답·펌웨어 |
| `qor` | 계산 거리가 0 이하 또는 80 m 초과 | 타임스탬프·antenna delay |
| `cal` | bias 보정 후 거리가 유효하지 않음 | bias 설정 |
| `height` | 높이 보정 결과가 유효하지 않음 | 태그·앵커 높이 |
| `gate` | 거리 변화 gate가 거부 | 순간 변화 또는 실제 급이동 |
| `off` | 비활성 앵커 | 빌드 설정 |

실측 15초 동안 네 앵커 모두 **57/57 프레임에서 `ok`** 였다. 즉 현재 문제는 통신 문제가 아니다.

---

## 7. 위치 계산 필드

### `raw_pose_x`, `raw_pose_y`

시간 필터를 거치지 않은 `horizontal_r*`로 계산한 XY 후보. 반응이 빠르지만 거리 튐에 민감하다.

### `pose_residual_max_m`, `pose_residual_rms_m`

- `max`: 앵커별 예상거리−측정거리 차이 중 최대값. 허용 **0.42 m 이하**
- `rms`: 사용 앵커 residual의 RMS. 허용 **0.28 m 이하**
- 실측(후보 생성 성공 34프레임): rms 평균 0.174 / 최대 0.244, max 평균 0.253 / 최대 0.385 → 기준 안에는 들어온다.

> **`null`은 0이 아니다** —
> 실측 상세 57프레임 중 **23프레임에서 이 값이 `null`** 이었다. 이는 residual이 0이라는 뜻이 아니라 **위치 후보 자체가 만들어지지 않아 계산할 대상이 없었다**는 뜻이다. 패널에서 `null`을 0으로 렌더링하면 "완벽하게 맞았다"로 정반대 해석이 된다. `null`은 반드시 `—` 또는 "후보 없음"으로 표시한다.

### `pose_residual_rejected` / `pose_jump_rejected` / `pose_speed_rejected`

각각 residual 기준 초과, 위치 급이동, 속도 이상으로 후보를 거부했는지 표시한다.
실측: residual 거부 23회, jump 거부 1회, speed 거부 0회. **거부의 거의 전부가 residual 문제**다.

### `raw_guard_healthy`

RAW 거리 기반 위치가 아래를 **모두** 통과했는지 표시한다.

- 배치 경계 안 (0.60 m 이상 벗어나지 않음)
- 최대 residual ≤ 0.42 m, RMS residual ≤ 0.28 m
- 거리 gate reject 없음
- filtered 위치와 차이 ≤ 0.15 m
- 이전 RAW 위치와 이동량 ≤ 0.30 m

### `hybrid_source`

| 값 | 뜻 | 패널 표현 | 실측 |
|---|---|---|---:|
| `raw_g` | RAW 위치가 검사를 통과해 채택됨 | 정상 - 빠른 경로 | 339 |
| `range_f` | RAW 불안정으로 필터 거리 위치 사용 | 정상 - 필터 대체 | 4 |
| `kf_pred` | 새 위치 없이 Kalman 예측만 사용 | 주의 - 예측 중 | 0 |
| `none` | 사용할 위치 후보가 없음 | **오류 - 위치 없음** | **225** |

> 실측 568프레임 중 **225회(40%)가 `none`** 이다. 이 중 상당수는 `fix=true`인 hold 구간이므로, `hybrid_source=none`은 화면에 위치가 떠 있어도 **새 위치는 만들어지지 않았다**는 뜻이다.

### `solve_mode`, `range_input`

실측값은 각각 `xy_only_height_corrected_hybrid`, `horizontal`로 고정이다. 높이 보정된 수평거리로 XY만 푼다는 뜻이며, 값이 바뀌면 펌웨어 설정이 바뀐 것이다.

### 앵커 bit mask — 두 필드의 의미가 반대다

**비트 규칙은 같다**: bit0=A1, bit1=A2, bit2=A3, bit3=A4.
**하지만 비트가 1일 때의 뜻이 정반대다.**

| 필드 | 비트=1의 뜻 |
|---|---|
| `solve_anchor_mask` | 그 앵커를 **사용했다** |
| `adaptive_excluded_mask` | 그 앵커를 **제외했다** |
| `adaptive_exclusion_state_mask` | 그 앵커가 **제외 대기/관찰 상태**다 |

| 값 | 이진수 | `solve_anchor_mask`일 때 | `adaptive_excluded_mask`일 때 |
|---:|---:|---|---|
| 15 | 1111 | A1~A4 전부 사용 | 전부 제외 (비정상) |
| 11 | 1011 | A1, A2, A4 사용 (**A3 빠짐**) | A1, A2, A4 제외 |
| 4 | 0100 | A3만 사용 | **A3만 제외** |
| 8 | 1000 | A4만 사용 | A4만 제외 |
| 0 | 0000 | **사용 앵커 없음 = 위치 계산 실패** | 제외 없음 |

실측에서 정상 프레임은 `solve_anchor_mask=11`, `adaptive_excluded_mask=4`였다. **두 값 모두 "A3를 빼고 3개로 풀었다"는 같은 사실**을 반대 방향으로 표현한 것이다.

---

## 8. 제어 출력 필드

| 필드 | 뜻 | 실측 |
|---|---|---:|
| `fix` | 제어 위치 사용 가능 | true 341 / false 227 |
| `xy_control_fresh` | 이번 사이클에서 만든 새 제어 위치 | true 320 |
| `xy_control_held` | 직전 유효 위치를 최대 500 ms 재사용 중 | true 21 |
| `xy_control_usable` | `fix`와 동일 (상세 프레임에만 출력) | fix와 57/57 일치 |

### 상태 조합표

| fix | fresh | held | 뜻 | 사용 가능 여부 | 실측 |
|---|---|---|---|---|---:|
| true | true | false | 새 유효 위치 | 사용 가능 | 320 |
| true | false | true | 새 값이 없어 직전 위치를 잠시 사용 | 짧게만 (≤500 ms) | 21 |
| false | false | false | 사용 가능한 위치 없음 | **사용 금지** | 227 |
| true | true | true | 정상 구현에서 발생 불가 | 버그/버전 불일치 점검 | 0 |

`held=true`가 500 ms를 넘기면 `fix=false`로 떨어진다.

---

## 9. Kalman 필드 — 현재 실측 로그에는 없다

### 먼저: 지금 상태

**2026-08-09 실측 로그 568프레임 전부에 아래 6개 필드가 하나도 없다.**

| 필드 | 저장소 소스 | 실측 로그 |
|---|---|---|
| `kf_measurement_x`, `kf_measurement_y` | 구현됨 | **없음 (0/568)** |
| `kf_measurement_sigma_m` | 구현됨 | **없음 (0/568)** |
| `kf_innovation_d2` | 구현됨 | **없음 (0/568)** |
| `kf_measurement_accepted` | 구현됨 | **없음 (0/568)** |
| `kf_innovation_rejected` | 구현됨 | **없음 (0/568)** |
| `kf_predict_only` | 구현됨 | **없음 (0/568)** |

`hybrid_source=kf_pred`도 실측 0회다. 즉 **현재 태그 펌웨어의 위치 출력 경로에는 Kalman 단계가 로그로 드러나지 않는다.**

가능한 원인은 셋이다.

1. 태그에 올라간 펌웨어가 현재 저장소보다 이전 버전이다.
2. 해당 빌드에서 Kalman 로깅이 비활성이다.
3. 패널이 다른 schema를 보고 있다.

> **다른 부서 전달 전 반드시 펌웨어 버전과 로그 schema를 고정한다. 없는 필드를 `false`로 간주하면 "Kalman이 계속 거부 중"이라는 완전히 잘못된 결론이 나온다. 반드시 항상 "미지원/없음"으로 표시한다.**

### 아래는 저장소 소스 기준 사전 (참고용)

이 절의 내용은 **로그에 해당 필드가 나타나기 시작하면** 적용된다.

`kf_measurement_accepted=true`의 뜻: 이번 사이클의 새 XY가 Kalman 예측과 통계적으로 충분히 가까워 상태 보정에 실제 사용되었다.

true가 되기까지 통과해야 하는 조건:

1. 유효 앵커 거리 3개 이상
2. 앵커 좌표 배치가 유효하고 동기화됨
3. 가중 NLS가 유한한 XY 후보를 계산
4. 위치가 배치 경계에서 0.60 m 이상 벗어나지 않음
5. 최대 residual ≤ 0.42 m
6. RMS residual ≤ 0.28 m
7. RAW 후보라면 filtered 후보와 차이 ≤ 0.15 m, 이전 RAW 위치 이동 ≤ 0.30 m
8. Kalman이 초기화됨 (연속 위치 6개 필요)
9. `kf_innovation_d2` ≤ 9.21

true일 때 내부 동작: `[x,y,vx,vy]` 상태 보정 → 공분산 Joseph form 갱신 → 마지막 유효 측정 시각 갱신 → `kf_predict_only=false`.

### Kalman boolean 3개는 반드시 함께 본다

| accepted | innovation_rejected | predict_only | 해석 |
|---|---|---|---|
| true | false | false | 정상: 새 XY를 Kalman에 반영 |
| false | true | true | 새 XY가 예측과 너무 달라 거부, 짧게 예측만 사용 |
| false | false | true | 유효한 새 XY가 없어 짧게 예측만 사용 |
| false | false | false | 초기화 중이거나 Kalman 출력도 유효하지 않음 |
| true | true | any | 정상 구현에서 불가능. 로그 버전·파싱 점검 |

`accepted=false` 하나만 보고 "Kalman 오류"라고 판단하면 안 된다. 위 4가지가 전부 `accepted=false`다.

predict-only 출력은 마지막 유효 측정 이후 **0.20초까지만** 유효하며, 500 ms 제어 hold 타이머를 연장하지 않는다.

---

## 10. `reason` 코드

| reason | 쉬운 설명 | 조치 | 실측 |
|---|---|---|---:|
| `raw_g` | 새 RAW 위치를 정상 사용 | 정상 | 319 |
| `range_f` | RAW가 불안정해 필터 거리로 계산 | 정상, RAW 품질 추이 확인 | 1 |
| `kf_pred` | 새 위치 없이 잠깐 예측 | 반복되면 UWB 품질 점검 | 0 |
| `hold_bridge` | 새 값을 못 만들어 직전 값 유지 | 500 ms 내 단기 보호 | 21 |
| `pose_residual_hold` | 앵커 거리들이 한 XY에서 맞지 않음 | **좌표·물리 ID·bias·NLOS 확인** | **222** |
| `pose_jump_hold` | 위치가 한 번에 너무 멀리 이동 | 실제 급이동인지 outlier인지 확인 | 5 |
| `range_gate_hold` | 앵커 거리 변동이 갑자기 큼 | 해당 앵커 `raw_r` 확인 | 0 |
| `cycle_slow` | 한 사이클 40 ms 초과 | UART/SPI/RF 부하 | 0 |
| `retry_budget` | 재시도 과다 | 앵커 통신 품질 | 0 |
| `layout_unsynced` | 앵커 좌표 미동기화 | layout 동기화 | 0 |
| `solve_failed` | 거리는 있지만 XY 계산 실패 | 유효 앵커 수·기하 확인 | 0 |
| `A1_rto` 등 | 표시된 앵커에서 Response 없음 | 해당 앵커 전원·주소 | 0 |
| `uwb_not_ready` | DW3000 준비 안 됨 | 초기화·SPI·전원 | 0 |

> 실측에서 **`pose_residual_hold`가 전체의 39%(222/568)** 다. 통신 실패는 0건이다. 즉 **문제는 통신이 아니라 거리 정확도**다.

---

## 11. 품질점수 `qscore` / `quality_score` — 함정 주의

계산 방식 (상세 프레임에만 출력):

```text
fix=false        → 무조건 0
정상 시작점       → 100
사이클 시간 초과   → -30
재시도 예산 초과   → -25
앵커 추가 재시도   → 회당 -10
거리 gate reject → -30
적응형 앵커 제외   → -15
Kalman predict-only → -40
Kalman innovation reject → -20
```

실측 분포: **85점 34회**(A3 적응형 제외 -15), **0점 22회**(fix=false), **100점 1회**.

> **함정 —**
> 100점이 나온 그 1회가 바로 seq=120이다. 이 프레임은 `hybrid_source=none`, `solve_anchor_mask=0`, `pose_residual_rejected=true`로 **새 위치를 하나도 만들지 못한 프레임**이다. `fix=true`(hold 중)라서 감점이 걸리지 않아 100점이 나왔을 뿐이다.
>
> **qscore는 "위치가 얼마나 정확한가"가 아니라 "이번 사이클 처리에 감점 사유가 있었나"를 뜻한다.** cm 오차와는 아무 관계가 없다. 실제로 이 100점 프레임의 위치 오차는 약 0.57 m다.
>
> 패널에서 qscore를 정확도처럼 보이는 큰 게이지로 만들면 안 된다. `fix`, `reason`과 항상 같이 표시한다.

---

## 12. 실측 진단 (2026-08-09, 15초, 정지 상태)

**기준 좌표**: 태그 `(0.70, 1.80, 1.20) m` / layout `w5x4p5_4anchor` / 태그 ID 5

### 12.1 거리 오차 — 진짜 원인

| 앵커 | 기준거리 | RAW 평균 ± SD | 필터 평균 ± SD | **기준 대비 오차** |
|---|---:|---|---|---:|
| A1 | 1.931 m | 1.932 ± 0.032 | 1.935 ± 0.028 | **+0.001 m** |
| A2 | 4.662 m | 5.468 ± 0.041 | 5.464 ± 0.038 | **+0.806 m** |
| A3 | 2.789 m | 3.124 ± 0.035 | 3.122 ± 0.018 | **+0.335 m** |
| A4 | 5.077 m | 5.284 ± 0.027 | 5.282 ± 0.026 | **+0.207 m** |

```text
A1 은 거의 맞는다.
A2 는 약 81 cm 길게 잰다.
A3 는 약 34 cm 길게 잰다.
A4 는 약 21 cm 길게 잰다.
넷 다 각각 서로 다른 답을 내므로 컴퓨터가 한 점을 고를 수 없다.
```

**필터는 흔들림(SD)만 3~4 cm 수준에서 약간 더 줄였을 뿐, 고정 오차는 1 cm도 줄이지 못한다.** 필터는 흔들림을 줄이는 도구이지 잘못된 앵커 좌표·물리 ID·antenna delay를 고치는 도구가 아니다. 더 강한 smoothing을 하면 화면만 부드럽게 만들 뿐 이 문제를 해결하지 못한다.

### 12.2 결과로 나타난 위치 오차

| 집계 대상 | 개수 | 평균 x | 평균 y | 기준(0.70,1.80)과의 거리 |
|---|---:|---:|---:|---:|
| 전체 프레임 | 568 | 0.145 | 1.914 | 0.566 m |
| `fix=true`만 | 341 | 0.064 | 1.968 | **0.657 m** |
| `fresh=true`만 | 320 | 0.063 | 1.969 | **0.659 m** |

> 제어에 실제로 쓰이는 값은 `fix=true` 쪽이다. **약 0.66 m 오차**이며 X축으로 크게 밀려 있다. (전체 평균 0.145는 fix=false 프레임까지 섞인 값이라 실제 제어 품질보다 좋아 보인다.)

### 12.3 A2가 가장 나쁜데 왜 A3가 제외되는가

성공한 위치 계산 34회 중 **33회에서 A3가 적응형 제외**된다 (`solve_anchor_mask=11`, `adaptive_excluded_mask=4`). A4는 4회 제외 관찰 상태(`state_mask=8`)에 들어간다.

| 앵커 | 기준 대비 bias | geo residual 평균 | weight 최신 | 제외 여부 |
|---|---:|---:|---:|---|
| A1 | +0.001 | 0.066 | 1.00 | 안 됨 |
| A2 | **+0.806** | 0.248 | 0.584 | 안 됨 |
| A3 | +0.335 | **0.532** | 0.08 | **됨 (33/34)** |
| A4 | +0.207 | 0.219 | 0.08 | 관찰 상태 |

이유: 적응형 제외는 **진짜 좌표를 모른다.** `range_geo_residual`, 즉 "다른 앵커들이 합의한 위치와 얼마나 모순되는가"만 본다. A2의 큰 bias가 이미 합의된 위치 자체를 끌어당겨 놓았기 때문에, 그 왜곡된 합의에서 가장 어긋나 보이는 A3가 대신 희생된다.

> **결론: 지금 시스템은 3개 앵커로, 그것도 틀린 앵커를 빼고 돌아가고 있다.** 이 상태에서 필터·Kalman 파라미터를 조정하는 것은 의미가 없다.

### 12.4 점검 순서 (우선순위)

```text
1. 앵커 A1~A4의 실제 (x,y,z)를 실측해 layout과 대조   ← 가장 먼저
2. 물리 앵커와 논리 ID(A1~A4) 매핑이 맞는지 사진으로 확인
3. antenna delay 교정 후 range_bias_a*_m 값 입력      ← 현재 전부 0
4. 다지점 bias 측정 (scripts/analyze_multipt_calibration.m)
5. 그 다음에서야 필터·Kalman 튜닝
```

통신 지표는 전부 정상이다: `range_fail_a*` 57/57 `ok`, 사이클 21~22 ms(예산 40 ms 내), `range_cycle_budget_ok=true` 100%.

---

## 13. 패널 표기 권장안

| 원본 필드 | 패널 표시 | 도움말 | 색상 |
|---|---|---|---|
| `fix` | 제어 사용 가능 | OFF면 x,y가 보여도 사용 금지 | 초록/빨강 |
| `xy_control_fresh` | 새 제어 위치 | ON일 때 가장 신뢰 가능 | 초록 |
| `xy_control_held` | 이전 위치 잠시 유지 | 최대 500 ms | 노랑 |
| `hybrid_source` | 위치 출처 | `none`이면 새 위치 없음 | 초록/노랑/빨강 |
| `pose_residual_rms_m` | 앵커 불일치 RMS | 0.28 m 초과 시 후보 거부. **`null`은 "후보 없음"** | 숫자+임계값 |
| `solve_anchor_mask` | 사용 앵커 | 비트=사용. 0이면 계산 실패 | 앵커 아이콘 4개 |
| `adaptive_excluded_mask` | 제외 앵커 | **비트=제외 (mask와 반대)** | 회색 처리된 아이콘 |
| `qscore` | 처리 감점 점수 | **정확도가 아님.** fix·reason과 함께만 표시 | 작은 배지 |
| `kf_*` | Kalman 상태 | 현재 펌웨어 미출력 | **항상 "미지원"** |

색상 규칙:

- **초록**: `fix=true` + `fresh=true` + `hybrid_source=raw_g`
- **노랑**: hold 중 또는 predict-only
- **빨강**: `fix=false`, residual 거부, 반복 통신 실패
- **회색**: 해당 프레임/펌웨어 버전에 필드 없음 → `false`로 칠하지 말 것

---

## 14. 문제를 전달할 때 필요한 정보

스크린샷만 보내면 원인 분리가 불가능하다. 아래를 함께 전달한다.

| 항목 | 예시 |
|---|---|
| 원본 JSONL | 문제 `seq` 전후 최소 5초 (압축·상세 프레임 모두) |
| 버전 | firmware version, Git commit, 로그 schema, `type` |
| 설정 좌표 | 태그 `(x,y,z)`, 앵커 A1~A4 `(x,y,z)`, `layout_id` |
| 물리 확인 | 앵커별 물리 ID 사진 |
| 환경 | LOS/NLOS, 장애물, 전원 상태 |
| 핵심 값 | `fix`, `reason`, `hybrid_source`, `range_fail_a*`, `raw_r*`/`solve_r*`, residual, `solve_anchor_mask` |

이 정보가 있으면 **통신 문제 / 거리 calibration 문제 / 위치 solver 문제 / Kalman 거부 / 패널 표시 문제**를 빠르게 분리할 수 있다.

---

## 부록 A. 실측 로그 한 줄 완전 해설 (seq=120)

이 프레임은 "숫자가 멀쩡해 보이는데 실제로는 위치가 없는" 대표적인 경우다.

```json
{"seq":120, "fix":true, "x":0.267, "y":1.833,
 "raw_r1":1.934, "raw_r2":5.441, "raw_r3":3.115, "raw_r4":5.307,
 "corrected_r1":1.934, "horizontal_r1":1.934,
 "solve_r1":1.947, "solve_r2":5.513, "solve_r3":3.125, "solve_r4":5.309,
 "range_fail_a1":"ok", "range_fail_a2":"ok", "range_fail_a3":"ok", "range_fail_a4":"ok",
 "range_geo_residual_a1_m":0.0817, "range_geo_residual_a2_m":0.3647,
 "range_geo_residual_a3_m":0.4349, "range_geo_residual_a4_m":0.1256,
 "pose_residual_max_m":null, "pose_residual_rms_m":null,
 "pose_residual_rejected":true, "solve_anchor_mask":0,
 "hybrid_source":"none", "raw_guard_healthy":false,
 "xy_control_fresh":false, "xy_control_held":true,
 "qscore":100, "reason":"hold_bridge"}
```

| 로그 값 | 쉬운 설명 | 이 프레임의 판정 |
|---|---|---|
| `seq=120` | 측정 사이클 번호 | 상세 프레임 (10 seq마다) |
| `range_fail_a*=ok` | 네 앵커 통신 전부 성공 | **통신은 정상** |
| `raw_r*` | 필터 전 거리 4개 모두 유효 | 거리 측정도 성공 |
| `corrected=horizontal=raw` | bias 0, 높이차 0 | 아직 교정 전 상태 |
| `solve_r*` | Median+EMA 통과 거리 | 필터도 정상 동작 |
| `geo_residual a3=0.435` | A3가 다른 앵커와 가장 모순 | A3 문제로 보이지만 실제 주범은 A2 |
| `pose_residual_rejected=true` | residual 검사 실패 | **위치 후보 거부** |
| `solve_anchor_mask=0` | 사용한 앵커 0개 | **위치 계산 자체가 중단** |
| `pose_residual_*=null` | 계산할 후보가 없음 | 0이 아니라 "없음" |
| `hybrid_source=none` | 채택된 위치 출처 없음 | 새 XY 없음 |
| `x=0.267, y=1.833` | 화면에 뜨는 좌표 | **이번에 계산한 값이 아니라 직전 값** |
| `fresh=false, held=true` | 직전 유효 위치 잠시 재사용 | 최대 500 ms |
| `fix=true` | 제어 사용 가능 | hold 덕분에 유지 중 |
| `qscore=100` | 감점 사유 없음 | **정확도 100%라는 뜻이 아님** |
| `reason=hold_bridge` | 짧은 공백을 직전 값으로 연결 | 지속되면 개선 필요 |

**한 문장 요약**: 통신도 거리 측정도 필터도 전부 성공했지만, **네 앵커 거리가 한 점에서 만나지 않아 새 위치를 만들지 못했고**, 시스템은 위치를 지어내는 대신 직전 좌표 `(0.267, 1.833)`을 잠시 재사용했다. `fix=true`와 `qscore=100`만 보면 완벽한 프레임처럼 보이지만 실제로는 **새 위치가 하나도 없는 프레임**이다.

---

## 부록 B. 필드 존재 여부 빠른 조회

| 찾는 필드 | 압축 프레임 | 상세 프레임 | 현재 펌웨어 |
|---|:-:|:-:|---|
| `fix`, `x`, `y`, `reason`, `hybrid_source` | O | O | 출력됨 |
| `xy_control_fresh`, `xy_control_held` | O | O | 출력됨 |
| `pose_interval_ms`, `range_cycle_ms`, `t`, `host_time_utc` | O | O | 출력됨 |
| `xy_control_usable` | X | O | 출력됨 (`fix`와 동일) |
| `raw_r*`, `corrected_r*`, `horizontal_r*`, `solve_r*` | X | O | 출력됨 |
| `range_fail_a*`, `range_sigma_a*`, `range_weight_a*` | X | O | 출력됨 |
| `pose_residual_*`, `solve_anchor_mask`, `raw_guard_healthy` | X | O | 출력됨 |
| `qscore`, `quality_score` | X | O | 출력됨 |
| `kf_measurement_accepted` | X | X | **미출력** |
| `kf_innovation_rejected`, `kf_innovation_d2` | X | X | **미출력** |
| `kf_predict_only`, `kf_measurement_x/y/sigma_m` | X | X | **미출력** |

`X` = 그 줄에는 아예 없음. **`false`가 아니다.**

---

## 부록 C. 이 문서를 검증하는 데 필요한 설정값

외부 담당자도 도구가 낸 숫자를 스스로 재계산할 수 있도록 설정값을 전부 적는다.

### 앵커 배치 (`include/UwbTagConfig.h`)

| 앵커 | short address | x (m) | y (m) | z (m) |
|---|---|---:|---:|---:|
| A1 | 0x00A1 | 0.0 | 0.0 | 1.20 |
| A2 | 0x00A2 | 5.0 | 0.0 | 1.20 |
| A3 | 0x00A3 | 0.0 | 4.5 | 1.20 |
| A4 | 0x00A4 | 5.0 | 4.5 | 1.20 |

즉 5.0 m × 4.5 m 직사각형의 네 모서리이며, 네 앵커 모두 높이 1.20 m다.

### 기준거리 재계산 (검산용)

태그 `(0.70, 1.80, 1.20)`, 높이차 0이므로 수평거리 = 경사거리다.

```text
A1 = sqrt(0.70² + 1.80²) = sqrt(3.730) = 1.931 m
A2 = sqrt(4.30² + 1.80²) = sqrt(21.73) = 4.662 m
A3 = sqrt(0.70² + 2.70²) = sqrt(7.780) = 2.789 m
A4 = sqrt(4.30² + 2.70²) = sqrt(25.78) = 5.077 m
```

12장 표의 기준거리 4개와 정확히 일치한다. **따라서 12장의 오차는 기준거리 계산 실수가 아니라 실제 측정 오차다.**

### 소스 설정과 실측 로그의 차이 (버전 확인 필요)

| 항목 | `include/UwbTagConfig.h` | 실측 로그 | 비고 |
|---|---|---|---|
| `layout_id` | `warehouse_5x4p5` | `w5x4p5_4anchor` | 런타임 override 가능하므로 단정 불가 |
| 태그 높이 | 0.80 m | 1.20 m | 런타임 override 가능 (측정 시 1.20 m로 설정) |
| `drone_id` | `5` | `5` | 일치 |
| `kf_*` 필드 | 구현됨 | 없음 | 9장 참조 |

`layout_id`와 `kf_*` 두 가지가 함께 어긋난다는 점에서 **태그 펌웨어가 현재 저장소 소스와 다른 빌드일 가능성**이 높다. 전달 전 `firmware_version`과 Git commit을 반드시 확인한다.

### 검증 재현 방법

```bash
# 이 문서의 모든 통계는 아래 파일 하나에서 나온다
data/ai_logs/pilot_20260809_115530_x070_y180_z120.jsonl   # 568 프레임, 15초

# PDF 재생성
.venv-ai/Scripts/python.exe scripts/build_operator_guide_pdf.py
```

> 위 경로는 HW 파트 개발 PC(Windows) 기준이다. 우리 워크스페이스에는 해당 로그·스크립트가 없다.
