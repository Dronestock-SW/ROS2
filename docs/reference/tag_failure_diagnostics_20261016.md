# 태그 최초 실패 진단 계약
Tag B 진단 필드와 집계 분모를 정하는 기준이다.
진단 펌웨어·기록 분석·복구 시험 전에 읽는다.

## 적용 범위

**이 변경은 실패 경로를 기록한다.**
기준 소스는 상원 통합 `70da279`다.
작업 브랜치는 `codex/tag-first-failure-diagnostics-20261016`이다.
운영 태그에 적용한 상태로 해석하지 않는다.
새 실물 성능·거리 정확도·비행 검증은 미실시다.

| 유지 항목 | 값 또는 동작 |
|---|---|
| 기본 RAW 주기 | 50Hz, 시작 간격 20ms |
| 회차 완료 목표 | 18ms |
| 재시도 | 첫 네 시도 뒤 최대 한 번 |
| 자연 UWB 재초기화 대기 | 1500ms |
| 새 RF session 대기 | 100ms |
| `uwb_recover` 시험 명령 | 1500ms 대기를 우회 |
| 동일 boot의 session 변경 | 기존 시계·pose 초기화 유지 |
| 거리·원본 시각·승인 규칙 | 기존 계약 유지 |
| LoRa·앵커·FC 설정 | 변경 없음 |

50Hz는 이번 변경에서 유지하는 설정이다.
진단을 켠 새 실물 성능을 보증하지 않는다.
기존 기준은 [통신 계약](single_tag_protocol_v2.md)을 따른다.
검증 순서는 [시험 절차](../runbooks/tag_failure_diagnostics_tests_20261016.md)에 있다.

## 데이터 경로

**최초 실패는 재시도 결과와 따로 보존한다.**

```text
한 앵커 첫 시도 ── 실패 상세 ─┐
    ↓                       │
다른 앵커의 첫 시도           ├→ 제한된 진단 큐 → USB 상세 사건
    ↓                       │    정상 첫 성공은 카운터만 증가
예산 내 재시도 ── 실패/성공 ─┘
    ↓
완결 RAW: 최종 거리 + 최초/최종 실패
    ↓
기존 수신·시각 대응·B_TF
```

진단은 관측을 대체하지 않는다.
진단 사건이 늦어도 RAW를 기다리게 하지 않는다.
이유: 진단 지연을 측정 지연으로 전파하지 않기 위해서다.

## 완결 RAW의 추가 필드

**기존 schema 2에 `diag_version=1`을 추가한다.**
기존 `failure`와 `attempts`의 의미는 유지한다.
모든 배열의 순서는 A1, A2, A3, A4다.

| 필드 | 형식 | 의미 |
|---|---|---|
| `diag_version` | 정수 1 | 추가 진단 계약 판본 |
| `transaction` | uint32[4] | 마지막 실제 시도의 RF 거래 순번 |
| `first_transaction` | uint32[4] | 최초 시도의 RF 거래 순번 |
| `first_failure` | 정수[4] | 최초 실패 코드, 기존 failure 열거형 |
| `cause` | 정수[4] | 마지막 시도의 상세 원인 코드 |
| `first_cause` | 정수[4] | 최초 시도의 상세 원인 코드 |

| 시도 수 | 최초 필드 | 최종 필드 |
|---|---|---|
| 0 | 거래 순번 0, NotReady 또는 Deadline | 최초와 같음 |
| 1 | 최초 결과 보존 | 최초와 같음 |
| 2 | 첫 실패와 첫 거래 순번 보존 | 재시도 결과와 새 거래 순번 |

시도 수 0은 송신 실패 횟수가 아니다.
해당 회차에 측정을 시작하지 않은 기회다.
실패 거리와 측정 시각은 계속 null이다.
최초 실패를 재시도 성공으로 덮어쓰지 않는다.
이유: 재시도가 가린 첫 실패율을 구해야 하기 때문이다.

## 상세 원인 코드

**코드는 소프트웨어가 관찰한 반환 경로다.**
물리적 RF 원인을 확정하는 라벨이 아니다.

| 값 | 이름 | 확인한 사실 |
|---|---|---|
| 0 | None | 아직 상세 원인을 지정하지 않음 |
| 1 | NotReady | UWB 미준비 또는 bench pause로 측정 미시작 |
| 2 | TxStartRejected | 송신 시작 API가 성공을 반환하지 않음 |
| 3 | RxTimeout | 읽은 상태에 RX 시간초과가 있음 |
| 4 | RxError | 읽은 상태에 RX 오류가 있음 |
| 5 | RadioDeadline | 읽은 RF 시각이 기한에 도달 |
| 6 | McuGuard | MCU 시간 기준 대기 가드 종료 |
| 7 | FrameLength | 읽은 프레임 길이 부적합 |
| 8 | FrameIdentity | 주소·판본·session·거래 등 불일치 |
| 9 | ReportRejected | REPORT 성공 코드가 아님 |
| 10 | TofInvalid | DS-TWR 시간 계산 검사 실패 |
| 11 | RangeInvalid | 거리 값 범위·유한값 검사 실패 |
| 12 | ClockMapInvalid | RF→MCU 시각 대응 검사 실패 |
| 13 | Budget | 회차 또는 교환 예산 부족 |
| 14 | Success | 해당 시도 성공 |

RX timeout만으로 앵커 고장을 확정하지 않는다.
태그 송신·앵커 수신·응답 경로를 따로 확인한다.
McuGuard만으로 DW3000 정지를 확정하지 않는다.
MCU 선점·SPI 지연도 같은 경로를 만들 수 있다.
이번 변경은 기존 복구 결정을 바꾸지 않는다.

## `uwb_rf_attempt` 상세 사건

**실패 시도와 성공한 재시도만 상세 출력한다.**
정상 첫 성공은 누적 시도 카운터에만 넣는다.
따라서 상세 사건 수를 전체 시도 분모로 쓰지 않는다.

| 필드 | 형식 또는 단위 | 의미 |
|---|---|---|
| `type/schema/diag_version/tag_id` | `uwb_rf_attempt` / 2 / 1 / `6` | 메시지 종류와 계약 |
| `boot_id/session/seq` | uint32 | 원본 부팅·RF session·회차 |
| `anchor/attempt/transaction` | 1~4 / 1~2 / uint32 | 해당 앵커·시도·RF 거래 |
| `failure/cause/stage` | 정수 | 기존 실패·상세 원인·단계 |
| `start_us/end_us` | MCU 단조 us | 시도 시작·종료 |
| `radio_start/radio_end` | DW 시각 상위 32비트 | 시도에서 관찰한 RF 시각 |
| `status` | uint32 비트마스크 | 마지막 관련 SYS_STATUS 관측 |
| `wait_stage` | 정수 0~7 | 마지막 대기의 단계. 현재 실패 단계와 다를 수 있음 |
| `wait_start_us/wait_end_us` | MCU 단조 us | 마지막 대기 구간 |
| `wait_radio_start/wait_radio_end` | DW 시각 상위 32비트 | 대기 중 RF 시각 관측 |
| `deadline_radio` | DW 시각 상위 32비트 | 해당 대기 기한 |
| `rf_progress` | bool | 대기 중 RF 시각 변화 관찰 여부 |
| `poll_tx_confirmed` | bool | POLL TX 완료 상태를 확인했는지 |

단계 값은 다음 순서를 따른다.

| 값 | 단계 | 값 | 단계 |
|---|---|---|---|
| 0 | NotStarted | 4 | ReportRx |
| 1 | PollTx | 5 | Distance |
| 2 | ResponseRx | 6 | ClockMap |
| 3 | FinalTx | 7 | Done |

대기가 없으면 대기 진단 필드는 0으로 남는다.
이는 RF 시각 0을 실제 관측했다는 뜻이 아니다.
송신 시작 실패는 직전 대기 기록을 보존할 수 있다.
이때 `stage`와 `wait_stage`를 함께 읽는다.
RF 시각 값은 기존 읽기 결과만 보존한다.
진단을 위해 새 SPI 시각 읽기를 추가하지 않는다.
`rf_progress=false`는 변화 미관측을 뜻한다.
칩 정지·전원 장애를 입증하지 않는다.
`poll_tx_confirmed=true`도 앵커 수신 증거는 아니다.

## 복구 사건과 큐 상태

**자연 복구와 시험 명령 복구를 분리한다.**
`uwb_rf_recovery`도 schema 2, diag_version 1이다.

| 필드 | 값 또는 의미 |
|---|---|
| `event` | scheduled / init_start / init_success / init_failed |
| `reason` | mcu_guard / tx_fault_streak / bench_request |
| 부팅 초기화 reason | startup. 최초 session 생성 전에는 session=0 가능 |
| `boot_id/session/seq` | 사건이 발생한 원본 식별자 |
| `at_us` | 사건 MCU 단조 시각 |
| `detected_us` | 복구를 요청한 이상·명령 감지 시각 |
| `next_cycle_us` | 사건 시점의 다음 측정 예정 시각 |
| `recoveries` | 기존 재초기화 시도 누적 횟수 |

초기화 성공 사건은 새 session을 담는다.
동일 boot·recoveries로 시작과 완료를 연결한다.
상세 사건 누락이 있으면 시간 구간을 추정하지 않는다.
복구 사건의 seq는 마지막으로 할당한 회차다.
아직 시작하지 않은 새 회차를 뜻하지 않는다.

| heartbeat 추가 필드 | 의미 |
|---|---|
| `diag_attempts` | 실제 실행한 모든 시도 누적 |
| `diag_failures[15]` | 실패 시도만 cause별 누적 |
| `diag_dropped` | 진단 큐 포화 등으로 누락한 수 |
| `diag_stale` | 2초 넘은 진단 폐기 수 |
| `diag_high_water` | 송신 큐 삽입 뒤 관찰한 점유 개수의 최댓값 |

Success 항목 14의 실패 누적은 0이다.
누적값은 MCU boot 단위로 해석한다.
RF session 변경만으로 서로 빼지 않는다.
카운터 역행은 wrap·순서·재시작 확인 대상이다.
임의의 큰 양수 차이로 바꾸지 않는다.

진단 큐의 크기는 16개다.
RAW의 송신 우선권을 유지한다.
진행 중인 JSON 한 줄은 끝까지 보낸다.
그 뒤 시각 응답·상태·RAW·보조·진단 순서로 선택한다.
동시 큐 소비로 순간 점유 최댓값을 놓칠 수 있다.
TTL 안의 이전 session 진단은 원래 ID로 보낸다.
이전 진단을 새 측정으로 사용하지 않는다.
큐 누락·만료와 호스트 기록 누락을 별도로 센다.

## 분석 분모와 결론의 한계

**수신한 유일 RAW의 네 측정 기회가 기본 분모다.**
분석기는 boot·session·seq로 RAW 중복을 제외한다.
같은 키의 내용 충돌도 별도로 보고한다.

| 지표 | 분자 | 분모 |
|---|---|---|
| 최초 성공률 | first_failure=0 | 수신 RAW 수 × 4 |
| 최종 성공률 | failure=0 | 수신 RAW 수 × 4 |
| 시도한 앵커의 최초 성공률 | first_failure=0 | attempts>0인 앵커 기회 |
| 시도한 앵커의 최종 성공률 | failure=0 | attempts>0인 앵커 기회 |
| 재시도 회복률 | attempts=2이며 최종 성공 | attempts=2인 앵커 기회 |
| 측정 함수 실행 수 `ranging_attempts` | attempts의 합 | 수신 RAW 범위만 집계 |

누락한 RAW의 결과는 생성하지 않는다.
측정 함수 실행에는 송신 시작 거절도 포함된다.
실제 무선 송신 완료 수로 읽지 않는다.
진단 필드가 없는 과거 RAW도 별도로 센다.
과거 최종 결과에서 최초 실패율을 복원하지 않는다.
상세 사건 집계는 수신한 사건의 분포다.
heartbeat 차이는 두 상태 알림 사이 구간이다.
RAW 집계 구간과 같다고 가정하지 않는다.

0건의 진단 drop은 완전 기록의 증명이 아니다.
시작·종료 부분과 호스트 누락이 남을 수 있다.
분석기는 파일 SHA-256과 파싱 실패 수를 남긴다.
원본 파일을 수정하거나 덮어쓰지 않는다.
이유: 같은 근거로 집계를 재현해야 하기 때문이다.
