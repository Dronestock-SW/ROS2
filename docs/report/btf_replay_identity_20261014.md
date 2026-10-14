# B_TF 독립 재생 전 입력·출력 대응 확인

2026-10-14 원본 읽기 감사 결과다. 재생 지표를 정할 때 읽는다.

## 결론

**원본 pose 3,097개를 판정 기록에 연결했다.**
stamp와 XY가 모두 일치했다.
해당 원본에서 중복 stamp는 없었다.
모든 실행에서 유일하다고 보장하지 않는다.

새 A–B–B–A 비교는 아직 수행하지 않았다.
이번 작업은 원본 파일 읽기와 소스 확인이다.
현장 서비스와 FC 설정은 변경하지 않았다.

## 원본과 방법

같은 13일 원본을 변경 없이 읽었다.

| 항목 | 값 |
|---|---|
| 원본 | `capture-1791884513951202644-a5b6e3e7/events.jsonl` |
| SHA-256 | `829d1055c8d7e49b3c4e019560cb5bf4b278b885496b06c030412da5cc30c91e` |
| 비교 A | `19cdb8b975b2d56107d7e470289da6e7779bec2a` |
| 비교 B | `d586e836402066269b1d7e85d4c803f2f748a724` |
| 확인 도구 | `src/drone_uwb/tools/audit_btf_capture_links.py` |
| 결과 | [감사 수치](evidence/btf_replay_identity_20261014.json) |

도구는 ROS를 기동하지 않는다.
원본 행을 읽어 식별자와 개수를 비교한다.

## 대응 결과

입력과 출력은 도착 순서로 연결하지 않았다.

```text
RAW (seq, cycle_end_us)
  -> 판정 (seq, source_cycle_end_us, stamp_ns, xy_m)
  -> pose (header.stamp, position.x/y)
```

| 확인 항목 | 결과 |
|---|---:|
| RAW cycle | 3,408 |
| TDMA epoch | 3,407 |
| RAW status | 17 |
| 판정 전체 | 3,407 |
| RAW에 유일하게 연결된 판정 | 3,370 |
| 식별 필드가 없는 판정 | 37 |
| published 판정 | 3,101 |
| 수집된 pose | 3,097 |
| stamp·XY가 유일하게 일치한 pose | 3,097 |
| published 판정에 비해 미관측인 pose | 4 |

식별 필드가 없는 37개는 `tdma_deadline_rejected`다.
해당 거부의 행별 지연은 추정하지 않는다.
미관측 pose 4개의 누락 지점도 미확정이다.
수집기·전달 경로·관찰 종료를 구분할 근거가 부족하다.

## 원본에 포함된 수집 지연

수집 순서 재생에는 기존 수집 지연도 포함된다.
수집기 ROS 시각에서 입력의 수신 ROS 시각을 뺐다.

| 종류 | 개수 | 150ms 초과 | p95 | 최대 |
|---|---:|---:|---:|---:|
| RAW cycle | 3,408 | 39 | 20.162ms | 755.441ms |
| TDMA epoch | 3,407 | 38 | 23.209ms | 767.831ms |
| RAW status | 17 | 0 | 21.728ms | 35.802ms |

150ms 초과 입력은 합계 77개다.
이는 당시 B_TF 콜백 대기시간과 다르다.
기존 A 재생의 만료 77회와 수치가 같다.
개별 입력 대조 전에는 인과를 확정하지 않는다.

## 시각 동기 경로 정정

`clock_unsynced`는 UWB ClockMap에서 나온다.
MAVROS ClockReadiness와 구분해야 한다.

```text
receiver_queue_expired
  -> reject_queued_input()
  -> TDMA pending 폐기 + UWB ClockMap 초기화
  -> UWB 시각 표본 재축적
  -> 준비 전 clock_unsynced 가능
```

[처리기](../../src/drone_uwb/drone_uwb/processing/measured_btf.py)의
`reject_queued_input()`과 `process()`에서 확인했다.
추가 만료가 준비 대기를 늘릴 수 있는 구조다.
수정본의 추가 만료 원인은 아직 미확정이다.
MAVROS 동기 상태는 센서 승인에 따로 영향을 준다.

## 지연 지표의 측정 경계

주 지연에는 RAW와 TDMA의 결합 대기를 포함한다.

| 지표 | 시작 → 종료 | 한계 |
|---|---|---|
| 주 지연 | RAW 발행 호출 직전 → pose 수신 콜백 진입 | 같은 host의 단조 시계로 측정 |
| 보조 지연 | 대응 epoch 발행 직전 → pose 수신 | epoch 대응이 유일할 때만 계산 |
| 기존 `processing_ms` | 입력 파싱 전 → 판정 처리 완료 | 출력 발행 제외. 순수 fit 시간이 아님 |
| 원본 데이터 나이 | 센서 시각과 판정 기준 시각의 차이 | 주 지연과 별도 보고 |

TDMA required 모드는 RAW를 먼저 보관한다.
계산은 뒤의 epoch 콜백에서 시작한다.
따라서 RAW 발행 시각만 보고 계산시간으로 부르지 않는다.

기존 시험기는 pose 수신을 저장하지 않는다.
행별 실제 발행 시각도 부족하다.
외부 시험기를 보완한 뒤 첫 비교를 수행한다.
시험 대상 A/B 코드는 첫 비교에서 유지한다.

## Claude 검토와 남은 일

두 차례 검토는 전달한 기술 요약에 대한 의견이다.
Claude가 소스를 직접 확인한 결과는 아니다.
전체 비교 뒤 후보를 분리하는 순서는 합의했다.
지연·출력 수·공백·거부를 함께 평가하기로 했다.

이 문서의 새 대응 결과는 아직 전달하지 않았다.
UWB/MAVROS 경로 정정도 추가 검토 대상이다.
외부 시험기의 절차 검토와 구현이 다음 작업이다.
실기 배포는 보류한다.

앞선 수신 수정과 재생 결과는
[기존 검증 기록](btf_sensor_timing_20261014.md)에 있다.
