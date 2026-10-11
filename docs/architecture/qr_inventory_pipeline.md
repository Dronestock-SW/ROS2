# QR 재고 판독 파이프라인

QR 리더기 우선 판독과 카메라 폴백의 실행 경계를 정한 설계 문서다.
FSM·BT에서 선반 스캔을 연결할 때 읽는다.

## 결론

**실패 횟수는 노드 기동 시점이 아니라 스캔 세션 시작 뒤부터 센다.**

```text
선반 정렬 완료
      |
      v
/qr_fallback_node/set_enabled(true)
      |
      v
DE2110 리더기 대기 -- 성공 --> /qr/data 1건 --> 세션 종료
      |
      +-- 3초 무응답 x 3회
                    |
                    v
             카메라 판독 활성화
                    |
                    +-- 성공 --> /qr/data 1건 --> 세션 종료
```

## 실행 단위

`qr_inventory.launch.py`가 아래 노드를 한 번에 실행한다.

| 노드 | 책임 |
|---|---|
| `gscam_node` | IMX219 영상을 `/camera/image_raw`로 발행 |
| `qr_reader_node` | DYSCAN DE2110 문자열을 발행 |
| `qr_decoder_node` | 활성화됐을 때만 카메라 QR을 판독 |
| `qr_fallback_node` | 스캔 세션과 3회 실패 전환을 관리 |
| `qr_parser_node` | 재고 JSON 스키마를 검증 |
| `qr_recorder_node` | 확정 관측을 SQLite에 저장 |

카메라는 `start_camera:=true`일 때만 launch가 연다.
이미 다른 launch가 열었다면 반드시 `false`를 사용한다.
이유: CSI 카메라는 두 프로세스가 동시에 열 수 없다.

## 스캔 세션 계약

| 경계 | 값 | 동작 |
|---|---|---|
| `/qr_fallback_node/set_enabled` | `true` | 새 세션 시작·실패 횟수 초기화 |
| `/qr_fallback_node/set_enabled` | `false` | 세션 취소·카메라 비활성화 |
| `/qr_reader/data` | 문자열 | 활성 세션의 첫 결과만 수락 |
| `/qr_code/data` | 문자열 | 카메라 폴백 중 첫 결과만 수락 |
| `/qr/data` | 문자열 | 수락한 결과 한 건을 통합 발행 |

결과 한 건을 수락하면 세션을 자동으로 끝낸다.
FSM·BT는 다음 선반에서 서비스를 다시 호출한다.
비활성 세션의 입력은 저장 경로로 전달하지 않는다.

## 기본 설정

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `attempt_timeout_sec` | 3.0초 | 리더기 무응답 시간 창 |
| `fallback_threshold` | 3회 | 카메라 폴백 전환 기준 |
| `scan_active_on_start` | `false` | 부팅 즉시 실패를 세지 않음 |
| `enabled_on_start` | 통합 launch에서 `false` | 카메라 디코더 초기 상태 |

설정은 `params/qr_fallback.yaml`과 `params/qr_decoder.yaml`에 둔다.
