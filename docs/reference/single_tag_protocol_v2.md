# 단일 태그 통신 계약 v2
Tag B·앵커·companion의 새 통신 규격이다.
수신기 구현과 저장된 RAW를 해석할 때 읽는다.

## 책임과 판본

**원본 측정 시각은 companion에서 한 번만 변환한다.**

```text
A1~A4 공통 응답기
    ↕ POLL → RESPONSE → FINAL → REPORT
Tag B: DS-TWR 거리 + RF 시각 + 완결 RAW
    ↕ USB RAW / 왕복 시각 교환 / 최신 pose
단일 serial 소유자: 계약 검사 + 시각 변환
    → /uwb/received → 기존 B_TF → /uwb/tag_pose_feedback
    ← 발행된 유효 결과의 원본 시각·순번
Tag B: 독립 LoRa 1Hz → 지상국
```

| 항목 | 값 |
|---|---|
| 태그 | Tag B, 문자열 ID `6` |
| 앵커 주소 | A1=`0xA1` ~ A4=`0xA4` |
| 무선 판본 | 2 |
| USB JSON schema | 2 |
| USB 속도 | 921600 baud |
| 태그 시각 | `esp32_monotonic_boot_us` |
| boot_id | MCU 부팅마다 난수 32비트, 0 제외 |
| session | UWB 초기화마다 난수 32비트, 0 제외 |
| host_session | serial 연결마다 companion 난수 |
| clock_generation | 시각 대응 재설정마다 새 식별자 |

`session`은 A1 동기 세션이 아니다.
Tag B의 UWB 초기화 세션이다.
`sf_seq`와 `schedule_id`를 생성하지 않는다.
구형 TDMA 응답을 새 관측으로 수용하지 않는다.

## RF 교환

**DS-TWR의 네 메시지와 실제 RF 시각을 유지한다.**

| 바이트 | 내용 |
|---|---|
| 0~1 | `41 88` 프레임 제어 |
| 2 | transaction 하위 8비트 |
| 3~4 | PAN `0xDECA`, little endian |
| 5~6 | 목적 주소 |
| 7~8 | 발신 주소 |
| 9 | POLL D0 / RESP D1 / FINAL D2 / REPORT D3 |
| 10 | 판본 2 |
| 11~14 | session 32비트 |
| 15~18 | transaction 32비트 |
| 19~30 | REPORT의 POLL-RX / RESP-TX / FINAL-RX |
| 31 | REPORT 성공 코드 1 |

앞의 세 메시지는 FCS 제외 19바이트다.
REPORT는 FCS 제외 32바이트다.
길이·주소·PAN·판본·세션·거래 순번을 검사한다.
응답기는 중복·역순 거래를 거부한다.
최근 종료한 세션 네 개도 거부한다.
새 세션은 마지막 승인 후 100ms 뒤 수용한다.
이는 신선도 검사이며 RF 인증 기능은 아니다.

| 시간 예산 | 값 |
|---|---|
| 주기 후보 | 20ms / 25ms |
| 개별 태그 교환 상한 | 3ms |
| 앵커 응답 종료 상한 | POLL 수신 후 2.3ms |
| 최초 네 시도 | 모두 먼저 수행, 명목 최대 12ms |
| 재시도 | 남은 예산에서 최대 한 앵커, 한 번 |
| 정리 여유 | 0.2ms |
| cycle 완료 목표 | 18ms |

주기는 절대 단조 시각을 기준으로 잡는다.
지연된 회차를 몰아서 실행하지 않는다.
누락한 회차는 seq와 skipped_cycles에 반영한다.

## 완결 RAW

**실패 거리와 시각은 null이며 이전 값을 복사하지 않는다.**

| 필드 | 의미 |
|---|---|
| seq | cycle 순번, uint32 순환 |
| cycle_start_us / cycle_end_us | 실제 회차 경계 |
| raw_slant_m[4] | A1~A4 원본 사선거리 |
| sample_time_us[4] | 각 거리의 RF 대표 시각 |
| valid_mask | 유효 거리 비트 0~3 |
| failure[4] | 아래 숫자 코드 |
| attempts[4] | 0 / 1 / 2 |
| rf_map_span_us[4] | RF→MCU 시각 읽기 구간 폭 |
| clock_sync_valid | 왕복 시각 대응 승인 유효 여부 |
| clock_generation | 승인한 대응 식별자 |
| deadline_missed | 18ms 완료 목표 초과 |
| skipped_cycles / tx_dropped | 회차 누락 / 대기 RAW 교체 누적 |

| 코드 | 의미 | 코드 | 의미 |
|---|---|---|---|
| 0 | 성공 | 7 | FINAL 송신 시간초과 |
| 1 | UWB 준비 안 됨 | 8 | REPORT 시간초과 |
| 2 | POLL 시작 실패 | 9 | REPORT 검증 실패 |
| 3 | POLL 송신 시간초과 | 10 | ToF 계산 조건 실패 |
| 4 | RESPONSE 시간초과 | 11 | 거리 범위 실패 |
| 5 | RESPONSE 검증 실패 | 12 | 시간 예산 부족 |
| 6 | FINAL 시작 실패 | 13 | RF 시각 대응 실패 |

RF 대표 시각은 POLL-TX와 FINAL-TX의 중간이다.
갱신한 DW SYS_TIME 읽기를 MCU 시각으로 감싼다.
그 중간 시각과 RF 차이를 이용한다.
읽기 구간이 500µs를 넘으면 거리를 무효화한다.
대표 시각이 실제 교환 밖이면 역시 거부한다.
DS-TWR 저위 32비트와 SYS_TIME 순환을 따로 처리한다.

## 왕복 시각 교환

**USB 수신 시각을 측정 시각으로 대신 쓰지 않는다.**

| 시각 | 기록 위치 |
|---|---|
| t1 | companion이 요청 전송을 시작할 때 |
| t2 | 태그가 완결 요청을 파싱할 때 |
| t3 | 태그 UART가 비워진 뒤 응답을 만들 때 |
| t4 | companion이 완결 응답을 수신할 때 |

요청은 `time_sync`다.
응답은 `time_sync_reply`다.
승인은 `time_sync_commit`이다.
세 메시지 모두 태그·부팅·세션을 검사한다.
host_session·seq·원본 t1도 대조한다.

왕복 잔여 지연은 다음과 같다.

```text
RTT = (t4 - t1) - (t3 - t2)
tag_mid = (t2 + t3) / 2
host_mid = (t1 + t4) / 2
host_us = host_anchor + alpha * (tag_us - tag_anchor)
```

RTT 0~12ms의 교환만 적합에 사용한다.
최대 120개·최근 30초를 보관한다.
5초 이상 자료에서 기울기를 추정한다.
허용 기울기는 0.999~1.001이다.
각 왕복이 허용하는 offset 구간을 교차한다.
교집합 중간을 대응점으로 사용한다.
반폭에 RF 읽기 여유 500µs를 더해 보고한다.
허용 불확실성은 5ms다.
이 값은 USB 비대칭의 검교정 결과가 아니다.

최소 네 교환이 있어야 준비 상태가 된다.
마지막 유효 교환에서 2초 뒤 만료된다.
태그 승인도 t3에서 2초 뒤 만료된다.
중복 승인은 만료를 연장하지 않는다.
연결·부팅·UWB 세션이 바뀌면 대응을 버린다.

변환 결과는 `/uwb/received`에 붙인다.
원본 `message`의 시각은 그대로 남긴다.
`single_tag.sample_host_monotonic_ns`가 변환 결과다.
B_TF는 이 값을 그대로 사용한다.
구형 `ClockMap.update()`를 호출하지 않는다.
원본 나이가 150ms를 넘으면 수용하지 않는다.
미래 시각을 수신 시각으로 잘라 맞추지 않는다.

## 출력과 LoRa

**LoRa는 UWB 회차와 무관하게 1Hz로 동작한다.**

| 설정 | 값 |
|---|---|
| RF | 915MHz / BW125kHz / SF7 / CR4/5 |
| 출력 | 10dBm |
| 네트워크 | private sync `0x1424`, CRC |
| 길이 | 최대 255바이트, 종료 NUL 제외 |
| 업링크 | 기존 `drone.status` JSON |
| 하향 | USB `lora_rx`, 원본 payload_hex·길이·RSSI·SNR |
| SPI | UWB 전용 HSPI, LoRa 전용 기본 SPI |
| 포트 소유자 | companion의 UWB 수신 노드 한 개 |

진행 중인 USB 줄은 끝까지 보낸다.
대기 RAW는 최신 완결 회차 하나만 유지한다.
보조 메시지 큐는 네 개로 제한한다.
시각 요청은 태그에서 최대 10Hz로 제한한다.
companion의 정상 요청은 4Hz다.

pose는 `/uwb/tag_pose_feedback`의 발행 결과만 사용한다.
큰 진단 메시지와 짧은 pose 피드백을 분리한다.
직렬 포트 소유자는 전용 수신 스레드를 쓴다.
ROS 타이머에 1ms 폴링을 맡기지 않는다.
최근 RAW의 boot/session/seq/원본 시각을 대조한다.
태그도 원본 시각에서 500ms 이내인지 검사한다.
quality는 해당 RAW의 유효 앵커 수 × 25다.
정확도나 보정된 확률을 뜻하지 않는다.
동일 시각·역순 pose는 거부한다.
반복 전송으로 만료 시각이 연장되지 않는다.
만료 시 fix=false와 null 좌표를 보낸다.
앵커 배치가 확인되지 않은 시험은 발행을 차단한다.
`single_tag_layout_confirmed=true`는 실제 배치 확인 뒤 설정한다.

LoRa TX-done은 지상국 수신 증거가 아니다.
실제 지상국 시험은 [별도 절차](../runbooks/esp32_tag_b_rebuild_execution_prompt_20261015.md#10-다른-장소에서-진행할-lora-시험-안내)를 따른다.

## 앵커 SPI 오류 복구

**앵커는 SPI 오류를 확인하면 무선부를 재초기화한다.**
원본 드라이버의 2ms DMA 시간 제한을 유지한다.
오류 래치는 `spi_transfer_faulted()`로 조회한다.
시간 초과 대기와 새 수신 시작 전에 검사한다.
`openspi()`와 DW3000 초기화를 다시 수행한다.
실패한 레지스터 읽기의 출력은 0으로 지운다.
과거 데이터가 성공 이벤트로 해석되는 일을 막는다.
수락한 세션·거래 순번 이력은 초기화하지 않는다.
복구 중인 앵커 외에는 독립적으로 거리 교환을 계속한다.
