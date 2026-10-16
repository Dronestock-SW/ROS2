# 다른 태그 재구성을 위한 LoRa 이식 명세

2026-10-15 추출한 Tag B 펌웨어의 LoRa 동작과 재구성 방법이다.
다른 태그에 같은 기능을 구현하거나 같은 역할의 장치를 교체할 때 읽는다.

**현재 LoRa는 상태와 젯슨 좌표 요약을 전달한다. UWB SYNC가 있어야 송신한다.**

## 1. 적용 범위와 근거

| 항목 | 기준 |
|---|---|
| 추출 장치 | ESP32-S3, Tag B, ID `6` |
| 펌웨어 | `uwb-tag-tdma40-v0.4.1-lora-sync1hz-z015` |
| 기준 보드 | Heltec WiFi LoRa 32 V3, 내장 SX1262 |
| UWB | 외부 DWM3000 계열, 기존 TDMA40 구성 |
| 전체 플래시 | 8,388,608바이트 |
| 추출 실행 이미지 | `app0`, 주소 `0x10000`, 364,608바이트 |
| 근거 구분 | 버전·메시지 템플릿은 실제 추출본, 세부 로직은 대응 소스 |

추출 바이너리에서 `drone.status`의 유효 XY/무효 XY 템플릿을 확인했다.
세부 로직 설명은 동일 버전의 생성 소스에 근거한다.
소스 재빌드와 추출본의 바이트 일치 검사는 수행하지 않았다.
이 문서를 작성하면서 새 장치 시험이나 업로드는 하지 않았다.

## 2. 통신 구조

```text
A1 앵커 ── UWB SYNC ──> 태그의 UWB/LoRa 송신 일정
A1~A4  <── UWB 거리 측정 ──> ESP32 태그
                              │
                              ├─ USB JSONL: UWB RAW ──> 젯슨
                              │                         │
                              │                    위치 계산/필터
                              │                         │
                              <─ USB: jetson_pose ──────┘
                              │
                              └─ LoRa: drone.status ──> 지상국

지상국 ── LoRa 하향 패킷 ──> 태그 ── USB: lora_rx ──> 젯슨
```

- UWB RAW 전송 목표는 40 Hz다. LoRa 요약은 태그당 명목 1 Hz다.
- LoRa 좌표는 젯슨이 돌려준 결과다. 태그의 RAW XY를 자동 대입하지 않는다.
- 젯슨 좌표가 없어도 SYNC 조건을 만족하면 `fix:false`를 송신한다.
- UWB SYNC가 없으면 생존 신호도 송신하지 않는다.
- 하향 패킷은 USB로 전달한다. 이 브리지가 비행 명령을 실행하는 것은 아니다.
- 라이브러리 이름에 LoRaWAN이 들어가지만, 이 경로는 `Radio.Send`/`Radio.Rx`로 직접 패킷을 주고받는다. LoRaWAN 가입 절차를 사용하지 않는다.

## 3. 하드웨어와 무선 설정

### 보드 핀

아래 핀은 Heltec WiFi LoRa 32 V3 기준이다.
다른 ESP32/SX1262 보드라면 보드 정의와 실제 배선을 함께 바꿔야 한다.

| 장치 | 신호 | ESP32 GPIO |
|---|---|---:|
| SX1262 | NSS / SCK / MISO / MOSI | 8 / 9 / 11 / 10 |
| SX1262 | RESET / BUSY / DIO1 | 12 / 13 / 14 |
| UWB | CS / SCK / MISO / MOSI | 40 / 41 / 42 / 39 |
| UWB | IRQ / RESET | 48 / 47 |

코드는 하나의 Arduino `SPI` 객체를 전환해 두 장치를 사용한다.
전환 전에 반대 장치의 CS를 HIGH로 올린다.
`selectUwbSpi()`와 `selectLoRaSpi()`의 호출 순서를 유지한다.
보드 라이브러리의 TCXO·RF 스위치 처리도 함께 유지한다.

### 무선 호환 값

| 설정 | 값 |
|---|---|
| 주파수 | 915,000,000 Hz |
| 대역폭 | 125 kHz, 라이브러리 인수 `0` |
| 확산계수 | SF7 |
| 부호율 | 4/5, 라이브러리 인수 `1` |
| 프리앰블 | 8 symbols |
| 송신 출력 | 10 dBm |
| 헤더 / CRC | 가변 길이 / 사용 |
| IQ 반전 / 주파수 도약 | 사용 안 함 / 사용 안 함 |
| 네트워크 | private, `Radio.SetPublicNetwork(false)` |
| SX1262 sync word | `0x1424` |
| 최대 송신 JSON | 255바이트, 종료용 NUL은 무선에 보내지 않음 |

수신 지상국은 주파수·변조·sync word·패킷 형식을 맞춰야 한다.
위 값은 추출 대상의 설정이며, 새 장소의 운용 조건을 판정한 값은 아니다.

## 4. 태그 ID와 송신 시간표

**현재 구현은 ID 5와 6의 두 태그만 지원한다.**

| 역할 | 빌드 환경 | ID | LoRa 송신 회차 |
|---|---|---:|---|
| Tag A | `tag_a_tdma40` | 5 | `sf_seq % 40 == 0` |
| Tag B | `tag_b_tdma40` | 6 | `sf_seq % 40 == 20` |

공통 UWB 주기는 25 ms다. 40회차가 1초다.
A/B의 LoRa 예정 회차는 약 500 ms 간격으로 나뉜다.
두 태그는 A1 앵커의 같은 UWB SYNC를 기준으로 움직인다.

송신에는 다음 조건이 모두 필요하다.

1. SX1262가 준비됐고 이전 송신이 진행 중이지 않다.
2. 현재 UWB 회차 처리가 끝났고 유효한 SYNC가 등록됐다.
3. 태그에 배정된 회차이며 그 회차에서 아직 송신하지 않았다.
4. 동기화 재획득 대기 1초가 끝났다.
5. 패킷 구성과 SPI 전환 뒤에도 SYNC 기준 24,500 µs 이내다.

| 시간 상수 | 값 | 의미 |
|---|---:|---|
| UWB A/B 슬롯 시작 | 1,000 / 12,500 µs | SYNC 기준 |
| LoRa 송신 승인 마감 | 24,500 µs | 실제 RF 시작 시각과는 다름 |
| 시계 대응 표본 구간 상한 | 500 µs | 초과하면 SYNC 등록 무효 |
| SYNC 공백 기준 | 100,000 µs | 초과 시 재획득 필요 |
| 재획득 대기 | 1,000,000 µs | 부팅·복구·세션 변경·순번 순환 포함 |

SYNC 누락, 시간 초과, 송신 중이면 해당 기회를 건너뛴다.
늦은 재전송이나 비동기 대체 송신은 없다.
소스의 255바이트 airtime 상한은 399,616 µs다.
RF 시작 지연 5 ms는 시간 여유 계산의 가정이며 실측 보장이 아니다.

### 다른 태그에 적용하는 경우

| 목적 | 적용 방법 |
|---|---|
| 기존 Tag B 장치 교체 | 새 장치를 ID 6으로 빌드. 기존 ID 6과 동시 운용하지 않음 |
| 기존 Tag A 장치 교체 | 새 장치를 ID 5로 빌드. 기존 ID 5와 동시 운용하지 않음 |
| 세 번째 태그 추가 | ID 변경만으로 불가. UWB 슬롯·허용 ID·LoRa 배정·지상국 매핑까지 확장 필요 |
| UWB 없이 LoRa만 사용 | 현재 동작 그대로는 송신 불가. 별도의 송신 일정 설계 필요 |

`JetsonTagConfig.h`와 `UwbTdma40.h`가 ID를 5/6으로 제한한다.
`LoRaTelemetrySchedule.h`도 A/B 두 위상만 배정한다.
따라서 `UWB_TAG_NODE_ID=7`만 추가하면 재구성되지 않는다.

## 5. 젯슨과 태그 사이의 USB 계약

921600 baud, UTF-8 JSON Lines를 사용한다.
메시지는 한 줄에 하나이며 줄 끝에 `\n`을 붙인다.
아래 JSON은 형식 설명용 예시이며 실제 측정값이 아니다.

### 젯슨 → 태그: `jetson_pose`

```json
{"type":"jetson_pose","schema":1,"tag_id":"6","seq":120,"sample_time_us":8451000,"fix":true,"x_m":0.701,"y_m":1.803,"quality":92,"reason":"ok"}
```

| 필드 | 요구 사항 |
|---|---|
| `type`, `schema` | `jetson_pose`, 정수 `1` |
| `tag_id` | 펌웨어 ID와 같은 문자열. Tag B는 `"6"` |
| `seq` | uint32 |
| `sample_time_us` | uint64, 원본 표본 시각 |
| `fix` | JSON boolean |
| `x_m`, `y_m` | `fix:true`일 때 유한한 숫자, 단위 m |
| `quality` | 정수 0~100. 보정된 정확도나 AI 확률을 뜻하지 않음 |
| `reason` | 선택. 31자 이하의 단순 문자열, 이스케이프 사용 안 함 |

한 줄 내용은 255바이트 이내로 유지한다.
`reason` 생략 시 `fix`에 따라 `ok` 또는 `no_fix`를 쓴다.
잘못된 입력은 기존의 마지막 정상 좌표를 교체하지 않는다.

**500 ms는 태그가 메시지를 수신한 뒤의 경과 시간이다.**
현재 코드는 `receivedAtMs = millis()`로 신선도를 계산한다.
`sample_time_us`는 저장하지만 그 값으로 원본 표본의 나이를 검사하지 않는다.
따라서 젯슨은 오래된 좌표를 새 메시지로 반복 전송하지 않도록 처리해야 한다.
기존 파서는 `seq`의 증가 여부도 검사하지 않는다.

### 태그 → 젯슨: `jetson_pose_ack`

```json
{"type":"jetson_pose_ack","schema":1,"tag_id":"6","accepted":true,"pose_seq":120,"fix":true,"lora_eligible":true,"reason":"ok"}
```

입력이 틀리면 `accepted:false`, `pose_seq:null`, `reason:"invalid_packet"`을 보낸다.
이 ACK는 USB 입력 수락 결과다. LoRa 송신이나 지상국 수신 ACK가 아니다.
`lora_eligible:true`도 좌표 형식의 적합성을 뜻한다. SYNC 송신 조건은 별도다.

## 6. LoRa 패킷 계약

### 태그 → 지상국: `drone.status`

좌표가 유효하고 태그 수신 후 500 ms 이내인 경우:

```json
{"type":"drone.status","id":"6","seq":120,"t":8500,"fix":true,"x":0.701,"y":1.803,"q":92,"src":"jetson","uwb_seq":121,"reason":"ok"}
```

좌표가 없거나 오래된 경우:

```json
{"type":"drone.status","id":"6","seq":120,"t":9500,"fix":false,"x":null,"y":null,"q":0,"src":"none","uwb_seq":161,"reason":"jetson_pose_stale"}
```

| 필드 | 의미 |
|---|---|
| `id` | 무선 태그 ID. 웹 계정·인증 device ID와 별도 |
| `seq` | 수락한 젯슨 메시지가 있으면 그 순번. 한 번도 없으면 UWB 순번 |
| `t` | 태그 부팅 후 ms. UTC나 젯슨 시각이 아님 |
| `fix`, `x`, `y` | 좌표 유효 여부와 XY(m). 유효 좌표는 소수점 셋째 자리로 직렬화 |
| `q`, `src` | 젯슨 품질·출처. 무효이면 `0`, `none` |
| `uwb_seq` | 송신 처리 시 UWB 회차의 측정 순번 |
| `reason` | 젯슨이 준 사유 또는 `jetson_pose_stale` |

이 메시지에는 Z, 배터리, 자세, 앵커별 RAW 거리, LiDAR가 없다.
유효 XY도 별도 좌표 변환 없이 젯슨 값을 사용한다.
이 패킷 자체에는 애플리케이션 인증 필드가 없다.
실제 지상국 수신 여부는 이 태그의 송신 카운트로 알 수 없다.

### 지상국 → 태그 → 젯슨: `lora_rx`

```json
{"type":"lora_rx","schema":1,"tag_id":"6","rssi_dbm":-73,"snr_db":8,"payload":"command text"}
```

LoRa 수신 데이터와 RSSI/SNR을 USB로 전달한다.
출력에서 인쇄 불가능한 바이트는 `?`로 바꾼다.
이 브리지는 `payload`를 해석해 기체 동작을 실행하지 않는다.

## 7. 실행 순서와 복구 로직

다음 모듈을 함께 이식한다.

| 파일 | 책임 |
|---|---|
| `src/JetsonLoRaBridge.cpp` | SPI 전환, USB 입력, LoRa 패킷, IRQ·복구 |
| `include/JetsonLoRaBridge.h` | API와 진단 구조체 |
| `include/JetsonTagConfig.h` | 핀, ID, 무선·시간 설정 |
| `include/LoRaTelemetrySchedule.h` | SYNC 기반 1 Hz 송신 승인 |
| `include/LoRaRecoveryPolicy.h` | ms 순환을 고려한 watchdog |
| `src/main.cpp` | UWB 회차와 LoRa 서비스 호출 연결 |
| `../common/UwbTdma40.h` | UWB SYNC·ID·슬롯 공통 계약 |

초기화는 UWB 준비 후 `JetsonLoRaBridge::begin()`을 호출한다.
SX1262 sync-word 레지스터가 `0x1424`이면 준비 상태로 둔다.
이 레지스터 확인은 안테나나 무선 수신 확인을 뜻하지 않는다.

UWB 회차가 끝난 뒤의 처리 순서는 다음과 같다.

1. 다음 UWB SYNC를 받을 수 있게 DW3000 RX를 연다.
2. `serviceSerialInput()`으로 젯슨 입력을 처리한다.
3. 현재 UWB 시각을 ESP32 시각으로 대응시켜 `observeSync(...)`에 전달한다.
4. `service(cycleSequence, gUwbReady)`를 호출한다.
5. 내부에서 패킷 작성·SPI 전환 후 송신 마감을 다시 판정한다.

단순히 `delay(1000)` 뒤 `Radio.Send()`를 호출하면 같은 동작이 아니다.
UWB 처리 구간과 배정된 LoRa 송신 시점을 유지해야 한다.

| 항목 | 구현 값 |
|---|---:|
| 일반 LoRa 서비스 간격 | 100 ms, 송신 예정 회차는 앞당겨 처리 |
| 라이브러리 송신 timeout | 3,000 ms |
| 독립 watchdog | 3,500 ms |
| 준비 실패 시 재초기화 간격 | 3,000 ms |
| USB heartbeat 주기 | 5,000 ms |

`Radio.IrqProcess()`와 `Mcu.timerhandler()`를 모두 호출한다.
재초기화 전 `TxTimeoutTimer`, `RxTimeoutTimer`, `CadTimeoutTimer`를 중지한다.
이 처리를 빠뜨리면 기존 타이머가 남은 채 재초기화될 수 있다.
재초기화 후에는 다시 SYNC와 1초 대기를 거친다.

## 8. 그대로 빌드하는 방법

아래 명령은 재구성 절차다. 이번 문서 작성에서 실행하지 않았다.

### 가져갈 소스

현재 생성된 소스를 복사하면 준비 스크립트를 다시 실행할 필요가 없다.
다른 PC에는 아래 두 폴더를 **같은 부모 폴더 아래** 복사한다.

```text
lora-tag-rebuild/
├─ tag/       ← experiments/anchor-low015/generated/tag 전체
│  ├─ platformio.ini
│  ├─ src/
│  ├─ include/
│  ├─ lib/                 번들 라이브러리 포함
│  └─ scripts/             빌드 전후 스크립트 포함
└─ common/    ← experiments/anchor-low015/generated/common 전체
```

`.pio`는 이전 빌드 산출물이므로 복사할 필요가 없다.
`lib/`, `scripts/`, `common/`은 생략하면 안 된다.
헤더가 `../../common/...`을 참조하므로 두 폴더의 상대 위치를 유지한다.

로컬 원본:

- [생성된 tag 소스](/C:/Users/ASWPC_NTBOOK/Desktop/Drone5/.integration/UWB/experiments/anchor-low015/generated/tag)
- [생성된 common 소스](/C:/Users/ASWPC_NTBOOK/Desktop/Drone5/.integration/UWB/experiments/anchor-low015/generated/common)

### 도구와 빌드 설정

| 항목 | 사용 기준 |
|---|---|
| PlatformIO Core | 기존 재현 기록: 6.1.18 |
| Platform | `espressif32@7.0.1` |
| Board / Framework | `heltec_wifi_lora_32_V3` / `arduino` |
| Heltec 라이브러리 | 번들 `Heltec ESP32 Dev-Boards` 2.1.6 |
| UWB 라이브러리 | 번들 `heltec_dw3000_br101` 0.1.2 |
| 기준 플래시 | ESP32-S3, 8 MB, DIO, 80 MHz |

번들 `lib/`와 기존 `platformio.ini`를 함께 사용한다.
`RADIO_CHIP_SX1262`, `WIFI_LORA_32_V3`, `HELTEC_BOARD=30` 등 기존 빌드 플래그를 유지한다.

새 PC의 PowerShell에서 `lora-tag-rebuild` 폴더를 기준으로 실행한다.

```powershell
py -m pip install platformio==6.1.18

# 현재 Tag B 역할로 빌드
py -m platformio run -d tag -e tag_b_tdma40

# 업로드할 때 COM6을 새 태그의 실제 포트로 바꾼다.
py -m platformio run -d tag -e tag_b_tdma40 -t upload --upload-port COM6
```

Tag A로 대체하려면 환경을 `tag_a_tdma40`으로 선택한다.
이 생성 소스의 기본 환경은 B지만 A 환경 정의도 들어 있다.
한 장치에 A/B를 연속으로 업로드하는 절차가 아니다.

`z015`는 앵커 높이 0.15 m 프로필이다. 드론 비행 고도가 아니다.
같은 생성 소스로 A를 빌드해도 이 0.15 m 프로필을 사용한다.
실제 앵커 배치가 다르면 태그와 젯슨의 배치 설정을 함께 맞춰야 한다.

### 새 코드에 LoRa 부분만 옮기는 경우

7절의 모듈과 호출 순서를 옮긴다.
같은 무선 설정만 복사해서는 송신되지 않는다.
실제 A1 SYNC의 세션·순번·기준 시각을 `observeSync()`에 제공해야 한다.
가짜 SYNC나 로컬 타이머로 대체하면 기존 태그와의 교대 송신 계약이 깨진다.

### 바이너리 사용 범위

`full-flash.bin`은 원본 장치의 NVS 등도 포함한 보관본이다.
다른 역할의 태그를 만드는 기본 방법은 ID를 선택한 소스 빌드다.
`app0.bin`만으로는 새 칩의 부트로더·파티션 구성이 완성되지 않는다.
PlatformIO 업로드가 필요한 이미지와 주소를 함께 처리하도록 한다.
별도 업로더를 만들 경우 빌드 산출물의 `flash_layout_export.json`을 기준으로 한다.

## 9. 젯슨·지상국 연동 지점

- 젯슨은 대상 태그의 실제 USB 포트를 사용한다. `/dev/uwb` 별칭은 다른 장치에 자동 승계되지 않는다.
- `uwb_raw_status`에서 ID·버전·921600 baud·1 Hz LoRa 모드를 처리한다.
- 구버전 수신기는 1000 ms 주기를 거부할 수 있다. 1 Hz 계약을 지원하는 수신기를 사용한다.
- 젯슨 송신 `tag_id`는 빌드한 태그 ID와 같아야 한다.
- 같은 포트를 별도 프로세스로 중복 열지 않고 기존 수신기의 송신 경로로 `jetson_pose`를 보낸다.
- 지상국은 `drone.status.id`로 태그를 구분하고 `fix:false` 좌표를 유효값으로 사용하지 않는다.
- 태그가 동작 중이어도 UWB SYNC가 끊기면 LoRa 수신이 끊긴다. LoRa LOST만으로 전원 꺼짐을 단정하지 않는다.
- 펌웨어의 좌표 전송 기능과 웹에서 실제로 어떤 필드를 사용하는지는 별도다. 이 문서는 웹 매핑 변경을 포함하지 않는다.

## 10. 기존 읽기에서 관측한 값

다음 값은 2026-10-15 앞선 펌웨어 추출 작업의 기록이다.
다른 태그나 현재 시점의 상태를 뜻하지 않는다.

| 항목 | 읽기 전 | 읽기 후 재부팅 |
|---|---:|---:|
| `lora_ready` | true | true |
| `lora_sync_recent` / `lora_schedule_ready` | true / true | true / true |
| `lora_tx_attempts` | 3446 | 4 |
| `lora_tx_done` | 3446 | 3 |
| `lora_tx_busy` | false | true |
| `lora_tx_timeouts` | 0 | 0 |
| `lora_rx_packets` | 0 | 0 |

재부팅하면 카운터가 초기화된다.
읽기 후에는 송신 중인 순간을 기록해 attempts와 done이 1 차이 난다.
`tx_done`은 로컬 SX1262 완료 콜백이며 지상국 ACK가 아니다.
지상국의 자동 ACK가 없으므로 `rx_packets:0`만으로 상향 송신 실패를 판정하지 않는다.
새 태그 업로드, RF 수신·거리·좌표 정확도 시험은 이 문서의 완료 항목에 포함하지 않는다.

## 11. 원본과 참고 파일

이 문서와 같은 폴더의 보관 자료:

- [전체 플래시](full-flash.bin)
- [추출 실행 이미지](app0.bin)
- [바이너리 문자열·파티션 요약](firmware_summary.json)
- [읽기 전후 장치 메시지](result.json)
- [실제 플래시 읽기 로그](read_flash.log)

소스 기준:

- [LoRa 브리지](/C:/Users/ASWPC_NTBOOK/Desktop/Drone5/.integration/UWB/experiments/anchor-low015/generated/tag/src/JetsonLoRaBridge.cpp)
- [태그 설정](/C:/Users/ASWPC_NTBOOK/Desktop/Drone5/.integration/UWB/experiments/anchor-low015/generated/tag/include/JetsonTagConfig.h)
- [송신 스케줄](/C:/Users/ASWPC_NTBOOK/Desktop/Drone5/.integration/UWB/experiments/anchor-low015/generated/tag/include/LoRaTelemetrySchedule.h)
- [패킷 계약](/C:/Users/ASWPC_NTBOOK/Desktop/Drone5/.integration/UWB/experiments/lora-sync1hz/firmware/tag/PACKET_CONTRACT.md)
- [0.15 m 프로필 생성기](/C:/Users/ASWPC_NTBOOK/Desktop/Drone5/.integration/UWB/experiments/anchor-low015/prepare.py)

프로필 기록의 기준 저장소는 `ArialHanho/uwb-multitag-research`다.
기록된 상위 커밋은 `80dc0ff791f0120fcc2bfe4434e0864a9da447be`다.
이는 상위 기준 커밋이며, 그 값 하나만으로 로컬 생성물 전체의 동일성을 보장하지 않는다.
