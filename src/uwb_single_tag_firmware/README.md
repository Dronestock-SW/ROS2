# Tag B 단일 태그 펌웨어
Tag B ID 6과 nRF52832 앵커 A1~A4의 소스다.
빌드·기록·복구 전에 이 파일과 실행 기록을 읽는다.

현재 실물 상태는 [실행 기록](../../docs/report/esp32_single_tag_execution_20261015.md)에 있다.
메시지는 [통신 계약](../../docs/reference/single_tag_protocol_v2.md)을 따른다.
최초 실패 추가 필드는 [진단 계약](../../docs/reference/tag_failure_diagnostics_20261016.md)을 따른다.
로컬 검사와 미실시 실물 항목은 [진단 시험](../../docs/runbooks/tag_failure_diagnostics_tests_20261016.md)에 있다.
R5의 [재시도·복구 후속 구현](../../docs/report/tag_retry_recovery_20261016.md)은 실물 적용 전 코드다.
빌드 산출물과 전체 플래시 백업은 Git 밖에 보관한다.

## 활성 소스

| 경로 | 책임 |
|---|---|
| common/SingleTagProtocol.h | v2 RF 봉투·순번·시간 예산 |
| tag/src/main.cpp | DS-TWR·RF 시각·절대 주기 |
| tag/src/SingleTagRuntime.cpp | 제한된 USB 큐·시각 교환·pose 입력 |
| tag/src/SingleTagLoRa.cpp | 독립 LoRa 1Hz·IRQ·복구 |
| tag/src/JetsonTagDisplay.cpp | 별도 OLED 작업 |
| anchor/src/anchor_nrf52dk_ref_1to1.c | 주소별 공통 응답기 |
| tools/ | 원본 추출·백업·기록·검증·지상 수집 |
| tools/summarize_tag_diagnostics.py | 원본 JSONL의 최초·최종 실패와 진단 손실 집계 |

`platformio.ini`가 위 태그 파일만 선택한다.
원본의 비활성 TDMA·bridge 파일은 출처 보존용이다.
비활성 파일을 따로 빌드해 새 펌웨어로 쓰지 않는다.
새 단일 태그 구성과 호환되는 API를 보장하지 않기 때문이다.

## 빌드

**원본 radiofix3의 플랫폼과 vendor를 사용한다.**

| 항목 | 값 |
|---|---|
| PlatformIO Core | 6.1.19 |
| ESP32 platform | espressif32@7.0.1 |
| Nordic platform | nordicnrf52@10.12.0 |
| ESP32 보드 | heltec_wifi_lora_32_V3 |
| 앵커 보드 | nrf52_dk, nRF52832 |
| 원본 | UPSTREAM_SOURCE.json의 ZIP·파일 해시 |

```text
python -m platformio run -d src/uwb_single_tag_firmware/tag -e tag_b_50 -e tag_b_40
python -m platformio run -d src/uwb_single_tag_firmware/anchor -e anchor_a1_single -e anchor_a2_single -e anchor_a3_single -e anchor_a4_single
```

`tag_b_50`와 `tag_b_40`는 제한된 시험 입력을 포함한다.
`tag_b_release`와 `tag_b_release_40`는 이를 제외한다.
벤치 입력으로 flight 명령을 실행할 수 없다.

build 후 `flash_layout_export.json`을 사용한다.
부트로더·파티션표·boot_app0·앱 주소가 포함된다.
`stage_tag_images.py`가 이 표를 휴대 가능한 묶음으로 만든다.
기록 도구는 대상 MAC·8MB·보안 상태·해시를 검사한다.

## 원본 보존과 장치별 값

**원본 라이선스와 DW3000 드라이버 수정을 보존한다.**
UPSTREAM_SOURCE.json의 해시는 수정 전 추출 파일 기준이다.
원본 ZIP 전체를 저장소에 넣지 않는다.
Br101의 SYS_TIME 갱신·reset·SPI burst 경로를 유지한다.
새 UWB 경로만 전용 SPIClass(HSPI)를 선택한다.
Heltec SX1262는 기존 기본 SPI를 사용한다.

태그 TX/RX antenna delay는 각각 16385다.
이 값은 원본 radiofix3 소스에서 가져왔다.
이번 작업에서 측정한 보정값으로 표현하지 않는다.
태그는 과거 멀티태그 NVS를 읽어 동작하지 않는다.
앵커 배치와 거리 bias는 companion 설정에 둔다.

## 기록과 복구

**실물 백업과 호환 조합을 확보한 뒤 기록한다.**
`backup_tag.py`가 신선한 DISARM 지상 상태를 확인한다.
`flash_tag.py prepare`는 수신기를 중단한다.
대상 Tag B를 ROM 모드에 둬 기존 RF 송신을 멈춘다.
이후 A1~A4의 지정 J-Link에 새 이미지를 기록한다.
새 태그·companion을 적용하고 heartbeat를 확인한다.

J-Link 연결이 보호 해제를 겸할 수 있다.
기존 사용자 설정의 자동 해제 여부를 먼저 확인한다.
보호 해제는 플래시와 UICR을 지운다.
신규 readback과 원본 ZIP 이미지를 구분해 보관한다.

`prepare_receiver_overlay.py`는 운영 소스 사본에 적용한다.
기존 A-cache와 native B_TF 계약을 보존한다.
`configure_receiver_service.py`는 별도 슬롯을 가리킨다.
기존 실행 파일을 덮어쓰지 않는다.
실제 배치가 확인되기 전에는 위치 발행을 차단한다.

태그·앵커·수신기를 한 호환 조합으로 복구한다.
구형 앵커와 새 태그를 섞어 정상 상태로 인계하지 않는다.
RF 판본이 달라 거리 교환이 성립하지 않기 때문이다.

LoRa 지상국의 실제 수신·하향 시험은 보류 상태다.
TX-done·지상국 수신·PX4 융합을 서로 구분한다.
