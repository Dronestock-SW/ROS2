# 단일 태그 릴리스·복구 절차
Tag B와 A1~A4, companion의 기록·복구 절차다.
실물 변경 전 [실행 기록](../report/esp32_single_tag_execution_20261015.md)과 함께 읽는다.

## 호환 조합 선택

**태그·앵커·companion을 한 조합으로 적용한다.**

| 조합 | 태그 | 앵커 | companion |
|---|---|---|---|
| 새 단일 태그 | R3 / RF v2 / schema 2 | SINGLE2-R2 / RF v2 | candidate-v4 schema 2 overlay |
| 원본 복구 | 업로드 전 8MB 전체 백업 | 원본 ZIP의 역할별 radiofix3 | 기존 A-cache 슬롯·시작 파일 |

구형 태그와 새 앵커는 메시지 판본이 다르다.
혼합 상태의 거리 측정을 정상으로 쓰지 않는다.
새 조합은 실제 배치 확인 전 위치 발행을 차단한다.
이 차단을 RAW 통신 성공만으로 해제하지 않는다.

## 대상과 백업 확인

**실제 연결·MAC·지상 상태부터 다시 확인한다.**

| 역할 | 이 실행에서 확인한 식별 |
|---|---|
| Tag B | ID 6, MAC `e8:3d:c1:e3:48:2c`, ESP32-S3, 8MB |
| 태그 USB | companion `/dev/uwb` → `/dev/ttyUSB1` |
| A1 | nRF52832, J-Link `1050323475` |
| A2 | nRF52832, J-Link `1050337085` |
| A3 | nRF52832, J-Link `1050376327` |
| A4 | nRF52832, J-Link `1050326715` |
| 수신 서비스 | arialhanho / `dronestock-manual-observe.service` |

위 표는 이번 실행의 확인값이다.
다른 PC의 COM 번호를 그대로 대입하지 않는다.
태그와 PX4 포트를 MAC·장치 종류로 구분한다.

원격 작업 루트는 다음 경로다.
`/home/arialhanho/esp32-single-tag-20261015/`

원본 태그 백업은 다음 파일이다.
`backup/tag-20261015-213115/full-flash-before.bin`

크기는 8,388,608바이트다.
SHA-256은 다음과 같다.
`ea9851c4bf1d07b659a7c59460381401916669d65423c11566e985a0e64a9a39`

원본 수신 소스·설정 묶음도 PC에 보관했다.
파일명은 `backup-with-source.tar.gz`다.
SHA-256은 다음과 같다.
`93721d06b0be36320fc95e96882f410ba4f610373daf001e8130b91df825b2a8`

A1은 최초 연결 때 삭제 전 실물 백업을 잃었다.
A2·A4는 보호 때문에 사전 읽기가 불가능했다.
이 세 장치의 복구는 원본 ZIP 이미지에 근거한다.
삭제 전 미기록 설정까지 복원된다고 주장하지 않는다.

## 새 태그 전체 기록

**해시가 맞는 주소표의 이미지 네 개를 기록한다.**

1. ROS 환경에서 `backup_tag.ground_snapshot()`을 실행한다.
   신선한 connected·DISARM·landed_state=1이 필요하다.
2. `flash_tag.py prepare --output <새 증거 폴더>`를 실행한다.
   15분 복구 타이머와 수신기 중단을 확인한다.
   대상 MAC·8MB가 다르면 후속 기록을 중단한다.
3. 최종 초기화는 다음 명령을 사용한다.
   `flash_tag.py write --erase --images <검증한 이미지 폴더> --output <같은 증거 폴더>`
   도구가 직전 전체 플래시를 새 파일로 백업한다.
   이어 전체 삭제와 네 이미지 기록·대조를 수행한다.
4. 서비스 시작 뒤 새 heartbeat를 확인한다.
   boot/session·주기·시각 동기·UWB·LoRa를 대조한다.
   배포용 판본은 `bench_controls=false`여야 한다.
5. 정상 수신을 확인한 뒤 복구 타이머를 해제한다.
   실제 전원 재인가와 정상 10분 기록을 추가한다.
   원격 reset을 실제 전원 차단으로 표기하지 않는다.

| 주소 | 이미지 |
|---|---|
| 0x0000 | bootloader.bin |
| 0x8000 | partitions.bin |
| 0xe000 | boot_app0.bin |
| 0x10000 | firmware.bin |

기록 도구는 eFuse를 변경하지 않는다.
과거 멀티태그 NVS를 새 릴리스에 복원하지 않는다.
TX/RX antenna delay 16385는 원본 소스에서 이관했다.
이 값은 새 실측 보정 결과가 아니다.

## 앵커 기록과 오류 복구

**역할별 이미지를 지정 J-Link에 기록한다.**
기록 전 읽기 가능 여부와 보호 상태를 확인한다.
J-Link의 저장된 Nordic 자동 잠금 해제를 확인한다.
보호 해제는 flash·UICR을 지울 수 있기 때문이다.
이번 작업 후 자동 잠금 해제 설정은 제거했다.

빌드 환경은 `anchor_a1_single`부터 `anchor_a4_single`이다.
각 역할의 HEX를 기록한 뒤 512KB를 읽는다.
`verify_anchor_readback.py`로 모든 HEX 바이트를 대조한다.
앵커 R2는 SPI DMA 오류를 감지해 자체 복구한다.
RF 판본과 주소는 네 역할에서 동일 규칙을 따른다.

## 원본 조합으로 복구

**원본의 제한을 보존한 상태로 함께 복구한다.**

1. 지상 상태를 확인하고 수신기를 멈춘다.
   태그를 ROM 모드에 둬 RF 송신을 멈춘다.
   기존 이미지·백업 해시를 먼저 대조한다.
2. A1~A4에 원본 ZIP의 역할별 HEX를 기록한다.
   이미지 바이트와 실물 readback을 대조한다.
3. Tag B의 원본 전체 백업을 주소 0에 기록한다.
   이 복구에만 원본 NVS도 함께 돌아간다.
   `verify_flash 0 full-flash-before.bin` 결과를 확인한다.
4. 이번 전용 서비스 drop-in만 제거한다.
   파일명은 `zzzzzz-single-tag-20261015.conf`다.
   다른 기존 drop-in과 시작 파일은 유지한다.
5. daemon-reload 후 기존 서비스를 시작한다.
   v0.4.1 원본 heartbeat·TDMA·새 RAW를 확인한다.
   확인 후 한시적 복구 타이머를 해제한다.

## 보류한 현장 확인

**지상국 수신과 위치 성능을 따로 확인한다.**
[LoRa 현장 시험](esp32_tag_b_rebuild_execution_prompt_20261015.md#10-다른-장소에서-진행할-lora-시험-안내)을 따른다.
실제 앵커 좌표를 확인한 뒤 companion 배치를 맞춘다.
그 후 새 유효 위치율·오차를 측정한다.
PX4 관측 송신·수용·융합은 별도 증거가 필요하다.
이 절차에는 비행·arming·모터 구동을 포함하지 않는다.

## LoRa 최대 길이 시험 실행

**지상국을 연결한 장소에서만 다음 절차를 수행한다.**
이번 실행에서는 이 실물 시험을 보류했다.
태그 TX-done만으로 수신 성공을 판정하지 않는다.

1. 현재 릴리스·전체 백업·서비스 상태를 보관한다.
   위 전체 기록 절차로 `tag_b_50` 시험판을 적용한다.
   이 판본은 `bench_controls=true`다.
   `tag_b_release`는 시험 명령을 허용하지 않는다.
2. 지상 상태와 실제 지상국 설정을 확인한다.
   위 연결 문서의 915MHz 무선 설정을 사용한다.
   지상국 원본 수신 바이트와 시각을 기록한다.
3. 수신 서비스와 직접 UART 도구의 점유를 분리한다.
   한시적 서비스 복구 타이머를 먼저 준비한다.
   서비스를 중단한 뒤 다음 도구를 실행한다.

```bash
# TASK_ROOT와 TOOLS는 실제 복사한 경로로 설정한다.
# ROS 환경과 candidate-v4 패키지의 PYTHONPATH가 필요하다.
python3 "$TOOLS/capture_single_tag.py" \
  --port /dev/uwb --seconds 30 \
  --lora-maxlen-probe --output "$TASK_ROOT/lora-maxlen-field"
```

도구는 heartbeat와 시각 동기를 기다린다.
현재 boot/session으로 시험 명령을 한 번 보낸다.
합성 pose나 비행 명령은 보내지 않는다.
태그는 다음 1Hz 송신 기회에 255바이트를 보낸다.
최초 15바이트는 ASCII `SINGLE2_MAXLEN:`이다.
나머지 인덱스 i=15..254는 ASCII 숫자 `i % 10`이다.
종료 NUL은 무선 payload에 포함하지 않는다.

4. 지상국의 255바이트를 위 규칙과 대조한다.
   태그의 길이·완료 카운터와 RAW 공백도 기록한다.
   `summary.json` 자체는 지상국 수신 증거가 아니다.
5. 1Hz 정상 상태 송신 중 고유 문자열을 하향 송신한다.
   `lora_rx.length`와 `payload_hex`를 대조한다.
   `payload`는 출력용이며 비ASCII를 `?`로 표시한다.
   원본 바이트 검증에는 `payload_hex`를 사용한다.
   RSSI·SNR과 TX 뒤 RX 복귀 결과를 보관한다.
6. 시험판에서 검증한 배포용 이미지를 다시 적용한다.
   서비스를 시작하고 새 heartbeat를 확인한다.
   확인 후 복구 타이머를 해제한다.
   `bench_controls=false`를 다시 확인한다.
