# 앵커 2.1m 배치 준비·전환
다음 앵커 배치의 준비 파일과 적용 절차다.
실제 설치 후 태그·companion 설정을 전환할 때 읽는다.

## 준비 상태

**2.1m 파일과 태그 빌드를 준비했고 운영본은 유지했다.**
사용자는 2026-10-16에 네 안테나 높이를 확인했다.
기준은 공통 바닥에서 UWB 안테나 중심까지다.
XY는 6.3×4.6m를 유지한다. 실제 이동은 아직 전이다.
독립 측량·거리 보정·위치 정확도 검증은 미실시다.

| 앵커 | x (m) | y (m) | 준비 z (m) |
|---|---:|---:|---:|
| A1 | 0 | 0 | 2.1 |
| A2 | 6.3 | 0 | 2.1 |
| A3 | 0 | 4.6 | 2.1 |
| A4 | 6.3 | 4.6 | 2.1 |

새 ID는 `warehouse-rectangle-6p3x4p6-z2p1-20261016`이다.
XY 원점은 A1의 바닥 투영점이다. Z=0은 공통 바닥이다.
X는 A1→A2, Y는 A1→A3 방향이다.

## 책임별 변경

**높이 좌표는 companion의 계산 설정에서 바꾼다.**

| 대상 | 준비한 변경 | 실물 적용 상태 |
|---|---|---|
| nRF52832 앵커 R2 | 높이 좌표를 저장하지 않아 소스 변경 없음 | 기존 R2 유지 |
| ESP32 Tag B | heartbeat 배치 ID·빌드 식별자 갱신 | 새 이미지 빌드, 미업로드 |
| UWB 수신기 | 새 앵커 JSON 선택 | 별도 준비본 |
| B_TF | 같은 ID와 네 z=2.1m | 별도 준비본 |
| 임무·지도 예제 | 새 배치 ID, 설치 확인 false | 로컬 소스만 갱신 |
| 기존 0.15m·2.2m 파일과 실측 로그 | 기록 당시 값·해시 보존 | 변경 없음 |

태그의 거리·RF 판본·50Hz·LoRa 1Hz는 그대로다.
거리 bias와 시각·품질 임계값도 그대로다.
`ground_antenna_height_m=0.15`는 태그의 지상 높이다.
운영 `tof_to_tag_body_flu_m=[0,0,0.15]`는 장착 간격이다.
두 값은 앵커 높이가 아니므로 바꾸지 않았다.
PX4 이륙 높이·고도 제어와 수평 LiDAR도 변경하지 않았다.

## 준비 파일

**현재 운영 설정의 복사본에서 배치만 바꿨다.**

소스 좌표는 [anchors_20261016_z2p1.json](../../src/drone_uwb/config/anchors/anchors_20261016_z2p1.json)이다.
수신 예제와 [B_TF 예제](../../src/drone_uwb/config/runtime/uwb_btf_tag_b.json)도 같은 좌표다.
일반 Tag B YAML·B_TF 예제의 구형 TDMA 선택은 유지했다.
실물 단일 태그에는 아래 운영 기반 준비본을 쓴다.
일반 예제를 운영 설정 위에 덮어쓰지 않는다.
이유: 운영본의 실측 장착값과 native backend를 보존해야 한다.

Jetson 준비 위치:
`/home/arialhanho/anchor-height-2p1-20261016/prepared/`

| 파일 | 용도 |
|---|---|
| `field-btf-cpp.json` | 실제 candidate-v4에서 만든 2.1m B_TF 설정 |
| `anchors_20261016_z2p1.json` | 수신기가 읽을 네 좌표 |
| `source-field-btf-cpp.json` | 변경 전 설정의 바이트 보존 사본 |
| `single_tag_observe.launch.py` | 현재 운영 launch 사본 |
| `run_observe.sh` | 새 설정과 기존 candidate-v4 소스를 연결 |
| `zzzzzzz-anchor-height-20261016.conf` | 설치 전 상태인 사용자 서비스 drop-in |
| `preparation.json` | 출처·전후 ID·파일별 SHA-256 |

drop-in은 systemd 설정 폴더에 설치하지 않았다.
서비스 daemon-reload·restart도 실행하지 않았다.
준비본의 `single_tag_layout_confirmed`는 false다.
`external_output_allowed`도 false를 유지한다.
현재 설정 파일 해시와 서비스 PID 37745가 같음을 확인했다.
비교 대상의 SHA-256은 다음과 같다.
`212700702fe941100810acc90d490faeca544d3c8fbe8b04136ee46c348baa5c`

재생성 도구는 [prepare_anchor_height.py](../../src/uwb_single_tag_firmware/tools/prepare_anchor_height.py)다.
기존 설정·같은 XY·새 배치 ID를 검사한다.
기존 출력 폴더는 덮어쓰지 않는다.

```bash
python3 prepare_anchor_height.py \
  --source-btf /home/arialhanho/esp32-single-tag-20261015/candidate-v4/field-btf-cpp.json \
  --anchors anchors_20261016_z2p1.json \
  --output /home/arialhanho/anchor-height-2p1-20261016/new-preparation
```

## 설치 후 전환

**실제 배치와 새 입력 경로를 확인한 뒤 적용한다.**

1. A1~A4 안테나 중심 높이·XY·순서를 현장에서 대조한다.
   설치 확인 기록을 남긴다. 준비 상태를 실측 완료로 바꾸지 않는다.
2. 최신 FC 지상·DISARM 상태와 현재 백업을 확인한다.
   [태그 기록·복구 절차](single_tag_release_and_recovery_20261015.md)를 따른다.
3. ESP32에는 새 50Hz 배포 이미지를 적용한다.
   버전은 `uwb-single-tag-v2.0-r3-z2p1-20261016`이다.
   새 전체 백업과 해시 검증을 먼저 수행한다.
   높이만 바꾸는 이번 전환에서 앵커 R2는 유지한다.
4. 준비 drop-in이 새 앵커 파일과 B_TF를 가리키는지 확인한다.
   기존 source와 native 라이브러리는 유지한다.
   적용 시에는 [A-cache 재시작](btf_acache_restart.md)을 따른다.
   실행 중인 프로세스는 파일 수정만으로 좌표를 다시 읽지 않는다.
5. 설치 기록과 좌표·RAW 상태를 대조한다.
   `single_tag_layout_confirmed`의 해제는 실제 배치 확인 후 수행한다.
   별도 PX4 출력·정렬·임무 확인을 한 번에 true로 바꾸지 않는다.
6. 새 heartbeat ID, 수신기 ID, B_TF 기하·캐시 해시를 확인한다.
   RAW 순번·유효 거리·ToF/IMU 최신성도 확인한다.
   지상 위치 결과와 FC 전송·융합은 각각 검증한다.

전환 취소 시 새 drop-in만 제거하고 기존 슬롯으로 돌아간다.
물리 배치와 설정을 함께 맞춰야 한다.
앵커가 이미 2.1m라면 0.15m 설정으로 위치를 발행하지 않는다.

## 검증 범위

**소스·합성 입력·준비 파일을 검사했으며 실물 적용은 미실시다.**

| 검사 | 결과 |
|---|---|
| 높이·배치 선택·schema 2·태그 계약 | 41개 통과 |
| 임무·지상 준비·정렬의 설치 확인 차단 | 28개 통과, ROS CLI 모듈 1개 건너뜀 |
| 전체 UWB 검사 | Windows의 Git 심볼릭링크가 일반 텍스트라 수집 중단, ROS launch 의존성도 없음 |
| ESP32 `tag_b_release` 빌드 | 성공, 앱 BIN 364,160 bytes |
| 2.1m 합성 거리 | 태그 z=0.85/1.3/2.4m에서 B4 XY와 오염 앵커 제외 B3 확인 |
| Jetson 준비 파일 | 7개 파일 해시 대조, bash 문법·launch 구문 검사 통과 |
| 운영 native 라이브러리와 새 설정 | 별도 Python 객체 생성 성공, C++ backend·캐시 builds=1·usable=true |
| 현재 운영본 보존 | 설정 SHA-256·PID·기동시각 동일 |
| 실물 설치·업로드·새 지상 관측·비행 | 미실시 |

앱 BIN SHA-256:
`6e18c31c37a2920deac2fe7604bf98c1b7d15076101beddb7a52de8f01470fb3`
빌드에는 기존 vendor의 GPIO 매크로 중복 경고가 남았다.
최종 이미지와 주소표는 작업 출력 폴더에 보관한다.
합성 계산 통과를 현장 위치 정확도 검증으로 해석하지 않는다.

새 설정으로 별도 생성한 캐시의 기하 SHA-256:
`45f59743a7d9752f2a3f6363c388fdca3ccaffd8343be2c91470c364426f4fe0`
이 검사는 ROS 노드나 운영 서비스를 기동하지 않았다.
