# 멀티태그·companion·웹 연결 기준
센서 처리와 웹 전송의 책임을 정한 기준이다.
코드·설정을 연결하거나 표시값을 해석할 때 읽는다.

## 연결 경로

ROS는 드론 탑재 companion에서 실행한다.
LoRa 지상국 USB는 별도 로컬 웹 PC에 연결한다.

```text
Tag A --USB 921600--> companion A (ROS 도메인 1) --Wi-Fi/WS--+
Tag B --USB 921600--> companion B (ROS 도메인 2) --Wi-Fi/WS--+--> 웹

Tag A/B --LoRa 1Hz--> 지상국 --USB 115200--> 로컬 웹 PC --> 웹
```

각 companion은 자기 Tag 하나만 받는다.
웹 좌표 전송은 `jetson_pose` 역송신을 요구하지 않는다.
두 경로의 연결 상태와 수신률은 별도로 센다.
LoRa 패킷으로 Wi-Fi 좌표의 신선도를 갱신하지 않는다.
이유: 두 값의 측정 시각과 출처가 다르다.

## TDMA 수신

새 A/B 설정은 `tdma_mode=required`를 사용한다.

| 항목 | 계약 |
|---|---|
| Tag ID | A=`"5"`, B=`"6"` |
| 주기/스케줄 | 25ms, `0x4001` |
| 대응 | RAW 다음 TDMA 진단, 같은 tag/seq, 수신 간격 0~50ms |
| 슬롯 | A 1000~11700us 미만, B 12500~23200us 미만 |
| 유효성 | 4/4 거리, deadline_valid, local_overrun=false, sample span 일치 |
| 손실 | 누락·중복·역순·log_drops 증가에서 해당 관측 제외 |
| 재시작 | 부트/세션 전환 때 계산 이력 초기화. 최근 종료 세션 32개 거절 |
| 원본 시각 | TDMA 수신 시각 대신 RAW의 최초 수신 시각 사용 |
| 보조 메시지 | TDMA·pose ACK·LoRa RX 정상 형식은 거리 이력을 훼손하지 않음 |

기존 일반 설정의 `auto`는 구형 RAW를 허용한다.
새 멀티태그 실측에서는 A/B strict 설정을 선택한다.
이유: 구형 호환과 TDMA 검증 완료의 의미가 다르다.

## 좌표 출처

웹 기본 입력은 B_TF XY 관측이다.
비행 상태추정은 계속 PX4 EKF2가 담당한다.

| 설정 | 토픽/형식 | 좌표계·기준점 | Z |
|---|---|---|---|
| `uwb_xy` | `/uwb_pose`, PoseWithCovarianceStamped | uwb_map·안테나 | null |
| `btf_xy` | `/uwb/btf_pose`, PoseWithCovarianceStamped | uwb_map·안테나 | null |
| `px4_local` | `/mavros/local_position/pose`, PoseStamped | 입력 map, 전송 px4_local_enu·FC | 같은 메시지의 Z |

PX4 로컬 XYZ는 창고 지도에 투영하지 않는다.
이유: 원점·축·안테나와 FC 기준 정렬이 미완료다.
`xyz_valid`는 메시지의 신선한 XYZ 유무다.
외부 UWB 융합 성공이나 위치 정확도를 보장하지 않는다.
XY Pose의 z=0 자리값은 실제 높이로 보내지 않는다.

앵커 기존 설정은 6.3×4.6m, 높이 2.2m다.
실제 배치를 이 값과 대조해야 한다.
Tag A의 기존 ToF→안테나 12cm 설정을 보존했다.
Tag B의 장착·바닥 확인은 false다.
이번 작업에서 두 기체의 장착값을 실측하지 않았다.

## 식별자와 시간

Tag 주소·ROS 도메인·웹 ID는 별도 식별자다.

| 항목 | A | B |
|---|---|---|
| Tag ID | 5 | 6 |
| ROS_DOMAIN_ID | 1 | 2 |
| 웹 ID | 서버에 등록한 ID | 다른 서버 등록 ID |
| 인증 ID | A용 HOST_OBSERVE 바인딩 | B용 HOST_OBSERVE 바인딩 |

원본 나이는 수신할 때 0으로 되돌리지 않는다.

| 필드/검사 | 의미 |
|---|---|
| `source_stamp_ns` | 원본 ROS 관측 시각 |
| `source_received_at_ms` | companion이 원본을 받은 ROS 시각 |
| 송신 `source_age_ms` | 최초 원본 나이 + companion 단조시계 경과 |
| `sent_at_ms` | WebSocket 송신 UTC |
| `server_received_at_ms` | 웹 수신 UTC |
| 웹 `source_age_ms` | 원본 나이 + 전송 지연 추정 + 시계 여유 250ms + 경과 |
| 좌표 만료 | 원본 나이 상한 500ms |
| 연결 만료 | 웹 수신 후 2000ms |

운용 PC와 companion의 UTC를 동기화한다.
허용 시계 차이는 250ms다.
웹은 송신 미래 250ms 초과·전송 2000ms 초과를 거절한다.
이는 검증한 시계 오차의 실측값이 아니라 설정 한계다.
ROS 시뮬레이션 시계와 UTC를 섞지 않는다.
이유: 원본 나이의 의미가 사라진다.

## 웹 인증과 수신

관측 WS는 기존 명령 계약과 별도 인증 분기를 쓴다.

| 항목 | 값 |
|---|---|
| 경로 | `/ws/drones/{id}/` |
| 업그레이드 헤더 | `X-DS-Observation-Version: 1.0` |
| 인증 | 기존 5줄 HMAC 서명, 빈 GET 본문 SHA256 |
| 기체 확인 | 서버 인증 ID→기체 ID 바인딩 |
| 패킷 계약 | `observation_contract="1.0"` |
| 순서 | companion_session_id + telemetry_seq |
| 수락 | `observation.ack`, accepted=true |
| 실패 | 거절 ACK, 송신기는 연결 재시도 |
| 기본 전송 목표 | 10Hz. 새 UWB/B_TF 관측률과 별도 |
| 임무 조회 | 기본 false. 구형 API가 필요한 경우에만 명시 설정 |
| 비행 권한 | 항상 false |

필드 검증·중복 검사·보관은 웹의 `observations.py`가 맡는다.
LoRa 수신은 기존 지상국 브리지를 사용한다.
현 검증 결과는 [패치 기록](../report/multitag_patch_validation_20261007.md)에 있다.
