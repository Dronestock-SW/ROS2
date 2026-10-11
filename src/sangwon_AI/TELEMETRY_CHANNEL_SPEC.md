# LoRa 생존 확인과 Jetson Wi-Fi 좌표 보고

기준: 2026-10-04 사용자 최종 정정. 이전 대화의 LoRa 좌표 전송 추정은 폐기한다.

## 통신 경로

```text
UWB 태그 -- LoRa 생존 신호 --> 지상국 --> 웹 [드론 생존 여부]

UWB 장치 -- 사양 수령 후 확정할 인터페이스 --> Jetson
                                            ├─ 측량/검증된 지도 좌표 변환
                                            └─ Wi-Fi --> 웹 [좌표 시각화]

웹 -- Wi-Fi 임무/지도/제어 요청 --> Jetson
웹 <-- Wi-Fi 점검/수락/실행 결과 -- Jetson
```

LoRa는 좌표를 전달하지 않는다. Jetson이 UWB를 받는 물리 포트·프로토콜은 아직 미정이다.
LoRa 생존 신호만으로 Wi-Fi 연결·위치 유효성·PX4 연결·준비 완료를 추정하지 않는다.
웹 로그인은 운영자 인증이고, Jetson HTTP/WS 자동 연결은 등록된 장치 인증으로 한다.
관리자 비밀번호를 Jetson 서비스에 저장하지 않는다.

## 웹 표시와 좌표 필드

| 정보 | 원천 / 계약 | 표시 원칙 |
|---|---|---|
| 드론 생존 | 지상국이 받은 LoRa heartbeat | 수신 시각·만료를 별도 표시. 좌표 갱신 시각으로 사용하지 않음 |
| Jetson 연결 | Wi-Fi 장치 세션/보고 | LoRa와 독립 배지 |
| UWB 원천 관측 | telemetry.uwb_observation | 실제 신규 관측·원천 시각·품질. 값이 같아도 신규 측정인지 따로 판단 |
| UWB 변환 좌표 | telemetry.uwb_map_position (이번 확장 후보) | Jetson이 변환한 WAREHOUSE_MAP 좌표. 변환 ID/개정 및 유효성 확인 |
| PX4 융합 위치 | telemetry.pose_fused | PX4 EKF 위치를 지도 좌표계로 변환한 별도 결과. UWB 변환만으로 융합 완료라고 표시하지 않음 |

`uwb_map_position`은 기존 draft.4에 새로 추가한 후보다. 서버 저장·WS 전달·UI 수락 시험은 아직 필요하다.
기존 flat x/y 필드로 묵시 변환하거나 LoRa 좌표로 가장하지 않는다.
웹은 UWB 변환 좌표와 PX4 융합 위치의 출처를 구분한다. 둘 중 무엇을 지도 아이콘으로 표시하는지 명시한다.

```json
{
  "position_transport": "JETSON_WIFI",
  "uwb_map_position": {
    "source": "JETSON_UWB_MAP_TRANSFORM",
    "frame": "WAREHOUSE_MAP",
    "valid": false,
    "position_m": null,
    "observed_at": null,
    "source_age_ms": null,
    "transform_ref": null,
    "reason_code": "UWB_SPEC_PENDING"
  }
}
```

위 값은 현재 개발 Jetson이 C++에서 생성하는 미수신 상태다. 0,0,0을 실제 위치로 보내지 않는다.
UWB 사양 수령 후에는 원천 frame/unit·새 관측·시각·축/측량·변환 참조를 검증하고 실제 좌표를 채운다.
z가 관측되지 않으면 웹 목표 z를 현재 고도로 대입하지 않는다. XY만 유효한 형식은 사양과 함께 확정한다.
Wi-Fi 재연결·좌표 만료 시 웹은 마지막 위치를 오래된 값으로 표시하고 새로운 관측으로 재사용하지 않는다.

## 현재 구현 범위

- C++가 Wi-Fi telemetry 본문과 UWB 원천/지도 변환 좌표의 미확인 상태를 생성한다.
- Python은 같은 본문을 WS로 전달한다. 변환 값·비행 가능 여부를 임의로 만들지 않는다.
- REPLAY의 가짜 위치는 TEST 기체와 FAKE_PX4 출처를 유지한다. 실제 UWB 변환 결과로 재명명하지 않는다.
- 실제 UWB 수신·측량 변환 적용·웹의 새 필드 표시는 사양/서버 계약 후 연결한다.
- HOST_OBSERVE 운영 서비스는 실제 서버 조회를 유지하며 장치 등록·계약 검증 전 쓰기를 활성화하지 않는다.

## 웹팀 적용 요청

1. LoRa 배지를 생존 상태로만 사용하고 좌표 갱신 코드와 분리한다.
2. 인증된 Jetson telemetry의 `uwb_map_position`을 저장·화면에 전달한다. `pose_fused`와 별도 출처를 보존한다.
3. frame·transform_ref·원천 시각·valid가 확인된 좌표만 현재 좌표로 시각화한다.
4. LoRa 연결 중 Wi-Fi 단절, Wi-Fi 연결 중 UWB 미수신, 좌표 만료, REPLAY 혼입 거부를 공동 검수한다.
