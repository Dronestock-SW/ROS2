# 멀티태그·웹 패치 검증 — 2026-10-07
이번 코드 수정과 시험 결과를 기록한다.
장치에 적용하기 전 검증 범위를 확인할 때 읽는다.

## 결론

TDMA 수신과 웹 관측 전송을 구현했다.
실제 companion 센서 연결과 위치 정확도는 미검증이다.

| 대상 | 기준과 결과 |
|---|---|
| UWB | `ArialHanho/uwb-multitag-research`, `e45419c` |
| ROS2 기준 | `Dronestock-SW/ROS2`, `1cc9ba4` |
| ROS2 작업 | `codex/uwb-web-handoff-20261007` |
| 웹 기준 | `ljh006008-blip/https---github.com-ljh006008-blip-Teamproject1`, `4f2b0d7` |
| 웹 작업 | `codex/uwb-observation-handoff-20261007` |
| 배포 범위 | 태그·지상국 펌웨어 적용. ROS·웹은 코드와 격리 시험까지 |
| 실제 연결 | 현재 Tag↔companion 연결 전. 지상국 웹 연결도 미실행 |

이전 [호환성 검토](multitag_web_compatibility_20261007.md)는 수정 전 기록이다.
이번 결과로 과거 시험 수치를 덮어쓰지 않았다.

## 구현 범위

각 입력의 출처와 유효 시각을 보존한다.

| 작업 | 반영 내용 |
|---|---|
| J1 | TDMA·ACK·LoRa RX를 정상 보조 메시지로 분리 |
| J2 | Tag A/B YAML·B_TF JSON, 공통 실행기, 도메인 1/2 |
| J3 | RAW와 다음 TDMA를 50ms 이내 대응. ID·seq·부트·세션·슬롯·시한·로그 손실 검사 |
| W1 | `uwb_xy`, `btf_xy`, `px4_local` 선택. 기본 `btf_xy` |
| W2 | 원본 시각과 나이·좌표계·기준점 보존. XY의 Z는 null |
| W3 | 웹 ID와 ROS 도메인 분리. 서명 인증 사용 |
| W4 | 웹 등록 기체와 인증 ID 대응. 중복·역순·이전 세션 거절. 수락 ACK 반환 |
| W5 | 소스·원본 나이 상한·기준점·만료 표시. PX4 로컬 좌표는 창고 지도에 미투영 |

Tag B 장착값은 미확인으로 유지했다.
0 벡터는 비활성 자리값이다.
Tag A의 12cm 설정은 기존 저장소 설정을 보존했다.
이번에 다시 실측한 장착값이 아니다.
PX4 브리지와 외부 출력 허용은 켜지 않았다.

## 실제 UWB·LoRa 벤치

무선 LoRa와 USB 수신 판정은 다르다.

| 시험 | 실제 결과 | 제한 |
|---|---|---|
| Tag B LoRa 원인 | 기존 허용 22ms보다 서비스 진입 23.41ms가 늦음 | UWB 시분할 슬롯 변경 없음 |
| 펌웨어 조치 | A/B `v0.4.1`, 발사 허용 24.5ms, 지상국 `v0.4` | A1~A4 radiofix3 유지 |
| 정상 70초 UWB | 준비 후 60초 A 39.500Hz, B 39.967Hz | 1초 창 최소 A 37회/B 38회, 최대 연속 누락 1회 |
| LoRa 장기 기록 | 1229.491초 창, A/B 각 1230개 | 간격 999~1001ms, LOST 0, 오류 증가 0 |
| Tag B·A1 재시작 | 새 부트/세션 뒤 통신 복구 확인 | 의도한 중단 포함. 연속성 시험과 분리 |
| 600초 USB 기준 | 미통과 | Windows RXOVER/OVERRUN과 원본 행 손상 확인 |
| 위치 정확도 | 미실시 | 임의 근접 배치. 독립 기준점 없음 |

상세 원본은 [UWB 벤치 기록](https://github.com/ArialHanho/uwb-multitag-research/tree/e45419c/experiments/lora-sync1hz/results/field-20261007)에 있다.
손상 로그도 보관했다.
기체별 companion에서 600초 수신을 다시 검사한다.

## 소프트웨어 시험

아래 수치는 실물 ROS·위치 정확도 결과가 아니다.

| 시험 | 결과 | 환경/분모 |
|---|---|---|
| UWB Python | 313 통과 | Linux 격리 디렉터리. 기존 MAVLink wire 시험은 `pymavlink` 없어 제외 |
| ROS 패키지 빌드 | 2개 성공 | `drone_uwb`, `drone_platform_link`; `colcon build --symlink-install` |
| Platform Python | 14 통과 | Windows Python 3.13, 실제 송신 모듈 |
| USB 기록 재생 | 5개 기록 묶음 | TDMA 대응 경계만 재생. 손상·시한 초과 거절 결과 보존 |
| 웹 Django | 67 통과 | 관측·좌표·companion·UI·조회·지상국 회귀 시험 |
| 새 JS 표시 시험 | 통과 | 만료·Z null·PX4 로컬 출처 |
| 기존 JS 대시보드 | 6 통과, 1 실패 | 변경 전 `4f2b0d7`에서도 같은 preflight 행 재생성 실패 |
| 실제 송신 Runtime→Django | A/B 각각 186/186 수락, 거절 0 | 합성 40Hz 입력, 약 20.03초, localhost WS, 목표 송신 10Hz |
| 웹 시각 확인 | 수행 | 합성 위치·출처·Z 미관측·입력 중단 후 만료 |

실제 송신율은 약 9.29Hz였다.
UWB RF 수신율이나 B_TF 계산률로 해석하지 않는다.
시험 종료 후 `websocket_connected=false`는 정상 종료 상태다.
기본 임무 조회는 껐다. HTTP 요청/실패는 0이다.
관측 인증과 구형 임무 API 계약 충돌을 피한다.
명령 실행 기능은 추가하지 않았다.

[검증 로그와 요약](evidence/multitag_patch_20261007/manifest.json)을 보관했다.
[화면 캡처](evidence/multitag_patch_20261007/web-observation-final.png)는 합성 입력 중단 후 만료 상태다.

## 다음 실물 확인

각 기체를 독립적으로 연결한 뒤 확인한다.

1. Tag A/B를 각 companion에 연결한다.
2. 장착값·ToF 토픽·자세 시각을 기체별 확인한다.
3. 준비 후 600초 RAW/TDMA를 보존한다.
4. B_TF 새 좌표율·공백·나이를 기록한다.
5. 독립 기준점과 실제 XYZ 오차를 비교한다.
6. 별도 웹 PC에서 지상국 USB를 연결한다.
7. 웹의 Wi-Fi 관측과 LoRa 상태를 각각 확인한다.

최종 PX4 XYZ의 창고 좌표 변환은 남아 있다.
외부 관측 융합·비행·고도 제어 검증도 미실시다.
자세한 적용 순서는 [실행 절차](../runbooks/multitag_jetson_web_handoff.md)를 따른다.
