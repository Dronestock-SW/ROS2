# 전체 미션 흐름 구현·검증 기록
명령부터 AI 경로 검사·복귀·END까지의 확인 기록이다.
2026-10-09 현장 시험과 다음 개발 인계 때 읽는다.

native 전체 흐름과 기존 AI 지도 검사를 구현했다.
작업 브랜치는 `codex/mission-flight-flow-20261008`이다.
main에 직접 작업하거나 병합하지 않았다.
기준 commit은 `128dfec4e7363a71499d6ca383ee105a014ffdfc`다.
실물 비행 성공을 확인한 기록은 아니다.

## 1. 연결 결과

하나의 실행기가 PX4 명령과 종료 확인을 책임진다.
기존 AI는 이륙 전에 지도와 전체 경로를 검사한다.
PX4는 상태추정·자세·고도를 제어한다.

```text
로컬 웹 START
 -> 기존 C++ AI 지도·전체 경로 검사
 -> 계획·START 기록 저장
 -> 3초 지상 정지
 -> AUTO.TAKEOFF 실제 모드 확인
 -> ARM 응답·실제 armed 확인
 -> native 상승 완료·XY/속도 2초 안정
 -> XY/yaw 목표 실제 적용·도착·정지
 -> ArUco 정렬 창·미세 이동·SCAN
 -> 결과 저장·대기점 복귀
 -> 다음 목표·방문 경로 역순 복귀
 -> LAND·새 ON_GROUND·disarm
 -> END·WebSocket 결과
```

| 부분 | 구현과 실제 시험 경계 |
|---|---|
| RAW UWB | 기존 DS-TWR·Tag B 슬롯·개별 측정 시각 사용 |
| ToF·IMU | 실제 처리기에 가상 거리·자세를 입력 |
| Flow | MAVLink optical_flow_rad를 실제 PX4에 입력 |
| 추정 | PX4 EKF2 하나. companion 위치 적분 없음 |
| AI | 기존 C++ 기하 검사 재사용. FC 출력 권한 없음 |
| 명령 | Python MissionChain 하나. 공통 writer 잠금 |
| 마커·스캔 | 명시적 가상 worker. 실물 adapter 미완성 |
| 웹 | 실제 로컬 HTTP·ROS2·MAVROS·WebSocket 경로 |
| 운영 웹 | v1.1 snapshot 직접 연결은 미완료 |

## 2. 최종 검증

Linux에서 실제 ROS2와 C++ 빌드를 검사했다.
ROS domain 99·localhost만 사용했다.
PX4는 v1.17의 고정 소스를 사용했다.
commit은 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`다.

| 시험 | 결과 |
|---|---|
| `drone_uwb` Python | 349/349 통과 · 37.10초 |
| `drone_mission` Python | 106/106 통과 · 55.34초 |
| `drone_platform_link` Python | 18/18 통과 · 2.86초 |
| C++ 등록 CTest 전체 | 30/30 통과 · 296.14초 |
| Python 합계 | 473개 통과 |
| AI 연결·1.3m·MAG_TYPE=6 | END·모사 작업 성공 |
| 최종 AI 연결 고장 행렬 | 11/11 기대 결과 통과 |

C++ 지도 계약은 정상 2개·거부 11개를 포함한다.
ROS 기록 오류·센서 원본 시각·RC 시작 조건도 검사했다.
회귀시험과 실제 PX4 시험은 서로 다른 근거다.

| 가상 시나리오 | 기대 결과 | 최종 결과 |
|---|---|---|
| 정상 | END·복귀·착륙·작업 성공 | 통과 · END · 작업 성공 |
| 상승 중 한 거리 +1.2m | END·작업 성공 | 통과 · END · 작업 성공 |
| NLOS 한 거리 +0.6m/0.8초 | END·작업 성공 | 통과 · END · 작업 성공 |
| 네 거리의 XY +0.8m 점프 | 격리·회복·END | 통과 · END · 작업 성공 |
| 잔차 ±0.035m·시각 차이 | END·작업 성공 | 통과 · END · 작업 성공 |
| UWB 0.65초 단절 | 정지·회복·END | 통과 · END · 작업 성공 |
| UWB 5.2초 단절 | LAND·FAILED | 통과 · FAILED · 착륙 확인 |
| ToF 0.65초 단절 | 정지·회복·END | 통과 · END · 작업 성공 |
| ToF 5.2초 단절 | LAND·FAILED | 통과 · FAILED · 착륙 확인 |
| 마커 응답 없음 | 복귀·END·작업 미완료 | 통과 · END · INCOMPLETE |
| 수동 POSCTL 전환 | PILOT_OVERRIDE·재획득 없음 | 통과 · PILOT_OVERRIDE |

최종 11개 ULog에서 EV XY·flow·range 융합을 확인했다.
비행 중 GPS와 EV velocity 융합은 0이었다.
fake position 융합도 0이었다. ULog dropout은 0개다.
정상 착륙 truth는 출발점에서 2.7cm였다.
정상 상승 후 안정화의 truth XY 반경은 7.3cm 이내였다.
coherent 점프는 RAW 단계에서 28개 관측을 격리했다.
짧은 UWB·ToF 단절은 RECOVERING을 거쳐 재개했다.
각 시행의 ARM 요청은 1회였다.
AI 계획 저장은 첫 비행 명령보다 먼저였다.
수동 전환 이후 자동 명령은 0개였다.
이 수치는 가상 모델의 관측 결과다.

## 3. 고친 연결 공백

관측 고장·명령 수락·실행 결과를 분리했다.
위치가 멈춘 것처럼 보여도 새 센서 입력을 위조하지 않는다.

| 공백 | 변경 |
|---|---|
| coherent 점프가 H80 창에서 완만해짐 | H80 전 RAW 좌표 점프 격리 |
| 반복 큰 점프가 새 원점으로 채택됨 | 원래 기준 유지·자동 재기준 금지 |
| 단절 중 이전 XY를 새 시각으로 재발행 | 관측 중지·PX4 현재 XY HOLD 한 번 |
| 이륙 직후 이동 | native 상승 완료·속도·XY 안정 2초 |
| ACK만으로 목표 도착 처리 | 실제 목표 피드백·FC 위치·속도 검사 |
| 목표 yaw가 무시됨 | heading 확인·MAG 정책/높이 조합 검사 |
| 스캔 라벨로 기체 이동 | 라벨과 기체 대기점 좌표 분리 |
| 문자열만 있는 경로 검증 참조 | C++ 지도 검사·SHA 고정·계획 저장 |
| 마커 미세 이동이 일반 이동 속도 사용 | native 요청 0.05m/s |
| 카메라/스캐너 결과의 창·시각 혼입 | 입력별 증가 시각·대상·창·저장 식별자 |
| 복귀 없이 LAND도 임무 완료 표시 | home·landing·작업 성공을 별도 판정 |
| 재시작·중복 START 재시동 | 영속 ledger·기체별 단일 writer |
| 저장 실패 뒤 가짜 END | END 저장 확인 후 발행·오류는 FAILED |
| RC 없는 시작·AUTO 인계 비활성 | 실제 RC 입력·RC-only·AUTO override 요구 |

Bridge 동기화는 RTT·증가 원본 시각을 검사한다.
불량 응답으로 정상 동기화의 나이를 갱신하지 않는다.
offset 점프 후 30개 정상 표본을 다시 모은다.
명령 효과를 모르면 UNCONFIRMED로 종료한다.
불명확한 ARM을 자동 재시도하지 않는다.
공중 disarm·수동 제어권 재획득을 하지 않는다.
이유: 결과가 모호할 때 후속 자동 명령이 충돌할 수 있다.

## 4. AI와 현장 범위

실물 START는 실측 지도와 현장 설정을 요구한다.
`native_map.template.json`은 UNVERIFIED로 거부된다.
SURVEYED 지도·원본 파일 SHA를 준비해야 한다.
`native_body_floor_height_m`은 실측한 기체 높이다.
MIS_TAKEOFF_ALT에 launch 높이를 임의로 더하지 않는다.
지도 검사는 companion z 명령을 만들지 않는다.

이번 실행기는 한 높이·정적 지도·짧은 구간을 지원한다.
기본 이동은 0.3m/s·구간 최대 1m·최대 20개 작업이다.
AI는 기체 여유 공간과 마커 보정 범위도 검사한다.
장애물 우회 경유점은 승인된 미션에 명시한다.
새 장애물의 동적 회피·여러 높이 임무는 미구현이다.
기존 C++ Behavior Tree는 REPLAY로 보존했다.
비행 명령을 내는 두 AI 실행기를 함께 사용하지 않는다.
이유: 명령·고도·모드의 책임이 충돌한다.

현재 실물 조회는 다음 조건이다.
추진 배터리 분리·조종기와 앵커 전원 꺼짐 상태다.

| 실물 항목 | 조회·적용 상태 |
|---|---|
| MIS_TAKEOFF_ALT | 1.3m 보존 |
| EKF2_MAG_TYPE | 0 보존 |
| COM_RC_IN_MODE | 3 보존 |
| COM_RC_OVERRIDE | 2 보존 |
| 실물 ARM·모드 명령 | 0회 |
| 실물 파라미터 쓰기 | 0회 |
| 기존 main·어제 통합 수정 | 원본 보존 |
| 기존 관측 서비스 | 실행 설정 보존 |
| 새 실행기의 실물 자동 기동 | 설정하지 않음 |

전체 실행기는 이 실물 상태에서 START를 거부한다.
RC-only와 AUTO override를 현장에서 검증해야 한다.
MAG_TYPE 0/1은 1.6m 미만 설정을 거부한다.
고정 PX4의 자력계 최종 정렬 문턱은 HAGL 1.5m다.
1.3m Init 정책은 가상으로만 확인했다.
실물 yaw 드리프트·자기장 확인은 미실시다.
높이·heading 정책은 현장 공간에 맞게 결정한다.

## 5. 가상 모델과 미확인

성공한 모사는 잡음·공분산을 명시한 조건부 결과다.
센서 모델을 현장 보정값으로 복사하지 않는다.
이유: 실제 센서 잡음과 RF 환경을 측정하지 않았다.

| 항목 | 모사 조건 | 실물 상태 |
|---|---|---|
| UWB | 3mm 잡음·알려진 편향 | 이동 편향·NLOS 미측정 |
| BTF 불확실성 | 표준편차 0.01m | 미보정 0.30m 보존 |
| H80 창 | 0.2초 | 0.8초 보존 |
| FC EV noise floor | 0.01m | 기존 FC 설정 보존 |
| Flow 최소 잡음 | 0.01rad/s | 장착·축·바닥·조도 미확인 |
| SIH IMU | upstream 잡음 배율 0.1 | 진동·실제 잡음 미측정 |
| 이륙 높이 | 행렬 1.7m·낮은 시험 1.3m | FC 1.3m 보존 |
| pose 유효시간 | 0.3초 | 0.2초 보존 |
| 기록 저장 | RAM 출력 후 영구 복사 | 지속 기록 지연 미측정 |
| 스캔 | SIMULATED-SCAN worker | 실물 adapter 미완성 |

센서 잡음 패치는 SIH 생성기에만 적용했다.
PX4 EKF·제어기를 수정하지 않았다.
upstream IMU 잡음 배율 1의 이전 시행은
3cm·3도·속도 0.03m/s 정렬 시간 제한을 관측했다.
그 시행은 복귀·착륙했지만 작업은 INCOMPLETE였다.
새 AI 연결 최종 행렬의 조건과 구분한다.
SD 출력의 freshness 실패도 이전 기록에 보존한다.
RAM의 fsync는 전원 단절 내구성 증거가 아니다.

PX4 HIL truth stream은 time_usec를 0으로 보낸다.
시험 도구는 loopback 수신 나이·MAVLink 순번을 쓴다.
100ms 넘게 truth를 못 받으면 센서 생성을 중지한다.
실물 센서 원본 시각 검사는 완화하지 않았다.
잘못된 HIL 시각 검사로 센서 0개였던 실패도 보존한다.

DW3000 RAW 계약에는 carrier phase 필드가 없다.
가상 위상 영향은 거리 잔차·측정 시각 차이로 주입했다.
느린 공통 편향은 UWB만으로 구분할 수 없다.
flow·IMU도 장기 절대 위치의 보장을 뜻하지 않는다.
실제 마커·실측 위치·EKF innovation 검사가 남는다.

## 6. 다음 실행

내일은 가까운 hover/waypoint 하나로 시작한다.
[현장 미션 절차](../runbooks/mission_chain_field.md)를 따른다.
그 뒤 yaw·복수 목표·복귀를 순서대로 검사한다.
실물 scan adapter가 준비될 때 scan 작업을 넣는다.
응답이 없으면 실패 기록·복귀가 정상 결과다.

[가상 재현 절차](../runbooks/mission_chain_virtual.md)와
[명령·관측 구조](../architecture/mission_chain.md)를 함께 읽는다.
시험 근거는 별도 `mission_flight_validation_20261008.tar.gz`다.
요약·ULog·RAW·웹 결과·계획·회귀 로그를 포함한다.
원본과 과거 실패를 성공 기록으로 덮지 않는다.
