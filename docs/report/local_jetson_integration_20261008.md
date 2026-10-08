# 로컬 Jetson 통합 기록 — 2026-10-08
웹 인계와 현장 코드를 합친 검토판의 기록이다.
다음 빌드·지상 검증을 이어갈 때 읽는다.

별도 브랜치에서 소스 통합과 분리 검사를 완료했다.
기존 서비스에 배포하거나 FC 설정을 바꾸지 않았다.
실제 PX4 SITL·실물 융합·비행은 아직 미실시다.

## 통합한 원본

조회 뒤 보존한 세 입력을 사용했다.

| 입력 | 기준 |
|---|---|
| 웹 인계 | `8c2aec3cf266a4fa00585c0fc6e191683b5594af` |
| 어제 통합 | `1cc7e3b7fbfe39a693968925bbec507db854380d` |
| Jetson 추가 작업 | native hover 관련 9개 파일, 43,540바이트 |
| 로컬 검토 브랜치 | `codex/local-jetson-integration-20261008` |
| Jetson 검토 폴더 | `/home/arialhanho/ROS2-review-20261008-codex` |

추가 파일의 원본·SHA-256은 따로 보존했다.
[직접 조회 기록](local_jetson_readback_20261008.md)에 장비 상태가 있다.
기존 main과 현장 작업 폴더는 수정하지 않았다.
새 검토 폴더에서만 빌드·시험 결과를 생성했다.

## 변경한 계약

파일 단위 선택 대신 양쪽 기능을 함께 합쳤다.

| 경계 | 통합 결과 |
|---|---|
| A/B 선택 | `tag`로 ID·domain·수신기·B_TF를 함께 선택 |
| 배치 | mission·B_TF·anchor JSON이 다르면 시작 전 거부 |
| Tag B 기본값 | ID 6·domain 2·임시 0.15m·이륙 기대값 1.3m |
| 물리 확인 | `layout_confirmed` 추가. 모든 확인값 false 유지 |
| TDMA | RAW/진단 짝·각 거리 원본 시각·재시작 처리 유지 |
| B_TF | 같은 시각 XYZ·높이 필요 조건 유지 |
| EV bridge | 배치·domain·ground-only·TIMESYNC·유일 발행자 검사 유지 |
| FC 장착값 | `antenna_body_frd_*_m` 단일 계약, 1mm 대조 |
| 웹 장착 설정 | mission의 `expected_ev_pos_*_m`를 위 FRD 필드로 변환 |
| bridge 최신성 | 원본 관측 stamp·10Hz 상태·1Hz 파라미터 조회 |
| 웹 관측 | 출처·좌표·원본 시각과 미션 상태·ACK를 함께 전달 |
| 웹 높이 | PX4 높이와 태그 높이 분리. XYZ는 같은 시각일 때만 유효 |
| HTTP/WS 인증 | POST 본문과 WebSocket GET 경로 서명을 모두 보존 |
| 로컬 웹 | 같은 `--config`로 기체·배치 ID 선택 |
| 지상 수집기 | RC 채널·배터리·RC/상실 관련 mirror 조회 추가 |
| MAVROS | `start_mavros=false` 기본값. 기존 소유자 확인 필요 |
| 종료 | callbacks 종료 뒤 ROS context 해제. 플랫폼 스레드 join |
| 검사 등록 | 미션·플랫폼에 pytest를 등록해 0개 검사 종료 수정 |

## PX4 명령 주체

웹 FSM과 native hover가 같은 호스트 잠금을 쓴다.

```text
Python 웹 FSM ----+
                 +-- 공통 PX4 명령 잠금 --> 하나의 실행기
C++ native hover-+                          |
                                           v
                                      MAVROS --> PX4

C++ BT 서비스 --> REPLAY / HOST_OBSERVE 유지
```

실물 domain 1/2는 같은 잠금 파일을 쓴다.
SITL·합성 domain은 별도 잠금 파일을 쓴다.
프로세스 종료 때 파일을 지우지 않고 잠금만 해제한다.
이유: 파일 삭제 뒤 서로 다른 파일을 잠그는 경쟁을 막는다.

이 잠금은 이번 수정에 참여한 실행기 사이의 규칙이다.
이전 배포본·외부 GCS·다른 호스트를 제어하지 않는다.
현재 Jetson 서비스 교체나 현장 단일 주체 검증은 미실시다.

## 확인 결과

검사 수는 실행 묶음별로 센다. 중복 합산하지 않는다.

| 검사 | 결과 |
|---|---|
| Jetson colcon 빌드 | 6개 패키지 성공 |
| UWB | 347개 통과 |
| 데모 | 116개 통과 |
| 플랫폼 | 18개 통과 |
| 미션 | 64개 통과 |
| 기동·lint | 58개 중 57개 통과·copyright 1개 skip |
| Python 합계 | 603 = 통과 602 + skip 1. 오류·실패 0 |
| 별도 C++ 핵심 검사 | 10개 모두 통과 |
| ROS/HTTP/WS 합성 시나리오 | 정상·UWB 단절·웹 단절·공중 재시작 4개 통과 |
| 프로세스 종료 | 위 시나리오의 모든 자식 종료 코드 0 |
| 보존본 | 기존 현장 소스 9개의 SHA-256 변화 없음 |

미션 64개에 합성 시나리오 4개가 포함된다.
C++ 검사는 ROS 관측 서비스 전체 시험을 포함하지 않는다.
이전 중간 검사 161개와 Windows 73개는 중복 합산하지 않는다.
기계 판독 결과는 [시험 영수증](evidence/local_jetson_integration_20261008.json)에 있다.

첫 전체 실행의 ROS 시나리오 4개는 환경 검사에서 멈췄다.
A/B launch 검사가 환경의 domain 값을 복구하지 않았다.
해당 검사의 종료 복구를 추가했다.
합성 입력의 domain 99·localhost 조건은 그대로 유지했다.

처음에는 작업 루트에서 lint를 함께 실행했다.
vendor와 다른 패키지까지 검사해 style 실패 2개가 났다.
이후 패키지별 표준 작업 경로에서 lint도 통과했다.
검사를 제외하거나 규칙을 완화하지 않았다.

종료 로그의 context 예외와 daemon thread 종료도 수정했다.
SIGTERM 뒤 노드·스레드가 정리되는 순서를 검사했다.
최종 로그에는 해당 Traceback·terminate 메시지가 없다.

Windows의 Git checkout은 링크를 일반 파일로 받았다.
UWB 전체 검사는 원래 링크가 유지되는 Linux에서 실행했다.
라이브러리 빌드는 고정된 BehaviorTree.CPP commit을 사용했다.
빌드한 실행기를 실제 FC에 연결하지 않았다.

## 남은 현장 조건

코드 검사 통과만으로 비행 조건이 충족되지 않는다.

| 항목 | 남은 확인 |
|---|---|
| 조종기·앵커 | 사용자 확인상 전원 꺼짐. 전원 켠 연속 수신 검사 필요 |
| UWB | 어제 0Hz 창·JSON 오류 원인과 600초 연속성 재검사 |
| 장착·배치 | Tag B 장착·네 앵커 실측, FRD/FLU·좌표 방향·지연 |
| FC | RC 인계·상실·저전압 정책과 실제 반응 |
| EV | ULog 융합·innovation과 독립 위치 기준 대조 |
| 수평 이동 | 실제 PX4 원점·DO_REPOSITION·정지·도착 |
| 실제 SITL | 펌웨어 기반 정상·단절·재시작·거부·수동 인계 |
| 실물 비행 | 조종자 준비·Hold 안정성과 위 조건 확인 뒤 별도 시행 |

현재 FC의 RC override 2와 GCS 상실 대응 0은 보존했다.
센서 보정값·모드 슬롯 차이도 임의 복원하지 않았다.
확인값을 true로 바꾸거나 자동 이륙을 실행하지 않았다.
실제 PX4 SITL 실행 파일은 이번 경로 조사에서 찾지 못했다.
합성 FC 검사를 실제 PX4 SITL 결과로 표기하지 않는다.
