# 기본 이착륙 시험 확인 기록
2026-10-08 명령 순서·실제 PX4 가상 시험 기록이다.
실물 시험을 준비하거나 결과를 이어받을 때 읽는다.

기본 실행기와 오류 경로를 보완했다.
PX4 가상 시험의 정상·취소·RC 인계를 통과했다.
실물 비행과 UWB·flow 공동 융합은 미완료다.

| 구분 | 확인 결과 |
|---|---|
| 작업 브랜치 | `codex/native-hover-test-20261008` |
| 출발점 | 로컬 통합 `24057d7690a2b04d8088b6871f61a823a03957cc` |
| Jetson 검토 폴더 | `/home/arialhanho/ROS2-review-20261008-codex` |
| 빌드 | `sangwon_ai_replay`, 성공 |
| CTest | 선택 11개 통과. 10개 기존 C++·재생 + ROS 연결 검사 1개 |
| ROS 연결 검사 | 실제 C++·HTTP·IPC, 모사 FC 시나리오 10개 |
| PX4 시험 | v1.17.0 SIH-as-SITL, 시나리오 3개 통과 |
| 실물 명령 | ARM·모드 명령 0개. FC 설정 쓰기 없음 |
| 보존 | 어제 현장 파일 9개 해시·원본 Git 상태 일치 |
| 운영 서비스 | 기존 관측 서비스 5개 active. 검토판 배포 없음 |

## 고친 동작

완료 표시는 ACK와 실제 비행 상태를 구분한다.

| 문제 | 적용한 처리 |
|---|---|
| ARM 먼저 보내는 순서 | 실제 AUTO.TAKEOFF 확인 후 ARM |
| 취소 중 늦은 ARM | 응답 대기. 지상 정상 disarm·POSCTL 확인 |
| 실패 뒤 착륙을 성공으로 표시 | FAIL·CANCELLED·PASS·UNCONFIRMED 분리 |
| 이전 landed 상태로 종료 | 응답 후 새 상태·landed 입력 요구 |
| 중복·역순 입력 | 상태 덮어쓰기와 만료 시각 갱신을 거절 |
| 무응답을 거부로 간주 | 효과 불명으로 반환. 재시동 자동 반복 없음 |
| RC 상실을 스틱 변화로 오인 | RC 유효성부터 검사 |
| companion의 강제 RC 인계 | 출력 중지. PX4 실제 인계 모드 관측 |
| 배터리·인계 설정 누락 | 배터리 실측·RC·override bit 확인 |
| 시작 직전 흔들림 | 최신 입력으로 3초 정지 유지 검사 |
| 시험 입력이 조회·서비스 대기에 정지 | 독립 입력 발행과 수신 처리 시간 제한 |
| 셸 결과가 다음 쿼리에 붙음 | echo와 최종 prompt·ANSI 처리를 확인 |

기존 고도·자세 제어는 PX4가 유지한다.
새 setpoint 발행자나 companion z 제어는 없다.
부팅은 시동·이륙을 실행하지 않는다.
새 실행기 세션 START만 순서를 시작한다.

## 모사 FC 오류 시험

10개는 성공 비행 10회라는 뜻이 아니다.
실패를 올바르게 판정했는지도 시험한다.

| 상황 | 확인한 결과 |
|---|---|
| 정상 | COMPLETE·PASS, 착륙·disarm 확인 |
| ARM 거절 | FAILED. 이륙 진행 없음 |
| TAKEOFF 접수 후 모드 미적용 | ARM 없음. 모드 확인 timeout |
| ARM 응답 지연 중 취소 | 대기 중 완료 금지. DISARM·POSCTL 후 CANCELLED |
| 같은 시각 메시지 반복 | freshness 상실, 출력 중지 |
| RC 상실 | FC failsafe에 반환. POSCTL 강제 없음 |
| 스틱 조작 후 PX4 미인계 | 인계 불명. 새 모드 강제 없음 |
| 수동 모드 전환 | 후속 명령·자율 재획득 없음 |
| LAND 거절 | 착륙 완료를 표시하지 않음 |
| ARM 응답 timeout·늦은 효과 | UNCONFIRMED. 자동 재시동 없음 |

잘못된 HTTP origin과 JSON null도 거절했다.
이전 세션 START·중복 START도 거절했다.

## 실제 PX4 가상 시험

명령·EKF·기체 동역학은 실제 PX4 프로세스다.
FC 소스는 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`다.
실물 조회의 펌웨어 해시와 같은 릴리스다.
가상 RC와 가상 위치 입력을 별도로 사용했다.

| 시험 | 결과·명령 경계 |
|---|---|
| 정상 | TAKEOFF→ARM→HOVER→LAND→COMPLETE·PASS |
| 호버 중 LAND | AUTO.LAND 후 지상 일반 disarm. CANCELLED |
| RC 스틱 | PX4 POSCTL 인계 확인. companion은 TAKEOFF·ARM 이후 명령 없음 |
| 인계 후 착륙 | 가상 조종자가 LAND. companion은 RELEASED 유지 |

정상 HOVER 상태는 2.0005초 유지했다.
목표는 1.3m다. SIH 실제 가상 위치 최고 높이는 1.262m다.
세 ULog 모두 해당 펌웨어 해시를 확인했다.
EV 위치·속도 융합을 확인했다. ULog dropout은 0개다.
FC 상태 확인 대기 중 같은 요청이 재전송될 수 있다.
원본 요청 횟수와 순서는 JSON·events에 모두 보존했다.

`ideal-ev`는 SIH 실제 위치를 ODOMETRY로 보낸다.
PX4 EKF가 융합한 위치를 MAVROS로 읽는다.
실행기의 위치·추정 gate를 직접 모사하지 않는다.
실물 FC와 직렬 장치는 열지 않는다.

| 센서 설정 | 실물 읽기 | 이번 양성 SITL |
|---|---|---|
| `EKF2_EV_CTRL` | 1, 수평 위치 | 5, 수평 위치·3D 속도 |
| `EKF2_GPS_CTRL` | 0 | 기본 SIH 값 7 |
| 위치 입력 | 현재 UWB 전달 없음 | SIH 위치를 이용한 이상적 EV |
| flow | 실물 수신·지상 미융합 | 실물 flow 재현 아님 |

기본 가상 GNSS 시험은 0.6m 편차로 중단됐다.
그 결과는 FAIL·착륙 확인으로 보존했다.
양성 시험을 위해 실물 편차·freshness 기준을 낮추지 않았다.
이 결과는 실물 센서 성능·UWB 융합 증거가 아니다.
SIH 빌드는 미사용 XRCE·Gazebo 모듈을 제외했다.
PX4 명령·EKF·제어·동역학 소스는 수정하지 않았다.
[공식 SIH 설명](https://docs.px4.io/v1.17/en/sim_sih/)을 따른다.

## 실물 센서 확인과 남은 연결

센서 수신과 융합 완료 사이의 공백을 확인했다.
Jetson·FC만 켜져 있다. 배터리·RC·앵커는 꺼져 있다.

| 읽은 항목 | 결과 |
|---|---|
| flow 허용 설정 | `EKF2_OF_CTRL=1` |
| flow 수신 | 최신 메시지. 마지막 quality 54/47 |
| flow 실제 융합 | 두 EKF 인스턴스 모두 `cs_opt_flow=false`, `fused=false` |
| 하방 거리 | 0.003m. 센서 최소 0.10m보다 작음 |
| range 실제 융합 | `cs_rng_hgt=true`, `fused=true` |
| 외부 위치 | `vehicle_visual_odometry` 발행 없음, `cs_ev_pos=false` |
| UWB 보정 코드 | B_TF의 ToF·IMU 자세 기하 보정 있음 |
| 보정 검증 | 장착 벡터·바닥·지도 회전·시각 지연 미확인 |
| FC 전달 | `enabled=false`, `ground_only=true` |
| 기본 시작 gate | RC·배터리·유효 위치·POSCTL·실제 인계 조건 미충족 |

ToF의 수 mm 값과 flow 최소 바닥 높이는 대조가 필요하다.
지상 가림인지 센서 문제인지는 미확정이다.
현재 지상 미융합만으로 공중 센서 불량을 단정하지 않는다.
다음 시험은 유효 높이·축 부호·실제 융합부터 확인한다.
기체 heading과 이동 궤적의 직진성도 각각 확인한다.

실물 `COM_RC_OVERRIDE=2`는 자동 모드 인계 bit가 꺼져 있다.
3은 기존 offboard bit를 유지하는 시험 후보다.
현재 값은 변경하지 않았다.
[센서 점검 절차](../runbooks/position_sensor_check.md)를 따른다.

## 이어서 할 일과 근거

다음 단계는 실물 융합 점검 후 짧은 이착륙이다.
앵커·RC·배터리 준비와 현장 조종자 확인이 필요하다.
임의 확인값·가짜 위치로 실물 gate를 열지 않는다.
이유: 가상 명령 시험과 독립 센서 검증은 다르다.
전체 UWB 이동·스캔·미션 완료는 이후 공동 시험이다.

[실행 명령](../runbooks/native_hover_test.md),
[검증 JSON](evidence/native_hover_test_20261008.json)을 따른다.
원본 CTest·ROS 로그·세 PX4 ULog를 별도 묶음에 보존했다.
이전 [통합 기록](local_jetson_integration_20261008.md)의
SITL 미실시는 해당 시점의 기록으로 유지한다.
