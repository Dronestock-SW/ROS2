# 상원 작업 통합과 관측 전달 제한
2026-10-13 통합 브랜치의 변경·검증 범위를 기록한다.
코드를 받거나 FC 설정과 LAND 원인을 대조할 때 읽는다.

브랜치는 `이번에-상원-한거-통합-브랜치`다.
Git은 공백을 허용하지 않아 요청 이름을 하이픈으로 연결했다.
최신 TEST1 `99a4a9b`에 로컬 C++ 실행기를 통합했다.
별도 LiDAR 연구 `288a061`은 합치지 않았다.
TEST1에 이미 있던 LiDAR 코드·기록은 보존했다.
실물 LiDAR의 비활성 설정도 유지했다.
main·FINAL·기존 TEST1 작업 트리는 수정하지 않았다.

## 통합 범위

| 부분 | 이번 처리 |
|---|---|
| 웹·미션 DB·원장·완료 상태 | 최신 TEST1 유지 |
| C++ AI | 공통 헤더로 분리. 최신 HANDOFF 변경 유지 |
| Python 실행기 | 최신 분리 이륙·HOLD·START 경로 유지 |
| C++ 실행기 | 단계·명령 응답·독립 20Hz 송신 연결 |
| 관측 Python | 센서·웹 계약 전용. 단계 기록은 비동기 |
| 공통 관측 서비스 | external-observation-bridge 인자 유지 |
| 텔레메트리 | 역할 이름으로 프로세스 선택. 인덱스 오류 수정 |
| 스캔 실패 후 계속 수행 | 기존 Python MissionChain 정책 유지 |
| 장착·점프 정책 | 최신 TEST1 유지. 이번에 추가 변경하지 않음 |

새 C++ 실행기는 ground_takeoff를 지원한다.
웹 START에서 Offboard→ARM→호버→복귀→LAND를 수행한다.
별도 TAKEOFF→HOLD→START는 기존 Python 실행기를 사용한다.
이 경로도 C++ AI 판단을 사용한다.
C++ 실행기에 takeoff_hold를 지정하면 시작 전에 거부한다.
지원하지 않는 경로를 비행 중에 발견하지 않기 위해서다.
두 실행기를 동시에 구동하지 않는다.

현재 C++ AI는 XY·hover·return 미션을 지원한다.
scan은 거부한다. 전체 스캔의 C++ 이식 완료를 뜻하지 않는다.
기존 QR·정렬·실패 후 계속 수행 소스는 통합 브랜치에 있다.
실물 전체 미션 성공은 여전히 미검증이다.

## 0.35초와 FC 파라미터의 구분

보고된 LAND 이유는 실행기의 전달 관측 검사다.
FC `COM_OF_LOSS_T`나 `EKF2_EV_DELAY` 검사와 다르다.

| 항목 | 의미·단위 | 이번 처리 |
|---|---|---|
| bridge_observation_timeout_s | 실행기가 허용하는 전달 관측 나이, 초 | 시험 예제 0.7 |
| COM_OF_LOSS_T | Offboard 연결 상실 후 FC 대응 대기, 초 | 실제 값 조회 대상. 임의 변경 없음 |
| EKF2_EV_DELAY | 외부 관측과 IMU의 시각 보정, ms | 실제 값 조회 대상. 700ms로 변경하지 않음 |
| EKF2_NOAID_TOUT | 수평 보조 없는 관성 추정 한계, μs | 실제 값 조회 대상. 임의 변경 없음 |

`EKF2_EV_DELAY`는 해당 고정 PX4 소스에서 0–300ms다.
0.7초 허용을 이 파라미터에 넣는 것은 이번 문제의 변경이 아니다.
공식 근거는 [Offboard 문서](https://docs.px4.io/main/en/flight_modes/offboard)와
[파라미터 설명](https://docs.px4.io/v1.14/en/advanced_config/parameter_reference#EKF2_EV_DELAY)이다.
실물 버전·값은 실제 readback으로 따로 확인한다.

## 적용값과 유지한 검사

Python·C++ AI·C++ 실행기가 같은 설정을 읽는다.
설정이 없는 기존 파일의 기본값은 0.35초다.
새 final_mission.example.json에는 0.7초를 명시했다.
사용 중인 비공개 설정은 자동으로 덮어쓰지 않았다.

| 검사 | 제한 |
|---|---:|
| 전달 관측 나이 | 선택한 설정, 시험 0.7초 |
| 원시 UWB 나이 | 기존 0.25초 |
| FC 위치 나이 | 기존 최대 0.2초 |
| C++ 판단 유효 시간 | 기존 0.15초 |
| 실행 주기 보호 | 기존 0.2초 |
| 높이 목표·상승 제한 | 기존 1.7m·0.5m/s |
| 출발 XY | 기존 고정 목표 |

관측의 원본 시각은 바꾸지 않는다.
이전 좌표를 새 관측으로 재발행하지 않는다.
0.48초 전달·0.13초 UWB 조건은 새 제한 안에 든다.
0.7초 초과나 별도 센서·FC 검사 실패는 계속 차단한다.
이 변경이 지연의 원인이나 물리 밀림을 해결했다는 뜻은 아니다.

## 검증과 현재 장비 확인

Windows의 선택 회귀는 486 passed·1 skipped다.
skip은 rclpy 부재로 완료 기록 ROS 시험을 건너뛴 것이다.
전체 저장소·실비행 통과 수로 해석하지 않는다.
Linux C++ 빌드와 비LiDAR ROS 종단 시험도 CI에서 통과했다.
검사 대상은 다음과 같다.

| 시험 | 확인 목적 |
|---|---|
| Python 경계·preflight·status | 0.48/0.7/0.700001초와 별도 UWB·FC 검사 |
| launcher 역할 4조합 | 공통 관측·C++ 선택 뒤 텔레메트리 보존 |
| C++ 계약 | 동일 전달 제한과 raw UWB 차단 |
| HTTP→ROS→C++→모사 FC | 실제 바이너리의 명령·착륙 결과 |
| GitHub ROS Humble 격리 CI | 로컬에 없는 Linux·ROS C++ 빌드 |

시험 ROS domain은 99·localhost다.
실물 FC 명령·비행을 수행하는 시험이 아니다.
전용 CI는 이 통합 브랜치의 소스 push로 실행한다.
main·FINAL push에는 이 작업을 실행하지 않는다.

첫 CI에서 ROS 4개 패키지와 C++ 실행기 빌드는 통과했다.
Linux Python은 246 passed였다.
C++ 계약은 실행기 16·AI 11·기존 LiDAR 13개다.
CTest 3개 묶음은 모두 통과했다.
정상·Python 중단·0.48초 전달은 모사 FC에서 END였다.
이후 기존 선택 LiDAR 공백 시험에서 FAILED·보호 LAND가 발생했다.
이 실패를 정상 완료로 계산하지 않았다.
[첫 실행](https://github.com/Dronestock-SW/ROS2/actions/runs/38325619173)의
로그와 실패 trace를 Git 밖에 보존했다.
요청 범위에 따라 LiDAR 추가 조사·활성화는 진행하지 않는다.
후속 종단 CI의 범위는 비LiDAR 5개로 명시했다.
정상·Python 중단·전달 지연·RC 인계·플랫폼 완료를 확인한다.
전체 선택 기능이나 실비행 안정성이 확인됐다는 뜻은 아니다.

최종 [비LiDAR CI](https://github.com/Dronestock-SW/ROS2/actions/runs/38326220154)는 통과했다.
검증한 구현 커밋은 `954fe787b8c942cf20376d8fb02c0582a4fcedb4`다.
이후 변경은 결과 문서다. 실행 소스는 같다.
ROS 패키지 4개·실제 C++ 실행기 빌드는 통과했다.
Linux 선택 Python 246개와 C++ 계약 40개도 통과했다.
Windows 486개와 Linux 246개의 중복 사례를 합산하지 않는다.

| 비LiDAR 시나리오 | 결과 | 모사 FC 수신 최대 간격 |
|---|---|---:|
| 정상 이륙·10초 호버·착륙 | END | 51.808ms |
| Python 어댑터 중단 1초 | END | 51.092ms |
| 0.48초 전달·0.13초 UWB | END | 50.982ms |
| ALTCTL 조종자 인계 | PILOT_OVERRIDE | 50.995ms |
| HTTP/WS 플랫폼→ROS→C++→웹 | END | 51.344ms |

정상 완료는 Offboard→ARM→LAND 순서였다.
착륙·DISARM의 새 상태를 확인한 뒤 END를 표시했다.
RC 인계에서는 LAND를 추가하지 않았다.
지연 사례의 종료 시 실제 C++ 입력 나이는
UWB 0.1357초·전달 관측 0.4857초였다.
관측의 원래 나이를 0으로 바꾸지 않았다.

검증 바이너리 SHA-256은 다음과 같다.
`77587edb035bafb975c3a7fec52af85dd2ad1cc4b02cf93e71404c28826e8199`.
[수치·분모·미실시 항목](evidence/sangwon_integration_20261013/verification.json)을 보존했다.
원본 성공·실패 trace는 PC `outputs/sangwon_integration_20261013/`에 있다.
이번 빌드는 Ubuntu 22.04·ROS Humble CI다.
Jetson의 현재 설치·실물 FC·실제 위치 유지 결과가 아니다.

이번 접속에서 Tailscale은 peer를 Online으로 표시했다.
하지만 실제 ping과 SSH는 시간 초과였다.
따라서 현재 FC 파라미터·착륙 상태를 확정하지 않았다.
과거 값이나 사용자 보고를 현재 readback으로 표시하지 않는다.
FC·서비스·운영 비공개 설정에 이번 변경은 적용하지 않았다.

다음 실물 확인은 [통합 브랜치 절차](../runbooks/sangwon_integration.md)를 따른다.
