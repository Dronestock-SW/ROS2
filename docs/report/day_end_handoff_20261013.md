# 13일 작업 종료 인계
2026-10-13 변경과 마지막 관측 상태를 기록한다.
14일 재개 전에 읽고 실제 상태를 다시 확인한다.

## 결론

**밀림 원인은 미확정이며 웹 비행 재개 상태가 아니다.**
R2 펌웨어 설치와 하방 거리 복구를 완료했다.
스캔 LiDAR의 실제 FC 융합은 아직 검증하지 않았다.
오늘 저장은 배포나 다음 비행 승인을 뜻하지 않는다.

## 소스와 배포

| 대상 | 저장·배포 상태 |
|---|---|
| 저장 브랜치 | `이번에-상원-한거-통합-브랜치` |
| 작업 시작 커밋 | `5db9c0ddf7be164b03e097acffac0440256994c8` |
| 기존 Jetson 실행 소스 | `b5ea7a46d6106fdd86d211ee016d537366542a7c` |
| 배포 신원 | slot의 `position-origin-release.json` |
| 과거 실행본 | `latency-release.json`의 `82e3bf1` |
| 새 PILOT_OVERRIDE 복구 코드 | 저장 후보. Jetson 미배포 |
| FC 펌웨어 | CRSF + LiDAR2D R2. CRC 검증 후 설치 |
| main | 이번 작업에서 변경하지 않음 |

Jetson은 `arialhanho@100.110.163.94`다.
실행 slot은 다음 경로다.
`/home/arialhanho/ROS2-review-20261008-codex/.review/fc-owned-runtime-20261013`

## 마지막 관측 상태

아래는 작업 중 마지막 조회다. 재개 시 실시간 값이 아니다.

| 대상 | 마지막 확인 |
|---|---|
| FC | 연결, DISARM, POSCTL, landed=1 |
| 하방 거리 | tf02pro bus 2, 0x10으로 복구 |
| 60초 검증 | 591개, 최대 간격 102.010ms, 통신 오류 0 |
| 임시 GPS2 UART tfmini | 종료 |
| MAVROS·기존 센서·스캔 수집 | 유지 |
| C++ 실행기·observer·웹·telemetry | 중지. 설치 뒤 미재개 |
| 공통 UWB 관측 송신 | 중지. 설치 뒤 미재개 |
| 원본 수집 서비스 | `dronestock-manual-capture.service` 중지 |
| FC logger | 프로세스 실행, 마지막 조회 `Not logging` |
| LiDAR 관측 송신 | 종료. status publisher 0 |
| FC LiDAR / aux velocity status | `never published` |

하방 복구 명령은 기존 bus 2 설정을 사용했다.
UART로 통신 모드를 변환하지 않았다.
버스 검사에는 reset 동작이 포함될 수 있다.
원래 통신 중단 원인과 영구 해결은 미확정이다.
자세한 근거는 [펌웨어·센서 복구 기록](lidar2d_crsf_flash_20261013.md)에 있다.

## 오늘 변경

| 변경 | 확인과 제한 |
|---|---|
| 첫 웹 이륙 분석 | TAKING_OFF 중 RC 인계. POSITION·별도 START 미도달 |
| 밀림 분석 | EV 공백과 FC XY reset 관측. reset량은 실제 이동량이 아님 |
| PILOT_OVERRIDE 복구 후보 | 착륙·DISARM·POSCTL·중립 등 연속 10초 충족 후 IDLE |
| 복구 권한 | 새 TAKEOFF 요청 필요. 같은 비행의 제어권 자동 회수 없음 |
| 펌웨어 | PX4 v1.17.0 기반, CRSF 유지, EKF2_LID2_VER 사용 표식 추가 |
| FC LiDAR 설정 | EKF2_LID2_EN=1, EKF2_LID2_AGE=180ms |
| 스캔 평면 | 사용자 수평 설치 확인 반영 |
| LiDAR 시각 분석 | 회전 2회 분석. 결과 불일치·IMU 공백 검토 필요 |
| 실제 LiDAR 송신 | timing_ref 비어 있음. 관측 송신 0건 |

두 번째 회전의 후보 지연은 -10ms였다.
bootstrap 범위는 [-20,+5]ms였다.
이 범위는 전체 체계 오차 상한이 아니다.
첫 회전의 불일치를 해소하지 못했다.
`calibration_complete=false`를 유지한다.
독립적인 평행 이동 정확도와 호버는 미검증이다.

1. [첫 이륙·PILOT_OVERRIDE 분석](first_takeoff_pilot_override_20261013.md)
2. [펌웨어 설치·시각 분석·하방 복구](lidar2d_crsf_flash_20261013.md)
3. [보존한 R2 펌웨어와 해시](../../src/drone_uwb/firmware/README.md)
4. [당시 지상 웹 대기 기록](ground_web_standby_20261013.md)

## 유지한 기준

이륙 높이 1.7m와 상승 제한 0.5m/s를 유지한다.
`EKF2_EV_DELAY=0ms`를 유지한다.
RC 인계와 기존 센서·안전 설정을 유지했다.
params 전체 복원은 하지 않았다.
캘리브레이션 차이는 설치 기록에 남겼다.

`home_xy_m`은 원래 이륙점이다.
`mission_start_xy_m`은 POSITION 안정 뒤 확정한다.
두 좌표를 같은 값으로 덮어쓰지 않는다.
복귀점이 바뀌면 원래 이륙점으로 돌아오지 못하기 때문이다.

관측 공백은 POSITION 5초 누적을 초기화한다.
POSITION 전 기존 이륙 안정 조건도 유지한다.
상태 보고 지연만으로 LAND하지 않는다.
UWB 제약을 물리적 밀림의 주원인으로 단정하지 않는다.

## 검증 범위

| 검증 | 결과 |
|---|---|
| 이번 저장 전 Python 선택 시험 | 41 passed, 1 skipped |
| skip | Windows에 rclpy 없음. completion_record 시험 미실시 |
| 기존 당일 C++ replay 근거 | 46 통과. 첫 이륙 보고서에 기록 |
| 이번 저장 전 WSL C++ 재구성 | SQLite3 개발 파일 부재로 중단 |
| 새 ROS 복구 시나리오 | 코드 추가. 통합 통과 미확인 |
| CI 38366040859, ROS 35개 | 이전 실행 소스 검증. 이번 저장본 결과 아님 |
| 실제 비행 | 상승 중 RC 인계. 수정 후 호버 성공 근거 없음 |

선택 시험은 LiDAR 설정·시각·motion과 관측·기록을 포함한다.
통과한 지상 시험을 실비행 성공으로 해석하지 않는다.
추정 오차와 실제 제어 응답을 함께 확인해야 하기 때문이다.

## 내일 재개 순서

실행 명령은 [현장 runbook](../runbooks/evening_web_flight_20261013.md)을 사용한다.
LiDAR 검증은 [별도 절차](../runbooks/lidar_physical_velocity.md)를 따른다.
재개 전 다음 확인이 남았다.

1. 새 부팅 신원·실제 DISARM·지상·RC·배터리·현장 조종자를 확인한다.
2. 기존 서비스·FC 포트 소유자를 확인한다. 중복 연결을 만들지 않는다.
3. 디스크 여유와 원본 수집·ULog의 실제 파일 증가를 확인한다.
4. 현재 기수 방향으로 지상 정렬을 새로 수행한다.
5. 실제 UWB XY·하방 거리·광류와 FC 융합 상태를 확인한다.
6. LiDAR 시각 검증과 실제 관측·FC 융합 증거를 분리해 확보한다.
7. 새 PILOT_OVERRIDE 복구 후보의 ROS 검증 후 배포 여부를 결정한다.
8. 현장 재승인 뒤 첫 시험만 수행한다. 자동 ARM·START는 하지 않는다.

첫 시험은 다음 범위다.
`TAKEOFF -> POSITION 연속 5초 -> 별도 START -> 제자리 10초 -> 원래 home 복귀·착륙·DISARM -> END`
좌표 이동 시험은 첫 시험 결과를 확인한 뒤 진행한다.
이 인계의 과거 승인을 다음 부팅의 비행 승인으로 쓰지 않는다.
전원과 현장 상태가 바뀌면 이전 확인이 유효하지 않기 때문이다.

## Git 외부 원본

PC 기준 루트는 다음 경로다.
`C:/Users/ASWPC_NTBOOK/Desktop/Drone5/outputs/`

| 폴더 | 내용 |
|---|---|
| ground_web_20261013 | 첫 비행 ULog·원시/decision·C++ 기록·영상 근거 |
| lidar2d-crsf-upgrade-20261013 | 원본/최종 펌웨어·params·flash·FC readback |
| lidar2d-enable-20261013 | 회전 원시 기록·시각 분석·UART 시험·bus 2 복구 |
| checkpoint-tests-20261013-b | 저장 전 선택 pytest 임시 결과 |

원본과 개인 설정은 Git에 포함하지 않았다.
주요 PC·Jetson 파일의 SHA256 대조는 상세 보고서에 있다.
`.runtime/`의 원시 로그·DB도 삭제하지 않고 제외했다.
