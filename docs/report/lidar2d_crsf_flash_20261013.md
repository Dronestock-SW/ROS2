# LiDAR2D + CRSF FC 설치 결과
2026-10-13 Pixhawk6C 펌웨어 설치 기록이다.
다음 지상 점검과 비행 재개 전에 읽는다.

**설치는 완료했다. 실제 LiDAR 융합은 미실시다.**
사용자가 프로펠러 제거와 안정 전원을 확인했다.
다른 비행·설정 작업의 중지도 확인했다.
ARM·이륙·모드 변경 명령은 보내지 않았다.

## 설치본

사용자가 준 CRSF 펌웨어의 기반을 유지했다.
기존 LiDAR2D 하드웨어 패치를 추가했다.

| 항목 | 결과 |
|---|---|
| PX4 기반 | v1.17.0, `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 대상 | FMUv6C, board ID 56 |
| 빌드 | `make px4_fmu-v6c_lidar2d -j4` |
| 최종 파일 | `PX4_v1.17.0_Pixhawk6C_CRSF_LIDAR2D_R2.px4` |
| SHA256 | `d52d045bdc8f474237a376f584c6c2a58fddc0b1b6ef12e98c773d0750bf628b` |
| 이미지 크기 | 1,956,324 / 1,966,080 bytes |
| 업로드 | board ID 검사, 프로그램, CRC 검증, 재부팅 성공 |
| 호스트 시험 | 관련 pytest 17개 통과. 비행 증거 아님 |

첫 설치본은 `VER`가 MAVROS 목록에 없었다.
FC 내부에는 값 1이 존재했다.
R2는 EKF2의 `ParamInt`에 표식을 등록했다.
재부팅 후 MAVROS에서도 값 1을 확인했다.
CRSF 옵션도 재현용 패치에 포함했다.
기존 원본 PX4 작업 폴더는 수정하지 않았다.
WSL의 별도 복사본에서 빌드했다.

## 파라미터

요청대로 기존 안전·센서 설정을 유지했다.
`perfect_holdv2.params` 전체 복원은 하지 않았다.
파일의 2.5m·1.2m/s 설정도 적용하지 않았다.

| 항목 | 최종 읽기값 | 처리 |
|---|---:|---|
| `EKF2_LID2_VER` | 1 | 펌웨어 표식 |
| `EKF2_LID2_EN` | 1 | 변경 |
| `EKF2_LID2_AGE` | 180ms | 변경 |
| `EKF2_AVEL_DELAY` | 5ms | 유지 |
| `EKF2_EV_DELAY` | 0ms | 유지 |
| `EKF2_EV_CTRL / OF_CTRL / RNG_CTRL` | 1 / 1 / 1 | 유지 |
| `COM_RC_OVERRIDE` | 3 | 유지 |
| `MIS_TAKEOFF_ALT` | 1.7m | 유지 |
| `MPC_TKO_SPEED` | 0.5m/s | 유지 |
| `MPC_Z_VEL_MAX_UP / MPC_Z_V_AUTO_UP` | 0.5 / 0.5m/s | 유지 |
| `MPC_LAND_SPEED` | 0.25m/s | 유지 |
| `EKF2_MAG_TYPE / SENS_FLOW_ROT` | 0 / 4 | 유지 |
| `MPC_THR_HOVER` | 0.35 | 유지 |
| `SDLOG_MODE / SDLOG_PROFILE` | 1 / 129 | 유지 |

작성 명령은 EN과 AGE 두 개뿐이었다.
최종 재부팅 후 강제 pull로 다시 비교했다.
최종 캐시의 FC 파라미터는 1,098개다.
미사용 파라미터 전체를 백업했다는 뜻은 아니다.

자력계 오프셋 3개는 초기 캐시와 달랐다.
`CAL_MAG0_XOFF/YOFF/ZOFF`가 해당한다.
FC 저장값을 유지했고 덮어쓰지 않았다.
새 설정 적용 전후에는 추가 변경이 없었다.
자동 보정 저장 경로는 PX4 소스에 존재한다.
하지만 이번 차이의 발생 시점·원인은 미확정이다.
전체 캘리브레이션이 백업과 같다고 판단하면 안 된다.
새 부팅에서 기수·자력계 정렬을 확인해야 한다.

## 지상 관측

설치 검증과 비행 준비 완료를 구분한다.

| 관측 | 최종 결과 |
|---|---|
| 기체 | DISARM, landed=true, USB 전원 유효 |
| CRSF | 16채널, rc_lost=false, failsafe=false, LQ 97 |
| 광류 원시 입력 | 갱신 확인, quality 60 |
| 거리 원시 입력 | 0.022m. 선언 최소거리 0.1m 미만 |
| LiDAR2D 상태 / aux velocity | `never published` |
| ULog | 파일 증가 확인, 약 2.84MB -> 3.04MB |
| FC SD 잔여 | 약 830MB |
| Jetson 잔여 | 약 1.25GB. 원시 수집 재개 전 확인 필요 |

ULog 경로는 `/fs/microsd/log/2026-10-12/04_36_34.ulg`다.
파일명 날짜는 FC 시계 기준이다.
호스트 검증 시각은 10월 13일 19:52 KST다.
플래시 전에 기존 ULog를 닫았다.
두 번째 설치 전 `logger off`는 정지를 못 했다.
DISARM·지상 검사 후 `logger stop`으로 닫았다.
재부팅 뒤 원래 SDLOG 설정으로 기록을 재개했다.

## 최초 설치 직후 상태

**LiDAR 송신 gate는 아직 닫혀 있다.**
FC의 EN=1만으로 실제 융합을 뜻하지 않는다.

| 구성 | 상태 |
|---|---|
| `dronestock-manual-observe.service` | active, 기존 MAVROS 하나 |
| `dronestock-manual-lidar.service` | active, 센서 관측 |
| `dronestock-manual-capture.service` | inactive, 원래 중지 상태 유지 |
| C++ 실행기·웹·텔레메트리·공통 UWB 송신기 | 플래시 준비 때 중지, 재개 안 함 |
| LiDAR 설정 `enabled` | false |
| `scan_plane_parallel_confirmed` | false |
| `timing_ref` | 빈 값 |

이전 FC 부팅의 정렬값으로 자동 재개하지 않았다.
재개 전 새 부팅의 시계·기수 정렬이 필요하다.
UWB 송신과 기록도 기존 runbook대로 재개한다.
LiDAR 수평 장착과 동적 시각을 실측해야 한다.
이후 실제 수신·허용·융합 여부를 별도로 확인한다.
이번 작업은 밀림 원인 확정이나 개선 증거가 아니다.

## 후속 활성화 요청 확인

기존 구현을 사용한 활성화 시험은 차단됐다.
사용자는 설치 후 LiDAR ON을 요청했다.
원격 두 브랜치의 완료 기록도 다시 확인했다.

| 원격 기준 | 기록 범위 |
|---|---|
| `codex/uwb-lidar-integration-20261012`, `288a061` | 회전 스캔·IMU·SITL 구현. 실기 배포는 미실시 |
| `FINAL-TEST1-SANGWON`, `0f8deaf` | 실측 노드 준비. 과거 bench 송신 0건 |

기존 장착 yaw -91.5도는 다시 만들지 않았다.
실행용 복사본만 `enabled=true`로 바꿨다.
운영 원본의 기하·시각 확인값은 바꾸지 않았다.
현재 부팅에서 기존 노드를 20초 실행했다.

| 항목 | 결과 |
|---|---:|
| IMU 입력 | 1,988개 |
| 점군 입력 | 199개 |
| timesync 준비 보류 | 33개 |
| `scan_plane_unconfirmed` 보류 | 165개 |
| FC 송신 | 0개 |
| FC 표식·활성·나이·지연 읽기 | 1 / 1 / 180ms / 5ms |

시험 노드는 20초 뒤 종료했다.
실제 융합 ON 완료로 처리하지 않는다.
수평면 확인과 동적 시각 근거가 여전히 필요하다.
완료 근거가 다른 커밋에 있다면 먼저 대조한다.
스캔 LiDAR와 하방 ToF는 별개의 센서다.
현재 노드는 ToF 유효 높이도 관측 검사에 쓴다.
기존 설정의 `timing_ref`는 빈 값이다.

증거는 PC `outputs/lidar2d-enable-20261013/`에 있다.
`guarded-on-check/summary.json`과 관측 기록을 보존했다.

이후 사용자가 수평면 검증 완료를 확인했다.
실행용 복사본의 수평 확인값을 true로 바꿨다.
이는 사용자 확인이며 독립 재측정은 아니다.
같은 복사본으로 10초 지상 시험을 수행했다.
점군 99개와 IMU 900개가 들어왔다.
초기 timesync 보류는 40개였다.
이후 59개는 `acquisition_timing_unconfirmed`로 보류됐다.
FC 송신은 0개였으며 시험 노드는 종료했다.
현재 별도 하방 Range는 약 0.02m다.
센서의 선언 최소거리 0.1m보다 작다.
웹 이륙 준비 완료로 판정하지 않았다.
증거는 같은 폴더의 `plane-confirmed-check/`다.

## 수동 회전과 거리 입력 중단

**시간 보정과 실제 융합은 완료되지 않았다.**
받침 위 정지 기록에서는 약 0.69m를 받았다.
사용자는 프로펠러 제거 후 수동 회전을 수행했다.
기록 안내와 첫 수집 종료가 겹쳐 추가 수집했다.
두 회전 기록 모두 보존하고 따로 분석했다.
분석에는 기존 점군 정합기를 재사용했다.
FC 출력이나 파라미터 변경은 없는 파일 분석이다.

| 항목 | 첫 회전 기록 | 두 번째 회전 기록 |
|---|---:|---:|
| 점군 / IMU 수 | 476 / 4,726 | 880 / 8,567 |
| 기록된 기체 상태 | 모두 DISARM, 지상 | 모두 DISARM, 지상 |
| 최대 기울기 | 3.34도 | 2.42도 |
| IMU 표본 간격 >25ms | 4회 | 20회 |
| 최대 IMU 표본 간격 | 192ms | 338ms |
| 원시 정합 회전 상관 | -0.413 | 0.952 |
| 회전 보정 후 공통 정합 쌍 | 37 | 24 |
| 회전 보정 후 시간차 후보 | -45ms | -10ms |

첫 기록의 회전 불일치를 해결하지 못했다.
두 번째 결과만으로 전체 성공 처리하지 않았다.
두 번째의 구간별 후보는 +5/-15/-10ms다.
그 기록의 재표집 구간은 -20~+5ms다.
이 구간은 체계적 오차를 포함한 보증값이 아니다.
정합의 약한 수평 구속과 수신 공백도 관측했다.
독립적인 이동 거리나 호버 정확도는 측정하지 않았다.
`timing_ref`와 시각 오프셋을 변경하지 않았다.
`EKF2_EV_DELAY=0ms`를 유지했다.

분석 도구는 `tools/analyze_lidar_yaw_timing.py`다.
새 시각 계산 시험 5개가 통과했다.
기존 설정 시험 8개와 합쳐 13개가 통과했다.
이는 계산 시험이며 실기 융합 증거가 아니다.

### 하방 센서 진단

정상 거리 입력이 회전 수집 중 중단됐다.
첫 기록에서는 거리 456개를 받았다.
두 번째에서는 22개 이후 수신이 끊겼다.
추가 12초 구독에서도 거리 수신은 0개였다.
같은 재확인의 IMU는 최대 간격 20ms였다.
따라서 앞선 IMU 공백을 상시 고장으로 단정하지 않는다.

FC 내부 거리 표본도 411초 이상 오래됐다.
그 표본의 장치 표기는 `I2C:2 (0x10)`이었다.
`tf02pro`의 I2C 통신 오류가 증가했다.
장치 표기를 실제 센서 모델·배선으로 단정하지 않는다.
사용자는 하방 모델을 TFmini Pro라고 설명했다.
연결은 GPS2 UART라고 설명했다.
기존 문서의 TFmini Plus 표기와 실물 모델은 미대조다.

| 점검 | 결과 |
|---|---|
| 기존 UART 설정 | `SENS_TFMINI_CFG=0`, 드라이버 미실행 |
| 기존 I2C 설정 | `SENS_EN_TF02PRO=1` |
| SD 부팅 스크립트 | `tf02pro start -X -b 2 -a 16 -f 400 -R 25` |
| GPS2 장치 | 빌드 설정의 `/dev/ttyS6` |
| GPS2 점유 | GPS2 설정 0. GPS는 ttyS0에서만 실행 |
| 임시 UART 시험 | tfmini를 ttyS6에서 약 15초 실행 |
| UART 시험 거리 수신 | 유효 표본 0개 |
| UART 시험 종료 | 시작한 tfmini를 종료, 저장 설정 불변 |
| 기존 드라이버 재시작 | 사용자 승인 후 1회, 같은 버스·주소·주파수 |
| 재시작 직전 통신 오류 | 10,750건 |
| 재시작 결과 | 장치 미검출. tf02pro 미실행 |
| 마지막 기체 상태 | 연결, DISARM, POSCTL, landed=1 |

GPS2에는 UART와 I2C 핀이 함께 있다.
[제조사 핀 배치](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports)를 확인했다.
포트 이름만으로 사용 중인 신호선을 확정하지 않는다.
UART의 유효 표본 0개도 단선 증명은 아니다.
현재 증거로 센서 전원·배선·통신 모드를 구분하지 못했다.
센서 응답 복구가 필요하며 추가 ARM은 하지 않는다.
FC 전체 재부팅이나 저장 파라미터 변경은 하지 않았다.

### 센서 복구 전 인계

스캔 LiDAR 원시 수신과 FC 융합은 별개다.
스캔 수집은 정상이나 실제 융합은 아직 미실시다.
실행 복사본은 enabled=true, 수평 확인=true다.
시각 근거는 빈 값이며 관측 송신 노드는 종료 상태다.
하방 거리 센서 드라이버도 현재 미실행이다.
기존 저장 설정과 부팅 스크립트는 보존했다.

FC 기록에는 RC ARM/DISARM 이력이 존재했다.
이 작업은 ARM 명령을 보내지 않았다.
해당 이력은 비행 수행 증거로 해석하지 않는다.
후속 조회에서 logger는 실행 중이나 Not logging이었다.
`SDLOG_MODE=1`만으로 실제 기록 중이라고 판정하지 않는다.
다음 시험 전에 실제 ULog 증가도 다시 확인해야 한다.
웹·임무 서비스는 재개하지 않았다.

PC `outputs/lidar2d-enable-20261013/`에 증거를 보존했다.
세 수집의 topics.jsonl과 UART 시험 기록을 해시 대조했다.
PC와 Jetson의 SHA256이 모두 일치했다.
시각 결과는 각 회전 폴더의 `yaw-timing-deskew.json`이다.
센서 조회와 재시작 원문은 같은 최상위 폴더에 있다.
현재 하방 통신 중단을 이전 비행 밀림 원인으로 단정하지 않는다.

## 후속 bus 2 복구

**하방 거리 수신을 복구하고 60초 확인했다.**
사용자가 기존 콘솔의 bus 설정으로 복구를 요청했다.
기존 SD 스크립트와 정상 수신 기록을 다시 대조했다.
UART 변환 명령의 과거 실행 근거는 찾지 못했다.
현재 센서 통신 모드를 변환하지 않았다.

DISARM·landed=1과 두 거리 드라이버의 정지를 확인했다.
같은 MAVROS 연결로 다음 명령을 순서대로 실행했다.

```text
i2cdetect -b 2
tf02pro start -X -b 2 -a 16 -f 400 -R 25
tf02pro status
listener distance_sensor -n 1
```

버스 검사에서 주소 0x10의 응답을 확인했다.
기존 400kHz 시작 명령으로 드라이버가 실행됐다.
이 버스 검사는 완전한 수동 조회가 아니다.
현재 빌드의 i2cdetect는 실패 시 버스 reset을 호출한다.
센서 프로토콜을 UART로 바꾸는 명령은 아니다.
어떤 reset 시점에 응답이 회복됐는지는 미측정이다.
따라서 원래 중단 원인이나 영구 해결을 확정하지 않는다.

| 60초 확인 항목 | 결과 |
|---|---|
| MAVROS 하방 표본 | 591개 |
| FC 거리 | 0.687m, 조회 당시 나이 58ms |
| MAVROS 거리 | 0.68m. MAVLink cm 단위 전달값 |
| 표본 간격 중앙값 / 최대 | 102.003 / 102.010ms |
| 드라이버 통신 오류 | 0건 |
| 마지막 ROS 표본 수신 나이 | 85ms |
| 기체 | DISARM, POSCTL, landed=1 |
| 저장 파라미터·프로토콜 모드 변경 | 0건 |
| ARM·비행 명령·FC 재부팅 | 0건 |

기존 tf02pro 드라이버 하나를 실행 상태로 남겼다.
임시 UART tfmini는 종료 상태를 유지했다.
부팅 스크립트와 저장 파라미터는 바꾸지 않았다.
스캔 LiDAR 융합과 비행 준비 완료는 별개다.
시각 검증·실제 융합·기록 재개 검증은 여전히 남았다.

증거는 PC와 Jetson의 기존 enable 작업 폴더에 있다.
두 JSON의 SHA256을 PC와 Jetson에서 대조했다.
`bus2-recovery-detect-01.json`은 버스 응답 원문이다.
`bus2-recovery-start-01.json`은 60초 표본과 FC 조회다.

## 증거 위치

PC 작업 폴더는 아래와 같다.
`C:/Users/ASWPC_NTBOOK/Desktop/Drone5/outputs/lidar2d-crsf-upgrade-20261013/`

원본 펌웨어와 params를 함께 보존했다.
최종 펌웨어와 재현 패치도 보존했다.
`jetson-evidence/`에 원격 기록을 복사했다.
핵심 파일 8개의 PC·Jetson SHA256을 대조했다.
펌웨어·백업·params·검증 기록이 모두 일치했다.
`final-r2-fresh-verified/`에 최종 params가 있다.
`flash-r2.log`에 CRC 검증 결과가 있다.
`final-r2-hardware-readback.json`에 원문이 있다.
`final-r2-ulog-growth.json`에 기록 증가가 있다.

원격 작업 폴더는 아래와 같다.
`/home/arialhanho/lidar2d-crsf-upgrade-20261013/`

자동비행 재개 절차는
[현장 runbook](../runbooks/evening_web_flight_20261013.md)을 따른다.
LiDAR 실측 gate는
[LiDAR 실행 절차](../runbooks/lidar_physical_velocity.md)를 따른다.
