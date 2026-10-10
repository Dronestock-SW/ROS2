# 10월 10일 수집 공백 수정

수동 호버 대기 중 수행한 코드·지상 기록이다.
수집 준비와 미확인 비행 항목을 구분할 때 읽는다.

## 확인한 문제

저장공간 부족으로 자동 수집이 대기했다.
서비스 active만으로 기록 중이라고 볼 수 없다.
원인은 `storage_reserve_waiting_no_flight_deletion`이다.
여유 약 1,247MiB는 시작 기준 1,280MiB보다 작았다.
상태가 불명확한 과거 조각은 자동 삭제하지 않았다.

원본 한 조각의 107,680행도 분석했다.
조각 ID는 `capture-1791612967702162304-f267f248`다.

| 입력 | 최대 수신 공백 | 공백 뒤 헤더 나이 |
|---|---:|---:|
| FC state | 17.958초 | 16.961초 |
| IMU | 17.719초 | 1.003초 |
| PX4 odom | 17.722초 | 3.332초 |
| MAVLink source | 17.726초 | 0.220초 |

같은 조각의 ROS−monotonic 차이 범위는 0.728ms다.
약 18초 공백을 큰 호스트 시계 점프로 설명하지 못한다.
이전 시각의 FC 메시지가 뒤늦게 도착했다.
수집기·DDS 대기열 정체와 일치하는 형태다.
UWB 무선 단절만으로 단정하지 않는다.
실제 정체의 모든 원인은 아직 확정하지 않았다.

## 코드 수정

수신 루프에서 상태 파일 쓰기를 분리했다.
기존 원본 events 기록은 이미 비동기 방식이었다.
하지만 summary 쓰기와 STOP 조회는 동기 방식이었다.
두 파일 작업을 상태 전용 스레드로 옮겼다.

진행 중 상태 하나와 최신 대기 상태 하나만 보관한다.
콜백에서 JSON 문자열을 확정해 원본 상태를 보존한다.
종료 상태는 앞선 상태 뒤에 기록한다.
용량 초과·쓰기 오류·종료 지연은 오류로 남긴다.
오류는 성공 기록이나 보정 완료로 처리하지 않는다.

`capture_transport_audit`는 파일 전용 분석기다.
수신 공백·헤더 진행·시계 차이를 정수 ns로 비교한다.
미래 시각·시계 역행·불완전 행을 숨기지 않는다.
좌표·시간 보정값을 자동으로 적용하지 않는다.
FC 파라미터·모드·융합·제어 경로는 변경하지 않았다.

## 검증

Windows와 Jetson에서 각각 51개가 통과했다.
대상은 checkpoint, recording, capture, boot,
transport audit, analysis, mirror다.
지연 파일 쓰기 중 콜백 지속을 시험했다.
최종 상태 순서·오류 전파·원본 ns 보존도 시험했다.
실제 과거 조각의 분석 수치를 재현했다.
Jetson `colcon build --symlink-install --packages-select drone_uwb`가 통과했다.
코드 배포 기준은 `52c43a0`이다.
수집 서비스만 재시작했다.
MAVROS PID 2674는 재시작 전후 동일했다.
직전 FC 연결·DISARM·최신 state를 확인했다.
새 조각의 source_revision도 같은 커밋이다.
SITL·수동 호버·자동 비행은 이번에 실행하지 않았다.

Jetson 시험 명령은 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/drone_uwb/test/integration/test_{async_checkpoint,async_recording,capture_transport_audit,manual_capture,boot_capture,manual_analysis,capture_mirror}.py -q
```

기존 현장 Python 의존성 경로도 함께 사용했다.

## 배포 후 지상 수신

180.585초 조각이 정상 종료됐다.
141,126행·119,583,430바이트를 기록했다.
저장 오류가 없고 writer_drained=true다.
같은 조각을 설치된 C++ 실행기로 읽었다.
종료 코드 0이며 schema 1 호환을 확인했다.
`can_start`·`flight_authority`·`fusion_verified`는 false다.

events SHA256은 다음과 같다.
`842d5b53a6c0a681b449f269ce2656c7735381928686bb943a639a0c9b40d542`

| 입력 | 전체 조각 최대 공백 | 별도 관측기와 겹친 120초의 최대 공백 |
|---|---:|---:|
| UWB 원시 메시지 | 0.0297초 | 0.0297초 |
| IMU | 0.3379초 | 0.0271초 |
| PX4 odom | 0.3530초 | 0.0468초 |
| FC state, 원래 1Hz | 1.0068초 | 1.0068초 |
| B_TF pose | 24.1272초 | 검사 탈락에 따른 발행 공백 존재 |

가벼운 별도 관측기도 120초 함께 기록했다.
이 관측기는 ROS 메시지의 시각·횟수만 보관했다.
별도 관측기의 IMU 최대 공백은 0.0356초다.
odom은 0.0449초다. 기록기는 0.0468초였다.
전체 조각의 0.35초 공백은 이 비교 구간 이전이다.
그 원인과 장시간 안정성은 아직 확정하지 않는다.
이 조각의 IMU·odom 미래 헤더 수는 0이다.
기존 약 18초 정체는 이 짧은 구간에서 재현되지 않았다.
동일 조건의 원인 제거 시험으로 확대 해석하지 않는다.

지상 기록 도중 앵커 원시 수신이 시작됐다.
주요 B_TF 사유는 `no_height_consistent_subset`이다.
일부 좌표가 나와도 연속 유효 좌표로 간주하지 않는다.
사용자는 현장 배치 준비 중이며 수동 비행은 하지 않았다.
설정은 6.3×4.6m, 앵커 높이 0.15m를 유지했다.
새 배치의 정확한 좌표·높이는 실측 대조가 필요하다.

증거 파일은 `after-gap-audit.json`, `reference-gap-audit.json`,
`overlap-gap-audit.json`, `after-cpp-replay.json`이다.
후속 조각 원본은 같은 증거 폴더의 `post-deploy/`에 둔다.
이 원본은 Jetson에도 보존한다.

## 저장공간 복구

종료 기록 29개를 PC에 백업했다.
events·manifest·summary 총 87개 파일이다.
원본 합계는 1,965,764,098바이트다.
PC 압축본을 열어 파일별 SHA256을 대조했다.
Jetson 원본도 다시 해시 대조한 뒤 정리했다.
현재 조각과 미완성 조각은 보존했다.
정리 직후 여유는 3,199,148,032바이트였다.
수집기는 공간 확보 뒤 자동 재개했다.

PC 증거 폴더는 다음 위치다.
`Documents/Drone5-evidence/20261010/capture-audit/`
파일별 목록은 `manifest.json`에 있다.
압축본 검증은 `verified.json`에 있다.
정리 결과는 `reclaimed-first.json`, `reclaimed-rest.json`이다.
원본·개인 현장 설정은 Git에 추가하지 않았다.

전체 기록을 무기한 보존할 공간은 아니다.
같은 저장속도라도 새 UWB 입력량에 따라 용량이 달라진다.
비행 전 `status.json`과 실제 파일 증가를 확인한다.
오래된 미검증 조각을 삭제해 통과시키지 않는다.

## 남은 작업

현장 전원 조작으로 Jetson이 재부팅됐다.
사용자가 해당 조작을 확인했다.
같은 부팅의 관측·기록 상태를 다시 확인한다.
앵커 수신과 수동 호버 성공은 별도 확인한다.
동일 시행의 PX4 ULog도 회수한다.
그 뒤 좌표·시각 보정과 UWB 융합을 진행한다.
C++ 관측 adapter에는 기존 schema 1을 유지한다.
