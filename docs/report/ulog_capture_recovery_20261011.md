# ULog 복구와 LiDAR 동시 기록 결과
2026-10-11 지상 기록 복구 결과다.
수동 비행 로그를 회수하거나 재현할 때 읽는다.

## 결론

FC ULog와 companion 동시 기록을 복구했다.
FC 파라미터·ARM·모드·재부팅 명령은 보내지 않았다.
수동 비행·LiDAR 이동량·PX4 융합 검증은 남았다.

| 구분 | 확인 결과 |
|---|---|
| FC ULog | 새 파일 생성·같은 파일의 크기 증가 확인 |
| companion 저장공간 | 검증 백업 후 1,131,502,043 bytes 확보 |
| UWB/FC/LiDAR 동시 기록 | 180.663초 조각 정상 종료·writer_drained=true |
| 다음 조각 | active, 필수 토픽 누락 없음, error=null |
| FC 파라미터 | 고정 조회 12개 전후 동일. 쓰기 명령 없음 |
| LiDAR 서비스 | 활성·부팅 enable. 이번 변경 후 전원 재인가 미시험 |
| 단위시험 | Windows 56개 통과 |
| shell·systemd | bash -n·systemd-analyze verify 통과 |
| 실물 비행·SITL·LiDAR 스캔매칭 | 이번 작업에서 미실시 |

## ULog가 없던 이유

이번 `Not logging`은 기록 정책과 일치했다.
`SDLOG_MODE=1`은 첫 DISARM에서 부팅 기록을 닫는다.
그 뒤 ARM에서 다시 기록한다.
`mode=all`은 백엔드 표시다. 연속 기록 설정값이 아니다.
[PX4 기록 설명](https://docs.px4.io/main/en/dev_log/logging)을 참고한다.

FC는 PX4 1.17.0 / FMU_V6C였다.
실행 펌웨어와 보존한 소스의 hash가 같았다.
hash: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
`logger.cpp`의 on/off·start_stop_logging을 대조했다.
`logger on`은 내부 기록 override만 켠다.
실제 FC ARM 상태를 바꾸는 명령이 아니다.

dmesg에는 이전 기록의 정상 close가 있었다.
이번 조회에서는 새 `errno27`을 확인하지 않았다.
과거 2GiB 파일 상한 문제와 구분한다.
SD는 약 1.50GiB가 남아 있었다.
FC가 과거 로그 폴더를 자동 삭제한 이력은 있었다.
이번 작업은 FC SD 파일을 삭제하지 않았다.

지상·DISARM 상태에서 logger on을 실행했다.
파일: `/fs/microsd/log/2026-10-10/11_04_00.ulg`.
최초 5초 확인에서 약 178KB→388KB로 증가했다.
후속 조회에서도 같은 파일 증가를 확인했다.
기록은 유지 중이다. 착륙·DISARM 후 닫아 회수한다.

FC 파일명 날짜는 companion 날짜와 달랐다.
companion 시각은 10월 11일이었다.
파일명만으로 같은 비행을 판정하지 않는다.
boot_id·원본 시각·timesync·ARM 구간을 대조한다.
ULog 파일 자체의 PC 회수·파싱은 아직 미실시다.

## 저장공간과 수집기

실제 기록 차단 원인은 companion 용량 부족이었다.
시작 시 여유는 1,310,892,032 bytes였다.
다음 조각 시작 기준 1,280MiB보다 작았다.
status는 `storage_reserve_waiting_no_flight_deletion`이었다.

종료 지상 조각 15개·45파일을 PC에 백업했다.
archive 전체와 각 파일의 SHA256을 대조했다.
원본도 다시 대조한 뒤 해당 파일만 삭제했다.
현재 기록·비행·중단·상태 불명 자료는 보존했다.
여유가 2,442,563,584 bytes로 늘어 수집이 재개됐다.
동시 기록 확인 시 여유는 약 1.83GiB였다.
원시 기록량은 약 0.85MB/s였다.
남은 시간은 보존 대상·새 기록에 따라 달라진다.

PC 증거 경로는 Git 밖에 있다.
`Documents/Drone5-evidence/20261011/ulog-recovery/`.
`ground-plan.json`, `ground-verified.json`, `deletion.json`을 보존한다.
archive SHA256은 다음과 같다.
`f2906a845d46079c1dd2ffcc8aa53264c3ff0a4dc4df411d63819fba03ebeb64`.

## LiDAR 연결

USB LiDAR의 `/scan`을 기존 수집기에 추가했다.
`/dev/lidar`는 USB `1-2.2`, ttyUSB0였다.
UWB는 USB `1-2.3`, ttyUSB1이었다.
LiDAR 포트의 기존 소유자는 없었다.
기존 observe·MAVROS는 재시작하지 않았다.

기존 T-mini Pro 설정을 사용했다.
230400 baud, 10Hz, reversion/inverted=true다.
제조사 ROS 드라이버 revision은 `4ef70d3`이다.
장착 방향이 검증됐다는 의미는 아니다.
manufacturer launch의 임의 2cm TF는 사용하지 않았다.
실측 장착 TF를 만들어 넣지 않았다.

| 180.663초 지상 구간 | 표본 수 | 최대 수신 공백 |
|---|---:|---:|
| `/scan` | 1,778 | 0.107574초 |
| `/uwb/received` | 14,481 | 0.045395초 |
| `/uwb/btf_pose` | 7,215 | 0.054049초 |

위 공백은 수신 간격이다. 보정된 센서 지연이 아니다.
이 조각도 PC archive에 별도로 검증 보존했다.
`first-lidar-ground-verified.json`에 파일별 SHA256이 있다.
B_TF header 나이는 최대 약 0.494초였다.
정지 수신 성공으로 동적 시각 보정을 승인하지 않는다.

첫 LiDAR 표본은 430광선 중 360점이 유효했다.
frame은 `laser_frame`이다.
`scan_time=time_increment=0` 표본을 확인했다.
ROS 드라이버는 SDK 값을 그대로 전달한다.
개별 광선 시각·deskew 검증은 남아 있다.
TF 미수신도 그대로 기록했다.

## 변경과 재현

기존 C++ 관측 경계를 유지했다.
LiDAR는 상대 이동량을 검증할 원본 자료다.
이번 변경은 이동량 추정기나 FC aiding 구현이 아니다.

| 파일 | 변경 |
|---|---|
| `px4_logger_session.py` | 지상 DISARM 기록 on/off·SD 용량·파일 증가 검사 |
| `test_logger_session.py` | 고정 명령·지상 신선도·출력 파싱 검사 |
| `run_manual_capture.sh` | 선택 LiDAR 노드와 설정 증거 기록 |
| `dronestock-manual-lidar.service` | 별도 센서 서비스. 기존 포트 소유자 보존 |
| `manual-capture.env.example` | 기본 비활성 LiDAR 설정 예시 |

현장 환경 파일·기존 wrapper를 먼저 백업했다.
현장에서 `MANUAL_CAPTURE_LIDAR=1`로 선택했다.
LiDAR 서비스 시작 뒤 수집기만 지상 재시작했다.
다음 전원 재인가 때 LiDAR 동시 기록을 재확인한다.

```bash
python3 src/sangwon_AI/ops/px4_logger_session.py \
  --ros-domain 2 --start --output /private/ulog-session.json
```

실행 환경·회수 명령은 [수집 절차](../runbooks/manual_capture_boot.md)를 따른다.

```powershell
$env:PYTHONPATH='src/drone_uwb'
python -m pytest src/sangwon_AI/tests/test_logger_session.py `
  src/sangwon_AI/tests/test_mavros_readback.py `
  src/drone_uwb/test/integration/test_manual_capture.py `
  src/drone_uwb/test/integration/test_boot_capture.py -q
```

첫 Windows 실행은 PYTHONPATH 누락으로 수집에 실패했다.
경로 설정 뒤 56개 시험을 통과했다.
FC 재조회 한 번은 빈 shell 응답으로 종료됐다.
자동 성공 처리하지 않았다. 재조회는 모두 완료됐다.

## 남은 실측

수동 비행 뒤 세 기록을 같은 구간으로 대응한다.
① FC ULog ② UWB/IMU/ToF ③ LiDAR 원본이다.
안정 호버·흐름·이동 구간을 구분해 분석한다.
LiDAR 장착 벡터·축·시각 의미부터 확인한다.
이후 상대 이동량·품질·공분산을 C++ 경계로 연결한다.
UWB로 보정한 값을 독립 검증값으로 재사용하지 않는다.
고도와 최종 비행 상태 추정은 PX4가 담당한다.
