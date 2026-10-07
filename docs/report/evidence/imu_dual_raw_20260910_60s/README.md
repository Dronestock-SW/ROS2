# 두 IMU의 60초 원본 기록 — 2026-09-10
이 문서는 이번 수집 결과의 설명이다.
센서별 CSV와 원본의 의미를 확인할 때 읽는다.

두 IMU를 같은 60초 동안 따로 읽었다.
각 센서에서 가속도 300개를 저장했다.
회전속도도 각 300개를 저장했다.
조회 간격은 약 0.2초다.
센서의 고속 출력 전체를 저장한 것은 아니다.

## 장비와 센서 구분

실제 장치 ID로 두 IMU를 구분했다.

| 항목 | 확인값 |
|---|---|
| 보드 | PX4_FMU_V6C / V6C002002 |
| PX4 | Release 1.16.0 |
| PX4 git hash | `6ea3539157ca358c70a515878b77077af7d4611d` |
| 수집 경로 | `/dev/pixhawk` → MAVLink 콘솔 |
| 내부 topic | `sensor_accel`, `sensor_gyro` |
| 수집 시간 | companion 시계로 60.00155초 |

| PX4 번호 | 모델 | 가속도 장치 ID | 회전속도 장치 ID |
|---|---|---:|---:|
| 0 | Bosch BMI088 | 6946826 | 6684682 |
| 1 | TDK/InvenSense ICM-42688-P | 2490378 | 2490378 |

번호는 이번 실행에서 확인한 값이다.
재부팅 후에도 같은 번호라고 가정하지 않는다.
장치 ID와 모델의 대응은
[PX4 v1.16.0 장치 정의](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/drivers/drv_sensor.h)로 확인했다.
실제 응답은 `console.raw`에 남겼다.

## 데이터 파일

CSV는 원본 콘솔의 숫자를 그대로 옮겼다.
companion에서 필터나 보정을 더하지 않았다.

| 파일 | 내용 |
|---|---|
| [imu0_raw.csv](imu0_raw.csv) | BMI088 가속도·회전속도 600행 |
| [imu1_raw.csv](imu1_raw.csv) | ICM-42688-P 가속도·회전속도 600행 |
| `imu0_accel.csv`, `imu0_gyro.csv` | BMI088 항목별 300행 |
| `imu1_accel.csv`, `imu1_gyro.csv` | ICM-42688-P 항목별 300행 |
| [console.raw](console.raw) | 조회 전후 상태를 포함한 콘솔 원문 |
| [queries.jsonl](queries.jsonl) | 명령별 응답과 companion 시각 |
| `mavlink_rx.bin` | 수신한 MAVLink 바이트 원문 |
| [summary.json](summary.json) | 개수·간격·장치 ID·통계 |
| [imu_comparison.png](imu_comparison.png) | 60초 비교 그래프 |
| `capture_window.json` | 수집 구간과 변경 여부 |
| `sha256.json` | 파일 무결성 확인값 |

가속도와 회전속도는 서로 다른 행이다.
같은 시각으로 보간하거나 묶지 않았다.
`mavlink_rx.bin`은 시간 포장을 넣은 tlog가 아니다.

## 숫자를 읽는 법

이 기록은 PX4 드라이버의 물리 단위 출력이다.
칩의 정수 레지스터 덤프는 아니다.
센서 장착 방향과 단위 변환이 반영된다.
드라이버가 여러 칩 샘플을 묶은 값도 반영된다.
[가속도 드라이버](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/drivers/accelerometer/PX4Accelerometer.cpp),
[회전속도 드라이버](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/drivers/gyroscope/PX4Gyroscope.cpp)를 기준으로 해석한다.

| CSV 필드 | 의미 |
|---|---|
| `imu_instance`, `device_id` | PX4 번호와 실제 장치 ID |
| `sensor` | `accel`: 가속도 / `gyro`: 회전속도 |
| `x`, `y`, `z` | 세 방향의 측정값 |
| `unit` | 가속도 m/s² / 회전속도 rad/s |
| `timestamp_sample` | PX4 부팅 기준 샘플 시각, µs |
| `timestamp` | PX4 부팅 기준 메시지 시각, µs |
| `host_send_monotonic_ns` | companion 조회 시작 시각, ns |
| `host_end_monotonic_ns` | companion 응답 수신 완료 시각, ns |
| `samples` | 이 메시지에 묶인 칩 샘플 수 |
| `temperature` | 온도 °C. `nan`은 값 미제공 |
| `error_count` | 드라이버가 보고한 누적 오류 수 |
| `clip_x`, `clip_y`, `clip_z` | 샘플의 축별 측정 범위 초과 수 |

축은 보드 FRD 기준이다.
+x는 앞, +y는 오른쪽, +z는 아래다.
앞서 MAVROS에서 본 FLU와 y·z 부호가 다르다.
수평으로 정지하면 가속도 z는 약 −9.8m/s²다.
중력의 영향이 포함되기 때문이다.
축과 필드는
[PX4 가속도 메시지 정의](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/msg/SensorAccel.msg)를 따른다.

두 시계의 원점은 같지 않다.
PX4 시각과 companion 시각을 바로 빼지 않는다.
이유: 그 차이가 센서 지연을 뜻하지 않는다.
각 조회는 순서대로 실행했다.
센서 간 시각이 완전히 같지는 않다.
그래프는 각 채널의 첫 샘플을 0초로 표시한다.

## 수집 품질과 한계

모든 예정 조회에서 네 채널을 확보했다.

| 채널 | 행 수 | 실제 조회율 | 최대 샘플 간격 |
|---|---:|---:|---:|
| BMI088 가속도 | 300 | 5.000Hz | 205.698ms |
| BMI088 회전속도 | 300 | 5.000Hz | 208.580ms |
| ICM-42688-P 가속도 | 300 | 5.000Hz | 207.554ms |
| ICM-42688-P 회전속도 | 300 | 5.000Hz | 210.056ms |

장치 ID 변경은 없었다.
샘플 시각의 중복·역전도 없었다.
확보한 행의 `error_count`는 모두 0이었다.
확보한 행의 측정 범위 초과도 0이었다.
ICM 누적 범위 초과 `[0, 28, 5]`는
수집 전부터 있었고 종료 후에도 같았다.

센서는 조회보다 훨씬 빠르게 측정한다.
종료 후 내부 보고값은 다음과 같다.
이는 CSV 저장률과 다른 수치다.

| 센서 | 가속도 내부 원시 측정률 | 회전속도 내부 원시 측정률 |
|---|---:|---:|
| BMI088 | 약 1,604Hz | 약 2,000Hz |
| ICM-42688-P | 약 7,998Hz | 약 7,998Hz |

조회 사이의 고속 샘플은 이 CSV에 없다.
따라서 전체 고속 데이터의 무결손을 뜻하지 않는다.
사용자가 완전 정지를 확인한 시험은 아니다.
기준 자세와 회전속도도 따로 측정하지 않았다.
이 기록만으로 정확도나 교정 완료를 판정하지 않는다.
두 센서의 숫자를 단순 평균하지 않았다.

## 재현에 사용한 도구

수집 도구는 읽기 전용 명령만 전송했다.
PX4 파라미터와 센서 발행률을 바꾸지 않았다.
시동·비행 명령도 전송하지 않았다.
수집 종료 후 연결을 닫았다.

- 수집: `src/drone_uwb/tools/px4_imu_raw_capture.cpp`
- CSV·그래프: `src/drone_uwb/tools/summarize_imu_raw.py`
- 명령: `listener sensor_accel -n 1`
- 명령: `listener sensor_gyro -n 1`

매번 두 번호를 모두 조회했다.
동작 근거는
[PX4 v1.16.0 listener 구현](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/systemcmds/topic_listener/listener_main.cpp)이다.
소스 컴파일과 전체 workspace 빌드가 통과했다.
