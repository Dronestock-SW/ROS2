# 사용자 제공 SITL 출력 발췌

이 문서는 대화에 제공된 출력의 발췌다. 시험 결과의 근거를 확인할 때 읽는다.

정리일: 2026-09-21. 실행 장비: Windows PC의 WSL Ubuntu.
사용자가 붙여 넣은 출력에서 필요한 줄만 골랐다.
블록마다 서로 다른 조회 시점이다.
전체 터미널 로그나 ULog 원본을 뜻하지 않는다.
작성자가 새로 실행해 얻은 출력은 아니다.

## Python 환경 — 사용자 텍스트

```text
/home/dronestock/.venvs/px4/bin/python
pip 22.0.2 from /home/dronestock/.venvs/px4/lib/python3.10/site-packages/pip (python 3.10)
```

Gazebo 버전 화면에는 `8.15.0`이 표시됐다.
같은 화면의 `pip check`는 `No broken requirements found.`였다.
화면 파일 자체는 이 저장소에 저장하지 않았다.

## 빌드 실패 당시 경로 — 사용자 텍스트

```text
protoc is /usr/bin/protoc
protoc is /bin/protoc
libprotoc 3.12.4
libprotoc 3.12.4
libgz-msgs10-dev 10.4.0-1~jammy
libprotobuf-dev 3.12.4-1ubuntu7.22.04.6
protobuf-compiler 3.12.4-1ubuntu7.22.04.6
Protobuf_DIR:PATH=/mnt/c/Users/A/anaconda3/Library/lib/cmake/protobuf
Protobuf_INCLUDE_DIR:PATH=/usr/include
```

## PX4 기동 — 사용자 텍스트

```text
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [logger] Opened full log file: ./fs/log/2026-09-21/01_24_46.ulg
INFO  [px4] Startup script returned successfully
```

로그 파일을 열었다는 메시지다. 파일 내용은 미수집이다.

## 가상 IMU — 사용자 텍스트에서 필드 발췌

```text
TOPIC: sensor_combined
    timestamp: 201652000 (0.008000 seconds ago)
    gyro_rad: [0.00153, 0.00056, 0.00005]
    accelerometer_m_s2: [0.00386, -0.00242, -9.80482]
    accelerometer_clipping: 0
    gyro_clipping: 0
```

## 지상 하방 거리 — 사용자 텍스트에서 필드 발췌

```text
TOPIC: distance_sensor
    timestamp: 265424000 (0.004000 seconds ago)
    device_id: 10092812 (Type: 0x9A, SIMULATION:1 (0x01))
    min_distance: 0.10000
    max_distance: 100.00000
    current_distance: 0.17701
    variance: 0.00000
    signal_quality: -1
    type: 0
    orientation: 25
```

## 위치·설정 — 사용자 텍스트에서 필드 발췌

```text
TOPIC: vehicle_local_position
    timestamp: 601548000 (0.000000 seconds ago)
    z: -0.13089
    dist_bottom: 0.17757
    xy_valid: True
    z_valid: True
    v_xy_valid: True
    v_z_valid: True
    heading_good_for_control: False
    dist_bottom_valid: True
```

```text
x     EKF2_RNG_CTRL [473,695] : 1
x     EKF2_HGT_REF [419,639] : 1
```

## EKF2 상태 — 사용자 텍스트에서 필드 발췌

```text
TOPIC: estimator_status_flags
    timestamp: 679704000 (0.940000 seconds ago)
    cs_tilt_align: True
    cs_yaw_align: True
    cs_gnss_pos: True
    cs_baro_hgt: True
    cs_rng_hgt: True
    cs_gps_hgt: True
    cs_ev_pos: False
    cs_ev_yaw: False
    cs_ev_hgt: False
    cs_ev_vel: False
    cs_rng_stuck: False
    cs_rng_fault: False
    cs_rng_kin_consistent: True
    cs_rng_terrain: True
    cs_gnss_vel: True
    cs_gnss_fault: False
```

## 지상국 연결 — 사용자 텍스트

실패 출력 뒤에 상대 주소와 준비 상태가 나왔다.

```text
INFO  [commander] Preflight check: FAILED
pxh> WARN  [health_and_arming_checks] Preflight Fail: No connection to the GCS
INFO  [mavlink] partner IP: 172.25.0.1
INFO  [commander] Ready for takeoff!
```

## 공중·착륙 후 거리 — 사용자 텍스트에서 필드 발췌

공중 조회:

```text
TOPIC: distance_sensor
    timestamp: 1474484000 (0.020000 seconds ago)
    current_distance: 2.52873
    orientation: 25
```

착륙 후 조회:

```text
TOPIC: distance_sensor
    timestamp: 1525764000 (0.004000 seconds ago)
    current_distance: 0.17701
    orientation: 25
```

이착륙 명령 자체의 전체 출력은 미수집이다.
시험 순서와 위 거리 변화를 함께 기록했다.

## 작업 보류 — 사용자 지시

> 음 잠깐 보류. 이유는 uwb HW팀에서 정비한댔어. ㄱㄷ

이후 UWB 관측을 SITL에 연결하는 작업은 진행하지 않았다.
