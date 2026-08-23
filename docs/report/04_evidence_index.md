# SW 증빙 목록 (중간보고서 첨부용)

> 이 문서는 `docs/report/evidence/` 에 수집한 실행 출력의 색인이다.
> 보고서 각 항목에 어떤 파일을 첨부할지 고를 때 읽는다.
> 수집일: 2026-08-16 / 수집 장비: 1호기 companion (Jetson Orin Nano, user-desktop)

---

## 결론 3줄

```text
1. 증빙 11건 수집. 성공 6건, 부분 실패 2건, 실패 3건.
2. 실패 5건은 감추지 않고 실패 출력 그대로 보관했다 — 미연결·미구현이 그대로 드러난다.
3. 실패 사유는 두 가지뿐 — 하드웨어 미연결, 또는 코드 미작성.
```

---

## 1. 증빙 색인

| ID | 파일 | 대상 | 결과 | 보고서 연결 항목 |
|---|---|---|:---:|---|
| D-01 | [evidence/01_env_versions.txt](evidence/01_env_versions.txt) | 개발환경 버전 | 성공 | [01_sw_status.md](01_sw_status.md) §1.4 / 초안 §1 |
| D-02 | [evidence/02_devices_udev.txt](evidence/02_devices_udev.txt) | 장치 노드 · udev 적용 상태 | **부분 실패** | 01 §2.1, §3 / 초안 §4 |
| D-03 | [evidence/03_colcon_build.txt](evidence/03_colcon_build.txt) | 워크스페이스 빌드 | 성공 | 01 §1.1 / [02_timeline_wbs.md](02_timeline_wbs.md) |
| D-04 | [evidence/04_lidar_launch.txt](evidence/04_lidar_launch.txt) | LiDAR launch + `/scan` | 성공 | 01 §2.1 / [05_architecture.md](05_architecture.md) §2 |
| D-05 | [evidence/06_uwb_fail.txt](evidence/06_uwb_fail.txt) | UWB 태그 통신 | **실패** | 01 §2.1 / [03_metrics_plan.md](03_metrics_plan.md) §1 |
| D-06 | [evidence/07_mavros_fail.txt](evidence/07_mavros_fail.txt) | MAVROS heartbeat | **실패** | 01 §2.2 / 초안 §4 |
| D-07 | [evidence/08_qr_marker_fail.txt](evidence/08_qr_marker_fail.txt) | QR / ArUco 인식 | **실패** | 01 §2.3 / 03 §3 |
| D-08 | [evidence/09_comm.txt](evidence/09_comm.txt) | Tailscale · LoRa · 웹 API | **부분 실패** | 01 §2.4 |
| D-09 | [evidence/05_camera_launch.txt](evidence/05_camera_launch.txt) | 카메라 launch + 스트림 | 성공 | 01 §2.3 / 05 §2 |
| D-10 | [evidence/10_node_graph.txt](evidence/10_node_graph.txt) | 전체 launch 동시 기동 · 노드 그래프 | 성공 | 01 §1.2 / 05 §1 |
| D-11 | [evidence/metrics_selftest/](evidence/metrics_selftest/) | 측정 스크립트 자체검증 | 성공 | 03 §7 / [scripts/README.md](scripts/README.md) §6 |

---

## 2. 증빙별 상세

### D-01 개발환경 버전 — 성공

| 항목 | 값 |
|---|---|
| 실행 명령 | `lsb_release -a`, `uname -a`, `cat /etc/nv_tegra_release`, `dpkg -l`, `colcon version-check` |
| 확인 내용 | Ubuntu 22.04.5 / L4T R36.4.7 / ROS2 Humble / Python 3.10.12 / ROS_DOMAIN_ID=1 |
| 증빙 용도 | 보고서 "개발환경" 표의 근거 |

### D-02 장치 노드 · udev — 부분 실패

| 장치 | 결과 |
|---|---|
| `/dev/lidar` | **성공** — `ttyUSB0` 심볼릭 링크 확인 |
| `/dev/video0` | **성공** — CSI CAM0 |
| `/dev/uwb` | **실패** — 장치 없음 (태그 미연결) |
| `/dev/pixhawk` | **실패** — 장치 없음 (기체 미연결) |

부가 발견: 저장소 원본과 `/etc/udev/rules.d/` 설치본이 **다르다**. 설치본은 2026-07-29자 구버전으로,
Pixhawk 규칙이 주석 처리되어 있고 UWB 주석에 갱신 전 값("약 12Hz")이 남아 있다.

조치: `sudo cp config/udev/99-dronestock.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`

### D-03 워크스페이스 빌드 — 성공

```text
Starting >>> ydlidar_ros2_driver     Finished <<< [3.04s]
Starting >>> drone_bringup           Finished <<< [3.43s]
Summary: 2 packages finished [7.60s]     종료코드 0
```

증빙 용도: "main은 항상 빌드되는 상태 유지" 규칙([README.md:32](../../README.md#L32))의 준수 근거.

### D-04 LiDAR — 성공

| 확인 항목 | 결과 |
|---|---|
| 포트 연결 | `Lidar successfully connected [/dev/lidar:230400]` |
| 모델 확인 | `Model: Tmini Pro`, Firmware 1.1, Hardware 2, Serial 2026010700095010 |
| 스캔 주파수 | `Scan Frequency: 10.00Hz` |
| 토픽 발행 | `/scan` 평균 9.964~9.970 Hz (10초, window 71) |
| 메시지 내용 | `frame_id: laser_frame`, `range_min 0.03 / range_max 12.0`, `angle_increment 0.014646` |
| QoS | BEST_EFFORT / VOLATILE |
| 파라미터 반영 | `port: /dev/lidar`, `frequency: 10.0` — yaml 값과 일치 |
| 구독자 | 0 (소비 노드 미구현) |

기록해 둔 경고: 기동 초기 `Checksum error` 4회, `Real points 640 > fixed points 430` 1회.
발행은 정상 지속. **정상 여부 판단은 미확인** — 제조사 문서 확인 필요.

### D-05 UWB — 실패

| 실패 사유 | 확인 방법 |
|---|---|
| 태그 미연결 | `ls -l /dev/uwb` → 없음. `lsusb`에 CP2102가 1개(LiDAR)뿐 |
| 노드 미구현 | `ls src/` → `drone_uwb` 패키지 없음. `ros2 topic list \| grep uwb` → 출력 없음 |

**두 가지가 동시에 걸려 있다.** 태그를 꽂아도 발행할 노드가 없다.
설계는 완료 상태 — [docs/uwb_node_design.md](../uwb_node_design.md) (단, git 미추적).

### D-06 MAVROS / Pixhawk — 실패

| 항목 | 결과 |
|---|---|
| MAVROS 패키지 | 성공 — `mavros`, `mavros_extras`, `mavros_msgs` 확인 |
| geoid 데이터셋 | 성공 — `egm96-5.pgm` 존재 |
| Pixhawk 연결 | **실패** — `lsusb \| grep 3185` 출력 없음 |
| MAVROS launch | **실패** — 저장소에 파일 없음 |

보고서 주의: roadmap.md:93이 "MAVROS connected"를 완료로 표기하나 **저장소에 그 확인 로그가 없다.**
초안에는 "환경 준비 완료 / 연동 미구현"으로 적었다.

### D-07 QR · ArUco — 실패

| 항목 | 결과 |
|---|---|
| 우리 코드 | 구현 없음. 주석 언급 2건뿐 |
| ROS2 패키지 | 미설치 (`aruco`, `apriltag`, `zbar` 검색 결과 없음) |
| OpenCV | 4.5.4, `QRCodeDetector`·`aruco` 모듈 사용 가능 |
| pyzbar | 미설치 |
| 카메라 내부 파라미터 | 빈 값 → 거리·각도 산출 불가 |

### D-08 통신 — 부분 실패

| 항목 | 결과 |
|---|---|
| Tailscale | **성공** — `tailscaled active`, IP 100.110.163.94, 피어 3대 |
| ROS_DOMAIN_ID | **성공** — `.bashrc:120`에 `export ROS_DOMAIN_ID=1` |
| LoRa | **실패** — 관련 코드 0건 |
| 웹 API 경계 토픽 5종 | **실패** — 구현 0건. 문서 계약만 존재 |

부가 기록: `tailscale status`에 iptables `connmark` 모듈 부재 경고 1건. 현재 접속에는 영향 없음.

### D-09 카메라 — 성공

| 확인 항목 | 결과 |
|---|---|
| GStreamer 파이프라인 | `nvarguscamerasrc ... 1640x1232 ... 30/1` 적용 확인 |
| 센서 모드 | GST_ARGUS `Camera mode = 3`, `1640 x 1232 FR = 29.999999 fps` |
| 이미지 메타 | 1640×1232, `rgb8`, step 4920, frame_id `camera_link` |
| `camera_info` | **빈 값** — `height: 0, width: 0, k: [0.0 ×9]` |
| 캘리브레이션 파일 | 부재 경고 — `~/.ros/camera_info/imx219.yaml not found` |

`camera_info`가 빈 값인 것은 **캘리브레이션 미실시에 따른 예상된 상태**다. 오류가 아니다
([camera_imx219.yaml:56-59](../../src/drone_bringup/params/camera_imx219.yaml#L56-L59)).

부가 기록: `RTPS_TRANSPORT_SHM Error ... Failed init_port` 경고가 반복 출력.
Fast-DDS 공유메모리 포트 관련 경고이며 토픽 통신은 정상 동작. **원인 미확인.**

### D-10 노드 그래프 — 성공

launch 2개를 동시 기동해 채취. 노드 4개(launch 내부 노드 1개 포함), 토픽 11개, 서비스 28개.
`ros2 node info` 출력으로 노드별 발행·구독 관계까지 포함.

[05_architecture.md](05_architecture.md)의 mermaid 다이어그램은 전부 이 파일에 근거한다.

### D-11 측정 스크립트 자체검증 — 성공

| 항목 | 결과 |
|---|---|
| 기록 | `record_metrics.sh -d 20 -l selftest` → bag 1.6 MB, 770 메시지 |
| 산출 | `analyze_metrics.py` → CSV 4개, PNG 3개, 요약 2개 |
| `/scan` | 192 메시지 / 19.149s / 평균 9.9743 Hz |
| `/camera/camera_info` | 578 메시지 / 19.234s / 평균 29.9985 Hz |
| 한글 그래프 | 정상 렌더링 |

이 항목은 **측정 도구가 동작함**의 증빙이지, 성능 목표 달성의 증빙이 아니다.

---

## 3. 실패 항목 정리

숨기지 않고 그대로 적는다.

| ID | 실패 대상 | 사유 분류 | 해소 조건 |
|---|---|---|---|
| D-02 | `/dev/uwb`, `/dev/pixhawk` | 하드웨어 미연결 | 장치 연결 |
| D-02 | udev 규칙 최신본 미적용 | 절차 미수행 | `sudo cp` + `udevadm` 재적용 |
| D-05 | UWB 통신 | 하드웨어 미연결 **+** 코드 미작성 | 태그 연결 + `uwb_node` 작성 |
| D-06 | MAVROS heartbeat | 하드웨어 미연결 **+** 코드 미작성 | 기체 연결 + MAVROS launch 작성 |
| D-07 | QR 인식 | 코드 미작성 | 인식 노드 작성 + 캘리브레이션 |
| D-08 | LoRa, 웹 API | 코드 미작성 | Phase 1 / Phase 4 |

사유는 두 종류뿐이다 — **하드웨어 미연결** 또는 **코드 미작성**. 설정 오류·빌드 실패로 인한 실패는 0건.

---

## 4. 재현 절차

수집 시점과 같은 출력을 다시 얻는 방법이다.

```bash
cd ~/drone_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# D-01 환경
lsb_release -a; uname -a; cat /etc/nv_tegra_release; dpkg -l | grep ros-humble-

# D-02 장치
ls -l /dev/lidar /dev/uwb /dev/pixhawk /dev/video0
diff config/udev/99-dronestock.rules /etc/udev/rules.d/99-dronestock.rules

# D-03 빌드
colcon build --symlink-install

# D-04 LiDAR
ros2 launch drone_bringup lidar.launch.py &
ros2 topic hz /scan; ros2 topic info /scan --verbose; ros2 param dump /ydlidar_ros2_driver_node

# D-09 카메라
ros2 launch drone_bringup camera.launch.py &
ros2 topic echo /camera/image_raw --once --no-arr; ros2 topic echo /camera/camera_info --once

# D-10 노드 그래프 (둘 다 띄운 상태에서)
ros2 node list; ros2 topic list -t; ros2 service list
ros2 node info /ydlidar_ros2_driver_node; ros2 node info /gscam_node

# D-11 측정
./docs/report/scripts/record_metrics.sh -d 20 -l selftest
python3 docs/report/scripts/analyze_metrics.py ~/drone_ws/metrics_bags/<bag이름>
```

`ros2 node list`가 빈 결과를 낼 때가 있다 — DDS discovery가 아직 안 끝난 경우다.
기동 후 15초 이상 기다린 뒤 실행한다 (D-04 수집 시 실제로 빈 결과가 나왔고, D-10에서는 정상).

---

## 5. 증빙에 없는 것 (수집 불가)

| 항목 | 사유 | 확보 방법 |
|---|---|---|
| UWB 실측 로그 | 장치 미연결 + 노드 미구현 | uwb_node 작성 후 D-11 절차 |
| 비행 로그 | 비행 제어 미구현 | Phase 1 이후 |
| PX4 파라미터 현황 | companion에서 조회 불가 (기체 미연결) | QGroundControl — HW팀 작성분 |
| TFmini Plus / PMW3901 출력 | PX4 직결이라 companion에 보이지 않음 | 동일 |
| 과거 시점 로그 | `.gitignore`가 `log/` 제외. 저장 이력 없음 | 재취득만 가능 |
