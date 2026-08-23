# SW 구현 현황 (중간보고서용)

> 이 문서는 저장소 스캔 + 실제 명령 실행 결과로 정리한 SW 파트 구현 현황이다.
> 보고서의 "개발 현황" 항목 작성 시 읽는다. 계획·목표가 아니라 현재 상태만 적는다.
> 기준 시점: 2026-08-16 / 기준 커밋: `7438c17` (미커밋 변경 2건 별도 표기)

---

## 결론 3줄

```text
1. 자체 작성 ROS2 노드 = 0개. 저장소의 코드는 외부 드라이버를 띄우는 launch 2개 + 파라미터 2개다.
2. 실기 검증 완료 = LiDAR(/scan), 카메라(/camera/image_raw) 2개 경로뿐.
3. UWB·MAVROS·QR·LoRa·임무제어는 전부 미착수. 문서 설계만 존재한다.
```

**작성 범위**: SW 영역만. 기구부·프레임·배선·부품 조달·비행 성능은 다른 팀 소관이라 제외.
단 SW가 제어·수신하는 하드웨어 인터페이스(드라이버, 토픽, 파라미터)는 포함.

---

## 1. 구성

### 1.1 디렉터리 구조와 패키지 역할

```text
drone_ws/
├── src/
│   ├── drone_bringup/          자체 패키지. launch + params 전용
│   │   ├── launch/             lidar.launch.py, camera.launch.py
│   │   ├── params/             lidar_tmini.yaml, camera_imx219.yaml
│   │   ├── drone_bringup/      __init__.py 만 존재 (노드 소스 없음)
│   │   └── test/               ament 표준 린트 3종 (copyright/flake8/pep257)
│   └── ydlidar_ros2_driver/    제조사 저장소 서브모듈. 수정하지 않음
├── config/udev/                99-dronestock.rules — 장치 고정 이름 규칙
└── docs/                       설계·기준 문서 (코드 아님)
```

| 패키지 | 종류 | 빌드 타입 | 역할 | 근거 |
|---|---|---|---|---|
| `drone_bringup` | 자체 | `ament_python` | launch·파라미터 보관. **실행 노드 미포함** | [package.xml](../../src/drone_bringup/package.xml), [setup.py:29-32](../../src/drone_bringup/setup.py#L29-L32) |
| `ydlidar_ros2_driver` | 서브모듈 | `ament_cmake` | YDLIDAR T-mini Pro 드라이버 | [.gitmodules](../../.gitmodules), 커밋 `bf3c96d` |

`drone_bringup`의 `entry_points.console_scripts`가 빈 리스트다 → **자체 실행 노드 0개**
([setup.py:29-32](../../src/drone_bringup/setup.py#L29-L32)).

서브모듈 고정 상태: `4ef70d3` (branch `humble`) — 증빙 [evidence/03_colcon_build.txt](evidence/03_colcon_build.txt)

빌드 결과: `colcon build --symlink-install` → 2 packages finished, 종료코드 0
(2026-08-16 실행, 증빙 [evidence/03_colcon_build.txt](evidence/03_colcon_build.txt))

---

### 1.2 노드 / 토픽 / 서비스 / 파라미터

아래는 저장소의 launch 2개를 **동시에 실제 기동해** 채취한 목록이다.
증빙 원문: [evidence/10_node_graph.txt](evidence/10_node_graph.txt)

#### 노드 (전부 외부 패키지 소속)

| 노드 | 소속 패키지 | 기동 경로 | 자체 작성 여부 |
|---|---|---|---|
| `/ydlidar_ros2_driver_node` | ydlidar_ros2_driver (제조사) | lidar.launch.py | 아니오 |
| `/static_tf_pub_laser` | tf2_ros (제조사 launch가 포함) | lidar.launch.py | 아니오 |
| `/gscam_node` | gscam (apt) | camera.launch.py | 아니오 |

#### 토픽

| 토픽 | 타입 | 발행자 | 상태 |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | ydlidar_ros2_driver_node | 발행 확인 |
| `/point_cloud` | `sensor_msgs/msg/PointCloud` | ydlidar_ros2_driver_node | 발행 확인 (미사용) |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | gscam_node | 발행 확인 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | gscam_node | 발행되나 **전 필드 0** (캘리브레이션 미실시) |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | image_transport 플러그인 | 자동 생성 |
| `/camera/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | image_transport 플러그인 | 자동 생성 |
| `/camera/image_raw/theora` | `theora_image_transport/msg/Packet` | image_transport 플러그인 | 자동 생성 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | static_tf_pub_laser | `base_link → laser_frame` (0, 0, 0.02) |

`/camera/camera_info` 실측 내용: `height: 0, width: 0, distortion_model: '', k: [0.0 ×9]`
→ 캘리브레이션 파일 부재 경고 후 빈 값 발행 ([evidence/05_camera_launch.txt](evidence/05_camera_launch.txt))

**README의 Topic 표와 현실의 차이** — [README.md:45-56](../../README.md#L45-L56)에 8개 토픽이 적혀 있으나
실제 존재하는 것은 `/scan` 하나다. 나머지 7개(`/uwb_pose`, `/target_pose`, `/target_valid`,
`/cmd_vel`, `/tfmini_range`, `/safety_stop`, `/flight_state`)는 **계약만 있고 발행자 없음**.
증빙: [evidence/09_comm.txt](evidence/09_comm.txt) (grep 결과 없음)

#### 서비스

| 서비스 | 타입 | 제공 노드 | 용도 |
|---|---|---|---|
| `/start_scan` | `std_srvs/srv/Empty` | ydlidar_ros2_driver_node | LiDAR 스캔 시작 |
| `/stop_scan` | `std_srvs/srv/Empty` | ydlidar_ros2_driver_node | LiDAR 스캔 정지 |
| `/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | gscam_node | 캘리브레이션 값 주입 |
| `/ydlidar_ros2_driver_node/change_state` | `lifecycle_msgs/srv/ChangeState` | ydlidar_ros2_driver_node | lifecycle 전이 |

자체 정의 서비스·액션·메시지 타입: **없음**. `msg/`, `srv/`, `action/` 디렉터리 부재.

#### 파라미터 (자체 관리분만)

| 파일 | 대상 노드 | 항목 수 | 우리가 바꾼 값 |
|---|---|---|---|
| [lidar_tmini.yaml](../../src/drone_bringup/params/lidar_tmini.yaml) | `ydlidar_ros2_driver_node` | 24 | `port: /dev/ttyUSB0 → /dev/lidar` **1개뿐** |
| [camera_imx219.yaml](../../src/drone_bringup/params/camera_imx219.yaml) | `gscam_node` | 7 | 전체 (제조사 기본값 없음) |

런타임 반영 확인: `ros2 param dump`로 `port: /dev/lidar`, `frequency: 10.0`,
`gscam_config: nvarguscamerasrc ... 1640x1232 ... 30/1`, `image_encoding: rgb8` 반영 확인
([evidence/04_lidar_launch.txt](evidence/04_lidar_launch.txt), [evidence/05_camera_launch.txt](evidence/05_camera_launch.txt))

`camera_imx219.yaml`에서 아직 꺼져 있는 항목: `camera_info_url` 주석 처리 상태
([camera_imx219.yaml:56-59](../../src/drone_bringup/params/camera_imx219.yaml#L56-L59))

---

### 1.3 launch 파일과 각각이 띄우는 구성

| launch | 띄우는 노드 | 파라미터 | 인자 | 근거 |
|---|---|---|---|---|
| [lidar.launch.py](../../src/drone_bringup/launch/lidar.launch.py) | 제조사 `ydlidar_launch.py` 포함 → `ydlidar_ros2_driver_node` + `static_tf_pub_laser` | `lidar_tmini.yaml` | `params_file` | 커밋 `bf3c96d` |
| [camera.launch.py](../../src/drone_bringup/launch/camera.launch.py) | `gscam/gscam_node` 직접 실행 | `camera_imx219.yaml` | `params_file` | 커밋 `3f7f136` |

launch 파일 총 2개. **통합 bringup launch 없음** — 두 센서를 한 번에 띄우는 구성은 미작성.

---

### 1.4 외부 의존성과 개발환경 버전

전부 2026-08-16에 명령으로 확인. 증빙 [evidence/01_env_versions.txt](evidence/01_env_versions.txt)

| 항목 | 값 | 확인 명령 |
|---|---|---|
| OS | Ubuntu 22.04.5 LTS (jammy) | `lsb_release -a` |
| 커널 | 5.15.148-tegra, aarch64 | `uname -a` |
| JetPack / L4T | R36 REVISION 4.7 (2025-09-18 빌드) | `cat /etc/nv_tegra_release` |
| ROS2 | Humble (`ros-humble-ros-base` 0.10.0) | `echo $ROS_DISTRO`, `dpkg -l` |
| Python | 3.10.12 | `python3 --version` |
| colcon-core | 0.21.0 | `colcon version-check` |
| ROS_DOMAIN_ID | 1 (`.bashrc:120`에 고정) | `echo $ROS_DOMAIN_ID` |

패키지 의존성 — `package.xml` 선언분과 실제 설치 버전:

| 의존 패키지 | 선언 위치 | 설치 버전 | 용도 |
|---|---|---|---|
| `ydlidar_ros2_driver` | package.xml:12 | 서브모듈 `4ef70d3` | LiDAR |
| `gscam` | package.xml:13 | 2.0.2 | CSI 카메라 |
| `launch`, `launch_ros` | package.xml:10-11 | ros-base 동봉 | launch |

`package.xml`에 선언되지 않았으나 설치되어 있는 것 (향후 Phase용):

| 패키지 | 버전 | 설치일 | 예정 용도 |
|---|---|---|---|
| `mavros` / `mavros-extras` / `mavros-msgs` | 2.14.0 | 2026-07-24 | Phase 1 PX4 연동 |
| `cv-bridge` | 3.2.1 | 2026-08-12 | Phase 3 영상 처리 |
| `image-transport` | 3.1.13 | 2026-08-12 | Phase 3 |
| `image-transport-plugins` | 2.5.5 | 2026-08-12 | Phase 3 |
| `camera-info-manager` | 3.1.13 | 2026-08-12 | Phase 3 캘리브레이션 |
| `camera-calibration` | 3.0.9 | 2026-08-12 | Phase 3 캘리브레이션 |
| `image-proc` | 3.0.9 | 2026-08-12 | Phase 3 |
| `rosbag2` / `ros2bag` | 0.15.16 | ros-base 동봉 | 측정 기록 |

설치 이력 근거: [docs/companion_setup.md:12-19](../companion_setup.md#L12-L19) (커밋 `707dfd8`, `889daa8`)

> **주의**: apt 패키지는 git이 추적하지 않는다. 위 표의 설치일은
> `companion_setup.md`의 수기 기록이며, 버전 문자열만 `dpkg -l`로 재확인한 값이다.

Python 분석 라이브러리 (측정 스크립트 전제): matplotlib 3.5.1 / numpy 1.21.5 / pandas 1.3.5 / OpenCV 4.5.4
한글 폰트: Noto Sans CJK KR 포함, `fc-list :lang=ko` 80건

---

## 2. 모듈별 상태표

판정 기준:

| 표기 | 뜻 |
|---|---|
| 구현완료 | 저장소에 코드·설정이 있고 의도한 출력이 나온다 |
| 부분구현 | 일부만 동작. 남은 것을 명시 |
| 미착수 | 저장소에 해당 코드가 없다 |
| 실기 검증 | 실제 하드웨어를 붙여 명령으로 출력을 확인했다 |

### 2.1 측위

| 모듈 | 상태 | 남은 것 | 실기 검증 | 근거 |
|---|---|---|---|---|
| UWB (`/uwb_pose`) | **미착수** | 노드 전체. 패키지 `drone_uwb` 미생성 | ✗ (장치 미연결) | [evidence/06_uwb_fail.txt](evidence/06_uwb_fail.txt), `ls src/` |
| LiDAR 드라이버 (`/scan`) | **구현완료** | — | ✓ 10Hz 발행 확인 | [evidence/04_lidar_launch.txt](evidence/04_lidar_launch.txt), 커밋 `b315759`·`bf3c96d` |
| LiDAR 스캔매칭 | **미착수** | 알고리즘 전체. Phase 2 범위 | ✗ | src/ 내 관련 코드 없음 |
| 광학흐름 (PMW3901) | **SW 범위 외 + 미착수** | PX4 파라미터 설정 (HW 파트) | ✗ | roadmap 결정 8, [altitude_policy.md:16](../altitude_policy.md#L16) |

**UWB 상세** — 설계는 끝났고 코드가 없다.

- 설계 문서 [docs/uwb_node_design.md](../uwb_node_design.md)에 인터페이스·파라미터 8종·금지사항 7종까지 확정
- 단, 이 파일은 **git 미추적 상태** (`git status` 기준 `??`) → 커밋 해시로 인용 불가
- 인터페이스 확정분: `/dev/uwb`, 921600 8N1, JSON 1줄/샘플, `type:"uwb_pose"`
  근거 커밋 `3ba253f` + [config/udev/99-dronestock.rules:43-66](../../config/udev/99-dronestock.rules#L43-L66)

> **"HW 파트 대기"는 이 항목의 착수 조건이 아니다 — 표기 정정**
>
> roadmap.md:98은 `fix:false` 지속(잔차 RMS 0.44m)을 HW 파트 대기 항목으로 둔다. 사실이나,
> **그것은 좌표값의 정확도 문제이지 노드 구현의 선행 조건이 아니다.**
>
> `uwb_node`가 하는 일은 시리얼 개방 → 첫 줄 `type` 검증 → 라인 프레이밍 → JSON 파싱 →
> 발행 판정 → 메시지 조립 → watchdog 이다. 어느 단계도 앵커 교정 결과에 의존하지 않는다.
> 설계 문서 자체가 이 노드를 통과(pass-through)로 규정한다
> ([uwb_node_design.md:232](../uwb_node_design.md#L232)).
>
> | 구분 | 교정 전 가능 여부 |
> |---|---|
> | 노드 구현 (파싱·프레이밍·watchdog·첫 줄 검증) | 가능 |
> | 태그 연결 후 `/uwb_pose` 발행 확인 | 가능 (`fix=true` 프레임 한정) |
> | 갱신 주기 지표 측정 | 가능 |
> | 측위 RMSE 산출 | **불가** — 교정 후 |
>
> 따라서 이 항목의 실제 상태는 "HW 대기"가 아니라 **"선행 조건 없음 / 착수 지연"**이다.
> 조치 계획은 [draft_sw_section.md](draft_sw_section.md) 5.2 참조.

**LiDAR 실측 (2026-08-16)**

| 항목 | 값 | 비고 |
|---|---|---|
| `/scan` 발행 주기 | 평균 9.964~9.970 Hz (표준편차 0.0006~0.0009s, window 71) | 설정값 10.0Hz와 일치 |
| 스캔 포인트 | `angle_increment` 0.01465 rad, 범위 ±π | |
| `range_min` / `range_max` | 0.03 / 12.0 m | 파라미터값과 일치 |
| QoS | BEST_EFFORT / VOLATILE | |
| 기동 로그 경고 | `Checksum error` 4회, `Real points 640 > fixed points 430` 1회 | 기동 초기 구간. 발행은 정상 지속 |

### 2.2 비행제어 연동 (MAVROS / PX4 인터페이스)

| 항목 | 상태 | 근거 |
|---|---|---|
| MAVROS 패키지 설치 | 완료 (2.14.0) | `ros2 pkg list` |
| geoid 데이터셋 | 완료 (`egm96-5.pgm` 존재) | [evidence/07_mavros_fail.txt](evidence/07_mavros_fail.txt) |
| MAVROS launch / 파라미터 | **미작성** | `ls src/drone_bringup/launch/` → 2개뿐 |
| Pixhawk udev 규칙 | 저장소에는 활성, **`/etc`에는 미반영** | 아래 §3 참조 |
| heartbeat 실행 확인 | **불가** (장치 미연결) | `lsusb \| grep 3185` 출력 없음 |
| 결정 문서 | 완료 — PX4 + MAVROS2, 2026-07-12 확정 | [pixhawk_decision.md](../pixhawk_decision.md), 커밋 `faa0e44` |

**종합 판정: 부분구현 (환경만 준비, 연동 코드 0줄)**

> roadmap.md:93은 "Pixhawk 6C Mini heartbeat (PX4 탑재, MAVROS connected)"를 `[x]`로 표기하나,
> **그 확인을 뒷받침하는 로그·커밋이 저장소에 없다.** 보고서에는 "환경 준비 완료, 연동 미구현"으로 적는다.
> 재현 명령: `ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/pixhawk:115200` 후 `ros2 topic echo /mavros/state`

### 2.3 인식

| 모듈 | 상태 | 남은 것 | 실기 검증 | 근거 |
|---|---|---|---|---|
| 카메라 스트림 | **부분구현** | `camera_info` 실값 (캘리브레이션), `base_link→camera_link` TF | ✓ 발행 확인 | [evidence/05_camera_launch.txt](evidence/05_camera_launch.txt), 커밋 `3f7f136` |
| 카메라 캘리브레이션 | **미착수** | 초점 조정 → 캘리브레이션 → `camera_info_url` 활성화 | ✗ | 담당 재확인 필요 — 아래 참조 |
| QR 인식 | **미착수** | 노드 전체 | ✗ | [evidence/08_qr_marker_fail.txt](evidence/08_qr_marker_fail.txt) |
| ArUco 마커 | **미착수** | 노드 전체. Phase 3 범위 | ✗ | 동일 |

**카메라 실측 (2026-08-16)**

| 항목 | 값 |
|---|---|
| 해상도 / 인코딩 | 1640 × 1232, `rgb8`, step 4920, 1프레임 6,061,440 byte |
| `frame_id` | `camera_link` |
| 센서 모드 | mode 3 (1640×1232 @ 29.999999 fps) — GST_ARGUS 확인 |
| 발행 주기 — `ros2 topic hz` 단독, 10초 | 평균 21.6 Hz (min 0.028s / max 0.302s) |
| 발행 주기 — `ros2 topic hz` LiDAR 동시, 5초 | 평균 28.0 Hz (min 0.028s / max 0.068s) |
| **발행 주기 — bag 기록 후처리, 19.2초** | **평균 29.9985 Hz** (중앙값 간격 0.033436s, p99 0.0367s, 결손 1회 0.323s) |

> **측정 방법에 따라 값이 달라진 항목 — 해석 정리**
>
> `ros2 topic hz`로 잰 21.6 / 28.0 Hz는 **구독자 측 손실이 섞인 값**이다.
> `/camera/image_raw`는 프레임당 6,061,440 byte라 구독자가 못 따라가면 그대로 낮게 잡힌다.
>
> bag 기록 후 후처리로 잰 `/camera/camera_info`는 **29.9985 Hz**다. gscam은 프레임마다
> 이미지와 camera_info를 함께 발행하므로 이 값이 카메라 파이프라인 속도를 대리한다.
> GST_ARGUS 로그의 센서 모드도 `1640x1232 @ 29.999999 fps`로 일치한다.
>
> **결론: 카메라는 30fps로 동작한다. 기존 문서의 30Hz 기록과 모순되지 않는다.**
> 다만 `ros2 topic hz` 수치는 조건을 병기하지 않으면 오해를 부르므로 보고서에 단독 인용하지 않는다.
>
> 증빙: [evidence/metrics_selftest/](evidence/metrics_selftest/) (summary.txt, plot_camera_camera_info.png)

> **렌즈 초점 담당 표기 — 사유 미기재 상태**
>
> 저장소 3곳이 초점 조정을 HW 파트로 표기한다 ([roadmap.md:102](../roadmap.md#L102),
> [companion_setup.md:22](../companion_setup.md#L22), [companion_setup.md:83](../companion_setup.md#L83)).
> **세 곳 모두 "왜 HW 파트인가"를 적지 않았다.** CLAUDE.md의 문서 규칙("금지·제약 조항에는
> 반드시 왜를 한 줄 붙인다") 미준수 상태다.
>
> 이 1건이 직렬로 막고 있는 것: 초점 → 캘리브레이션 → QR 인식 → 비주얼 서보잉.
> **Phase 3 항목 3건 전부가 여기에 종속된다.**
>
> 확인 필요 사항: ArduCAM B0191(IMX219)의 렌즈가 수동 초점 조절식인지
> ([equipment_inventory.md:17](../equipment_inventory.md#L17)에 모델명만 기재, 렌즈 방식 미기재).
>
> | 확인 방법 | 결과에 따른 조치 |
> |---|---|
> | 렌즈 경통 회전 여부 확인 + 제조사 사양서 대조 | 수동 조절 가능 → **SW 담당으로 재배정**, 즉시 착수 |
> | 동일 | 고정 초점·전용 공구 필요 → HW 파트 유지, **단 사유를 문서에 명기** |
>
> 조절 가능할 경우의 절차: `ros2 launch drone_bringup camera.launch.py` 기동 →
> `rqt_image_view` 등으로 영상 확인 → 선명해질 때까지 조절 → 확정 후 `camera_calibration` 실행.

**QR / 마커 상세**

- `src/` 전체에 `qr`·`aruco`·`apriltag`·`solvePnP` 구현 코드 없음. 주석 언급 2건뿐
  ([camera.launch.py:13](../../src/drone_bringup/launch/camera.launch.py#L13), [camera_imx219.yaml:14](../../src/drone_bringup/params/camera_imx219.yaml#L14))
- ROS2 마커 패키지 미설치: `ros2 pkg list | grep -E 'aruco|apriltag|zbar'` 출력 없음
- Python OpenCV 4.5.4에 `QRCodeDetector`·`aruco` 모듈 존재 → 라이브러리 차원의 장애물은 없음
- `pyzbar` 미설치 (`ModuleNotFoundError`)
- **거리·각도 산출 불가**: `camera_info`가 빈 값이라 solvePnP에 넣을 내부 파라미터가 없음

### 2.4 통신

| 모듈 | 상태 | 남은 것 | 실기 검증 | 근거 |
|---|---|---|---|---|
| Tailscale | **구현완료** (환경) | — | ✓ `active`, IP 100.110.163.94 | [evidence/09_comm.txt](evidence/09_comm.txt) |
| ROS_DOMAIN_ID 격리 | **구현완료** | 2호기 배포 시 값 2 적용 | ✓ `.bashrc:120` 고정 확인 | 동일 |
| LoRa 텔레메트리 | **미착수** | 송신 노드 전체. Phase 1 범위 | ✗ | grep `lora|sx127|sx126|rfm9` 결과 없음 |
| 웹 API 경계 토픽 5종 | **미착수** | 전부. 문서 계약만 존재 | ✗ | grep 결과 없음, roadmap.md:76-86 |

> roadmap.md:100의 `[ ] ROS_DOMAIN_ID=1 설정을 1호기 .bashrc에 고정`은 **실제로는 완료 상태**다.
> `~/.bashrc:120`에 `export ROS_DOMAIN_ID=1` 존재. roadmap 체크박스가 갱신되지 않은 것.

Tailscale 부가 사항: `tailscale status`에 iptables `connmark` 모듈 부재 경고 1건.
현재 접속에는 영향 없으나 기록해 둔다 ([evidence/09_comm.txt](evidence/09_comm.txt)).

### 2.5 임무제어 (FSM / BT)

| 모듈 | 상태 | 근거 |
|---|---|---|
| FSM 임무 골격 | **미착수** | src/ 내 상태기계 코드 없음. roadmap Phase 1 범위 |
| Behavior Tree | **미착수** | roadmap Phase 3~4 범위 |
| 가상벽 / keepout | **미착수** | `virtual_wall_node` 미생성. [altitude_policy.md:9-11](../altitude_policy.md#L9-L11)이 TODO 한 줄만 예약 |
| 위치제어 PID / 게인 스케줄링 | **미착수** | roadmap Phase 1 범위 |

### 2.6 요약표

| 모듈 | 구현완료 | 부분구현 | 미착수 | 실기 검증 |
|---|:---:|:---:|:---:|:---:|
| 측위 — UWB | | | ● | ✗ |
| 측위 — LiDAR 드라이버 | ● | | | ✓ |
| 측위 — LiDAR 스캔매칭 | | | ● | ✗ |
| 측위 — 광학흐름 | | | ● | ✗ |
| 비행제어 연동 (MAVROS/PX4) | | ● | | ✗ |
| 인식 — 카메라 스트림 | | ● | | ✓ |
| 인식 — 카메라 캘리브레이션 | | | ● | ✗ |
| 인식 — QR / ArUco | | | ● | ✗ |
| 통신 — Tailscale | ● | | | ✓ |
| 통신 — ROS_DOMAIN_ID | ● | | | ✓ |
| 통신 — LoRa | | | ● | ✗ |
| 통신 — 웹 API | | | ● | ✗ |
| 임무제어 — FSM / BT | | | ● | ✗ |

자체 작성 ROS2 노드 수: **0**
실기 검증 완료 데이터 경로: **2** (`/scan`, `/camera/image_raw`)

---

## 3. 문서와 실제가 어긋난 항목 (보고서 작성 전 정정 필요)

| # | 항목 | 문서 기재 | 실제 확인 | 근거 |
|---|---|---|---|---|
| 1 | udev 규칙 적용 | roadmap.md:99 "적용(sudo) 미완" | `/etc/udev/rules.d/99-dronestock.rules` **존재하나 구버전**(2026-07-29자, 커밋 `6737223` 시점). Pixhawk 규칙 주석 상태, UWB 주석에 구값 "약 12Hz" | [evidence/02_devices_udev.txt](evidence/02_devices_udev.txt) diff |
| 2 | ROS_DOMAIN_ID | roadmap.md:100 미체크 | `.bashrc:120`에 적용 완료 | [evidence/09_comm.txt](evidence/09_comm.txt) |
| 3 | 카메라 발행률 | roadmap.md:101 "30.1Hz 실측" | **일치 확인** (bag 후처리 29.9985 Hz). 단 측정 조건 미기재라 `ros2 topic hz` 값(21.6/28.0)과 혼동 소지 | [evidence/metrics_selftest/summary.txt](evidence/metrics_selftest/summary.txt) |
| 4 | MAVROS connected | roadmap.md:93 `[x]` | 저장소에 확인 로그·커밋 없음. 현재 장치 미연결로 재확인 불가 | [evidence/07_mavros_fail.txt](evidence/07_mavros_fail.txt) |
| 5 | README Topic 표 | 8개 토픽 명시 | 실존 1개(`/scan`) | [evidence/09_comm.txt](evidence/09_comm.txt) |
| 6 | `uwb_node` 착수 조건 | roadmap.md:98이 HW 파트 대기 항목으로 배치 | 노드 구현에 교정 결과 불요. **선행 조건 없음** | [uwb_node_design.md:232](../uwb_node_design.md#L232), 위 §2.1 |
| 7 | 렌즈 초점 담당 | 3개 문서가 HW 파트로 표기 | 지정 **사유 미기재**. 렌즈 방식 확인 후 재배정 필요 | 위 §2.3 |

1번 조치: `sudo cp config/udev/99-dronestock.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`
6·7번 조치 계획: [draft_sw_section.md](draft_sw_section.md) 5장

---

## 4. 미확인 항목과 확인 방법

| 항목 | 미확인 사유 | 확인 방법 |
|---|---|---|
| UWB 태그 실동작 | 장치 미연결 | 태그를 USB 1-2.1에 연결 → `ls -l /dev/uwb` → `cat /dev/uwb` (921600) |
| Pixhawk heartbeat | 장치 미연결 | 기체 연결 → `ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/pixhawk:115200` → `ros2 topic echo /mavros/state` |
| TFmini Plus 동작 | PX4 직결이라 companion에서 보이지 않음 | QGroundControl에서 `EKF2_RNG_*` 및 거리 확인 (HW 파트) |
| PMW3901 동작 | PX4 직결 | 동일 (HW 파트) |
| 카메라 렌즈 초점 | HW 파트 미실시 | [companion_setup.md:83](../companion_setup.md#L83) |
| 태그 펌웨어 버전 | HW 파트 미회신 | [hw_handoff/README.md:44](../hw_handoff/README.md#L44) |
| `rosdep` 의존성 해결 | `rosdep update` 미실행 | [companion_setup.md:82](../companion_setup.md#L82) |

---

## 5. 미커밋 변경 (기준 커밋 `7438c17` 이후)

| 파일 | 상태 | 내용 |
|---|---|---|
| [docs/glossary.md](../glossary.md) | 수정(M) | 용어 3개 추가 — 공분산 행렬, 라인 프레이밍, watchdog |
| [docs/uwb_node_design.md](../uwb_node_design.md) | 미추적(??) | uwb_node 설계 문서 신규 |

두 파일 모두 **커밋 해시로 인용할 수 없다.** 보고서 제출 전 커밋 필요.
