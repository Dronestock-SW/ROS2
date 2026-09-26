# companion ROS2 환경 설치 절차

> 이 문서의 목적: 빈 Jetson을 현재 1호기 companion 상태로 만드는 순서.
> 2호기 배포와 재설치(SD카드 재이미징) 때 이 순서대로 실행한다.
> 최종 갱신: 2026-09-21 (개인 계정 확인 기록 연결)

결론: 아래 5단계는 시스템 설치·빌드 절차다.
실행 계정의 환경과 장치 권한도 별도로 확인한다.

apt 패키지는 git이 추적하지 않는다. 설치 이력은 아래에 기록한다.
리포지토리에 있는 것은 `src/` 소스와 설정뿐이다.

## 개인 계정에서 이어서 작업할 때

공용 패키지 설치와 개인 계정 설정을 구분한다.
2026-09-21 `pgyxn` 계정의 상태를 확인했다.
[기본 설정 확인](report/setup_status_20260921.md)을 먼저 본다.

| 항목 | 확인한 상태 |
|---|---|
| 개인 저장소 | `/home/pgyxn/github/ROS2` |
| ROS2 기반 | Humble 설치·소싱 후 패키지 조회 가능 |
| 계정 시작 파일 | ROS 소싱·`ROS_DOMAIN_ID` 설정 줄 없음 |
| 개인 저장소 빌드 | 산출물 없음. 실행 검증 미실시 |
| 시리얼 접근 | `pgyxn`에서 `/dev/uwb`·`/dev/lidar` 접근 불가 |

아래 `~/drone_ws`는 기존 계정의 경로 예시다.
개인 저장소를 쓰면 실제 경로로 바꾼다.
기존 패키지 XML 오류도 새 확인 기록에 적었다.
빌드 전에 해당 항목을 해결해야 한다.

## 설치 이력 (1호기 실제 실행분)

| 날짜 | 설치 | 목적 | 관련 Phase |
|---|---|---|---|
| 2026-07-12 | `ros-humble-ros-base`, `ros-dev-tools` | ROS2 Humble 기반 + colcon | 전체 |
| 2026-07-24 | `ros-humble-mavros`, `ros-humble-mavros-extras` | Pixhawk 연결 (2.14.0) | Phase 1 |
| 2026-08-12 | `cv-bridge`, `image-transport`, `image-transport-plugins`, `camera-info-manager`, `camera-calibration`, `image-proc` | 카메라 영상 처리·캘리브레이션 | Phase 3 |
| 2026-08-12 | `ros-humble-gscam` | CSI 카메라를 ROS2 topic으로 발행 | Phase 3 |

카메라는 2026-08-12에 launch까지 붙었다. `/camera/image_raw`가 30Hz로 나온다.
남은 것은 캘리브레이션이며, 그 앞에 렌즈 초점 조정(HW 파트)이 선행 조건이다.

## 절차

### 1. ROS2 Humble 기반
```bash
sudo apt install -y ros-humble-ros-base ros-dev-tools
```

### 2. MAVROS + geoid 데이터셋
```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```
두 번째 줄을 빠뜨리면 MAVROS 노드가 실행 직후 죽는다.
이유: MAVROS는 고도 변환에 EGM96 지오이드 데이터를 필수로 읽는다.
확인: `ls /usr/share/GeographicLib/geoids/egm96-5.pgm`

### 3. 카메라 영상
```bash
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-image-transport-plugins ros-humble-camera-info-manager \
  ros-humble-camera-calibration ros-humble-image-proc
sudo apt install -y ros-humble-gscam
```
gscam을 쓰는 이유: IMX219는 CSI 카메라라 Jetson ISP(`nvarguscamerasrc`)를 거쳐야 한다.
일반 USB 카메라용 `v4l2_camera`로는 이 경로를 타지 못한다.

### 4. 워크스페이스 빌드
```bash
cd ~/drone_ws
git submodule update --init --recursive   # ydlidar_ros2_driver
colcon build --symlink-install
source install/setup.bash
```

### 5. 장치 udev 규칙
```bash
sudo cp config/udev/99-dronestock.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 확인

| 명령 | 기대 결과 |
|---|---|
| `ros2 pkg list \| grep mavros` | mavros, mavros_extras, mavros_msgs |
| `ls /usr/share/GeographicLib/geoids/` | egm96-5.pgm |
| `ls -l /dev/lidar /dev/uwb /dev/pixhawk` | 세 링크 모두 존재 |
| `ls /dev/video0` | 존재 (CSI CAM0) |
| `ros2 launch drone_bringup camera.launch.py` | `/camera/image_raw` 30Hz, rgb8 1640x1232 |
| `echo $ROS_DOMAIN_ID` | 1호기 = 1, 2호기 = 2 |

기존 `user` 계정은 `~/.bashrc`에서 ROS 환경을 설정했다.
개인 계정의 설정은 앞의 확인 기록을 따른다.
전달 경로는 glossary의 `.bashrc` / systemd 항목을 참조한다.

## 당시 남은 작업 (2026-08-12 기록)

아래 표는 설치 이력 당시의 상태다.
현재 계정·장비 상태는 날짜별 확인 기록을 따른다.

| 항목 | 상태 | 언제 |
|---|---|---|
| rosdep 초기화 (`rosdep update`) | 캐시 없음 | 소스 빌드 의존성이 늘어날 때 |
| 카메라 렌즈 초점 조정 | 미실시 (HW 파트) | 캘리브레이션보다 먼저 |
| 카메라 내부 파라미터 캘리브레이션 | 미실시 | 초점 확정 직후 |
| `base_link → camera_link` TF | 미정의 | 장착 위치 실측 후 |
