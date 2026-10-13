# 실측 LiDAR 입력 실행 절차
수동·OFFBOARD 공통 노드의 배포 절차다.
현재 상태에서 다음 실측을 준비할 때 읽는다.

현재 저장 설정은 `enabled=false`다.
10월 13일 CRSF 유지 펌웨어를 FC에 설치했다.
[설치·파라미터 검증 결과](../report/lidar2d_crsf_flash_20261013.md)를 먼저 읽는다.
수평면은 사용자가 확인했다. 동적 시각 검증은 미완료다.
후속 지상 시험 중 하방 센서 응답이 중단됐다.
이후 bus 2 검사와 기존 드라이버 시작으로 복구했다.
60초 표본 591개·통신 오류 0건을 확인했다.
현재 하방 드라이버는 실행 중이다. 비행 검증은 별개다.
설치 기록의 후속 bus 2 복구와 남은 검증을 읽는다.

1. 기존 파라미터와 원시 기록을 보존한다.
   이번 백업은 Jetson의 아래 경로에 있다.
   `/home/pgyxn/lidar-physical-20261012/pre-firmware-backup/`
2. FMUv6C용 펌웨어와 manifest SHA256을 확인한다.
   파일은 `src/drone_uwb/firmware/`에 있다.
   10월 12일 산출물은 그날 업로드하지 않았다.
   현재 FC는 10월 13일 보고서의 R2 설치본이다.
3. 펌웨어 설치 후 파라미터를 대조한다.
   `EKF2_LID2_VER=1`, `EKF2_LID2_EN=1`이다.
   `EKF2_LID2_AGE`는 180–200ms 범위다.
   `EKF2_AVEL_DELAY=5`를 확인한다.
   기본 펌웨어 설정은 EN=0, AGE=150이다.
4. 실제 수평면과 동적 시각 근거를 기록한다.
   복사한 설정에 확인값과 참조를 넣는다.
   확인 없이 gate를 참으로 바꾸지 않는다.
   실제 관측 정확도를 아직 측정하지 않았기 때문이다.
5. `colcon build --symlink-install` 후 환경을 읽는다.
   ROS Humble, numpy, scipy, pymavlink가 필요하다.
6. 실행 중 센서 스택 위에 공통 노드를 하나만 띄운다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2 MAVLINK20=1
ros2 launch drone_uwb shared_observation_fusion.launch.py \
  uwb_config:=/absolute/path/uwb-local.json \
  lidar_config:=/absolute/path/lidar-velocity.json
```

두 설정은 같은 tag_id여야 한다.
tag 5는 domain 1, tag 6은 domain 2다.
기존 UWB 송신 노드가 있으면 중복 실행하지 않는다.
한 관측을 두 번 입력할 수 있기 때문이다.
LiDAR만 추가할 때는 다음 명령을 쓴다.

```bash
ros2 launch drone_uwb lidar_velocity.launch.py \
  config:=/absolute/path/lidar-velocity.json
```

수동 스택은 `lidar_velocity_config` 인자를 지원한다.
`LIDAR_VELOCITY_CONFIG` 환경변수도 지원한다.
별도 service 예시는 `deployment/`에 있다.
이번 작업에서 service는 설치하지 않았다.

OFFBOARD의 `final_field_session.py`에는
`--external-observation-bridge`를 추가했다.
공통 UWB 송신기가 이미 실행 중일 때 사용한다.
기존 readiness 검사와 임무 인자는 유지한다.

`/lidar/velocity_status`와 관측 기록을 확인한다.
FC의 `lidar_velocity2d_status`도 함께 확인한다.
송신과 융합, 정확도, 비행 검증을 각각 기록한다.
관측 중지 시 재개 여부는 DISARM 지상에서 먼저 확인한다.
