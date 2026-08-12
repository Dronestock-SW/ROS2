"""CSI 카메라(ArduCAM B0191 / IMX219) 기동 launch — Dronestock 1호기.

gscam 노드를 띄워 카메라 영상을 ROS2 topic으로 발행한다.
LiDAR와 달리 제조사 launch가 없어서 노드를 직접 실행한다.

발행 topic:
    /camera/image_raw     sensor_msgs/Image      (bgr8, 1640x1232, 30Hz)
    /camera/camera_info   sensor_msgs/CameraInfo (캘리브레이션 전에는 빈 값)

topic 이름이 camera/ 로 시작하는 이유:
    gscam 이 스스로 camera/ 아래에 발행한다. launch 에서 namespace 를 또 주면
    /camera/camera/image_raw 로 두 번 겹친다 (2026-08-12 확인).
    이 이름은 Phase 3에 쓸 aruco_opencv 의 기본값(cam_base_topic: camera/image_raw)과
    그대로 맞아서 나중에 설정을 고칠 일이 없다.
    ※ 센서 묶음 이름이지 드론 구분 접두어가 아니다 — 군집 규칙(결정 7)과 무관하다.

실행:
    ros2 launch drone_bringup camera.launch.py

다른 파라미터 파일로 시험할 때:
    ros2 launch drone_bringup camera.launch.py params_file:=/경로/다른.yaml

확인:
    ros2 topic hz /camera/image_raw     # 30 근처면 정상
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')

    default_params = os.path.join(bringup_share, 'params', 'camera_imx219.yaml')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='카메라 파라미터 yaml 경로',
    )

    camera_node = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    return LaunchDescription([
        params_declare,
        camera_node,
    ])
