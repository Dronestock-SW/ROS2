"""LiDAR(T-mini Pro) 기동 launch — Dronestock 1호기.

제조사 launch(ydlidar_ros2_driver/ydlidar_launch.py)를 그대로 불러 쓰고,
파라미터 파일만 우리 것(drone_bringup/params/lidar_tmini.yaml)으로 바꿔 넘긴다.
제조사 코드는 수정하지 않는다 — 제조사가 새 버전을 내도 충돌하지 않게 하기 위함.

실행:
    ros2 launch drone_bringup lidar.launch.py

다른 파라미터 파일로 시험할 때:
    ros2 launch drone_bringup lidar.launch.py params_file:=/경로/다른.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    driver_share = get_package_share_directory('ydlidar_ros2_driver')

    default_params = os.path.join(bringup_share, 'params', 'lidar_tmini.yaml')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='LiDAR 파라미터 yaml 경로',
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_share, 'launch', 'ydlidar_launch.py')
        ),
        launch_arguments={'params_file': LaunchConfiguration('params_file')}.items(),
    )

    return LaunchDescription([
        params_declare,
        driver_launch,
    ])
