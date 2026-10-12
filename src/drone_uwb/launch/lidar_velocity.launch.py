"""Shared manual/OFFBOARD LiDAR observation service; no flight commands."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory('drone_uwb'))/'config/runtime/lidar_velocity_field.json')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='drone_uwb', executable='lidar_velocity_bridge', output='screen',
             arguments=['--config', LaunchConfiguration('config')]),
    ])
