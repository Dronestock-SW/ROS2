"""Shared UWB XY and LiDAR NE aids for both manual and OFFBOARD flight."""
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def components(context):
    read = lambda key: LaunchConfiguration(key).perform(context)
    uwb_path, lidar_path = Path(read('uwb_config')), Path(read('lidar_config'))
    uwb = json.loads(uwb_path.read_text(encoding='utf-8'))['settings']
    lidar = json.loads(lidar_path.read_text(encoding='utf-8'))['settings']
    if uwb['tag_id'] != lidar['tag_id'] or uwb['tag_id'] not in ('5','6'):
        raise ValueError('shared_observation_tag_mismatch')
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', '1' if uwb['tag_id']=='5' else '2'),
        Node(package='drone_uwb',executable='uwb_local_odometry_bridge',output='screen',
             arguments=['--config',str(uwb_path)]),
        Node(package='drone_uwb',executable='lidar_velocity_bridge',output='screen',
             arguments=['--config',str(lidar_path)])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('uwb_config'),
        DeclareLaunchArgument('lidar_config'),OpaqueFunction(function=components)])
