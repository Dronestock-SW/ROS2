"""Observation receiver plus disabled-by-default PX4 bridge."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('drone_uwb'))/'config/uwb.yaml')
    receiver = Node(package='drone_uwb', executable='uwb_node', output='screen',
                    parameters=[LaunchConfiguration('config'), {
                        'record_directory': ParameterValue(LaunchConfiguration('record_directory'), value_type=str),
                        'stop_after_s': ParameterValue(LaunchConfiguration('stop_after_s'), value_type=float),
                    }])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('record_directory', default_value=''),
        DeclareLaunchArgument('stop_after_s', default_value='0.0'),
        RegisterEventHandler(OnProcessExit(target_action=receiver,
                             on_exit=[EmitEvent(event=Shutdown(reason='UWB receiver stopped'))])),
        receiver,
        Node(package='drone_uwb', executable='uwb_px4_bridge', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
