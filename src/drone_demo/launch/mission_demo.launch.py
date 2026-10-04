"""Run one bounded input/assessment cycle; stop the monitor with its source."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def argument(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    source = Node(package='drone_demo', executable='demo_node', output='screen', parameters=[{
        'config_file': argument('config_file'), 'scenario': argument('scenario'),
        'real_tof_topic': argument('real_tof_topic'),
        'tof_timeout_s': ParameterValue(LaunchConfiguration('tof_timeout_s'), value_type=float),
    }])
    monitor = Node(package='drone_demo', executable='demo_mission_node', output='screen', parameters=[{
        'mission_config': argument('mission_config'), 'record_directory': argument('record_directory'),
    }])
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='gap'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('real_tof_topic', default_value='/tof/range'),
        DeclareLaunchArgument('tof_timeout_s', default_value='0.2'),
        DeclareLaunchArgument('mission_config', default_value=''),
        DeclareLaunchArgument('record_directory', default_value=''),
        RegisterEventHandler(OnProcessExit(target_action=source, on_exit=[
            EmitEvent(event=Shutdown(reason='Demo input source finished'))])),
        RegisterEventHandler(OnProcessExit(target_action=monitor, on_exit=[
            EmitEvent(event=Shutdown(reason='Demo monitor finished'))])),
        monitor, source,
    ])
