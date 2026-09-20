"""Launch only the reusable input generator in the local demo domain."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('scenario', default_value=''),
        Node(package='drone_demo', executable='demo_node', output='screen', parameters=[{
            'config_file': ParameterValue(LaunchConfiguration('config_file'), value_type=str),
            'scenario': ParameterValue(LaunchConfiguration('scenario'), value_type=str),
        }]),
    ])
