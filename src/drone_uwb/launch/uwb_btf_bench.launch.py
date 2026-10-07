"""Real UWB + FC sensor telemetry + B_TF observations; no flight output bridge."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('drone_uwb'))
    receiver = Node(package='drone_uwb', executable='uwb_node', output='screen',
        parameters=[str(share/'config/runtime/uwb.yaml'), {
            'record_directory':ParameterValue(PathJoinSubstitution([LaunchConfiguration('record_directory'),'uwb']),value_type=str),
            'stop_after_s':ParameterValue(LaunchConfiguration('stop_after_s'),value_type=float)}])
    btf = Node(package='drone_uwb', executable='uwb_btf_node', output='screen',
        parameters=[{'config_file':str(share/'config/runtime/uwb_btf_real.json'),
            'record_directory':ParameterValue(PathJoinSubstitution([LaunchConfiguration('record_directory'),'btf']),value_type=str)}])
    mavros = Node(package='mavros',executable='mavros_node',namespace='mavros',output='screen',
        parameters=[str(share/'config/runtime/mavros_btf_bench.yaml'),
            {'fcu_url':'/dev/pixhawk:921600','tgt_system':1,'tgt_component':1,'fcu_protocol':'v2.0'}])
    result = [DeclareLaunchArgument('record_directory'),DeclareLaunchArgument('stop_after_s',default_value='60.0')]
    for child in (receiver,btf,mavros):
        result.append(RegisterEventHandler(OnProcessExit(target_action=child,
            on_exit=[EmitEvent(event=Shutdown(reason='BTF bench component stopped'))])))
    return LaunchDescription(result+[mavros,btf,receiver])
