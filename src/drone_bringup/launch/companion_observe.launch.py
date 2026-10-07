"""
Compose one Tag's ground observations with the C++ AI PX4 observer.

No mission core, flight writer, calibration override or boot-service change.
Existing MAVROS can be reused with start_mavros:=false (the default).
"""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    """Validate Tag identity before assembling observation processes."""
    def value(name):
        return LaunchConfiguration(name).perform(context)
    role = value('tag').lower()
    if role not in ('a', 'b'):
        raise ValueError('tag must be A or B')
    domain = 1 if role == 'a' else 2
    root = Path(value('ai_root')).resolve(strict=True)
    config = root / f'config/px4.tag_{role}.observe.json'
    data = json.loads(config.read_text(encoding='utf-8'))
    if (data['ros_domain_id'] != domain or data['profile'] != 'HOST_OBSERVE'
            or data['flight_authority'] is not False
            or data['physical_output_enabled'] is not False):
        raise ValueError('AI observation profile must match selected Tag')
    binary = root.parents[1] / 'install/sangwon_ai_replay/bin/sangwon_px4_observer'
    if not binary.is_file():
        raise ValueError('Build sangwon_ai_replay in this workspace first')
    share = Path(get_package_share_directory('drone_uwb'))
    # The bridge stays disabled even if someone edited the profile for another trial.
    bridge = Node(package='drone_uwb', executable='uwb_px4_bridge',
                  parameters=[str(share / f'config/runtime/uwb_tag_{role}.yaml'),
                              {'enabled': False, 'ground_only': True}], output='screen')
    observer = ExecuteProcess(cmd=[str(binary), '--root', str(root), '--config', str(config)],
                              output='screen')
    bench = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/uwb_btf_bench.launch.py')),
        launch_arguments={name: value(name) for name in
                          ('tag', 'uwb_port', 'btf_config', 'fcu_url', 'start_mavros',
                           'record_directory', 'stop_after_s')}.items())
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain))]
    for child in (bridge, observer):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=child,
            on_exit=[EmitEvent(event=Shutdown(reason='Observation component stopped'))])))
    return actions + [bridge, observer, bench]


def generate_launch_description():
    """Keep physical output off and explicitly select the source checkout."""
    return LaunchDescription([
        DeclareLaunchArgument('tag', default_value='B'),
        DeclareLaunchArgument('ai_root'),
        DeclareLaunchArgument('record_directory'),
        DeclareLaunchArgument('stop_after_s', default_value='60.0'),
        DeclareLaunchArgument('uwb_port', default_value='/dev/uwb'),
        DeclareLaunchArgument('btf_config', default_value=''),
        DeclareLaunchArgument('fcu_url', default_value='/dev/pixhawk:921600'),
        DeclareLaunchArgument('start_mavros', default_value='false'),
        OpaqueFunction(function=setup),
    ])
