"""Real sensor observations and web flight trials with one MAVROS owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_mission.profiles import trial_profile


def components(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    uwb = Path(get_package_share_directory('drone_uwb'))
    settings, domain, config, btf_config, anchors, bridge = trial_profile(
        value('tag'), get_package_share_directory('drone_mission'), uwb,
        value('config'), value('btf_config'), value('anchor_file'))
    enabled = value('bridge_enabled').lower() == 'true'
    execute = value('execute').lower() == 'true'
    record = value('record_directory')
    record_path = lambda name: str(Path(record)/name) if record else ''
    # Only explicit execution allows armed observations; ground launch retains the gate.
    bridge.update(enabled=enabled, ground_only=not execute)
    role = value('tag').lower()
    children = [
        Node(package='drone_uwb', executable='uwb_node', output='screen',
             parameters=[str(uwb/f'config/runtime/uwb_tag_{role}.yaml'),
                         {'port':value('uwb_port'), 'anchor_file':str(anchors),
                          'record_directory':record_path('uwb')}]),
        Node(package='drone_uwb', executable='uwb_btf_node', output='screen',
             parameters=[{'config_file':str(btf_config),
                          'require_height_for_pose':True, 'record_directory':record_path('btf')}]),
        Node(package='drone_uwb', executable='uwb_px4_bridge', output='screen', parameters=[bridge]),
        Node(package='drone_mission', executable='flight_mission', output='screen',
             parameters=[{'config_file':str(config), 'execute':execute, 'record_directory':record_path('mission'),
                          'native_plan_binary':value('native_plan_binary'), 'native_map_file':value('native_map_file'),
                          'native_map_sha256':value('native_map_sha256')}]),
        Node(package='drone_platform_link', executable='platform_link', output='screen',
             additional_env={
                 'DRONESTOCK_SERVER_URL':value('server_url'),
                 'DRONESTOCK_WS_URL':value('ws_url') or f'ws://127.0.0.1:8002/ws/drones/{settings.drone_id}/',
                 'DRONESTOCK_DRONE_ID':settings.drone_id,
                 'DRONESTOCK_UWB_TOPIC':'/uwb/btf_pose',
                 'DRONESTOCK_POSE_SOURCE':'btf_xy',
                 'DRONESTOCK_MISSION_FORWARDING':'true',
                 **({'DRONESTOCK_STATE_DIR':record_path('platform')} if record else {})}),
    ]
    if value('start_mavros').lower() == 'true':
        children.insert(0, Node(package='mavros', executable='mavros_node', namespace='mavros', output='screen',
             parameters=[str(uwb/'config/runtime/mavros_test_flight.yaml'),
                         {'fcu_url':value('fcu_url'), 'tgt_system':1, 'tgt_component':1, 'fcu_protocol':'v2.0'}]))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=child,
        on_exit=[EmitEvent(event=Shutdown(reason='flight trial component stopped'))])) for child in children]
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain))]+handlers+children


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tag',default_value='A',choices=['A','B','a','b']),
        DeclareLaunchArgument('config',default_value=''),
        DeclareLaunchArgument('btf_config',default_value=''),
        DeclareLaunchArgument('anchor_file',default_value=''),
        DeclareLaunchArgument('uwb_port',default_value='/dev/uwb'),
        DeclareLaunchArgument('start_mavros',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('bridge_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('execute',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('record_directory',default_value=''),
        DeclareLaunchArgument('native_plan_binary',default_value=''),
        DeclareLaunchArgument('native_map_file',default_value=''),
        DeclareLaunchArgument('native_map_sha256',default_value=''),
        DeclareLaunchArgument('fcu_url',default_value='/dev/pixhawk:921600'),
        DeclareLaunchArgument('server_url',default_value='http://127.0.0.1:8001'),
        DeclareLaunchArgument('ws_url',default_value=''),
        OpaqueFunction(function=components),
    ])
