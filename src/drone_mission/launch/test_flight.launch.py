"""Real sensor observations and web flight trials with one MAVROS owner."""

from dataclasses import asdict
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_mission.contracts import Settings


def components(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = value('config')
    settings = Settings(**json.loads(Path(config).read_text(encoding='utf-8')))
    enabled = value('bridge_enabled').lower() == 'true'
    execute = value('execute').lower() == 'true'
    record = value('record_directory')
    record_path = lambda name: str(Path(record)/name) if record else ''
    uwb = Path(get_package_share_directory('drone_uwb'))
    bridge = {name: getattr(settings, name) for name in (
        'alignment_confirmed', 'timing_confirmed', 'sensor_mount_confirmed',
        'enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m', 'expected_ev_delay_ms',
        'expected_ev_pos_x_m', 'expected_ev_pos_y_m', 'expected_ev_pos_z_m')}
    bridge.update(enabled=enabled, pose_topic='/uwb/btf_pose', verify_ev_sensor_position=True)
    children = [
        Node(package='mavros', executable='mavros_node', namespace='mavros', output='screen',
             parameters=[str(uwb/'config/runtime/mavros_test_flight.yaml'),
                         {'fcu_url': value('fcu_url'), 'tgt_system':1, 'tgt_component':1, 'fcu_protocol':'v2.0'}]),
        Node(package='drone_uwb', executable='uwb_node', output='screen',
             parameters=[str(uwb/'config/runtime/uwb.yaml'), {'record_directory':record_path('uwb')}]),
        Node(package='drone_uwb', executable='uwb_btf_node', output='screen',
             parameters=[{'config_file':str(uwb/'config/runtime/uwb_btf_real.json'),
                          'require_height_for_pose':True, 'record_directory':record_path('btf')}]),
        Node(package='drone_uwb', executable='uwb_px4_bridge', output='screen', parameters=[bridge]),
        Node(package='drone_mission', executable='flight_mission', output='screen',
             parameters=[{'config_file':config, 'execute':execute, 'record_directory':record_path('mission')}]),
        Node(package='drone_platform_link', executable='platform_link', output='screen',
             additional_env={
                 'DRONESTOCK_SERVER_URL':value('server_url'),
                 'DRONESTOCK_WS_URL':value('ws_url'),
                 'DRONESTOCK_DRONE_ID':settings.drone_id,
                 'DRONESTOCK_UWB_TOPIC':'/uwb/btf_pose',
                 'DRONESTOCK_MISSION_FORWARDING':'true',
                 **({'DRONESTOCK_STATE_DIR':record_path('platform')} if record else {})}),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=child,
        on_exit=[EmitEvent(event=Shutdown(reason='flight trial component stopped'))])) for child in children]
    return handlers+children


def generate_launch_description():
    default = str(Path(get_package_share_directory('drone_mission'))/'config/flight.json')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=default),
        DeclareLaunchArgument('bridge_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('execute',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('record_directory',default_value=''),
        DeclareLaunchArgument('fcu_url',default_value='/dev/pixhawk:921600'),
        DeclareLaunchArgument('server_url',default_value='http://127.0.0.1:8001'),
        DeclareLaunchArgument('ws_url',default_value='ws://127.0.0.1:8002/ws/drones/5/'),
        OpaqueFunction(function=components),
    ])
