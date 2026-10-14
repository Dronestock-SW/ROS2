"""Sensors only at boot; explicit opt-in adds passive local setpoint transport."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from drone_uwb.integration.ros.layout_selection import matched_anchor_path


def components(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    role = value('tag').lower()
    tag, domain = ('5', '1') if role == 'a' else ('6', '2')
    share = Path(get_package_share_directory('drone_uwb'))
    config = Path(value('btf_config'))
    measured = json.loads(config.read_text(encoding='utf-8'))
    if measured['tag_id'] != tag or measured.get('tdma_mode') != 'required':
        raise ValueError('manual_capture_tag_config_mismatch')
    anchors = matched_anchor_path(share, measured, value('anchor_file'))
    mavros_options = {'fcu_url':value('fcu_url'), 'tgt_system':1, 'tgt_component':1,
                      'fcu_protocol':'v2.0'}
    if value('local_setpoints') == 'true':
        # The plugin only transports explicit setpoints. No boot mission is added.
        mavros_options['plugin_allowlist'] = ['sys_status','sys_time','command','imu',
            'distance_sensor','local_position','param','rc_io','setpoint_raw']
    children = [
        Node(package='mavros', executable='mavros_node', namespace='mavros', output='screen',
             parameters=[str(share/'config/runtime/mavros_btf_bench.yaml'),
                         mavros_options]),
        Node(package='drone_uwb', executable='uwb_node', output='screen',
             parameters=[str(share/f'config/runtime/uwb_tag_{role}.yaml'),
                         {'port':value('uwb_port'), 'anchor_file':str(anchors),
                          'record_directory':'', 'stop_after_s':0.0}]),
        Node(package='drone_uwb', executable='uwb_btf_node', output='screen',
             additional_env=({'PYTHONPATH':value('btf_python_path')+os.pathsep+os.environ.get('PYTHONPATH','')}
                             if value('btf_python_path') else {}),
             parameters=[{'config_file':str(config), 'record_directory':'',
                          'require_height_for_pose':True}]),
        Node(package='drone_uwb', executable='manual_observation_streams', output='screen'),
    ]
    essential = list(children)
    lidar_config = value('lidar_velocity_config')
    if lidar_config:
        # One sensor-level lifecycle serves manual/POSCTL and OFFBOARD alike.
        # A LiDAR failure withholds its aid, without shutting down UWB/MAVROS.
        children.append(Node(package='drone_uwb', executable='lidar_velocity_bridge',
            output='screen', arguments=['--config', lidar_config, '--same-user-dds']))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=child,
        on_exit=[EmitEvent(event=Shutdown(reason='manual observation component stopped'))]))
        for child in essential]
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', domain)]+handlers+children


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tag', default_value='B', choices=['A','B']),
        DeclareLaunchArgument('btf_config'), DeclareLaunchArgument('anchor_file', default_value=''),
        DeclareLaunchArgument('btf_python_path', default_value=''),
        DeclareLaunchArgument('uwb_port', default_value='/dev/uwb'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/pixhawk:921600'),
        DeclareLaunchArgument('lidar_velocity_config', default_value=os.environ.get('LIDAR_VELOCITY_CONFIG', '')),
        DeclareLaunchArgument('local_setpoints', choices=['true','false'],
            default_value='true' if os.environ.get('DRONESTOCK_LOCAL_SETPOINTS') == '1' else 'false'),
        OpaqueFunction(function=components),
    ])
