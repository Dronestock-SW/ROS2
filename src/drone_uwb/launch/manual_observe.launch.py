"""Manual-flight sensors only: no mission, EV bridge, vision or setpoint plugin."""
import json
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
    children = [
        Node(package='mavros', executable='mavros_node', namespace='mavros', output='screen',
             parameters=[str(share/'config/runtime/mavros_btf_bench.yaml'),
                         {'fcu_url':value('fcu_url'), 'tgt_system':1, 'tgt_component':1,
                          'fcu_protocol':'v2.0'}]),
        Node(package='drone_uwb', executable='uwb_node', output='screen',
             parameters=[str(share/f'config/runtime/uwb_tag_{role}.yaml'),
                         {'port':value('uwb_port'), 'anchor_file':str(anchors),
                          'record_directory':'', 'stop_after_s':0.0}]),
        Node(package='drone_uwb', executable='uwb_btf_node', output='screen',
             parameters=[{'config_file':str(config), 'record_directory':'',
                          'require_height_for_pose':True}]),
        Node(package='drone_uwb', executable='manual_observation_streams', output='screen'),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=child,
        on_exit=[EmitEvent(event=Shutdown(reason='manual observation component stopped'))]))
        for child in children]
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', domain)]+handlers+children


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tag', default_value='B', choices=['A','B']),
        DeclareLaunchArgument('btf_config'), DeclareLaunchArgument('anchor_file', default_value=''),
        DeclareLaunchArgument('uwb_port', default_value='/dev/uwb'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/pixhawk:921600'),
        OpaqueFunction(function=components),
    ])
