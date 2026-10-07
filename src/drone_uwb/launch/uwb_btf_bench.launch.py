"""One onboard companion, selected Tag and sensors; no flight output bridge."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from drone_uwb.integration.ros.layout_selection import matched_anchor_path


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    role = value('tag').lower()
    if role not in ('a', 'b'):
        raise ValueError('tag must be A or B')
    tag_id, domain = ('5', '1') if role == 'a' else ('6', '2')
    share = Path(get_package_share_directory('drone_uwb'))
    config = value('btf_config') or str(share/f'config/runtime/uwb_btf_tag_{role}.json')
    measured = json.loads(Path(config).read_text(encoding='utf-8'))
    if measured['tag_id'] != tag_id or measured.get('tdma_mode') != 'required':
        raise ValueError('BTF config must match selected Tag with tdma_mode=required')
    anchors = matched_anchor_path(share, measured,
                                  LaunchConfiguration('anchor_file', default='').perform(context))
    directory = Path(value('record_directory'))
    receiver = Node(package='drone_uwb', executable='uwb_node', output='screen',
        parameters=[str(share/f'config/runtime/uwb_tag_{role}.yaml'), {
            'port':value('uwb_port'), 'record_directory':str(directory/'uwb'),
            'anchor_file': str(anchors),
            'stop_after_s':float(value('stop_after_s'))}])
    btf = Node(package='drone_uwb', executable='uwb_btf_node', output='screen',
        parameters=[{'config_file':config,'record_directory':str(directory/'btf')}])
    children = [btf, receiver]
    use_mavros = value('start_mavros').lower()
    if use_mavros not in ('true', 'false'):
        raise ValueError('start_mavros must be true or false')
    if use_mavros == 'true':
        children.insert(0, Node(package='mavros', executable='mavros_node',
            namespace='mavros', output='screen',
            parameters=[str(share/'config/runtime/mavros_btf_bench.yaml'), {
                'fcu_url':value('fcu_url'),'tgt_system':1,'tgt_component':1,'fcu_protocol':'v2.0'}]))
    result = [SetEnvironmentVariable('ROS_DOMAIN_ID', domain)]
    for child in children:
        result.append(RegisterEventHandler(OnProcessExit(target_action=child,
            on_exit=[EmitEvent(event=Shutdown(reason='BTF bench component stopped'))])))
    return result + children


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tag', default_value='A'),
        DeclareLaunchArgument('uwb_port', default_value='/dev/uwb'),
        DeclareLaunchArgument('btf_config', default_value=''),
        DeclareLaunchArgument('anchor_file', default_value=''),
        DeclareLaunchArgument('fcu_url', default_value='/dev/pixhawk:921600'),
        DeclareLaunchArgument('start_mavros', default_value='true'),
        DeclareLaunchArgument('record_directory'),
        DeclareLaunchArgument('stop_after_s', default_value='60.0'),
        OpaqueFunction(function=setup),
    ])
