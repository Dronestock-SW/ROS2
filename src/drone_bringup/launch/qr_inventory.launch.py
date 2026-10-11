"""QR 재고 판독 파이프라인을 한 번에 실행한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """카메라와 QR 판독·저장 노드를 하나의 실행 단위로 묶는다."""
    bringup_share = get_package_share_directory('drone_bringup')
    decoder_params = os.path.join(bringup_share, 'params', 'qr_decoder.yaml')
    fallback_params = os.path.join(bringup_share, 'params', 'qr_fallback.yaml')

    start_camera = DeclareLaunchArgument(
        'start_camera', default_value='true',
        description='true면 IMX219 gscam도 함께 실행한다',
    )
    scan_active_on_start = DeclareLaunchArgument(
        'scan_active_on_start', default_value='false',
        description='시험 전용. true면 launch 직후 실패 시간 창을 센다',
    )
    output_path = DeclareLaunchArgument(
        'output_path',
        default_value='~/.local/share/dronestock/qr_inventory.sqlite3',
        description='확정 QR 관측을 저장할 SQLite 파일',
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    return LaunchDescription([
        start_camera,
        scan_active_on_start,
        output_path,
        camera,
        Node(
            package='drone_bringup', executable='qr_reader_node',
            output='screen',
        ),
        Node(
            package='drone_bringup', executable='qr_decoder_node',
            parameters=[decoder_params, {'enabled_on_start': False}],
            output='screen',
        ),
        Node(
            package='drone_bringup', executable='qr_fallback_node',
            parameters=[
                fallback_params,
                {'scan_active_on_start': ParameterValue(
                    LaunchConfiguration('scan_active_on_start'), value_type=bool)},
            ],
            output='screen',
        ),
        Node(
            package='drone_bringup', executable='qr_parser_node',
            output='screen',
        ),
        Node(
            package='drone_bringup', executable='qr_recorder_node',
            parameters=[{'output_path': LaunchConfiguration('output_path')}],
            output='screen',
        ),
    ])
