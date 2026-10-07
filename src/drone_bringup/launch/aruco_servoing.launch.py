"""
ArUco 비주얼 서보잉 파이프라인 기동 launch — Dronestock 1호기 (Phase 3).

세 노드를 한 번에 띄운다:
    aruco_tracker_autostart  ArUco 검출 + pose      → /aruco_detections
    aruco_alignment_node     목표 대비 오차 계산     → /aruco_alignment/error
    aruco_servo_node         비례제어 속도 제안값    → /aruco_alignment/velocity_suggestion

MAVROS/PX4에는 아무것도 보내지 않는다. 제안값까지만 만든다 (UWB팀 위치제어
인프라가 이 토픽을 구독해 실제 명령으로 바꾸는 구조 — 인터페이스는 잠정).

카메라:
    기본은 카메라를 안 띄운다 (start_camera:=false).
    이유: CSI 카메라는 프로세스 둘이 동시에 열 수 없다. 이미 camera.launch.py 가
    떠 있는 상태에서 또 띄우면 두 번째가 "Failed to create CaptureSession" 을
    반복한다 (2026-09-13 실측). 카메라까지 같이 띄우려면 start_camera:=true.

aruco_tracker 실행파일:
    일반 aruco_tracker 는 lifecycle 노드라 activate 전환이 없으면 토픽이 안 나온다.
    자동 활성화되는 aruco_tracker_autostart 를 쓴다.

실행:
    ros2 launch drone_bringup aruco_servoing.launch.py
    ros2 launch drone_bringup aruco_servoing.launch.py start_camera:=true

다른 ArUco 파라미터 파일로 시험할 때 (마커 크기·딕셔너리를 바꿀 때):
    ros2 launch drone_bringup aruco_servoing.launch.py aruco_params_file:=/경로/다른.yaml

확인:
    ros2 topic echo /aruco_detections
    ros2 topic echo /aruco_alignment/error
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')

    start_camera = DeclareLaunchArgument(
        'start_camera',
        default_value='false',
        description='true 면 camera.launch.py 도 같이 띄운다 (이미 떠 있으면 false)',
    )
    aruco_params = DeclareLaunchArgument(
        'aruco_params_file',
        default_value=os.path.join(bringup_share, 'params', 'aruco_tracker.yaml'),
        description='aruco_tracker 파라미터 yaml 경로',
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    tracker = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        parameters=[LaunchConfiguration('aruco_params_file')],
        output='screen',
    )
    alignment = Node(
        package='drone_bringup',
        executable='aruco_alignment_node',
        output='screen',
    )
    servo = Node(
        package='drone_bringup',
        executable='aruco_servo_node',
        output='screen',
    )

    return LaunchDescription([
        start_camera,
        aruco_params,
        camera,
        tracker,
        alignment,
        servo,
    ])
