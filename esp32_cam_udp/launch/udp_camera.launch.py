import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('esp32_cam_udp'),
                       'config', 'udp_camera.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5005'),
        DeclareLaunchArgument('show', default_value='false'),
        DeclareLaunchArgument('detect_qr', default_value='true'),
        DeclareLaunchArgument('gripper', default_value='true'),
        DeclareLaunchArgument('gripper_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('trigger_text', default_value=''),
        DeclareLaunchArgument('gripper_angle', default_value='0'),
        DeclareLaunchArgument('release_angle', default_value='90'),
        DeclareLaunchArgument('release_timeout', default_value='1.5'),
        DeclareLaunchArgument('rotate_180', default_value='true'),
        Node(
            package='esp32_cam_udp', executable='udp_camera_node',
            name='udp_camera_node', output='screen',
            parameters=[cfg, {
                'port': LaunchConfiguration('port'),
                'show': LaunchConfiguration('show'),
                'detect_qr': LaunchConfiguration('detect_qr'),
                'rotate_180': LaunchConfiguration('rotate_180'),
            }],
        ),
        Node(
            package='esp32_cam_udp', executable='qr_gripper_node',
            name='qr_gripper_node', output='screen',
            condition=IfCondition(LaunchConfiguration('gripper')),
            parameters=[cfg, {
                'port': LaunchConfiguration('gripper_port'),
                'trigger_text': LaunchConfiguration('trigger_text'),
                'gripper_angle': LaunchConfiguration('gripper_angle'),
                'release_angle': LaunchConfiguration('release_angle'),
                'release_timeout': LaunchConfiguration('release_timeout'),
            }],
        ),
    ])
