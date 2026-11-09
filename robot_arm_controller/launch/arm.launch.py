import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_arm_controller'),
                       'config', 'arm_params.yaml')
    port = LaunchConfiguration('port')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='robot_arm_controller', executable='arm_gui',
             name='robot_arm_controller', output='screen',
             parameters=[cfg, {'port': port}], condition=IfCondition(gui)),
        Node(package='robot_arm_controller', executable='arm_node',
             name='robot_arm_controller', output='screen',
             parameters=[cfg, {'port': port}], condition=UnlessCondition(gui)),
    ])
