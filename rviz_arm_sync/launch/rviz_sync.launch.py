import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('robotic_arm_description')
    ctrl_share = get_package_share_directory('robot_arm_controller')
    sync_share = get_package_share_directory('rviz_arm_sync')

    port = LaunchConfiguration('port')
    gui = LaunchConfiguration('gui')

    # 1) RViz + joint sliders (your existing package, unchanged)
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'display.launch.py')))

    # 2) Real arm driver (your existing package, unchanged). Headless by default so the
    #    Tk GUI sliders do not fight with the RViz sliders.
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl_share, 'launch', 'arm.launch.py')),
        launch_arguments={'port': port, 'gui': gui}.items())

    # 3) /joint_states (RViz) -> /arm/joint_command (real arm)
    sync = Node(
        package='rviz_arm_sync', executable='rviz_to_arm', name='rviz_to_arm',
        parameters=[os.path.join(sync_share, 'config', 'sync_map.yaml')],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gui', default_value='false'),
        display, arm, sync,
    ])
