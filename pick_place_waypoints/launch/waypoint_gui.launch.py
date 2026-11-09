from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick_place_waypoints',
            executable='waypoint_gui',
            name='pick_place_waypoints',
            output='screen',
        )
    ])
