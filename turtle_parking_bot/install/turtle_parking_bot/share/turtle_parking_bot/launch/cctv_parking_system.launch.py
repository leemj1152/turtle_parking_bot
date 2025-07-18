# turtle_parking_bot/launch/cctv_parking_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_parking_bot',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='turtle_parking_bot',
            executable='cctv_detector',
            name='cctv_detector',
            output='screen'
        ),
        Node(
            package='turtle_parking_bot',
            executable='cctv_find_empty',
            name='cctv_find_empty',
            output='screen'
        ),
        Node(
            package='turtle_parking_bot',
            executable='center_controller',
            name='center_controller',
            output='screen'
        )
    ])
