from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'main_package'

    return LaunchDescription([
        Node(
            package=package_name, 
            executable='ultrasonic_node.py',  # No absolute paths!
            output='screen'
        ),
        Node(
            package=package_name, 
            executable='camera_node.py', 
            output='screen'
        ),
        Node(
            package=package_name, 
            executable='gps_node.py', 
            output='screen'
        ),
        Node(
            package=package_name, 
            executable='imu_node.py', 
            output='screen'
        ),
    ])
