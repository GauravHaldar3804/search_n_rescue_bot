from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='main_package', executable='ultrasonic_node.py', output='screen'),
        Node(package='main_package', executable='camera_node.py', output='screen'),
        Node(package='main_package', executable='gps_node.py', output='screen'),
        Node(package='main_package', executable='imu_node.py', output='screen'),
    ])
