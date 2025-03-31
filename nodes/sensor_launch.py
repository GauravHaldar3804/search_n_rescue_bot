from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sensor_package', executable='ultrasonic_node', output='screen'),
        Node(package='sensor_package', executable='camera_node', output='screen'),
        Node(package='sensor_package', executable='gps_node', output='screen'),
        Node(package='sensor_package', executable='imu_node', output='screen'),
    ])
