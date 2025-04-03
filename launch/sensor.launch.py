from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = 'main_package'
    package_path = os.path.join('/home/gaurav/searchnrescue_dev_ws/install/', package_name, 'lib', package_name, 'nodes')

    return LaunchDescription([
        Node(
            package=package_name, 
            executable=os.path.join(package_path, 'ultrasonic_node.py'), 
            output='screen'
        ),
        Node(
            package=package_name, 
            executable=os.path.join(package_path, 'camera_node.py'), 
            output='screen'
        ),
        Node(
            package=package_name, 
            executable=os.path.join(package_path, 'gps_node.py'), 
            output='screen'
        ),
        Node(
            package=package_name, 
            executable=os.path.join(package_path, 'imu_node.py'), 
            output='screen'
        ),
    ])
