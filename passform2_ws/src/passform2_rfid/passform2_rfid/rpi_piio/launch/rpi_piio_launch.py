from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rpi_piio', executable='main', name='rpi_piio_node')
    ])
