from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rpi_i2c', executable='main', name='rpi_i2c_node')
    ])
