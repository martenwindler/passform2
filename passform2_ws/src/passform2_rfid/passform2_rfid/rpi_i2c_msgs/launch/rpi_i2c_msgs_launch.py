from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rpi_i2c_msgs', executable='main', name='rpi_i2c_msgs_node')
    ])
