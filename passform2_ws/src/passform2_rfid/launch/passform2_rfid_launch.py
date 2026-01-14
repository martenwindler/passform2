from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='passform2_rfid', executable='main', name='passform2_rfid_node')
    ])
