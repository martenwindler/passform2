from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='passform2_agent_frontend', executable='main', name='passform2_agent_frontend_node')
    ])
