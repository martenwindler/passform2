from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package="rpi_piio",
        executable="piio_node",
        name = "piio_node",
        output = "screen"
    ),
    Node(
        package="rpi_piio",
        executable="piio_test_publisher",
        name = "test_talker",
        output = "screen"
    )
    ])
