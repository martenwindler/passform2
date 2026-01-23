import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('passform_driver'),
        'config',
        'params.yaml'
        )

    gpio_server = Node(
        package="rpi_gpio",
        name="rpi_gpio",
        executable="rpi_gpio",
        parameters = [config],
    )

    ld.add_action(gpio_server)
    return ld
