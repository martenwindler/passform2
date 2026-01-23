import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('passform_base'),
        'config',
        'params.yaml'
        )

    base_manager = Node(
        package="passform_base",
        name="base_manager",
        executable="base_manager",
        namespace='base',
        parameters = [config]
    )

    base_manager = Node(
        package="passform_base",
        name="task_manager",
        executable="task_manager",
        namespace='base',
        parameters = [config]
    )

    ld.add_action(base_manager)

    return ld
