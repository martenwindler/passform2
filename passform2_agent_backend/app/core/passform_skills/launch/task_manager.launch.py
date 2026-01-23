import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    action_node = Node(
        package="passform_skills",
        name="task_manager",
        namespace="",
        executable="task_manager",
        # arguments=['--ros-args', '--log-level', 'debug']

    )
    ld.add_action(action_node)

    return ld
