import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Name des Rust-Pakets
    package_name = 'passform_skills_rs'

    # Optionale Konfigurationsdatei laden (falls vorhanden)
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )

    action_node = Node(
        package=package_name,
        name="task_manager",
        namespace="",
        executable="task_manager", # Name der Binary in Cargo.toml
        parameters=[
            config,
            # Hier könnten auch Parameter direkt überschrieben werden:
            # {"response_wait_time": 3.0}
        ],
        output='screen',
        emulate_tty=True, # Wichtig für die farbige Rust-Logausgabe
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    ld.add_action(action_node)

    return ld