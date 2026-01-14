from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='passform2',
            executable='manager',  # = Name in [project.scripts] 
            name='passform_manager_node',
            output='screen',
            parameters=[
                {'json_path': './data.json'},
                {'heartbeat_timeout': 60.0}
            ]
        ),
      # weitere oh-no-des
    ])
