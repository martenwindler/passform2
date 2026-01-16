# passform_agent_planning/launch/single_agent.launch.py
#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # 1) Launch‐Argumente
    declare_position = DeclareLaunchArgument(
        'position',
        default_value='[0, 0]',
        description='Initial grid position as [x, y]'
    )
    declare_module_type = DeclareLaunchArgument(
        'module_type',
        default_value='greifer',
        description='Type of the module (greifer, rollen_ns, rollen_ow, mensch)'
    )

    # 2) Node‐Definition
    agent_node = Node(
        package='passform_agent_planning',
        executable='agent_node.py',
        output='screen',
        parameters=[{
            'position':    LaunchConfiguration('position'),
            'module_type': LaunchConfiguration('module_type'),
        }]
    )

    # 3) Zusammenbauen
    return LaunchDescription([
        declare_position,
        declare_module_type,
        agent_node,
    ])
