#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --------------------------------------------------------
    # 1) Launch-Argument: log_level (default = INFO)
    # --------------------------------------------------------
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='ROS logger level for all agent nodes'
    )
    log_level = LaunchConfiguration('log_level')

    # --------------------------------------------------------
    # 2) Grid-Konfiguration laden
    # --------------------------------------------------------
    pkg_share   = get_package_share_directory('passform_agent_planning')
    config_file = os.path.join(pkg_share, 'config', 'grid.yaml')

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # --------------------------------------------------------
    # 3) Für jedes Modul einen Node anlegen
    #    und den gewünschten Log-Level durchreichen
    # --------------------------------------------------------
    agents = []
    for m in config['modules']:
        pos_x, pos_y = m['position']
        agents.append(
            Node(
                package='passform_agent_planning',
                executable='agent_node.py',
                name=f"module_{pos_x}_{pos_y}",
                parameters=[{
                    'agent_id':   f"A_{pos_x}_{pos_y}",
                    'position':    m['position'],
                    'module_type': m['module_type'],
                    'is_goal':     m.get('is_goal', False),
                }],
                # --ros-args --log-level <level>
                arguments=['--ros-args', '--log-level', log_level],
                output='screen',
            )
        )

    # --------------------------------------------------------
    # 4) „Sleeper“, damit der Launch-Prozess selbst aktiv bleibt
    # --------------------------------------------------------
    sleeper = ExecuteProcess(
        cmd=['sleep', '9999'],
        name='sleeper',
        output='screen'
    )

    # --------------------------------------------------------
    # 5) LaunchDescription zurückgeben
    # --------------------------------------------------------
    return LaunchDescription([
        log_level_arg,
        *agents,
        sleeper,
    ])
