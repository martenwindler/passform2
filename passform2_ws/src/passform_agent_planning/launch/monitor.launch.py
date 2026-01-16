#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Pfad zum RViz-Config-File im share/config-Verzeichnis
    pkg_share = get_package_share_directory('passform_agent_planning')
    rviz_config = os.path.join(pkg_share, 'config', 'monitor.rviz')

    # 1) Monitor-Node starten
    monitor_node = Node(
        package='passform_agent_planning',
        executable='monitor_node.py',
        name='monitor_node',
        output='screen',
    )

    # 2) RViz starten mit der passenden Config
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        monitor_node,
        rviz_node,
    ])
