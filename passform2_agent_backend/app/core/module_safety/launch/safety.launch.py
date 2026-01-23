#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    auto_activate = LaunchConfiguration('auto_activate', default='False')
    namespace = LaunchConfiguration('ns', default='example')

    ld = LaunchDescription()

    DeclareLaunchArgument(
        'auto_activate',
        default_value=auto_activate,
        description='Specifying auto activate node')

    DeclareLaunchArgument(
        'ns',
        default_value=namespace,
        description='Specifying namespace to node')

    safety_node  = LifecycleNode(
        namespace=namespace,
        package='module_safety',
        executable='safety_node',
        name='safety_node',
        # parameters=[{'robot_name': 'C3PO'}],
        arguments=['-a', auto_activate],
        output='screen'
    )

    watchdog_node = LifecycleNode(
        package='sw_watchdog',
        executable='windowed_watchdog',
        namespace='',
        name='windowed_watchdog',
        output='screen',
        arguments=['220', '3', '--publish', '--activate']
        #arguments=['__log_level:=debug']
    )

    # When the watchdog reaches the 'inactive' state, log a message
    watchdog_inactive_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = watchdog_node,
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "Watchdog transitioned to 'INACTIVE' state." ),
            ],
        )
    )

    ld.add_action(safety_node)
    ld.add_action(watchdog_node)
    ld.add_action(watchdog_inactive_handler)

    return ld
