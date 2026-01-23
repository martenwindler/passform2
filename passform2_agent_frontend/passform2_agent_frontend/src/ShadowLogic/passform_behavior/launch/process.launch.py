import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    ## getting path
    share_dir = get_package_share_directory('passform_behavior')
    config = os.path.join(share_dir, 'config', 'params.yaml')
    behavior_file = os.path.join(share_dir, 'config', context.perform_substitution(LaunchConfiguration('process_file')))

    load_nodes = GroupAction(
        actions=[
            Node(
                package="passform_behavior",
                name="process_node",
                namespace="",
                executable="process_node",
                parameters=[
                        {'behavior_file': behavior_file},
                        config
                    ],
            )
        ]
    )

    # Add any actions
    return [load_nodes]


def generate_launch_description():

    default_config_file = 'params.yaml'
    declare_config_file_cmd = DeclareLaunchArgument(
        name='config_file',
        default_value=default_config_file,
        description='Name of config file')

    default_process_file = 'example_process.xml'
    declare_process_file_cmd = DeclareLaunchArgument(
        name='process_file',
        default_value=default_process_file,
        description='Name of process file')

    return launch.LaunchDescription([
        declare_config_file_cmd,
        declare_process_file_cmd,
        OpaqueFunction(function = launch_setup)
        ])