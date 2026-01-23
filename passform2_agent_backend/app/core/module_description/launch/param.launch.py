import launch
import os
import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from passform_util.helper import load_yaml_file

# thanks to https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L21

def launch_setup(context, *args, **kwargs):

    ## getting path
    config_file = LaunchConfiguration('config_file').perform(context)
    config = load_yaml_file(config_file)

    urdf_path = os.path.join(
        get_package_share_directory(config['model_pkg']),
        config['model_path']
    )
    point_file = os.path.join(
        get_package_share_directory( LaunchConfiguration('point_pkg').perform(context)),
        LaunchConfiguration('point_file').perform(context)
    )

    xacro_command = LaunchConfiguration('xacro_command').perform(context)

    # needs context to concatenate arugment with string if the '/' should be part of the prefix
    ## https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/?answer=396630#post-id-396630
    prefix_subsitution = context.perform_substitution(LaunchConfiguration('uuid'))# +'/'
    launch_joint_state_publisher = LaunchConfiguration('launch_joint_state_publisher')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': config['use_sim_time'],
            'robot_description': Command([
                # 'xacro ', LaunchConfiguration('model'),
                'xacro ', urdf_path,
                ' prefix:=', prefix_subsitution,
                ' mesh_path:=', config['mesh_path'],
                ' ',xacro_command,
            ])}],
        )

    param = {'source_list': config['source_list']} if 'source_list' in config else dict()
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[param],
        condition=IfCondition(launch_joint_state_publisher)
    )

    reach_publisher = Node(
        package='module_description',
        executable='reach_publisher',
        name='reach_publisher',
        parameters=[
            {'point_file': point_file},
            {'uuid': LaunchConfiguration('uuid').perform(context)}
            ]
    )

    return [robot_state_publisher, joint_state_publisher, reach_publisher]

def generate_launch_description():

    default_config_file = os.path.join(
        get_package_share_directory('module_description'), 'config','params.yaml')
    declare_config_file_cmd = DeclareLaunchArgument(
        name='config_file',
        default_value=default_config_file,
        description='Absolute path to description config')
    declare_uuid_cmd = DeclareLaunchArgument(
        name='uuid',
        description='UUID of the model')
    declare_point_pkg = DeclareLaunchArgument(
        name='point_pkg',
        default_value='module_description',
        description='Package with the reach point data."')
    declare_point_file = DeclareLaunchArgument(
        name='point_file',
        default_value='config/points.yaml',
        description='Full path to point file within the specified package. Example: "config/points.yaml"')
    declare_xacro_command = DeclareLaunchArgument(
        name='xacro_command',
        default_value='',
        description='Further xacro commands for robot launch. Example: "name:=value"')
    declare_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='launch_joint_state_publisher',
        default_value='True',
        description='Defines if the joint_state_publisher for the given models shall be launched. \
        Set to "false" to prevent launching multiple joint_state_publisher in case another import \
        also launches one.')

    return launch.LaunchDescription([
        declare_config_file_cmd,
        declare_uuid_cmd,
        declare_point_pkg,
        declare_point_file,
        declare_xacro_command,
        declare_joint_state_publisher_cmd,
        OpaqueFunction(function = launch_setup)
        ])
