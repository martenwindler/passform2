import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from passform_util.helper import load_yaml_file

# thanks to https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L21

def launch_setup(context, *args, **kwargs):

    ## getting path
    config_file = LaunchConfiguration('config_file').perform(context)
    config = load_yaml_file(config_file)

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    bay_state_publisher = Node(
        package='passform_base_description',
        executable='bay_state_publisher',
        name='bay_state_publisher')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_base',
        parameters=[{
            'use_sim_time': config['use_sim_time'],
            'robot_description': Command([
                # 'xacro ', LaunchConfiguration('model'),
                'xacro ', os.path.join(
                    get_package_share_directory(config['model_pkg']), config['model_path']),
                ' bay_file:=', os.path.join(
                    get_package_share_directory(config['model_pkg']), config['coordinate_path']),
            ])}],
        )

    product_relay = Node(
        package='passform_base_description',
        executable='product_relay',
        name='product_relay')

    return [bay_state_publisher, robot_state_publisher, product_relay]

def generate_launch_description():

    default_config_file = os.path.join(
        get_package_share_directory('passform_base_description'), 'config','params.yaml')

    declare_config_file_cmd = DeclareLaunchArgument(
        name='config_file',
        default_value=default_config_file,
        description='Absolute path to description config')

    return LaunchDescription([
        declare_config_file_cmd,
        OpaqueFunction(function = launch_setup)
        ])
