import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package='passform_base_description').find('passform_base_description')
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_model_path = os.path.join(pkg_share, 'model/base.urdf.xacro')
    robot_name_in_urdf = 'passform_base'

    # Launch configuration variables specific to simulation
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare the launch arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # Specify the actions

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_bay_state_publisher = Node(
        package='passform_base_description',
        executable='bay_state_publisher',
        name='bay_state_publisher')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', model])
            }],
        arguments=[default_model_path])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_bay_state_publisher)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
