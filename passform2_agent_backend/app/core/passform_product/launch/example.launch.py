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
    config_file = os.path.join(
        get_package_share_directory('passform_product'), 'config',LaunchConfiguration('config_file').perform(context))
    config = load_yaml_file(config_file)

    nodes = list()
    for uuid, data in config.items():
        nodes.append(
            Node(
                package='passform_product',
                executable='product_publisher',
                name='product_'+str(uuid).replace('-','_'),
                parameters=[data, {'uuid': str(uuid)}]
            )
        )

    return nodes

def generate_launch_description():

    default_config_file = 'params.yaml'
    declare_config_file_cmd = DeclareLaunchArgument(
        name='config_file',
        default_value=default_config_file,
        description='Config file name. Stored in config folder.')

    return LaunchDescription([
        declare_config_file_cmd,
        OpaqueFunction(function = launch_setup)
        ])
