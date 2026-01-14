import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('rpi_i2c'),
        'config',
        'params.yaml'
        )

    rpi_i2c_di = Node(
        package="rpi_i2c",
        name="rpi_i2c_di",
        executable="rpi_i2c_di",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rpi_i2c_do = Node(
        package="rpi_i2c",
        name="rpi_i2c_do",
        executable="rpi_i2c_do",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rpi_i2c_ao = Node(
        package="rpi_i2c",
        name="rpi_i2c_ao",
        executable="rpi_i2c_ao",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rpi_gpio_us = Node(
        package="rpi_i2c",
        name="rpi_gpio_us",
        executable="rpi_gpio_us",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    angle_sensor = Node(
        package="rpi_i2c",
        name="angle_sensor",
        executable="angle_sensor",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rpi_i2c_ai = Node(
        package="rpi_i2c",
        name="rpi_i2c_ai",
        executable="rpi_i2c_ai",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    gpio_us_node = Node(
        package="rpi_i2c",
        name="gpio_us_node",
        executable="gpio_us_node",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # modul_logic = Node(
    #     package="rpi_i2c",
    #     name="modul_logic",
    #     executable="modul_logic",
    #     parameters = [config],
    #     arguments=['--ros-args', '--log-level', 'info']
    # )

    compartment = Node(
        package="rpi_i2c",
        name="compartment",
        executable="compartment",
        parameters = [config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(rpi_i2c_di)
    ld.add_action(rpi_i2c_ao)
    #ld.add_action(rpi_i2c_do)
    ld.add_action(compartment)
    ld.add_action(rpi_i2c_ai)
    ld.add_action(angle_sensor)
    ld.add_action(gpio_us_node)
    return ld
