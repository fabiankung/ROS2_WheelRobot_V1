# Launch file
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    serial_com_node = Node(
        package="hw_interface_py_pkg",
        executable="serial_com"
    )

    ld.add_action(serial_com_node)
    return ld