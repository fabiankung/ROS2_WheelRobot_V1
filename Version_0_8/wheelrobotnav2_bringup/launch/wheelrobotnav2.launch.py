# Launch file for wheel robot with nav2 support
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    serial_com_node = Node(
        package="hw_int_nav2_py_pkg",
        executable="serial_com"
    )

    rcstate_pub_node = Node(
        package="hw_int_nav2_py_pkg",
        executable="rcstate_pub"
    )

    ld.add_action(serial_com_node)
    ld.add_action(rcstate_pub_node)
    return ld