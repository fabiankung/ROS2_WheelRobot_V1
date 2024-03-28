# Launch file for wheel robot with nav2 support
# Last modified: 18 March 2024
# Author:        Fabian Kung
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    #pkg_share = launch_ros.substitutions.FindPackageShare(package='wheelrobotnav2_description').find('wheelrobotnav2_description')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_bringup').find('robot_bringup')
    #default_model_path = os.path.join(pkg_share, 'urdf/wheelrobot1.urdf')
    default_model_path = os.path.join(pkg_share, 'urdf/wheelrobot1.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot_rangesensor_config.rviz')

    print('serial com')
    serial_com_node = launch_ros.actions.Node(
        package="hw_int_nav2_py_pkg",
        executable="serial_com"
    )

    print('hw state')
    rcstate_pub_node = launch_ros.actions.Node(
        package="hw_int_nav2_py_pkg",
        executable="rcstate_pub"
    )

    print('cmd_vel_RC_command')
    cmd_vel_RC_command_node = launch_ros.actions.Node(
        package="hw_int_nav2_py_pkg",
        executable="cmd_vel_RC_command"
    )


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        serial_com_node,
        rcstate_pub_node,
        cmd_vel_RC_command_node,
        robot_state_publisher_node,
        rviz_node
    ])