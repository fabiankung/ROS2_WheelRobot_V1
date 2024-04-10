# Launch file for wheel robot with nav2 support
# Include calling the Lidar launch file.
# Last modified: 10 March 2024
# Author:        Fabian Kung

import os

import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Specified the path to hte 'robot_bringup' package, and also to the URDF file and default
    # rviz configuration.
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_bringup').find('robot_bringup')
    default_model_path = os.path.join(pkg_share, 'urdf/wheelrobot1.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot_rangesensor_config.rviz')

    # Find the folder where Lidar driver package is installed, form a path pointing 
    # to the Lidar launch file.
    LIDAR_LAUNCH_FILE = '/ldlidar.launch.py'
    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('ldlidar'), 'launch')
    )

    print('serial_com_node')
    serial_com_node = launch_ros.actions.Node(
        package="hw_int_nav2_py_pkg",
        executable="serial_com"
    )
 
    print('rcstate_pub_node')
    rcstate_pub_node = launch_ros.actions.Node(
        package="hw_int_nav2_py_pkg",
        executable="rcstate_pub"
    )

    print('cmd_vel_RC_command_node')
    cmd_vel_RC_command_node = launch_ros.actions.Node(
        package="hw_int_nav2_py_pkg",
        executable="cmd_vel_RC_command"
    )

    print('ROS2 robot_state_publisher')
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

    print('v4l2 camera support node')
    v4l2_node = launch_ros.actions.Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[
            {'image_size': [320,240]},
            {'camara_frame_id': 'camera_link_optical'}
        ]
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
        #rviz_node,
        v4l2_node,

        # --- Python launch files from other packages ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LIDAR_LAUNCH_FILE])
        )
    ])
