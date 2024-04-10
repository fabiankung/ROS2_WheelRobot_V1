# ROS2_WheelRobot_V1
Experimental codes for standard differential drive wheel robot
Hardware Target: Raspberry Pi 4B (4GB RAM)
OS: Ubuntu 22.04
ROS Distribution: ROS2 Humble LTS
Lidar Sensor: LD06
Camera: Raspberry Pi camera V1 (5 Mega pixels)

This is an attempt to learn up the eco-system of ROS2, and also the Navigation2 library. I am building a basic wheel robot to test out my codes.
For video of the robot, please see: https://youtu.be/fhEOr7UUopw

To install on Ubuntu in the Raspberry Pi 4B apart from ROS2 Humble:
1. SSH server
2. ros-humble-navigation2
3. ros-humble-nav2-bringup
5. ldlidar (see https://github.com/linorobot/ldlidar)
6. v4l_utils
7. ros-humble-v4l2-camera
