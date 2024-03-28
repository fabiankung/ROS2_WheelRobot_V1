#!/usr/bin/env python3      # Only use python 3.
'''
Description:    Code to scan capture /cmd_vel message from ROS2.
                /cmd_vel is off the type geometry_msgs/msg/Twist
                which contains 3D velocity and 3D angular velocity
                settings issued by various packages from ROS2. For
                wheeled robot only the velocity vx and angular 
                velocity wz are used. This node converts vx and wz
                the command to /t_rc_command message for controlling
                the movement of my custom wheel robot.
Author:         Fabian Kung
Last modified:  18 March 2024
'''

import rclpy                # Library for ROS2 in python. Note that this
                            # is a package, i.e. it is a folder named 
                            # 'rclpy' and contains a number of *.py 
                            # files which are the various modules making 
                            # up the library.
from rclpy.node import Node
from wheelrobot_interface.msg import RCCommand
from geometry_msgs.msg import Twist # Import String datatype for messaging.

#import robot_physical_params as robot
_TURN_ANGLE_PER_HEADING_TICK = 0.004 

class Cmd_VelRCCommandNode(Node):  # Create a class, inherited from the Node class.

    #dw = robot._WHEEL_RADIUS # Wheel pitch, meter, this needs to tally with URDF file.
    #rw = robot._WHEEL_PITCH  # Wheel radius, meter, this needs to tally with URDF file.
    dw = 0.156              # Wheel pitch, meter, this needs to tally with URDF file.
    rw = 0.042              # Wheel radius, meter, this needs to tally with URDF file.
    coeff_speed_to_RPS_scaled_100 = 100.0/(6.2832*rw)
    coeff_angular_to_tick = 1.0/_TURN_ANGLE_PER_HEADING_TICK  

    def __init__(self):     # Constructor.
        super().__init__("Cmd_Vel_RCCommand_node") # Create a node, initialize node name. 
                            # Basically here we run all the parent's constructors. 
                            # In python, the super() function is used to give
                            # access to methods and properties of a parent or
                            # sibling class.  
        self.get_logger().info("Cmd_Vel_RCCommand_node starting up...")

        # Create a publisher using the interface RCCommand,
        # with topic name "t_rc_command", and queue size of 6.
        self.publisher_ = self.create_publisher(RCCommand, "t_rc_command", 6)

        # Create a subscriber using the interface Twist, with topic name 
        # "cmd_vel" and queue size of 6.                    
        self.subscriber_ = self.create_subscription(Twist, 
                                                    "cmd_vel",self.Subscriber_Callback,6)

    # Linear velocity setting in ROS2 is in m/sec.
    # Linear velocity specification in my custom robot is in rotation-per-second (RPS), multiply
    # by 100 and approximate to signed integer, then offset by 512.
    # Angular velocity setting in ROS2 is rad/sec.
    # Angular velocity specification in my custom robot is in heading-ticks per second, and
    # then offset by 512.
    def Subscriber_Callback(self, msg):
        objRCCommand = RCCommand()      # Create a message object.
        #self.get_logger().info("/cmd_vel is " + str(msg.linear.x)) 
        #self.get_logger().info("/cmd_vel is " + str(msg.angular.z)) 
        set_vx = msg.linear.x
        set_wz = msg.angular.z
        nset_vx_rps = int(set_vx * self.coeff_speed_to_RPS_scaled_100) + 512
        nset_wz_tps = int(set_wz * self.coeff_angular_to_tick) + 512
        wz_stream = (nset_wz_tps).to_bytes(2, byteorder = 'big')
        vx_stream = (nset_vx_rps).to_bytes(2, byteorder = 'big')

        command = 116   # ASCII value for character 't'
        arg3 = wz_stream[0]
        arg4 = wz_stream[1]
        arg1 = vx_stream[0]
        arg2 = vx_stream[1] 
        # Calculate the checksum value using sum complement method.
        sum = command + arg1 + arg2 + arg3 + arg4
        sum = ~sum + 1  # Creat a 2's complement of the sum of all bytes.
        objRCCommand.command = command
        objRCCommand.arg1 = arg1
        objRCCommand.arg2 = arg2
        objRCCommand.arg3 = arg3
        objRCCommand.arg4 = arg4
        objRCCommand.checksum = sum & 0xFF       # Mask out all bits except the lower 8 bits.
        self.publisher_.publish(objRCCommand)
        
        


def main(args=None):
    rclpy.init(args=args)   # Initialize ROS2 communication library.
    node = Cmd_VelRCCommandNode()  # Instantiate class.
    rclpy.spin(node)        # Block the current node so that other process can run.
    rclpy.shutdown()        # Shutdown.

if __name__ == "__main__":  # Only run this part if this file (or module as python file
                            # is called) is invoked directly by the python intepreter.
    main()