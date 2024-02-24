#!/usr/bin/env python3      # Only use python 3.
'''
Description:    Code to scan for key press on keyboard or equivalent 
                input devices, decode the key command and publishes a 
                valid command message.
Author:         Fabian Kung
Last modified:  25 Dec 2023
'''

import rclpy                # Library for ROS2 in python. Note that this
                            # is a package, i.e. it is a folder named 
                            # 'rclpy' and contains a number of *.py 
                            # files which are the various modules making 
                            # up the library.
from rclpy.node import Node
from wheelrobot_interface.msg import RCCommand
#from example_interfaces.msg import String # Import String datatype for messaging.
import tty, termios
import sys, select

UserMessage = '''
Press the following keys to move the robot: 
f = forward 
t = backward 
r = turn right 
l = turn left
c = to stop the process
Any other key = stop
''' 

class TeleopKeyNode(Node):  # Create a class, inherited from the Node class.
    def __init__(self):     # Constructor.
        super().__init__("teleop_key_node") # Create a node, initialize node name. 
                            # Basically here we run all the parent's constructors. 
                            # In python, the super() function is used to give
                            # access to methods and properties of a parent or
                            # sibling class.
        #self.counter = 0    
        self.get_logger().info("Teleop_key node starting up...")
        self.get_logger().info(UserMessage)
        
        # Create a publisher using the interface RCCommand,
        # with topic "t_rc_command", and queue size of 8.
        self.publisher_ = self.create_publisher(RCCommand, "t_rc_command", 8)

        # Local variables.
        self.RCCmd = 1;
        self.RCarg1 = 0;
        self.RCarg2 = 0;
        self.RCarg3 = 0;

        # Create a timer
        # Timer period = 100 msec
        self.objTimer = self.create_timer(0.10,self.Timer_Callback)
        self.settings = termios.tcgetattr(sys.stdin)

    """
    Callback function for timer,
    """
    def Timer_Callback(self):    
        # The following codes to access keyboard in Linux
        # are copied from Robotis Turtlebot3 codes:
        # https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop.
        # Only for Linux and MacOS. For Windows need to use other low-level
        # system calls. See the original code to include support for Windows.
        tty.setraw(sys.stdin.fileno()) # Change the mode of terminal.
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)   # Access select() and poll()                                                                 
        if rlist:                                               # functions in the OS system 
            self.key = sys.stdin.read(1)                        # calls.
        else:
            self.key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) # Set attribute of file 
                                                                # descriptor sys.stdin
        # Decode the key.
        if self.key == 'c':
            #rclpy.shutdown() # Tested this will not work!
            self.destroy_timer(self.objTimer)
            self.get_logger().info("Press CTRL+C to quit")
        elif self.key != ' ':
            self.get_logger().info("Key pressed =" + str(self.key))
            msg = RCCommand()
            # Start parsing the key press.
            if self.key == 'f': # Move forward command         
                command = 70    # ASCII value for character 'F'
                arg1 = 48       # ASCII value for character '0'
                arg2 = 49       # ASCII value for character '1'
                arg3 = 48       # ASCII value for character '0'
                arg4 = 48       # ASCII value for character '0'
            elif self.key == 't': # Reverse slow   
                command = 70    # ASCII value for character 'F'
                arg1 = 45       # ASCII value for character '-'
                arg2 = 49       # ASCII value for character '1'
                arg3 = 48       # ASCII value for character '0'
                arg4 = 48       # ASCII value for character '0'                
            elif self.key == 'r': # Turn right a bit
                command = 84    # ASCII value for character 'T'
                arg1 = 45       # ASCII value for character '-'
                arg2 = 48       # ASCII value for character '0'
                arg3 = 49       # ASCII value for character '1'
                arg4 = 48       # ASCII value for character '0'  
            elif self.key == 'l': # Turn left a bit
                command = 84    # ASCII value for character 'T'
                arg1 = 48       # ASCII value for character '0'
                arg2 = 48       # ASCII value for character '0'
                arg3 = 49       # ASCII value for character '1'
                arg4 = 48       # ASCII value for character '0'                              
            else:               # Stop command
                command = 88    # ASCII value for character 'X'
                arg1 = 48       # ASCII value for character '0'
                arg2 = 48       # ASCII value for character '0'
                arg3 = 48       # ASCII value for character '0'
                arg4 = 48       # ASCII value for character '0'
            # Calculate the checksum value using sum complement method.
            sum = command + arg1 + arg2 + arg3 + arg4
            sum = ~sum + 1  # Creat a 2's complement of the sum of all bytes.
            msg.command = command
            msg.arg1 = arg1
            msg.arg2 = arg2
            msg.arg3 = arg3
            msg.arg4 = arg4
            msg.checksum = sum & 0xFF       # Mask out all bits except the lower 8 bits.
            self.publisher_.publish(msg)    # Publish message.

def main(args=None):
    rclpy.init(args=args)   # Initialize ROS communication library.
    node = TeleopKeyNode()  # Instantiate class.
    rclpy.spin(node)        # Block the current node so that other process can run.
    rclpy.shutdown()        # Shutdown.

if __name__ == "__main__":  # Only run this part if this file (or module as python file
                            # is called) is invoked directly by the python intepreter.
    main()