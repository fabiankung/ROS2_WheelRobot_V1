#!/usr/bin/env python3      # Only use python 3.
'''
Description:    Code to handle outgoing and incoming data packets from
                serial port.
Author:         Fabian Kung
Last modified:  28 Aug 2023
'''

import rclpy                # Library for ROS2 in python. Note that this
                            # is a package, i.e. it is a folder named 
                            # 'rclpy' and contains a number of *.py 
                            # files which are the various modules making 
                            # up the library.
from rclpy.node import Node
#from example_interfaces.msg import String # Import String datatype for messaging.
from custom_robot_interface.msg import RCStatus
from custom_robot_interface.msg import RCCommand

import serial


class SerialComNode(Node):  # Create a class, inherited from the Node class.

    ReadData = []           # Class level variable, captured data from serial port.

    def __init__(self):     # Constructor.
        super().__init__("serial_com_node") # Create a node, initialize node name. 
                            # Basically here we run all the parent's constructors. 
                            # In python, the super() function is used to give
                            # access to methods and properties of a parent or
                            # sibling class.
        #self.counter = 0    
        self.get_logger().info("Serial COM node starting up...")
        
        # Create a publisher using the interface RCStatus,
        # with topic "tpc_RC_Status", and queue size of 10.
        self.publisher_ = self.create_publisher(RCStatus, "tpc_RC_Status", 10)

        # Create a subscriber using the interface RCCommand, with topic "tpc_RC_Command"
        # and queue size of 10.                    
        self.subscriber_ = self.create_subscription(RCCommand, 
                                                    "tpc_RC_Command",self.Subscriber_Callback,10)

        # Variables for Robot Controller (RC) serial interface.
        #self.RC_TX_Busy = False         # TX busy flag. Set to 'true' to
                                        # initiate sending of command to RC.
        #self.utf8TXCommandString = ""   # Command string to send to RC.
        #self.TXCount = 0                # TX count.

        # Create a timer
        # Timer period = 50 msec
        self.create_timer(0.05,self.Timer_Callback)
        
        # Create a serial port handle. Settings:
        # Baud rate = 38400
        # Stop bit = 1
        # Parity = None
        # Byte length = 8 bits
        # Read timeout = 10 msec. External hardware usually responds
        # within 5 msec.
        # Write timeout = 10 msec.
        self.sport = serial.Serial('/dev/ttyUSB0', 38400) # USB-to-serial converter
        #self.sport = serial.Serial('/dev/ttyACM0', 38400) # Arduino Micro
        self.sport.timeout = 0.01 # Read timeout.
        self.sport.write_timeout = 0.01 # Write timeout.
        self.sport.reset_input_buffer() # Clear receive buffer first.

    def Timer_Callback(self):   # Callback function for timer.
        if self.sport.is_open == True:
            bytetoread = self.sport.in_waiting
            if bytetoread > 9:  # At least 10 bytes
                ReadData = self.sport.read(bytetoread)
                self.sport.reset_input_buffer()
                #self.get_logger().info(str(ReadData))  # Echo receive data to terminal.
                    
                # Verify that the checksum value is correct. 
                # Here we are using the sum complement method, So all the bytes in the
                # received buffer should adds up to 256 (lower 8-bits is 0) or 0.
                sum = 0
                for data in ReadData:
                    sum = sum + data
                if (sum == 0) or (sum == 256): # Make sure checksum is correct.
                    msg = RCStatus() # Note RCStatus is a function or method.
                    # Get distance. Convert the bytes into a 32-bits integer.
                    tempdata = ReadData[0] + (ReadData[1]<<8) + (ReadData[2]<<16) + (ReadData[3]<<24)
                    msg.distance = tempdata
                    # Get velocity. Convert the bytes into 16-bits integer.
                    msg.velocity = ReadData[4] + (ReadData[5]<<8)
                    # Get heading. Convert the bytes into 16-bits integer.
                    msg.heading = ReadData[6] + (ReadData[7]<<8)
                    msg.hwstatus1 = ReadData[8]     # Get hardware status 1.
                    self.publisher_.publish(msg)    # Publish message.
                        
                    self.get_logger().info("Checksum check " + str(sum))                                          

    def __del__(self):      # Destructor.
        if self.sport.is_open == True:    # Close the serial port.
            self.sport.close()
    
    def Subscriber_Callback(self, msg):
        # Check checksum value. The sum of all bytes in the 
        # message should adds up to 0 or 256 (lower 8-bits are zero)
        command = msg.command
        arg1 = msg.arg1
        arg2 = msg.arg2
        arg3 = msg.arg3
        checksum = msg.checksum
        sum = command + arg1 + arg2 + arg3 + checksum   # Make sure checksum is correct.
        #self.get_logger().info(str(sum))
        if (sum == 0) or (sum == 256): # Make sure checksum is correct.
            if self.sport.is_open == True: # Make sure serial port is opened.
                self.sport.write(bytes([command, arg1, arg2, arg2, checksum]))    
            

def main(args=None):
    rclpy.init(args=args)   # Initialize ROS communication library.
    node = SerialComNode()  # Instantiate class.
    rclpy.spin(node)        # Block the current node so that other process can run.
    rclpy.shutdown()        # Shutdown.

if __name__ == "__main__":  # Only run this part if this file (or module as python file
                            # is called) is invoked directly by the python intepreter.
    main()