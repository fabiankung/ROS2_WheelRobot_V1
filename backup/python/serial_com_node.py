#!/usr/bin/env python3      # Only use python 3.
'''
Description:    Code to handle outgoing and incoming data packets from
                serial port.
Author:         Fabian Kung
Last modified:  8 Aug 2023
'''

import rclpy                # Library for ROS2 in python. Note that this
                            # is a package, i.e. it is a folder named 
                            # 'rclpy' and contains a number of *.py 
                            # files which are the various modules making 
                            # up the library.
from rclpy.node import Node
from example_interfaces.msg import String # Import String datatype for messaging.

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

        self.publisher_ = self.create_publisher(String, "tpc_RC_Status", 5)
                            # Create a topic publisher using the interface String,
                            # with name "tpc_RC_Status", and queue size of 5.
        # Variables for Robot Controller (RC) serial interface.
        self.RC_TX_Busy = False         # TX busy flag. Set to 'true' to
                                        # initiate sending of command to RC.
        self.utf8TXCommandString = ""   # Command string to send to RC.
        self.TXCount = 0                # TX count.

        # Create a timer
        # Timer period = 100 msec
        self.create_timer(0.1,self.Timer_Callback)
        
        # Create a serial port handle. Settings:
        # Baud rate = 38400
        # Stop bit = 1
        # Parity = None
        # Byte length = 8 bits
        # Read timeout = 10 msec. External hardware usually responds
        # within 5 msec.
        # Write timeout = 20 msec.
        self.sport = serial.Serial('/dev/ttyUSB0', 38400)
        self.sport.timeout = 0.01 # Read timeout.
        self.sport.write_timeout = 0.02 # Write timeout.
        self.sport.reset_input_buffer() # Clear receive buffer first.

    def Timer_Callback(self):   # Callback function for timer.
        #self.counter += 1
        #self.get_logger().info("Hello " + str(self.counter))
        if self.sport.is_open == True:
            if self.RC_TX_Busy == True:
                self.sport.write(self.utf8TXCommandString)
                self.RC_TX_Busy = False
            else:
                bytetoread = self.sport.in_waiting
                if bytetoread > 0:
                    ReadData = self.sport.read(bytetoread)
                    self.sport.reset_input_buffer()
                    self.get_logger().info(str(ReadData))  # Echo receive data to terminal.
                    msg = String()
                    msg.data = str(ReadData) + " (Python version)"
                    self.publisher_.publish(msg)           

    def __del__(self):      # Destructor.
        if self.sport.is_open == True:    # Close the serial port.
            self.sport.close()


    def SendtoRC(self, string):
        if self.RC_TX_Busy == False:
            self.RC_TX_Busy = True          # Inform the Timer_Callback there is
                                            # data to send to RC.
            self.utf8TXCommandString = string
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)   # Initialize ROS communication library.
    node = SerialComNode()  # Instantiate class.
    rclpy.spin(node)        # Block the current node so that other process can run.
    rclpy.shutdown()        # Shutdown.

if __name__ == "__main__":  # Only run this part if this file (or module as python file
                            # is called) is invoked directly by the python intepreter.
    main()