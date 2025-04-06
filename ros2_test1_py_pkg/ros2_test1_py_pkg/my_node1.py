#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode1(Node):    # Create a class, inherited from the Node class.
    def __init__(self): # Constructor.
        super().__init__("My_node1")    # Create a node, initialize node name.
                                        # Basically here we will run all the
                                        # paranets constructor.
        self.counter = 0
        self.get_logger().info("MyNode1 is starting up...")

    # def __del__(self): # Destructor.
    # enable this if you have any codes to run when this node 
    # stops.
        
def main(args=None):
    rclpy.init(args=args)
    node = MyNode1()        # Instantiate the class.
    rclpy.spin(node)        # Keep the node running, non-blocking mode so that 
                            # other processes can also run.
    rclpy.shutdown()        # Shutdown the node.

if __name__ == '__main__':  # Means this class cannot be run as python module.
    main()