#!/usr/bin/env python3      # Only use python 3.
'''
Description:    Code to convert the raw robot controller raw state parameters
                into Navigation2 required messages, and publish these.
Author:         Fabian Kung
Last modified:  23 Jan 2024
'''

import rclpy                # Library for ROS2 in python. Note that this
                            # is a package, i.e. it is a folder named 
                            # 'rclpy' and contains a number of *.py 
                            # files which are the various modules making 
                            # up the library.
from rclpy.node import Node
from wheelrobot_interface.msg import RCStateRaw
from sensor_msgs.msg import Range
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import cos, sin

# Declare constants of wheel robot platform
_RC_OS_TICK_SECONDS = 0.0001667
_NO_TICK_ROTATION = 450.0
_RAD_PER_TICK = 6.2832/_NO_TICK_ROTATION # 2*pi/(num_tick_rotation)

class RCStatePubNode(Node):  # Create a class, inherited from the Node class.

    # Static varaibles in the class.
    ranger_msg = Range()    # Object for storing IR range sensor data, right.
    rangel_msg = Range()    # Object for storing IR range sensor data, left.
    rangef_msg = Range()    # Object for storing IR range sensor data, front.
    jointstate_msg = JointState() # Object for storing joint state data.
    odom_msg = Odometry()   # Object for storing odometry information (from wheel encoders).
    tfodombase = TransformStamped() # Object for storing transform (tf) data, odom-->base_footprint mapping.

    dw = 0.156              # Wheel pitch, this needs to tally with URDF file.
    rw = 0.04               # Wheel radius, this needs to tally with URDF file.
    rw_dw = rw/dw
    rw_2 = rw/2.0

    def __init__(self):     # Constructor.
        super().__init__("rcstate_pub_node") # Create a node, initialize node name. 
                            # Basically here we run all the parent's constructors. 
                            # In python, the super() function is used to give
                            # access to methods and properties of a parent or
                            # sibling class.  
        self.get_logger().info("RC State Publisher node starting up...")

        # Create publisheres using the interface sensor_msgs.msg.Range,
        # with topic name "tpc_RightRange", "tpc_LeftRange", "tpc_FrontRange"
        # and queue size of 8 for each topic.
        self.publisher_r = self.create_publisher(Range, "t_rightrange", 8)
        self.publisher_l = self.create_publisher(Range, "t_leftrange", 8)
        self.publisher_t = self.create_publisher(Range, "t_frontrange", 8)

        # Create a publisher using the interface odom_msgs.msg.Odometry.
        self.publisher_odom = self.create_publisher(Odometry, "odom", 8)

        # Create a publisher using the interface sensor_msgs.msg.JointState.
        self.publisher_jointstate = self.create_publisher(JointState, "joint_states", 8)

        # Create a subscriber using the interface RCStateRaw, with topic "t_rc_status"
        # and queue size of 8.                    
        self.subscriber_ = self.create_subscription(RCStateRaw, 
                                                    "t_rc_status",self.Subscriber_Callback,8)
        
        # Create a tf transform broadcaster for TransformStamped.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Setup constant parameters of range sensors message: Sharp GP2Y0A41SK0F
        self.ranger_msg.header.frame_id = "right_rsen_link"
        self.ranger_msg.min_range = 0.04        # In meter.
        self.ranger_msg.max_range = 0.3         # In meter.
        self.ranger_msg.radiation_type = Range.INFRARED
        self.ranger_msg.field_of_view = 0.16    # To be determined.

        self.rangel_msg.header.frame_id = "left_rsen_link"
        self.rangel_msg.min_range = 0.04
        self.rangel_msg.max_range = 0.3  
        self.rangel_msg.radiation_type = Range.INFRARED
        self.rangel_msg.field_of_view = 0.16    # To be determined.    

        self.rangef_msg.header.frame_id = "front_rsen_link"
        self.rangef_msg.min_range = 0.04
        self.rangef_msg.max_range = 0.3  
        self.rangef_msg.radiation_type = Range.INFRARED
        self.rangef_msg.field_of_view = 0.16    # To be determined.     

        # Setup constant parameters of Jointstate message:
        self.jointstate_msg.header.frame_id = ""    # Leave it empty, follow joint_state_publisher_gui package.
        self.jointstate_msg.name = ['base_left_wheel_joint','base_right_wheel_joint']
        
        # Setup constant parameters of Odometry message:
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_footprint'      
        self.odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        self.odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        # Setup constant parameters of TF message:
        self.tfodombase.header.frame_id = "odom"
        self.tfodombase.child_frame_id = 'base_footprint'
        
        # Initial wheel angles or count.
        self.wrotationr = 0     # Wheel count
        self.wrotationl = 0
        self.wthetar = 0.0      # Wheel angle
        self.wthetal = 0.0
        # Initial pose and time.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lasttimestamp = 0
        

    # Subscriber callback function, publish range sensor data
    def Subscriber_Callback(self, msg):

        #self.get_logger().info("Raw state received")        
        timestamp = self.get_clock().now()      # Get current time-stamp in nano seconds.
        timestamp_msg = timestamp.to_msg()      # Convert time-stamp to message.
        
        self.ranger_msg.header.stamp = timestamp_msg
        self.ranger_msg.range = msg.sen_ranger/1000.0   # Normalized to meter.       
        self.rangel_msg.header.stamp = timestamp_msg
        self.rangel_msg.range = msg.sen_rangel/1000.0   # Normalized to meter.
        self.rangef_msg.header.stamp = timestamp_msg
        self.rangef_msg.range = msg.sen_rangef/1000.0   # Normalized to meter.

        self.jointstate_msg.header.stamp = timestamp_msg

        self.odom_msg.header.stamp = timestamp_msg

        self.tfodombase.header.stamp = timestamp_msg

        # Calculate time interval since last message.
        dt = (msg.timestamp - self.lasttimestamp)*_RC_OS_TICK_SECONDS 
        self.lasttimestamp = msg.timestamp # Update current timestamp.
        #self.get_logger().info("dt in sec " + str(dt))
        
        wcoeff = _RAD_PER_TICK / dt
        # Update estimated wheel angles. The reported values from robot controller (RC) is
        # number of wheel encoder ticks.
        self.wthetar = msg.rotater * _RAD_PER_TICK 
        self.wthetal = msg.rotatel * _RAD_PER_TICK   
        # Estimate the wheel angular velocity in rad/sec from change in wheel rotation count.
        wr = (msg.rotater - self.wrotationr)*wcoeff 
        wl = (msg.rotatel - self.wrotationl)*wcoeff 
        self.wrotationl = msg.rotatel # Update current rotation count for left wheel.
        self.wrotationr = msg.rotater # Update current rotation count for right wheel.

        # Calculate v and w from wheel angular velocities.
        v = (self.rw_2)*(wr + wl)           # 
        w = (self.rw_dw)*(wr - wl)
    
        # Updating the estimated pose of the robot using finite difference, based on the 
        # kinematic formulation by S. Thrun et-al, "Probabilistic robotics", 2005, MIT Press.
        dtheta = w * dt
        theta_new = self.theta + dtheta

        if (dtheta < 0.01):             # Use straight line approximation when angular velocity
                                        # is too small.
            Vdt = v*dt
            dx = Vdt*cos(self.theta)
            dy = Vdt*sin(self.theta)        
        else:                           # General curvilinear motion where robot move and turn 
                                        # at the same time.
            R = v/w
            dx = R*(-sin(self.theta)+sin(theta_new))
            dy = R*(cos(self.theta) - cos(theta_new))

        self.x = self.x + dx
        self.y = self.y + dy
        self.theta = theta_new
        
   
        # Debug messages
        #self.get_logger().info("wr " + str(wr))
        #self.get_logger().info("wl " + str(wl))
        #self.get_logger().info("v in rps " + str(v/(self.rw*6.283)))
        #self.get_logger().info("v in m/s " + str(v))
        #self.get_logger().info("w in rad/s" + str(w))  
        #self.get_logger().info("th " + str(self.theta)) 
        #self.get_logger().info("x " + str(self.x))
        #self.get_logger().info("y " + str(self.y)) 
        #self.get_logger().info("wthetar " + str(self.wthetar))
        #self.get_logger().info("wthetal " + str(self.wthetal))

        '''
        Set the pose, in (x,y,z) and unit quarternion (x,y,z,w).
        Unit quarternion is a compact way to hold the rotation
        information in equivalent angle-axis representation (J. Craig, "Introduction
        to robotics - mechanics and control", 4th edition 2022, Pearson). The equi-
        valent axis is a unit vector. Assuming axis = kx*i+ky*j+kz*k, where i,j and k
        are the unit vectors along x,y and z axis, and angle = theta, then the unit
        quarternion elements are given by:
        x = kx*sin(theta/2)
        y = ky*sin(theta/2)
        z = kz*sin(theta/2)
        w = cos(theta/2)

        For our robot, axis = 0*i+0*j+1*k  (Rotation along z-axis)
        angle = theta_new, thus q = (x,y,z,w) is:
        x = 0
        y = 0
        z = sin(theta_new/2)
        w = cos(theta_new/2)
        '''
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.x = 0.0  # Orientation is in quarternion format.
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = sin(theta_new/2.0)
        self.odom_msg.pose.pose.orientation.w = cos(theta_new/2.0)

        self.tfodombase.transform.translation.x = self.x
        self.tfodombase.transform.translation.y = self.y
        self.tfodombase.transform.translation.z = 0.0
        self.tfodombase.transform.rotation.x = 0.0
        self.tfodombase.transform.rotation.y = 0.0
        self.tfodombase.transform.rotation.z = sin(theta_new/2.0)
        self.tfodombase.transform.rotation.w = cos(theta_new/2.0)

        # Set the twist.
        self.odom_msg.twist.twist.linear.x = v*cos(theta_new/2.0)
        self.odom_msg.twist.twist.linear.y = v*sin(theta_new/2.0)
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.z = w

        # Set all joint states.
        self.jointstate_msg.position = [self.wthetal, self.wthetar]

        self.publisher_r.publish(self.ranger_msg)       # Publish right range sensor message.
        self.publisher_l.publish(self.rangel_msg)       # Publish left range sensor message.
        self.publisher_t.publish(self.rangef_msg)       # Publish front range message.
        self.publisher_jointstate.publish(self.jointstate_msg)  # Publish joint state message.
        self.publisher_odom.publish(self.odom_msg)      # Publish odometry message.
        
        self.tf_broadcaster.sendTransform(self.tfodombase)


def main(args=None):
    rclpy.init(args=args)   # Initialize ROS communication library.
    node = RCStatePubNode()  # Instantiate class.
    rclpy.spin(node)        # Block the current node so that other process can run.
    rclpy.shutdown()        # Shutdown.

if __name__ == "__main__":  # Only run this part if this file (or module as python file
                            # is called) is invoked directly by the python intepreter.
    main()