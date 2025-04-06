// Basic C++ code to create and instantiate a ROS2 node.
// Author: Fabian Kung
// Date: 6 April 2025

#include <chrono>               // Contains header for timer  objects.
#include "rclcpp/rclcpp.hpp"                    // Header for ROS C++ library.
#include "example_interfaces/msg/string.hpp"    // Header for example ROS messages.

class MyNode2 : public rclcpp::Node // Create a node object.
{
public:
    MyNode2() : Node("My_Node2")
    {
        RCLCPP_INFO(this->get_logger(), "My_Node2 has started ..."); // Display a message.
    }

private:
    unsigned int unCount;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                     // Initialize rclcpp object.
  rclcpp::spin(std::make_shared<MyNode2>());    // Run node in background, non-blocking mode.
                                                // Note here we create a shared pointer to the
                                                // MyNode2 class first before we spin the object.
                                                // Shared pointer is a smart pointer in C++ standard
                                                // library for cases where more than one owners need
                                                // to manage the object simultaneously. See
                                                // Microsoft explanation. Here the class needs to be
                                                // access by our codes and also ROS processes in the
                                                // background. An alternative way to declare this 
                                                // 'auto' keyword in C++ is shown below:
//  auto node = std::make_shared<MyNodde2>();
//  rclcpp::spin(node);
  rclcpp::shutdown();                           // If CTRL-C key is received, execute shutdown()
                                                // method to perform some house-keeping tasks.
  return 0;
}