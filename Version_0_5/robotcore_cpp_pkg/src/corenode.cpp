// Description:    Example code to handle interfacing with
//                 serial port code.
// Author:         Fabian Kung
// Last modified:  28 Aug 2023

#include "rclcpp/rclcpp.hpp"
//#include "example_interfaces/msg/string.hpp"
#include "custom_robot_interface/msg/rc_status.hpp"
#include "custom_robot_interface/msg/rc_command.hpp"

class RobotCoreNode : public rclcpp::Node   // Create a node object.
{
public:
    RobotCoreNode() : Node("robot_core_node") // Constructor.
    {
        // Create a subscriber for topic 'tpc_RC_Status'.
        subscriber_ = this->create_subscription<custom_robot_interface::msg::RCStatus>(
            "tpc_RC_Status", 10,
            std::bind(&RobotCoreNode::callbackRC_Status, this, std::placeholders::_1));
        // Note: The callback function has 1 parameter, so we need to add the argument
        // std::placeholders::_1 after the 'this' keyword. 
        // If the callback function has 2 parameters, then we also need to add a 2nd placeholder
        // argument, i.e. std::placeholders::_2
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&RobotCoreNode::publishCommand, this));        
        // Create a publisher for topic 'tpc_RC_Command'.
        publisher_ = this->create_publisher<custom_robot_interface::msg::RCCommand>(
            "tpc_RC_Command",10);

        RCcommand = 1;
        RCarg1 = 0;
        RCarg2 = 0;
        RCarg3 = 0;

        RCLCPP_INFO(this->get_logger(), "Robot core node has been started.");
    }

private:
    // Subscriber callback function for subscriber to RCStatus topic
    void callbackRC_Status(const custom_robot_interface::msg::RCStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"Distance %d", msg->distance);
        RCLCPP_INFO(this->get_logger(),"Heading %d", msg->heading);
    }

    // Publisher method for RCCommand
    void publishCommand()
    {
        u_int8_t bytSum;

        auto msg = custom_robot_interface::msg::RCCommand(); // Note that RCCommand is
                                                             // a function or method.
        msg.command = RCcommand;                             // Populate the message packet   
        msg.arg1 = RCarg1;                                   // from private variables.
        msg.arg2 = RCarg2;
        msg.arg3 = RCarg3;

        bytSum = ~(RCcommand + RCarg1 + RCarg2 + RCarg3);   // Calculate the 1st complement of the sum.
        bytSum = bytSum + 1;                                // Calculate the 2nd complement.
        msg.checksum = bytSum;                              // Checksum is computed using sum complement algorithm.                       
        publisher_->publish(msg);                                                     
    }

    rclcpp::Subscription<custom_robot_interface::msg::RCStatus>::SharedPtr subscriber_;
    rclcpp::Publisher<custom_robot_interface::msg::RCCommand>::SharedPtr publisher_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    u_int8_t RCcommand;
    u_int8_t RCarg1;
    u_int8_t RCarg2;
    u_int8_t RCarg3;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);   // Initialize rclcpp object.
    auto node = std::make_shared<RobotCoreNode>();      
    rclcpp::spin(node);     // Run node in the background, non-blocking.
    rclcpp::shutdown();     // If CTRL-C key is received, execute shutdown() method.
    return 0;
}
// 1. 'std' is a namespace in modern C++ that denotes standard library. Advisable to use this 
// when invoking system variables, classes and functions to avoid conflict.
// 2. Create a shared pointer to a node class. Shared pointer is a newer feature of C++,
// easier for garbage collection when we exit the application or go out of scope. 
// Importantly several shared pointer objects may own the same object.
// 3. We do not need to manually perform the garbage collection like in tradisional pointer. 
// 4. Shared pointer is similar to C# smart pointer.
// 5. make_shared< > is a function in C++ to create shared pointer. We can also use 'new' and 
// 'shared_ptr< >' keywords but less efficient.
//
// Note also the 'auto' keyword. This will automatically select the correct datatype.
// 'auto' is a simple to declare a variable that has a complicated type. This is a feature
// in C++ 11 or newer. Something like python where we do not have to explicitly specify 
// the datatype of a variable and leave it to the compiler.