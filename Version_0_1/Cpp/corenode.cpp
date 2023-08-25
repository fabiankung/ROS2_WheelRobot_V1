#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotCoreNode : public rclcpp::Node   // Create a node object.
{
public:
    RobotCoreNode() : Node("robot_core_node") // Constructor.
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>("tpc_RC_Status", 10,
            std::bind(&RobotCoreNode::callbackRC_Status, this, std::placeholders::_1));
        //timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
        //                                 std::bind(&RobotCoreNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot core node has been started.");
    }

private:
    void callbackRC_Status(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"%s", msg->data.c_str());
    }

    
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
    //rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);   // Initialize rclcpp object.
    auto node = std::make_shared<RobotCoreNode>();      // 1. 'std' is a namespace in modern C++ that 
                                                        // denotes standard library. Advisable to use this 
                                                        // when invoking system variables, classes and 
                                                        // functions to avoid conflict.
                                                        // 2. Create a shared pointer to a node class. 
                                                        // Shared pointer is a newer feature of C++,
                                                        // easier for garbage collection when we
                                                        // exit the application or go out of scope. 
                                                        // Importantly several shared pointer objects may
                                                        // own the same object.
                                                        // 3. We do not need to manually perform the 
                                                        // garbage collection like in tradisional 
                                                        // pointer. 
                                                        // 4. Shared pointer is similar to C# smart pointer.
                                                        // 5. make_shared< > is a function in C++ to create
                                                        // shared pointer. We can also use 'new' and 
                                                        // 'shared_ptr< >' keywords but less efficient.
                                                        //
                                                        // Note also the 'auto' keyword. This will
                                                        // automatically select the correct datatype.
                                                        // 'auto' is a simple to declare a variable that
                                                        // has a complicated type. This is a feature
                                                        // in C++ 11 or newer. Something like python 
                                                        // where we do not have to explicitly specify 
                                                        // the datatype of a variable and leave it to 
                                                        // the compiler.
    rclcpp::spin(node);     // Run node in the background, non-blocking.
    rclcpp::shutdown();     // If CTRL-C key is received, execute shutdown() method.
    return 0;
}
