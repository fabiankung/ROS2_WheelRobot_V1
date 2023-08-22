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
    auto node = std::make_shared<RobotCoreNode>();      // Create a shared pointer to a node class. 
                                                        // Shared pointer is a newer feature of C++,
                                                        // easier for garbage collection when we
                                                        // exit the application or go out of scope.
                                                        // Note also the 'auto' keyword. This will
                                                        // automatically select the correct datatype.
    rclcpp::spin(node);     // Run node in the background, non-blocking.
    rclcpp::shutdown();     // If CTRL-C key is received, execute shutdown() method.
    return 0;
}
