#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "consts.h"

using std::placeholders::_1;

class ArduinoNode : public rclcpp::Node {
public:
    ArduinoNode() : Node(RM_ARDUINO_NODE_NAME) {
        arduinoInputSubscription_ = this->create_subscription<std_msgs::msg::String>(
                RM_ARDUINO_INPUT_TOPIC_NAME, 10, std::bind(&ArduinoNode::arduino_input_topic_callback, this, _1));
    }
private:
    void arduino_input_topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arduinoInputSubscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoNode>());
    rclcpp::shutdown();
    return 0;
}
