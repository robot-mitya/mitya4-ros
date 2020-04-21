//
// Created by dmitrydzz on 21.04.2020.
//

#include "joystick_node.hpp"
#include "consts.hpp"

using namespace robot_mitya;

JoystickNode::JoystickNode() : Node(RM_JOYSTICK_NODE_NAME, RM_NAMESPACE) {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickNode>());
    rclcpp::shutdown();
    return 0;
}
