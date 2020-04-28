//
// Created by dmitrydzz on 28.04.2020.
//

#include "herkulex_node.hpp"
#include "consts.hpp"

using namespace robot_mitya;

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

HerkulexNode::HerkulexNode() : Node(RM_HERKULEX_NODE_NAME, RM_NAMESPACE) {

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HerkulexNode>());
    rclcpp::shutdown();
    return 0;
}
