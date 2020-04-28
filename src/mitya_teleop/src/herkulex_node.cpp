//
// Created by dmitrydzz on 28.04.2020.
//

#include "herkulex_node.hpp"
#include "consts.hpp"

using namespace robot_mitya;

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

HerkulexNode::HerkulexNode() : Node(RM_HERKULEX_NODE_NAME, RM_NAMESPACE) {
    herkulexInputSubscription_ = this->create_subscription<std_msgs::msg::String>(
            RM_HERKULEX_INPUT_TOPIC_NAME, 100, std::bind(&HerkulexNode::herkulex_input_topic_callback, this, std::placeholders::_1));
    herkulexOutputPublisher_ = this->create_publisher<std_msgs::msg::String>(RM_HERKULEX_OUTPUT_TOPIC_NAME, 100);
    headPositionSubscription_ = this->create_subscription<mitya_interfaces::msg::HeadPosition>(
            RM_HEAD_POSITION_TOPIC_NAME, 100, std::bind(&HerkulexNode::head_position_topic_callback, this, std::placeholders::_1));
    headMoveSubscription_ = this->create_subscription<mitya_interfaces::msg::HeadMove>(
            RM_HEAD_MOVE_TOPIC_NAME, 100, std::bind(&HerkulexNode::head_move_topic_callback, this, std::placeholders::_1));

//    headImuInputPublisher_ = nodeHandle.advertise<std_msgs::String>(RM_HEAD_IMU_INPUT_TOPIC_NAME, 10);
//    headImuOutputSubscriber_ = nodeHandle.subscribe(RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100, &HerkulexNode::headImuOutputCallback, this);
//    controllerImuSubscriber_ = nodeHandle.subscribe(RM_CONTROLLER_IMU_TOPIC_NAME, 100, &HerkulexNode::controllerImuCallback, this);
//    driveTowardsSubscriber_ = nodeHandle.subscribe(RM_DRIVE_TOWARDS_TOPIC_NAME, 100, &HerkulexNode::driveTowardsCallback, this);

    herkulexOutputPublisher_ = this->create_publisher<std_msgs::msg::String>(RM_ARDUINO_INPUT_TOPIC_NAME, 100);

//    ros::NodeHandle privateNodeHandle("~");
//    privateNodeHandle.param("serial_port", serialPortName, (std::string) "/dev/ttyUSB0");
//    privateNodeHandle.param("baud_rate", serialBaudRate, 115200);
//
//    initServos();
//
//    ros::NodeHandle commonNodeHandle("");
//    commonNodeHandle.param("head_horizontal_min_degree", headHorizontalMinDegree, -120.0f);
//    commonNodeHandle.param("head_horizontal_center_degree", headHorizontalCenterDegree, 0.0f);
//    commonNodeHandle.param("head_horizontal_max_degree", headHorizontalMaxDegree, 120.0f);
//    commonNodeHandle.param("head_vertical_min_degree", headVerticalMinDegree, -120.0f);
//    commonNodeHandle.param("head_vertical_center_degree", headVerticalCenterDegree, -15.0f);
//    commonNodeHandle.param("head_vertical_max_degree", headVerticalMaxDegree, 10.0f);
//
//    // 2856 mSec is the longest duration for single movement of the Herkulex servo.
//    // To calculate to lowest speed we divide the longest path in degrees by this time.
//    // See herkulex.cpp (search "2856") for more information.
//    headMoveMinSpeed_ = floor(
//            MAX(abs(headHorizontalMaxDegree - headHorizontalMinDegree), abs(headVerticalMaxDegree - headVerticalMinDegree)) *
//            1000.0f / 2856.0f + 1);
//    headMoveSpeed_ = headMoveMinSpeed_;
//
//    previousHeadMoveValues_.horizontal = 0;
//    previousHeadMoveValues_.vertical = 0;
//
//    centerHeadImuStarted_ = false;
//
//    targetQuaternion_ = tf2::Quaternion::getIdentity();
//    targetMode_ = false;
//    targetModeVelocity_ = 0;

    factor1_ = 0.0f;
    factor2_ = 0.0f;
}

void HerkulexNode::herkulex_input_topic_callback(std_msgs::msg::String::SharedPtr msg) const {
}

void HerkulexNode::head_position_topic_callback(mitya_interfaces::msg::HeadPosition::SharedPtr msg) const {
}

void HerkulexNode::head_move_topic_callback(mitya_interfaces::msg::HeadMove::SharedPtr msg) const {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HerkulexNode>());
    rclcpp::shutdown();
    return 0;
}
