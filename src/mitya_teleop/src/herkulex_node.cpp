//
// Created by dmitrydzz on 28.04.2020.
//

#include "herkulex_node.hpp"
#include "consts.hpp"
#include "robo_com.hpp"

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

    headImuInputPublisher_ = this->create_publisher<std_msgs::msg::String>(RM_HEAD_IMU_INPUT_TOPIC_NAME, 10);
    headImuOutputSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100, std::bind(&HerkulexNode::head_imu_output_callback, this, std::placeholders::_1));
    controllerImuSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            RM_CONTROLLER_IMU_TOPIC_NAME, 100, std::bind(&HerkulexNode::controller_imu_callback, this, std::placeholders::_1));
    driveTowardsSubscription_ = this->create_subscription<std_msgs::msg::Int8>(
            RM_DRIVE_TOWARDS_TOPIC_NAME, 100, std::bind(&HerkulexNode::drive_towards_callback, this, std::placeholders::_1));

    herkulexOutputPublisher_ = this->create_publisher<std_msgs::msg::String>(RM_ARDUINO_INPUT_TOPIC_NAME, 100);

    serialPortName = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    serialBaudRate = this->declare_parameter<int>("baud_rate", 115200);

    initServos();

    headHorizontalMinDegree = this->declare_parameter<float>("head_horizontal_min_degree", -120.0f);
    headHorizontalCenterDegree = this->declare_parameter<float>("head_horizontal_center_degree", 0.0f);
    headHorizontalMaxDegree = this->declare_parameter<float>("head_horizontal_max_degree", 120.0f);
    headVerticalMinDegree = this->declare_parameter<float>("head_vertical_min_degree", -120.0f);
    headVerticalCenterDegree = this->declare_parameter<float>("head_vertical_center_degree", -15.0f);
    headVerticalMaxDegree = this->declare_parameter<float>("head_vertical_max_degree", 10.0f);

    // 2856 mSec is the longest duration for single movement of the Herkulex servo.
    // To calculate to lowest speed we divide the longest path in degrees by this time.
    // See herkulex.cpp (search "2856") for more information.
    headMoveMinSpeed_ = floor(
            MAX(abs(headHorizontalMaxDegree - headHorizontalMinDegree), abs(headVerticalMaxDegree - headVerticalMinDegree)) *
            1000.0f / 2856.0f + 1);
    headMoveSpeed_ = headMoveMinSpeed_;

    previousHeadMoveValues_.horizontal = 0;
    previousHeadMoveValues_.vertical = 0;

    centerHeadImuStarted_ = false;

    targetQuaternion_ = tf2::Quaternion::getIdentity();
    targetMode_ = false;
    targetModeVelocity_ = 0;

    factor1_ = 0.0f;
    factor2_ = 0.0f;
}

void HerkulexNode::herkulex_input_topic_callback(std_msgs::msg::String::SharedPtr msg) const {
    //TODO
}

void HerkulexNode::head_position_topic_callback(mitya_interfaces::msg::HeadPosition::SharedPtr msg) const {
    //TODO
}

void HerkulexNode::head_move_topic_callback(mitya_interfaces::msg::HeadMove::SharedPtr msg) const {
    //TODO
}

void HerkulexNode::head_imu_output_callback(sensor_msgs::msg::Imu::SharedPtr msg) const {
    //TODO
}

void HerkulexNode::controller_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) const {
    //TODO
}

void HerkulexNode::drive_towards_callback(std_msgs::msg::Int8::SharedPtr msg) const {
    //TODO
}

void HerkulexNode::publishDriveCommands(int leftMotorVelocity, int rightMotorVelocity) {
    std_msgs::msg::String stringMessage;
    stringMessage.data = RoboCom::getDriveLeftCommand((signed char) leftMotorVelocity);
    arduinoInputPublisher_->publish(stringMessage);
    stringMessage.data = RoboCom::getDriveRightCommand((signed char) rightMotorVelocity);
    arduinoInputPublisher_->publish(stringMessage);
}

void HerkulexNode::initServos() {
    bool portOpened = herkulex_.begin(serialPortName.c_str(), serialBaudRate);
    if (portOpened)
        RCLCPP_INFO(this->get_logger(), "Serial port \'%s\' is opened", serialPortName.c_str());
    else
        RCLCPP_ERROR(this->get_logger(), "Error %d opening %s: %s", errno, serialPortName.c_str(), strerror(errno));
    herkulex_.initialize();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HerkulexNode>());
    rclcpp::shutdown();
    return 0;
}
