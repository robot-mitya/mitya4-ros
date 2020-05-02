//
// Created by dmitrydzz on 28.04.2020.
//

//#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>
#include "herkulex_node.hpp"
#include "consts.hpp"
#include "robo_com.hpp"
#include "madgwick.hpp"

using namespace robot_mitya;

#define CORRECTION_DURATION 0
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

HerkulexNode::HerkulexNode(rclcpp::Clock &clock)
        : Node(RM_HERKULEX_NODE_NAME, RM_NAMESPACE), clock_(clock) {

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

void HerkulexNode::herkulex_input_topic_callback(std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Received in %s.%s: %s",
            RM_HERKULEX_NODE_NAME, RM_HERKULEX_INPUT_TOPIC_NAME, msg->data.c_str());

    YAML::Node node = YAML::Load(msg->data);

    std::string commandName = node["n"] ? node["n"].as<std::string>() : "null name";

    if (commandName == "mode")
    {
        if (!node["m"])
        {
            RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: mode is not defined",
                    commandName.c_str());
            return;
        }
        int mode = node["m"].as<int>();
        RCLCPP_INFO(get_logger(), "HerkuleX command (%s): mode = %d", commandName.c_str(), mode);
        herkulex_.torqueState(HEAD_HORIZONTAL_SERVO_ID, (TorqueState) mode);
        herkulex_.torqueState(HEAD_VERTICAL_SERVO_ID, (TorqueState) mode);
    }
    else if (commandName == "pointing")
    {
        if (!node["v"])
        {
            RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: value is not defined", commandName.c_str());
            return;
        }
        int value = node["v"].as<int>();
        RCLCPP_INFO(get_logger(), "HerkuleX command (%s): value = %d", commandName.c_str(), value);
        stopHead();
        targetMode_ = value != 0;
        publishDriveCommands(0, 0); // stop motors anyway
    }
    else if (commandName == "f1")
    {
        if (!node["v"])
        {
            RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: value is not defined", commandName.c_str());
            return;
        }
        auto value = node["v"].as<float>();
        RCLCPP_INFO(get_logger(), "HerkuleX command (%s): value = %f", commandName.c_str(), value);
        factor1_ = value;
    }
    else if (commandName == "f2")
    {
        if (!node["v"])
        {
            RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: value is not defined", commandName.c_str());
            return;
        }
        auto value = node["v"].as<float>();
        RCLCPP_INFO(get_logger(), "HerkuleX command (%s): value = %f", commandName.c_str(), value);
        factor2_ = value;
    }
    else
    {
        if (!node["a"])
        {
            RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: servo ID is not defined", commandName.c_str());
            return;
        }
        int address = node["a"].as<int>();
        if (address != HEAD_HORIZONTAL_SERVO_ID && address != HEAD_VERTICAL_SERVO_ID && address != HEAD_BROADCAST_SERVO_ID)
        {
            RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: unknown servo ID (%d)", commandName.c_str(), address);
            return;
        }

        if (commandName == "stat")
        {
            uint8_t statusError;
            uint8_t statusDetail;
            signed char result = herkulex_.stat(address, &statusError, &statusDetail);
            if (result == 0)
            {
                RCLCPP_DEBUG(get_logger(), "Sending HerkuleX response to %s: statusError=%d, statusDetail=%d", RM_HERKULEX_INPUT_TOPIC_NAME, statusError, statusDetail);

                YAML::Emitter out;
                out << YAML::BeginMap;
                out << YAML::Key << "n";
                out << YAML::Value << "stat";
                out << YAML::Key << "a";
                out << YAML::Value << (int) address;
                out << YAML::Key << "e";
                out << YAML::Value << (int) statusError;
                out << YAML::Key << "d";
                out << YAML::Value << (int) statusDetail;
                out << YAML::EndMap;

                std_msgs::msg::String stringMessage;
                stringMessage.data = out.c_str();
                herkulexOutputPublisher_->publish(stringMessage);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: wrong checksum in response (error code %d)", commandName.c_str(), result);
            }
        }
        else if (commandName == "led")
        {
            if (!node["c"])
            {
                RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: LED color is not defined", commandName.c_str());
                return;
            }
            int color = node["c"].as<int>();
            if (color < 0 || color > 7)
            {
                RCLCPP_ERROR(get_logger(), "HerkuleX command (%s) processor error: bad color value (%d)", commandName.c_str(), color);
                return;
            }
            herkulex_.setLed(address, color);
        }
        else if (commandName == "center")
        {
            int delay;
            if (address == HEAD_BROADCAST_SERVO_ID)
            {
                int durationH = headMoveCenter(HEAD_HORIZONTAL_SERVO_ID);
                int durationV = headMoveCenter(HEAD_VERTICAL_SERVO_ID);
                delay = MAX(durationH, durationV);
            }
            else
                delay = headMoveCenter(address);
            centerHeadImu(delay);
        }
        else if (commandName == "reboot")
        {
            headServoReboot(address);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Unknown command name: \'%s\'", commandName.c_str());
            return;
        }
    }
}

void HerkulexNode::head_position_topic_callback(mitya_interfaces::msg::HeadPosition::SharedPtr msg) {
    if (targetMode_ || centerHeadImuStarted_) return;
//    RCLCPP_INFO(get_logger(), "Received in %s.%s: %f, %f",
//            RM_HERKULEX_NODE_NAME, RM_HEAD_POSITION_TOPIC_NAME, msg->horizontal, msg->vertical);
    setHeadPositionHorizontal(msg->horizontal, 0);
    setHeadPositionVertical(msg->vertical, 0);
}

void HerkulexNode::head_move_topic_callback(mitya_interfaces::msg::HeadMove::SharedPtr msg) {
    if (targetMode_ || centerHeadImuStarted_) return;

    if (msg->horizontal != previousHeadMoveValues_.horizontal)
    {
        if (msg->horizontal != 0)
        {
            float currentAngle = herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID);
            float targetAngle = msg->horizontal > 0 ? headHorizontalMaxDegree : headHorizontalMinDegree;
            int duration = calculateDurationInMillis(targetAngle - currentAngle, headMoveSpeed_);
//RCLCPP_DEBUG(get_logger(), "H: currentAngle=%.3f, targetAngle=%.3f, duration=%d", currentAngle, targetAngle, duration);
            herkulex_.moveOneAngle(HEAD_HORIZONTAL_SERVO_ID, targetAngle, duration, 0);
        }
        else
        {
            herkulex_.moveOneAngle(HEAD_HORIZONTAL_SERVO_ID, herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID), CORRECTION_DURATION, 0);
        }
        previousHeadMoveValues_.horizontal = msg->horizontal;
    }

    if (msg->vertical != previousHeadMoveValues_.vertical)
    {
        if (msg->vertical != 0)
        {
            float currentAngle = herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID);
            float targetAngle = msg->vertical > 0 ? headVerticalMaxDegree : headVerticalMinDegree;
            int duration = calculateDurationInMillis(targetAngle - currentAngle, headMoveSpeed_);
//RCLCPP_DEBUG(get_logger(), "V: currentAngle=%.3f, targetAngle=%.3f, duration=%d", currentAngle, targetAngle, duration);
            herkulex_.moveOneAngle(HEAD_VERTICAL_SERVO_ID, targetAngle, duration, 0);
        }
        else
        {
            herkulex_.moveOneAngle(HEAD_VERTICAL_SERVO_ID, herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID), CORRECTION_DURATION, 0);
        }
        previousHeadMoveValues_.vertical = msg->vertical;
    }
}

void HerkulexNode::head_imu_output_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    headQuaternion_.setValue(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void HerkulexNode::controller_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    targetQuaternion_.setValue(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void HerkulexNode::drive_towards_callback(std_msgs::msg::Int8::SharedPtr msg) {
    targetModeVelocity_ = msg->data;
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

void HerkulexNode::updateCenterHeadState() {
    rclcpp::Time now = clock_.now();
    if (centerHeadImuStarted_ && now >= centerHeadImuStartTime_)
    {
        //ROS_DEBUG("Centering Head Imu [nowSeconds = %f]", now.toSec());
        centerHeadImuStarted_ = false;

        std_msgs::msg::String stringMessage;
        stringMessage.data = "center";
        headImuInputPublisher_->publish(stringMessage);
    }
}

void HerkulexNode::updateHeadDirectedToTarget() {
    if (!targetMode_ || centerHeadImuStarted_) return;

    tf2Scalar imuYaw;
    tf2Scalar imuPitch;
    MadgwickImu::getEulerYP(headQuaternion_, imuYaw, imuPitch);
    tf2Scalar targetYaw;
    tf2Scalar targetPitch;
    MadgwickImu::getEulerYP(targetQuaternion_, targetYaw, targetPitch);

    tf2Scalar deltaYaw = targetYaw - imuYaw;
    tf2Scalar deltaPitch = targetPitch - imuPitch;
    float speed = headMoveSpeed_ * 4.0f;
    int yawDuration = calculateDurationInMillis(deltaYaw, speed);
    int pitchDuration = calculateDurationInMillis(deltaPitch, speed);
    int duration = MAX(yawDuration, pitchDuration);
    float aYaw = herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID);
    float aPitch = herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID);
    float yaw = aYaw + static_cast<float>(deltaYaw);
    float pitch = aPitch - static_cast<float>(deltaPitch); // (should be plus, but pitch servo's axis is directed in negative direction)
//  ROS_INFO("iY/iP: %+9.3f    %+9.3f    tY/tP: %+9.3f    %+9.3f    aY/aP: %+9.3f    %+9.3f    Y/P: %+9.3f    %+9.3f",
//           imuYaw, imuPitch, targetYaw, targetPitch, aYaw, aPitch, yaw, pitch);
//  ROS_INFO("iY(aY): %+9.3f (%+9.3f)    iP(aP): %+9.3f (%+9.3f)", imuYaw, aYaw, imuPitch, aPitch);
//  ROS_INFO("iY/iP: %+9.3f    %+9.3f    aY/aP: %+9.3f    %+9.3f    Y/P: %+9.3f    %+9.3f",
//           imuYaw, imuPitch, aYaw, aPitch, yaw, pitch);
    setHeadPositionHorizontal(yaw, duration);
    setHeadPositionVertical(pitch, duration);
}

void HerkulexNode::updateDriveToTarget() {
    if (!targetMode_ || centerHeadImuStarted_) return;

    float yaw = herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID);
    const float yawLimit = 45;
    if (yaw > yawLimit) yaw = yawLimit;
    else if (yaw < -yawLimit) yaw = -yawLimit;

    int velocityLeft = targetModeVelocity_;
    int velocityRight = targetModeVelocity_;
    int deltaVelocity = static_cast<int>(static_cast<float>(targetModeVelocity_) * yaw / yawLimit);
    if (targetModeVelocity_ >= 0)
    {
        if (yaw >= 0) velocityLeft -= deltaVelocity;
        else velocityRight += deltaVelocity;
    }
    else
    {
        if (yaw >= 0) velocityRight -= deltaVelocity;
        else velocityLeft += deltaVelocity;
    }
//  ROS_INFO("driveTowardsCallback => velocity = %d, yaw = %.1f, velocityLeft = %d, velocityRight = %d",
//           velocity, yaw, velocityLeft, velocityRight);

    publishDriveCommands(velocityLeft, velocityRight);
}

void HerkulexNode::stopHead() {
    setHeadPositionHorizontal(herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID), 0);
    setHeadPositionVertical(herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID), 0);
}

void HerkulexNode::setTorqueMode(HerkulexTorqueState mode) {
    switch (mode)
    {
        case HTS_BREAK_ON:
        case HTS_TORQUE_ON:
            herkulex_.torqueState(HEAD_HORIZONTAL_SERVO_ID, (TorqueState) mode);
            herkulex_.torqueState(HEAD_VERTICAL_SERVO_ID, (TorqueState) mode);
            break;
        default:
            herkulex_.torqueState(HEAD_HORIZONTAL_SERVO_ID, TS_TORQUE_FREE);
            herkulex_.torqueState(HEAD_VERTICAL_SERVO_ID, TS_TORQUE_FREE);
            break;
    }
}

void HerkulexNode::logPosition() {
}

int HerkulexNode::calculateDurationInMillis(float deltaAngle, float degreesPerSecond) const {
    if (degreesPerSecond < headMoveMinSpeed_)
        degreesPerSecond = headMoveMinSpeed_;
    return abs(static_cast<int>(deltaAngle * 1000.0f / degreesPerSecond));
}

void HerkulexNode::setHeadPositionHorizontal(float angle, int duration) {
    if (angle < headHorizontalMinDegree)
        angle = headHorizontalMinDegree;
    else if (angle > headHorizontalMaxDegree)
        angle = headHorizontalMaxDegree;
    herkulex_.moveOneAngle(HEAD_HORIZONTAL_SERVO_ID, angle, duration, 0);
}

void HerkulexNode::setHeadPositionVertical(float angle, int duration) {
    if (angle < headVerticalMinDegree)
        angle = headVerticalMinDegree;
    else if (angle > headVerticalMaxDegree)
        angle = headVerticalMaxDegree;
    herkulex_.moveOneAngle(HEAD_VERTICAL_SERVO_ID, angle, duration, 0);
}

int HerkulexNode::headMoveCenter(int servoAddress) {
    float currentAngle = herkulex_.getAngle(servoAddress);
    float targetAngle = servoAddress == HEAD_HORIZONTAL_SERVO_ID ? headHorizontalCenterDegree : headVerticalCenterDegree;
    int duration = calculateDurationInMillis(targetAngle - currentAngle, headMoveSpeed_);
    herkulex_.moveOneAngle(servoAddress, targetAngle, duration, 0);
    return duration;
}

void HerkulexNode::headServoReboot(int servoAddress) {
    herkulex_.reboot(servoAddress);

    usleep(1000000);
    initServos();
}

void HerkulexNode::centerHeadImu(double millis) {
    rclcpp::Time now = clock_.now();
    //ROS_DEBUG("Function centerHeadImu(%f) is called [nowSeconds = %f]", millis, now.toSec());
    centerHeadImuStartTime_ = now + rclcpp::Duration(millis / 1000.0);
    centerHeadImuStarted_ = true;
}

std::shared_ptr<HerkulexNode> pNode = nullptr;

// Calls on shutting down the node.
void sigintHandler(int sig)
{
    if (pNode != nullptr)
    {
        RCLCPP_INFO(pNode->get_logger(), "Shutting down %s", RM_HERKULEX_NODE_NAME);
        pNode->stopHead();
        pNode->setTorqueMode(HTS_TORQUE_FREE);
    }

    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::Clock clock(RCL_ROS_TIME);
    pNode = std::make_shared<HerkulexNode>(clock);
    rclcpp::Rate loop_rate(100); // 100 Hz (?)
    while (rclcpp::ok()) {
        pNode->updateCenterHeadState();
        pNode->updateHeadDirectedToTarget();
        pNode->updateDriveToTarget();
        pNode->logPosition();
        rclcpp::spin_some(pNode);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
