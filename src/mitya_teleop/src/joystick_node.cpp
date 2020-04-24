//
// Created by dmitrydzz on 21.04.2020.
//

#include "joystick_node.hpp"
#include "consts.hpp"
#include "robo_com.hpp"
#include <yaml-cpp/yaml.h>

using namespace robot_mitya;

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

JoystickNode::JoystickNode() : Node(RM_JOYSTICK_NODE_NAME, RM_NAMESPACE) {
    headControlMode_ = HEAD_MOVE;

    rebootButtonIndex_ = this->declare_parameter<int>("reboot_button", 8);

    driveAxisX_ = this->declare_parameter<int>("drive_axis_x", 3);
    driveAxisY_ = this->declare_parameter<int>("drive_axis_y", 4);
    driveBoost_ = this->declare_parameter<int>("drive_boost", 2);
    driveBoostMinFactor_ = this->declare_parameter<float>("drive_boost_min_factor", 0.25f);
    driveBoostMaxFactor_ = this->declare_parameter<float>("drive_boost_max_factor", 1.0f);
    driveBoostWasChanged_ = false;
    driveMaxValue = this->declare_parameter<int>("drive_max_value", 100);
    driveInvertX = this->declare_parameter<bool>("drive_invert_x", true);
    driveInvertY = this->declare_parameter<bool>("drive_invert_y", false);
    driveSignX = driveInvertX ? -1.0f : 1.0f;
    driveSignY = driveInvertY ? -1.0f : 1.0f;

    headModeButtonIndex_ = this->declare_parameter<int>("head_mode_button", 9);
    headAxisX_ = this->declare_parameter<int>("head_axis_x", 0);
    headAxisY_ = this->declare_parameter<int>("head_axis_y", 1);
    headMoveHorizontalAxis_ = this->declare_parameter<int>("head_move_horizontal_axis", 6);
    headMoveVerticalAxis_ = this->declare_parameter<int>("head_move_vertical_axis", 7);
    headInvertHorizontal = this->declare_parameter<bool>("head_invert_horizontal", true);
    headInvertVertical = this->declare_parameter<bool>("head_invert_vertical", true);
    headMoveCenterButtonIndex_ = this->declare_parameter<int>("head_move_center_button", 4); // XBOX LB

    greenButtonIndex_ = this->declare_parameter<int>("green_button", 0); // Smile or swing tail(+ALT) by default
    redButtonIndex_ = this->declare_parameter<int>("red_button", 1); // Angry face
    blueButtonIndex_ = this->declare_parameter<int>("blue_button", 2); // Blue face or LED1(+ALT) by default
    yellowButtonIndex_ = this->declare_parameter<int>("yellow_button", 3); // Ill or LED2(+ALT) by default
    altButtonIndex_ = this->declare_parameter<int>("alt_button", 5); // XBOX RB by default

    headHorizontalMinDegree = this->declare_parameter<float>("head_horizontal_min_degree", -120.0f);
    headHorizontalCenterDegree = this->declare_parameter<float>("head_horizontal_center_degree", 0.0f);
    headHorizontalMaxDegree = this->declare_parameter<float>("head_horizontal_max_degree", 120.0f);
    headVerticalMinDegree = this->declare_parameter<float>("head_vertical_min_degree", -120.0f);
    headVerticalCenterDegree = this->declare_parameter<float>("head_vertical_center_degree", -15.0f);
    headVerticalMaxDegree = this->declare_parameter<float>("head_vertical_max_degree", 10.0f);

    headHorizontalAmplitude = MAX(
            abs(headHorizontalMinDegree - headHorizontalCenterDegree),
            abs(headHorizontalMaxDegree - headHorizontalCenterDegree));
    headVerticalAmplitude = MAX(
            abs(headVerticalMinDegree - headVerticalCenterDegree),
            abs(headVerticalMaxDegree - headVerticalCenterDegree));

    if (headInvertHorizontal)
        headHorizontalAmplitude *= -1.0f;
    if (headInvertVertical)
        headVerticalAmplitude *= -1.0f;

    rebootButton_ = new ButtonEvent(this, &JoystickNode::rebootButtonHandler);

    greenButton_ = new ButtonEvent(this, &JoystickNode::greenButtonHandler);
    redButton_ = new ButtonEvent(this, &JoystickNode::redButtonHandler);
    blueButton_ = new ButtonEvent(this, &JoystickNode::blueButtonHandler);
    yellowButton_ = new ButtonEvent(this, &JoystickNode::yellowButtonHandler);
    altButton_ = new ButtonEvent(this, &JoystickNode::altButtonHandler);

    altButtonState_ = false;
    faceType_ = OK;

    headModeButton_ = new ButtonEvent(this, &JoystickNode::headModeButtonHandler);
    headMoveLeftButton_ = new ButtonEvent(this, &JoystickNode::headMoveLeftButtonHandler);
    headMoveRightButton_ = new ButtonEvent(this, &JoystickNode::headMoveRightButtonHandler);
    headMoveUpButton_ = new ButtonEvent(this, &JoystickNode::headMoveUpButtonHandler);
    headMoveDownButton_ = new ButtonEvent(this, &JoystickNode::headMoveDownButtonHandler);
    headMoveCenterButton_ = new ButtonEvent(this, &JoystickNode::headMoveCenterButtonHandler);

    headMoveMessage_.horizontal = 0;
    headMoveMessage_.vertical = 0;

    joystickSubscription_ = this->create_subscription<sensor_msgs::msg::Joy>(RM_JOY_TOPIC_NAME, 10,
            std::bind(&JoystickNode::joy_topic_callback, this, std::placeholders::_1));

    drivePublisher_ = this->create_publisher<mitya_interfaces::msg::Drive>(RM_DRIVE_TOPIC_NAME, 100);
    headPositionPublisher_ = this->create_publisher<mitya_interfaces::msg::HeadPosition>(RM_HEAD_POSITION_TOPIC_NAME, 100);
    headMovePublisher_ = this->create_publisher<mitya_interfaces::msg::HeadMove>(RM_HEAD_MOVE_TOPIC_NAME, 100);
    arduinoInputPublisher_ = this->create_publisher<std_msgs::msg::String>(RM_ARDUINO_INPUT_TOPIC_NAME, 100);
    herkulexInputPublisher_ = this->create_publisher<std_msgs::msg::String>(RM_HERKULEX_INPUT_TOPIC_NAME, 100);
    facePublisher_ = this->create_publisher<std_msgs::msg::String>(RM_FACE_TOPIC_NAME, 100);
}

void JoystickNode::rebootButtonHandler(bool state)
{
    if (!state) return;
    publishRebootHerkulex();
    publishRebootArduino();
}

void JoystickNode::greenButtonHandler(bool state)
{
    if (!state) return;
    if (altButtonState_)
        publishSwingTailMessage();
    else
    {
        if (faceType_ != HAPPY)
        {
            publishFaceMessage(RoboCom::getFaceHappyCommand());
            faceType_ = HAPPY;
        }
        else
        {
            publishFaceMessage(RoboCom::getFaceOkCommand());
            faceType_ = OK;
        }
    }
}

void JoystickNode::redButtonHandler(bool state)
{
    if (!state) return;
    if (!altButtonState_)
    {
        if (faceType_ != ANGRY)
        {
            publishFaceMessage(RoboCom::getFaceAngryCommand());
            faceType_ = ANGRY;
        }
        else
        {
            publishFaceMessage(RoboCom::getFaceOkCommand());
            faceType_ = OK;
        }
    }
}

void JoystickNode::blueButtonHandler(bool state)
{
    if (!state) return;
    if (altButtonState_)
        publishSwitchLed1Message();
    else
    {
        if (faceType_ != BLUE)
        {
            publishFaceMessage(RoboCom::getFaceBlueCommand());
            faceType_ = BLUE;
        }
        else
        {
            publishFaceMessage(RoboCom::getFaceOkCommand());
            faceType_ = OK;
        }
    }
}

void JoystickNode::yellowButtonHandler(bool state)
{
    if (!state) return;
    if (altButtonState_)
        publishSwitchLed2Message();
    else
    {
        if (faceType_ != ILL)
        {
            publishFaceMessage(RoboCom::getFaceIllCommand());
            faceType_ = ILL;
        }
        else
        {
            publishFaceMessage(RoboCom::getFaceOkCommand());
            faceType_ = OK;
        }
    }
}

void JoystickNode::altButtonHandler(bool state)
{
    altButtonState_ = state;
}

void JoystickNode::headModeButtonHandler(bool state)
{
    if (!state) return;
    if (headControlMode_ == HEAD_MOVE)
        headControlMode_ = HEAD_POSITION;
    else if (headControlMode_ == HEAD_POSITION)
        headControlMode_ = HEAD_MOVE;
}

void JoystickNode::headMoveLeftButtonHandler(bool state)
{
    headMoveMessage_.horizontal = state ? 1 : 0;
    if (headInvertHorizontal)
        headMoveMessage_.horizontal = -headMoveMessage_.horizontal;
    headMovePublisher_->publish(headMoveMessage_);
}

void JoystickNode::headMoveRightButtonHandler(bool state)
{
    headMoveMessage_.horizontal = state ? -1 : 0;
    if (headInvertHorizontal)
        headMoveMessage_.horizontal = -headMoveMessage_.horizontal;
    headMovePublisher_->publish(headMoveMessage_);
}

void JoystickNode::headMoveUpButtonHandler(bool state)
{
    headMoveMessage_.vertical = state ? 1 : 0;
    if (headInvertVertical)
        headMoveMessage_.vertical = -headMoveMessage_.vertical;
    headMovePublisher_->publish(headMoveMessage_);
}

void JoystickNode::headMoveDownButtonHandler(bool state)
{
    headMoveMessage_.vertical = state ? -1 : 0;
    if (headInvertVertical)
        headMoveMessage_.vertical = -headMoveMessage_.vertical;
    headMovePublisher_->publish(headMoveMessage_);
}

void JoystickNode::headMoveCenterButtonHandler(bool state)
{
    if (!state) return;
    publishCenterHerkulex(HEAD_BROADCAST_SERVO_ID);
}

void JoystickNode::joy_topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    float driveBoost =  joy->axes[driveBoost_];
    // driveBoost values should be in interval [+1..-1].
    // +1 - the trigger is released, -1 - the trigger is fully pressed.
    // By some reason the initial value of the trigger is wrong - it is set to 0.
    // To fix the bug in joy node I have to use a flag driveBoostWasChanged_:
    if (!driveBoostWasChanged_) {
        if (driveBoost < -0.01 || driveBoost > 0.01)
            driveBoostWasChanged_ = true;
        else
            driveBoost = 1.0;
    }

    publishDriveMessage(joy->axes[driveAxisX_], joy->axes[driveAxisY_], driveBoost);

    if (headControlMode_ == HEAD_POSITION) {
        publishHeadPositionMessage(joy->axes[headAxisX_], joy->axes[headAxisY_]);
    }

    rebootButton_->update(joy->buttons[rebootButtonIndex_] == 1);

    greenButton_->update(joy->buttons[greenButtonIndex_] == 1);
    redButton_->update(joy->buttons[redButtonIndex_] == 1);
    blueButton_->update(joy->buttons[blueButtonIndex_] == 1);
    yellowButton_->update(joy->buttons[yellowButtonIndex_] == 1);
    altButton_->update(joy->buttons[altButtonIndex_] == 1);

    headModeButton_->update(joy->buttons[headModeButtonIndex_] == 1);
    if (headControlMode_ == HEAD_MOVE) {
        headMoveLeftButton_->update(joy->axes[headMoveHorizontalAxis_] > 0);
        headMoveRightButton_->update(joy->axes[headMoveHorizontalAxis_] < 0);
        headMoveUpButton_->update(joy->axes[headMoveVerticalAxis_] > 0);
        headMoveDownButton_->update(joy->axes[headMoveVerticalAxis_] < 0);
        headMoveCenterButton_->update(joy->buttons[headMoveCenterButtonIndex_] == 1);
    }
}

int8_t JoystickNode::getSpeedValue(float joystickValue) const
{
    auto result = (int8_t) roundf(joystickValue * (float) driveMaxValue);
    if (result < -driveMaxValue) return -driveMaxValue;
    else if (result > driveMaxValue) return driveMaxValue;
    return result;
}

void JoystickNode::publishDriveMessage(float x, float y, float boost) {
    x *= driveSignX;
    y *= driveSignY;

    float alpha = atan2f(y, x) * RAD_TO_DEG;
    if (alpha < 0) alpha += 360;
    if (alpha < 0) alpha += 360;
    else if (alpha >= 360) alpha -= 360;

    float radius = sqrtf(x * x + y * y);
    if (radius < 0) radius = 0.0f;
    else if (radius > 1) radius = 1.0f;

    float left = 0;
    float right = 0;
    if (alpha >= 0 && alpha < 90)
    {
        left = 1.0f;
        right = 2.0f / 90.0f * alpha - 1.0f;
    }
    else if (alpha >= 90 && alpha < 180)
    {
        left = -2.0f / 90.0f * alpha + 3.0f;
        right = 1.0f;
    }
    else if (alpha >= 180 && alpha < 270)
    {
        left = -1.0f;
        right = - 2.0f / 90.0f * alpha + 5.0f;
    }
    else
    {
        left = 2.0f / 90.0f * alpha - 7.0f;
        right = -1.0f;
    }
    left *= radius;
    right *= radius;

    // Calculating boost_factor according to the <boost> argument:
    // <boost> value is in interval [+1..-1].
    // When <boost> is +1 - the trigger is released and boost_factor should be equal to driveBoostMinFactor_.
    // When <boost> is -1 - the trigger is fully pressed and boost_factor should be equal to driveBoostMaxFactor_.
    float boost_factor = driveBoostMinFactor_ + (driveBoostMaxFactor_ - driveBoostMinFactor_) * (boost - 1.0f) / (-2.0f);
    left *= boost_factor;
    right *= boost_factor;

    //ROS_INFO("x=%+5.3f y=%+5.3f R=%+5.3f A=%+8.3f    Left=%+6.3f Right=%+6.3f", x, y, radius, alpha, left, right);
    //ROS_INFO("Left=%+6.3f  Right=%+6.3f  Boost=%+6.3f  BoostFactor=%+6.3f", left, right, boost, boost_factor);

    driveMessage_.left = getSpeedValue(left);
    driveMessage_.right = getSpeedValue(right);
    drivePublisher_->publish(driveMessage_);
}

void JoystickNode::publishHeadPositionMessage(float x, float y) {
    x *= headHorizontalAmplitude;
    x += headHorizontalCenterDegree;
    if (x < headHorizontalMinDegree) x = headHorizontalMinDegree;
    else if (x > headHorizontalMaxDegree) x = headHorizontalMaxDegree;
    headPositionMessage_.horizontal = x;

    y *= headVerticalAmplitude;
    y += headVerticalCenterDegree;
    if (y < headVerticalMinDegree) y = headVerticalMinDegree;
    else if (y > headVerticalMaxDegree) y = headVerticalMaxDegree;
    headPositionMessage_.vertical = y;

    headPositionPublisher_->publish(headPositionMessage_);
}

void JoystickNode::publishSwitchLed1Message()
{
    std_msgs::msg::String msg;
    msg.data = RoboCom::getSwitchLed1Command();
    arduinoInputPublisher_->publish(msg);
}

void JoystickNode::publishSwitchLed2Message()
{
    std_msgs::msg::String msg;
    msg.data = RoboCom::getSwitchLed2Command();
    arduinoInputPublisher_->publish(msg);
}

void JoystickNode::publishSwingTailMessage()
{
    std_msgs::msg::String msg;
    msg.data = RoboCom::getSwingTailCommand();
    arduinoInputPublisher_->publish(msg);
}

void JoystickNode::publishFaceMessage(const char* command)
{
    std_msgs::msg::String msg;
    msg.data = command;
    facePublisher_->publish(msg);
}

void JoystickNode::publishCenterHerkulex(uint8_t address)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "n";
    out << YAML::Value << "center";
    out << YAML::Key << "a";
    out << YAML::Value << (int) address;
    out << YAML::EndMap;

    std_msgs::msg::String stringMessage;
    stringMessage.data = out.c_str();
    herkulexInputPublisher_->publish(stringMessage);
}

void JoystickNode::publishModeHerkulex(HerkulexTorqueState mode)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "n";
    out << YAML::Value << "mode";
    out << YAML::Key << "m";
    out << YAML::Value << (int) mode;
    out << YAML::EndMap;

    std_msgs::msg::String stringMessage;
    stringMessage.data = out.c_str();
    herkulexInputPublisher_->publish(stringMessage);
}

void JoystickNode::publishRebootHerkulex()
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "n";
    out << YAML::Value << "reboot";
    out << YAML::Key << "a";
    out << YAML::Value << (int) HEAD_BROADCAST_SERVO_ID;
    out << YAML::EndMap;

    std_msgs::msg::String stringMessage;
    stringMessage.data = out.c_str();
    herkulexInputPublisher_->publish(stringMessage);
}

void JoystickNode::publishRebootArduino()
{
    std_msgs::msg::String msg;
    msg.data = RoboCom::getRebootCommand();
    arduinoInputPublisher_->publish(msg);
}

void JoystickNode::sendStopHead()
{
    publishModeHerkulex(HTS_TORQUE_FREE);
}

std::shared_ptr<JoystickNode> pNode = nullptr;

// Calls on shutting down the node.
void sigintHandler(int sig) {
    if (pNode != nullptr) {
        RCLCPP_INFO(pNode->get_logger(), "Shutting down %s (signal=%d)", RM_JOYSTICK_NODE_NAME, sig);
        pNode->sendStopHead();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    pNode = std::make_shared<JoystickNode>();
    signal(SIGINT, sigintHandler);
    rclcpp::spin(pNode);
//    rclcpp::spin(std::make_shared<JoystickNode>());
//    rclcpp::shutdown();
    return 0;
}
