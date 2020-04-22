//
// Created by dmitrydzz on 21.04.2020.
//

#include "joystick_node.hpp"
#include "consts.hpp"

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

    RCLCPP_INFO(this->get_logger(), "greenButtonIndex_: %d", greenButtonIndex_);
    RCLCPP_INFO(this->get_logger(), "redButtonIndex_: %d", redButtonIndex_);
    RCLCPP_INFO(this->get_logger(), "blueButtonIndex_: %d", blueButtonIndex_);
    RCLCPP_INFO(this->get_logger(), "yellowButtonIndex_: %d", yellowButtonIndex_);

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
}

void JoystickNode::rebootButtonHandler(bool state)
{
//    if (!state) return;
//    publishRebootHerkulex();
//    publishRebootArduino();
}

void JoystickNode::greenButtonHandler(bool state)
{
    RCLCPP_INFO(this->get_logger(), "greenButtonHandler: %s", (state ? "ON" : "OFF"));
//    if (!state) return;
//    if (altButtonState_)
//        publishSwingTailMessage();
//    else
//    {
//        if (faceType_ != HAPPY)
//        {
//            publishFaceMessage(RoboCom::getFaceHappyCommand());
//            faceType_ = HAPPY;
//        }
//        else
//        {
//            publishFaceMessage(RoboCom::getFaceOkCommand());
//            faceType_ = OK;
//        }
//    }
}

void JoystickNode::redButtonHandler(bool state)
{
    RCLCPP_INFO(this->get_logger(), "redButtonHandler: %s", (state ? "ON" : "OFF"));
//    if (!state) return;
//    if (!altButtonState_)
//    {
//        if (faceType_ != ANGRY)
//        {
//            publishFaceMessage(RoboCom::getFaceAngryCommand());
//            faceType_ = ANGRY;
//        }
//        else
//        {
//            publishFaceMessage(RoboCom::getFaceOkCommand());
//            faceType_ = OK;
//        }
//    }
}

void JoystickNode::blueButtonHandler(bool state)
{
    RCLCPP_INFO(this->get_logger(), "blueButtonHandler: %s", (state ? "ON" : "OFF"));
//    if (!state) return;
//    if (altButtonState_)
//        publishSwitchLed1Message();
//    else
//    {
//        if (faceType_ != BLUE)
//        {
//            publishFaceMessage(RoboCom::getFaceBlueCommand());
//            faceType_ = BLUE;
//        }
//        else
//        {
//            publishFaceMessage(RoboCom::getFaceOkCommand());
//            faceType_ = OK;
//        }
//    }
}

void JoystickNode::yellowButtonHandler(bool state)
{
    RCLCPP_INFO(this->get_logger(), "yellowButtonHandler: %s", (state ? "ON" : "OFF"));
//    if (!state) return;
//    if (altButtonState_)
//        publishSwitchLed2Message();
//    else
//    {
//        if (faceType_ != ILL)
//        {
//            publishFaceMessage(RoboCom::getFaceIllCommand());
//            faceType_ = ILL;
//        }
//        else
//        {
//            publishFaceMessage(RoboCom::getFaceOkCommand());
//            faceType_ = OK;
//        }
//    }
}

void JoystickNode::altButtonHandler(bool state)
{
//    altButtonState_ = state;
}

void JoystickNode::headModeButtonHandler(bool state)
{
//    if (!state) return;
//    if (headControlMode_ == HEAD_MOVE)
//        headControlMode_ = HEAD_POSITION;
//    else if (headControlMode_ == HEAD_POSITION)
//        headControlMode_ = HEAD_MOVE;
}

void JoystickNode::headMoveLeftButtonHandler(bool state)
{
//    headMoveMessage_.horizontal = state ? 1 : 0;
//    if (headInvertHorizontal)
//        headMoveMessage_.horizontal = -headMoveMessage_.horizontal;
//    headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveRightButtonHandler(bool state)
{
//    headMoveMessage_.horizontal = state ? -1 : 0;
//    if (headInvertHorizontal)
//        headMoveMessage_.horizontal = -headMoveMessage_.horizontal;
//    headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveUpButtonHandler(bool state)
{
//    headMoveMessage_.vertical = state ? 1 : 0;
//    if (headInvertVertical)
//        headMoveMessage_.vertical = -headMoveMessage_.vertical;
//    headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveDownButtonHandler(bool state)
{
//    headMoveMessage_.vertical = state ? -1 : 0;
//    if (headInvertVertical)
//        headMoveMessage_.vertical = -headMoveMessage_.vertical;
//    headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveCenterButtonHandler(bool state)
{
//    if (!state) return;
//    publishCenterHerkulex(HEAD_BROADCAST_SERVO_ID);
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

void JoystickNode::publishDriveMessage(float x, float y, float boost) {
}

void JoystickNode::publishHeadPositionMessage(float x, float y) {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickNode>());
    rclcpp::shutdown();
    return 0;
}
