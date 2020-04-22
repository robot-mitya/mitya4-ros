//
// Created by dmitrydzz on 21.04.2020.
//

#ifndef MITYA_TELEOP_JOYSTICK_NODE_H
#define MITYA_TELEOP_JOYSTICK_NODE_H

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sensor_msgs/msg/joy.hpp>
#include "mitya_interfaces/msg/drive.hpp"
#include "mitya_interfaces/msg/head_move.hpp"
#include "mitya_interfaces/msg/head_position.hpp"
#include "button_event.hpp"

namespace robot_mitya {

    enum HeadControlMode { HEAD_MOVE, HEAD_POSITION };
    enum FaceType { OK, ANGRY, BLUE, HAPPY, ILL };

    class JoystickNode : public rclcpp::Node {
    public:
        JoystickNode();
        void sendStopHead();
    private:
        static constexpr float RAD_TO_DEG = 180.0f / M_PI;

        HeadControlMode headControlMode_;

        int rebootButtonIndex_;

        int driveAxisX_;
        int driveAxisY_;
        int driveBoost_;
        bool driveBoostWasChanged_; // (To fix a bug in joy node - bad initial values in triggers' axes)
        float driveBoostMinFactor_;
        float driveBoostMaxFactor_;

        int headModeButtonIndex_;
        int headAxisX_;
        int headAxisY_;
        int headMoveHorizontalAxis_;
        int headMoveVerticalAxis_;
        int headMoveCenterButtonIndex_;

        int greenButtonIndex_;
        int redButtonIndex_;
        int blueButtonIndex_;
        int yellowButtonIndex_;
        int altButtonIndex_;

        bool altButtonState_;
        FaceType faceType_;

        int driveMaxValue;
        bool driveInvertX;
        bool driveInvertY;
        float driveSignX;
        float driveSignY;

        float headHorizontalMinDegree;
        float headHorizontalCenterDegree;
        float headHorizontalMaxDegree;
        float headVerticalMinDegree;
        float headVerticalCenterDegree;
        float headVerticalMaxDegree;
        bool headInvertHorizontal;
        bool headInvertVertical;
        float headHorizontalAmplitude;
        float headVerticalAmplitude;

        ButtonEvent *rebootButton_;
        void rebootButtonHandler(bool state);

        ButtonEvent *greenButton_;
        void greenButtonHandler(bool state);
        ButtonEvent *redButton_;
        void redButtonHandler(bool state);
        ButtonEvent *blueButton_;
        void blueButtonHandler(bool state);
        ButtonEvent *yellowButton_;
        void yellowButtonHandler(bool state);
        ButtonEvent *altButton_;
        void altButtonHandler(bool state);

        ButtonEvent *headModeButton_;
        void headModeButtonHandler(bool state);

        ButtonEvent *headMoveLeftButton_;
        void headMoveLeftButtonHandler(bool state);
        ButtonEvent *headMoveRightButton_;
        void headMoveRightButtonHandler(bool state);
        ButtonEvent *headMoveUpButton_;
        void headMoveUpButtonHandler(bool state);
        ButtonEvent *headMoveDownButton_;
        void headMoveDownButtonHandler(bool state);
        ButtonEvent *headMoveCenterButton_;
        void headMoveCenterButtonHandler(bool state);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystickSubscription_;
//        ros::Publisher drivePublisher_;
//        ros::Publisher headPositionPublisher_;
//        ros::Publisher headMovePublisher_;
//        ros::Publisher arduinoInputPublisher_;
//        ros::Publisher herkulexInputPublisher_;
//        ros::Publisher facePublisher_;
        void joy_topic_callback(sensor_msgs::msg::Joy::SharedPtr msg);
        int8_t getSpeedValue(float joystickValue);
        void publishDriveMessage(float x, float y, float boost);
        void publishHeadPositionMessage(float x, float y);
        void publishSwitchLed1Message();
        void publishSwitchLed2Message();
        void publishSwingTailMessage();
        void publishFaceMessage(const char* command);
        void publishCenterHerkulex(uint8_t address);
//        void publishModeHerkulex(HerkulexTorqueState mode);
        void publishRebootHerkulex();
        void publishRebootArduino();

        mitya_interfaces::msg::Drive driveMessage_;
        mitya_interfaces::msg::HeadMove headMoveMessage_;
        mitya_interfaces::msg::HeadPosition headPositionMessage_;
    };
}

#endif //MITYA_TELEOP_JOYSTICK_NODE_H
