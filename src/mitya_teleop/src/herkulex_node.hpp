//
// Created by dmitrydzz on 28.04.2020.
//

#ifndef MITYA_TELEOP_HERKULEX_NODE_HPP
#define MITYA_TELEOP_HERKULEX_NODE_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "mitya_interfaces/msg/head_position.hpp"
#include "mitya_interfaces/msg/head_move.hpp"

namespace robot_mitya {
    class HerkulexNode : public rclcpp::Node {
    public:
        std::string serialPortName;
        int serialBaudRate;

        HerkulexNode();
//        void updateCenterHeadState();
//        void updateHeadDirectedToTarget();
//        void updateDriveToTarget();
//        void stopHead();
//        void setTorqueMode(HerkulexTorqueState mode);
//        void logPosition();
    private:
//        HerkulexClass herkulex_;
//        void initServos();
//
//        float headHorizontalMinDegree;
//        float headHorizontalCenterDegree;
//        float headHorizontalMaxDegree;
//        float headVerticalMinDegree;
//        float headVerticalCenterDegree;
//        float headVerticalMaxDegree;
//        float headMoveMinSpeed_; // degrees per second
//
//        float headMoveSpeed_; // degrees per second
//
//        bool targetMode_;
//        tf2::Quaternion headQuaternion_;
//        tf2::Quaternion targetQuaternion_;
//
//        int targetModeVelocity_;

        // Topic RM_HERKULEX_INPUT_TOPIC_NAME ('herkulex_input') subscriber:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr herkulexInputSubscription_;
        void herkulex_input_topic_callback(std_msgs::msg::String::SharedPtr msg) const;

        // Topic RM_HERKULEX_OUTPUT_TOPIC_NAME ('herkulex_output') publisher:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr herkulexOutputPublisher_;

        // Topic RM_HEAD_POSITION_TOPIC_NAME ('head_position') subscriber:
        rclcpp::Subscription<mitya_interfaces::msg::HeadPosition>::SharedPtr headPositionSubscription_;
        void head_position_topic_callback(mitya_interfaces::msg::HeadPosition::SharedPtr msg) const;

        // Topic RM_HEAD_MOVE_TOPIC_NAME ('head_move') subscriber:
        rclcpp::Subscription<mitya_interfaces::msg::HeadMove>::SharedPtr headMoveSubscription_;
        void head_move_topic_callback(mitya_interfaces::msg::HeadMove::SharedPtr msg) const;

//        // Topic RM_HEAD_IMU_INPUT_TOPIC_NAME ('head_imu_input') publisher:
//        ros::Publisher headImuInputPublisher_;
//        // Topic RM_HEAD_IMU_OUTPUT_TOPIC_NAME ('herkulex_output') subscriber:
//        ros::Subscriber headImuOutputSubscriber_;
//        void headImuOutputCallback(const sensor_msgs::Imu::ConstPtr& msg);
//
//        // Topic RM_CONTROLLER_IMU_TOPIC_NAME ('controller_imu') subscriber:
//        ros::Subscriber controllerImuSubscriber_;
//        void controllerImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
//
//        // Topic RM_DRIVE_TOWARDS_TOPIC_NAME ('drive_towards') subscriber:
//        ros::Subscriber driveTowardsSubscriber_;
//        void driveTowardsCallback(const std_msgs::Int8ConstPtr& msg);

        // Topic RM_ARDUINO_INPUT_TOPIC_NAME ('arduino_input') publisher:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduinoInputPublisher_;

//        void publishDriveCommands(int leftMotorVelocity, int rightMotorVelocity);
//
//        struct HeadMoveValues
//        {
//            int horizontal;
//            int vertical;
//        };
//        HeadMoveValues previousHeadMoveValues_;
//        int calculateDurationInMillis(float deltaAngle, float degreesPerSecond);
//
//        /*
//         * Returns movement duration in millis.
//         */
//        int headMoveCenter(int servoAddress);
//
//        void setHeadPositionHorizontal(float angle, int duration);
//        void setHeadPositionVertical(float angle, int duration);
//
//        void headServoReboot(int servoAddress);
//
//        void centerHeadImu(double millis);
//        ros::Time centerHeadImuStartTime_;
//        bool centerHeadImuStarted_;

        float factor1_;
        float factor2_;
//        static constexpr tf2Scalar POINTING_DEFAULT = 1000.0f;
    };
}

#endif //MITYA_TELEOP_HERKULEX_NODE_HPP
