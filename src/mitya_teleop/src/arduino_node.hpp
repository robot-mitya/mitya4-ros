//
// Created by dmitrydzz on 19.04.2020.
//

#ifndef MITYA_TELEOP_ARDUINO_NODE_H
#define MITYA_TELEOP_ARDUINO_NODE_H

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "consts.hpp"

namespace robot_mitya {
    #define SERIAL_BUFFER_SIZE 1000

    class ArduinoNode : public rclcpp::Node {
    public:
        ArduinoNode();
        bool openSerial();
        void closeSerial();
        void readSerial(void (*func)(ArduinoNode*, char*));
        void writeSerial(char const* message);
        void publishArduinoOutput(char *message);
    private:
        const char *serialPortParamName = "serial_port";
        const char *baudRateParamName = "baud_rate";

        std::string serialPortName;
        int serialBaudRate;
        int fd;
        bool isPortOpened{false};
        char serialIncomingMessage[MAX_MESSAGE_SIZE];
        int serialIncomingMessageSize;
        char buffer[SERIAL_BUFFER_SIZE];
        bool setInterfaceAttributes(int speed, int parity);
        bool setBlocking(int should_block);
        int baudRateToBaudRateConst(int baudRate);
        void arduino_input_topic_callback(std_msgs::msg::String::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arduinoInputSubscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduinoOutputPublisher_;
    };
}

#endif //MITYA_TELEOP_ARDUINO_NODE_H
