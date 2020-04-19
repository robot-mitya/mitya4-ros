#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "consts.h"
#include "robo_com.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

using std::placeholders::_1;

#define MAX_MESSAGE_SIZE 200
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

ArduinoNode::ArduinoNode() : Node(RM_ARDUINO_NODE_NAME, RM_NAMESPACE) {
    this->declare_parameter<std::string>(serialPortParamName, "");
    this->declare_parameter<int>(baudRateParamName, 115200);

    arduinoInputSubscription_ = this->create_subscription<std_msgs::msg::String>(RM_ARDUINO_INPUT_TOPIC_NAME,
            100, std::bind(&ArduinoNode::arduino_input_topic_callback, this, _1));
    arduinoOutputPublisher_ = this->create_publisher<std_msgs::msg::String>(RM_ARDUINO_OUTPUT_TOPIC_NAME, 100);
}

bool ArduinoNode::setInterfaceAttributes(int speed, int parity)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %d from tcgetattr", errno);
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 0;            // 0 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %d from tcsetattr", errno);
        return false;
    }
    return true;
}

bool ArduinoNode::setBlocking(int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %d from tcgetattr", errno);
        return false;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 0;            // 0 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %d setting term attributes", errno);
        return false;
    }

    return true;
}

bool ArduinoNode::openSerial()
{
//    serialPortName = "/dev/ttyACM0";
//    serialBaudRate = 115200;
//
    rclcpp::Parameter serialPortParam;
    if (this->get_parameter(serialPortParamName, serialPortParam)) {
        serialPortName = serialPortParam.as_string();
        RCLCPP_INFO(this->get_logger(), "%s param value: %s", serialPortParamName, serialPortName.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s param not found", serialPortParamName);
        return false;        
    }

    rclcpp::Parameter baudRateParam;
    if (this->get_parameter(baudRateParamName, baudRateParam)) {
        serialBaudRate = baudRateParam.as_int();
        RCLCPP_INFO(this->get_logger(), "%s param value: %d", baudRateParamName, serialBaudRate);
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s param not found", baudRateParamName);
        return false;
    }

    closeSerial();
    fd = open(serialPortName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %d opening %s: %s", errno, serialPortName.c_str(), strerror(errno));
        return false;
    }
    isPortOpened = true;

    if (!setInterfaceAttributes(baudRateToBaudRateConst(serialBaudRate), 0)) // set speed bps, 8n1 (no parity)
        return false;
    if (!setBlocking(0)) // set no blocking
        return false;

    RCLCPP_INFO(this->get_logger(), "Serial port %s is opened at %d baud rate", serialPortName.c_str(), serialBaudRate);
    return true;
}

void ArduinoNode::closeSerial()
{
    if (isPortOpened)
    {
        close(fd);
        isPortOpened = false;
        RCLCPP_INFO(this->get_logger(), "Serial port is closed");
    }
}

int ArduinoNode::baudRateToBaudRateConst(int baudRate)
{
    switch (baudRate)
    {
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            RCLCPP_ERROR(this->get_logger(),
                    "Wrong serial baud rate value. The default value is 9600.");
            return B9600;
    }
}

void ArduinoNode::readSerial(void (*func)(ArduinoNode*, char*))
{
    int n = read(fd, buffer, SERIAL_BUFFER_SIZE);
    char ch;
    for (int i = 0; i < n; i++)
    {
        ch = buffer[i];
        if (ch < 32) continue;
        serialIncomingMessage[serialIncomingMessageSize++] = ch;
        if (ch == COMMAND_SEPARATOR)
        {
            serialIncomingMessage[serialIncomingMessageSize] = '\0';
            func(this, serialIncomingMessage);
            serialIncomingMessageSize = 0;
            serialIncomingMessage[serialIncomingMessageSize] = '\0';
        }
    }
}

void ArduinoNode::writeSerial(char const* message)
{
    write(fd, message, strlen(message));
}

void ArduinoNode::arduino_input_topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    this->writeSerial(msg->data.c_str());
}

void ArduinoNode::publishArduinoOutput(char *message)
{
    std_msgs::msg::String msg = std_msgs::msg::String();
    msg.data = message;
    arduinoOutputPublisher_->publish(msg);
}

void onReceiveSerialMessage(ArduinoNode *arduinoNode, char *message)
{
    RCLCPP_DEBUG(arduinoNode->get_logger(), "I heard: '%s'", message);
    Command command;
    int param1, param2, param3;
    RoboCom::parseMessage(message, command, param1, param2, param3);
    switch (command)
    {
        case CMD_STATUS_RESPONSE:
        {
            if (param1 > 0)
                RCLCPP_ERROR(arduinoNode->get_logger(), "Arduino controller's status response error: %s (%d)",
                        RoboCom::getStatusText(param1), param1);
            break;
        }
        case CMD_L1_RESPONSE:
        {
            //TODO #1
//            arduinoNode->publishLED(1, param1);
            arduinoNode->publishArduinoOutput(message);
            break;
        }
        case CMD_L2_RESPONSE:
        {
            //TODO #1
//            arduinoNode->publishLED(2, param1);
            arduinoNode->publishArduinoOutput(message);
            break;
        }
        case CMD_DIST_RESPONSE:
        {
            float meters = (float) param2;
            meters /= 1000;
            meters += (float) param1;
            //TODO #1
//            arduinoNode->publishDistance(meters);
            break;
        }
        case CMD_SPD_RESPONSE:
        {
            float kilometersPerHour = (float) param1;
            kilometersPerHour /= 1000;
            //TODO #1
//            arduinoNode->publishSpeed(kilometersPerHour);
            break;
        }
        default:
        {
            arduinoNode->publishArduinoOutput(message);
            break;
        }
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArduinoNode>();
    if (node->openSerial()) {
        rclcpp::Rate loop_rate(100); // 100 Hz (?)
        while (rclcpp::ok()) {
            node->readSerial(onReceiveSerialMessage);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        node->closeSerial();
    } else {
        std::cout << "NOT OPENED" << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
