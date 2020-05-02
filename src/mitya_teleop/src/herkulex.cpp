/*
 * Herkulex.cpp
 * Copyright (c) 2017, Robot Mitya.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Mitya nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Aug 4, 2017
 *      Author: Dmitry Dzakhov
 */

#include "herkulex.hpp"

#include <unistd.h>
#include <cstring>
#include <ctime>

HerkulexClass::HerkulexClass() noexcept {
    isPortOpened_ = false;
    conta_ = 0;
    ck1_ = 0;
    ck2_ = 0;
    cmd_ = 0;
    fd_ = 0;
    lengthString_ = 0;
    pID_ = 0;
    playTime_ = 0;
    pSize_ = 0;
    XOR_ = 0;
}

HerkulexClass::~HerkulexClass() {
    end();
}

// Herkulex serial port initialization
bool HerkulexClass::begin(char const *portName, int baud) {
    end();

    fd_ = open(portName, static_cast<int>(
            static_cast<unsigned int>(O_RDWR) |
            static_cast<unsigned int>(O_NOCTTY) |
            static_cast<unsigned int>(O_SYNC)));
    if (fd_ < 0)
        return false;

    isPortOpened_ = true;
    if (!setInterfaceAttribs(fd_, baudRateToBaudRateConst(baud), 0)) // set speed bps, 8n1 (no parity)
        return false;
    return setBlocking(fd_, 0);
}

// Herkulex serial port initialization
void HerkulexClass::end() {
    if (isPortOpened_) {
        close(fd_);
        isPortOpened_ = false;
    }
}

// initialize servos
void HerkulexClass::initialize() {
    conta_ = 0;
    lengthString_ = 0;
    delay(100);
    clearError(BROADCAST_ID); // clear error for all servos
    delay(10);
    ACK(1); // set ACK
    delay(10);
    torqueState(BROADCAST_ID, TS_TORQUE_ON); // torqueON for all servos
    delay(10);
}

// stat
uint8_t HerkulexClass::stat(int servoID, uint8_t *statusError, uint8_t *statusDetail) {
    pSize_ = 0x07; //3.Packet size
    pID_ = servoID; //4.Servo ID - 0XFE=All servos
    cmd_ = HSTAT; //5.CMD

    ck1_ = (pSize_ ^ pID_ ^ cmd_) & 0xFE;
    ck2_ = (~(pSize_ ^ pID_ ^ cmd_)) & 0xFE;

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2

    sendData(dataEx_, pSize_);
    delay(2);
    readData(9); // read 9 bytes from serial

    pSize_ = dataEx_[2]; // 3.Packet size 7-58
    pID_ = dataEx_[3]; // 4. Servo ID
    cmd_ = dataEx_[4]; // 5. CMD
    data_[0] = dataEx_[7];
    data_[1] = dataEx_[8];
    lengthString_ = 2;

    ck1_ = (dataEx_[2] ^ dataEx_[3] ^ dataEx_[4] ^ dataEx_[7] ^ dataEx_[8]) & 0xFE;
    ck2_ = checksum2(ck1_);

    if (ck1_ != dataEx_[5])
        return -1; //checksum verify
    if (ck2_ != dataEx_[6])
        return -2;

    if (statusError != nullptr)
        *statusError = dataEx_[7];
    if (statusDetail != nullptr)
        *statusDetail = dataEx_[8];
    return 0;
}

void HerkulexClass::torqueState(int servoID, TorqueState state) {
    pSize_ = 0x0A; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID
    cmd_ = HRAMWRITE; // 5. CMD
    data_[0] = 0x34; // 8. Address
    data_[1] = 0x01; // 9. Length
    data_[2] = state; // 10. 0x00=Torque Free, 0x40=Break ON, 0x60=Torque ON
    lengthString_ = 3; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address 52
    dataEx_[8] = data_[1]; // Length
    dataEx_[9] = data_[2]; // State

    sendData(dataEx_, pSize_);
}

void HerkulexClass::setTorqueFree(int servoID) {
    torqueState(servoID, TS_TORQUE_FREE);
}

void HerkulexClass::setBreakOn(int servoID) {
    torqueState(servoID, TS_BREAK_ON);
}

void HerkulexClass::setTorqueOn(int servoID) {
    torqueState(servoID, TS_TORQUE_ON);
}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::ACK(int valueACK) {
    pSize_ = 0x0A; // 3.Packet size 7-58
    pID_ = 0xFE; // 4. Servo ID
    cmd_ = HRAMWRITE; // 5. CMD
    data_[0] = 0x34; // 8. Address
    data_[1] = 0x01; // 9. Length
    data_[2] = valueACK; // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
    lengthString_ = 3; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address 52
    dataEx_[8] = data_[1]; // Length
    dataEx_[9] = data_[2]; // Value

    sendData(dataEx_, pSize_);
}

// model - 1=0101 - 2=0201
uint8_t HerkulexClass::model() {
    pSize_ = 0x09; // 3.Packet size 7-58
    pID_ = 0xFE; // 4. Servo ID
    cmd_ = HEEPREAD; // 5. CMD
    data_[0] = 0x00; // 8. Address
    data_[1] = 0x01; // 9. Length
    lengthString_ = 2; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address
    dataEx_[8] = data_[1]; // Length

    sendData(dataEx_, pSize_);

    delay(1);
    readData(9);

    pSize_ = dataEx_[2]; // 3.Packet size 7-58
    pID_ = dataEx_[3]; // 4. Servo ID
    cmd_ = dataEx_[4]; // 5. CMD
    data_[0] = dataEx_[7]; // 8. 1st uint8_t
    lengthString_ = 1; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    if (ck1_ != dataEx_[5])
        return -1; //checksum verify
    if (ck2_ != dataEx_[6])
        return -2;

    return dataEx_[7]; // return status
}

// setID - Need to restart the servo
void HerkulexClass::set_ID(int ID_Old, int ID_New) {
    pSize_ = 0x0A; // 3.Packet size 7-58
    pID_ = ID_Old; // 4. Servo ID OLD - original servo ID
    cmd_ = HEEPWRITE; // 5. CMD
    data_[0] = 0x06; // 8. Address
    data_[1] = 0x01; // 9. Length
    data_[2] = ID_New; // 10. ServoID NEW
    lengthString_ = 3; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address 52
    dataEx_[8] = data_[1]; // Length
    dataEx_[9] = data_[2]; // Value

    sendData(dataEx_, pSize_);
}

// clearError
void HerkulexClass::clearError(int servoID) {
    pSize_ = 0x0B; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID - 253=all servos
    cmd_ = HRAMWRITE; // 5. CMD
    data_[0] = 0x30; // 8. Address
    data_[1] = 0x02; // 9. Length
    data_[2] = 0x00; // 10. Write error=0
    data_[3] = 0x00; // 10. Write detail error=0

    lengthString_ = 4; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address 52
    dataEx_[8] = data_[1]; // Length
    dataEx_[9] = data_[2]; // Value1
    dataEx_[10] = data_[3]; // Value2

    sendData(dataEx_, pSize_);
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAll(int servoID, int Goal, int iLed) {
    if (Goal > 1023 || Goal < 0)
        return; //0 <--> 1023 range

    int iMode = 0; //mode=position
    int iStop = 0; //stop=0

    // Position definition
    int posLSB = Goal & 0X00FF; // MSB Pos
    int posMSB = (Goal & 0XFF00) >> 8; // LSB Pos

    //led
    int iBlue = 0;
    int iGreen = 0;
    int iRed = 0;
    switch (iLed) {
        case 1:
            iGreen = 1;
            break;
        case 2:
            iBlue = 1;
            break;
        case 3:
            iRed = 1;
            break;
        default:
            break;
    }

    int SetValue = iStop + iMode * 2 + iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

    addData(posLSB, posMSB, SetValue, servoID); //add servo data_ to list, pos mode
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAllAngle(int servoID, float angle, int iLed) {
    if (angle > 160.0 || angle < -160.0)
        return; // out of the range
    int position = (int) (angle / 0.325) + 512;
    moveAll(servoID, position, iLed);
}

// move all servo at the same time with different speeds: servo list building
void HerkulexClass::moveSpeedAll(int servoID, int Goal, int iLed) {
    if (Goal > 1023 || Goal < -1023)
        return; //-1023 <--> 1023 range

    int iMode = 1; // mode=continous rotation
    int iStop = 0; // Stop=0

    // Speed definition
    int GoalSpeedSign;
    if (Goal < 0) {
        GoalSpeedSign = (-1) * Goal;
        GoalSpeedSign |= 0x4000; //bit n�14
    } else {
        GoalSpeedSign = Goal;
    }

    int speedGoalLSB = GoalSpeedSign & 0X00FF; // MSB speedGoal
    int speedGoalMSB = (GoalSpeedSign & 0xFF00) >> 8; // LSB speedGoal

    //led
    int iBlue = 0;
    int iGreen = 0;
    int iRed = 0;
    switch (iLed) {
        case 1:
            iGreen = 1;
            break;
        case 2:
            iBlue = 1;
            break;
        case 3:
            iRed = 1;
            break;
        default:
            break;
    }

    int SetValue = iStop + iMode * 2 + iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

    addData(speedGoalLSB, speedGoalMSB, SetValue, servoID); //add servo data_ to list, speed mode
}

// move all servo with the same execution time
void HerkulexClass::actionAll(int pTime) {
    if ((pTime < 0) || (pTime > 2856))
        return;

    pSize_ = 0x08 + conta_; // 3.Packet size 7-58
    cmd_ = HSJOG; // 5. CMD SJOG Write n servo with same execution time
    playTime_ = int((float) pTime / 11.2); // 8. Execution time

    pID_ = 0xFE ^ playTime_;
    ck1_ = checksum1(moveData_, conta_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    pID_ = 0xFE;
    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = playTime_; // Execution time

    for (int i = 0; i < conta_; i++)
        dataEx_[i + 8] = moveData_[i]; // Variable servo data_

    sendData(dataEx_, pSize_);

    conta_ = 0; //reset counter
}

// get Position
int HerkulexClass::getPosition(int servoID) {
    int Position = 0;

    pSize_ = 0x09; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID - 253=all servos
    cmd_ = HRAMREAD; // 5. CMD
    data_[0] = 0x3A; // 8. Address
    data_[1] = 0x02; // 9. Length

    lengthString_ = 2; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address
    dataEx_[8] = data_[1]; // Length

    sendData(dataEx_, pSize_);

    delay(1);
    readData(13);

    dataEx_[2] = 13; // Fixing a firmware bug (DmitryDzz)
    pSize_ = dataEx_[2]; // 3.Packet size 7-58
    pID_ = dataEx_[3]; // 4. Servo ID
    cmd_ = dataEx_[4]; // 5. CMD
    data_[0] = dataEx_[7];
    data_[1] = dataEx_[8];
    data_[2] = dataEx_[9];
    data_[3] = dataEx_[10];
    data_[4] = dataEx_[11];
    data_[5] = dataEx_[12];
    lengthString_ = 6;

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    if (ck1_ != dataEx_[5])
        return -1;
    if (ck2_ != dataEx_[6])
        return -1;

    Position = ((dataEx_[10] & 0x03) << 8) | dataEx_[9];
    return Position;
}

float HerkulexClass::getAngle(int servoID) {
    return static_cast<float>(getPosition(servoID) - 512) * 0.325f;
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void HerkulexClass::reboot(int servoID) {

    pSize_ = 0x07; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID - 253=all servos
    cmd_ = HREBOOT; // 5. CMD
    ck1_ = (pSize_ ^ pID_ ^ cmd_) & 0xFE;
    ck2_ = (~(pSize_ ^ pID_ ^ cmd_)) & 0xFE;;

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2

    sendData(dataEx_, pSize_);
}

// LED  - see table of colors
void HerkulexClass::setLed(int servoID, int valueLed) {
    pSize_ = 0x0A; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID
    cmd_ = HRAMWRITE; // 5. CMD
    data_[0] = 0x35; // 8. Address 53
    data_[1] = 0x01; // 9. Length
    data_[2] = valueLed; // 10.LedValue
    lengthString_ = 3; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address
    dataEx_[8] = data_[1]; // Length
    dataEx_[9] = data_[2]; // Value

    sendData(dataEx_, pSize_);
}

// get the speed for one servo - values between -1023 <--> 1023
int HerkulexClass::getSpeed(int servoID) {
    int speedy = 0;

    pSize_ = 0x09; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID
    cmd_ = HRAMREAD; // 5. CMD
    data_[0] = 0x40; // 8. Address
    data_[1] = 0x02; // 9. Length

    lengthString_ = 2; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address
    dataEx_[8] = data_[1]; // Length

    sendData(dataEx_, pSize_);

    delay(1);
    readData(13);

    pSize_ = dataEx_[2]; // 3.Packet size 7-58
    pID_ = dataEx_[3]; // 4. Servo ID
    cmd_ = dataEx_[4]; // 5. CMD
    data_[0] = dataEx_[7];
    data_[1] = dataEx_[8];
    data_[2] = dataEx_[9];
    data_[3] = dataEx_[10];
    data_[4] = dataEx_[11];
    data_[5] = dataEx_[12];
    lengthString_ = 6;

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    if (ck1_ != dataEx_[5])
        return -1;
    if (ck2_ != dataEx_[6])
        return -1;

    speedy = ((dataEx_[10] & 0xFF) << 8) | dataEx_[9];
    return speedy;
}

// move one servo with continous rotation
void HerkulexClass::moveSpeedOne(int servoID, int Goal, int pTime, int iLed) {
    if (Goal > 1023 || Goal < -1023)
        return; // speed (goal) non correct
    if ((pTime < 0) || (pTime > 2856))
        return;

    int GoalSpeedSign;
    if (Goal < 0) {
        GoalSpeedSign = (-1) * Goal;
        GoalSpeedSign |= 0x4000; //bit n�14
    } else {
        GoalSpeedSign = Goal;
    }
    int speedGoalLSB = GoalSpeedSign & 0X00FF; // MSB speedGoal
    int speedGoalMSB = (GoalSpeedSign & 0xFF00) >> 8; // LSB speedGoal

    //led
    int iBlue = 0;
    int iGreen = 0;
    int iRed = 0;
    switch (iLed) {
        case 1:
            iGreen = 1;
            break;
        case 2:
            iBlue = 1;
            break;
        case 3:
            iRed = 1;
            break;
        default:
            break;
    }
    int SetValue = 2 + iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

    playTime_ = int((float) pTime / 11.2); // 8. Execution time

    pSize_ = 0x0C; // 3.Packet size 7-58
    cmd_ = HSJOG; // 5. CMD

    data_[0] = speedGoalLSB; // 8. speedLSB
    data_[1] = speedGoalMSB; // 9. speedMSB
    data_[2] = SetValue; // 10. Mode=0;
    data_[3] = servoID; // 11. ServoID

    pID_ = servoID ^ playTime_;

    lengthString_ = 4; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    pID_ = servoID;

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = playTime_; // Execution time
    dataEx_[8] = data_[0];
    dataEx_[9] = data_[1];
    dataEx_[10] = data_[2];
    dataEx_[11] = data_[3];

    sendData(dataEx_, pSize_);
}

// move one servo at goal position 0 - 1024
void HerkulexClass::moveOne(int servoID, int Goal, int pTime, int iLed) {
    if (Goal > 1023 || Goal < 0)
        return; // speed (goal) non correct
    if ((pTime < 0) || (pTime > 2856))
        return;

    // Position definition
    int posLSB = Goal & 0X00FF; // MSB Pos
    int posMSB = (Goal & 0XFF00) >> 8; // LSB Pos

    //led
    int iBlue = 0;
    int iGreen = 0;
    int iRed = 0;
    switch (iLed) {
        case 1:
            iGreen = 1;
            break;
        case 2:
            iBlue = 1;
            break;
        case 3:
            iRed = 1;
            break;
        default:
            break;
    }
    int SetValue = iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

    playTime_ = int((float) pTime / 11.2); // 8. Execution time

    pSize_ = 0x0C; // 3.Packet size 7-58
    cmd_ = HSJOG; // 5. CMD

    data_[0] = posLSB; // 8. speedLSB
    data_[1] = posMSB; // 9. speedMSB
    data_[2] = SetValue; // 10. Mode=0;
    data_[3] = servoID; // 11. ServoID

    pID_ = servoID ^ playTime_;

    lengthString_ = 4; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    pID_ = servoID;

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = playTime_; // Execution time
    dataEx_[8] = data_[0];
    dataEx_[9] = data_[1];
    dataEx_[10] = data_[2];
    dataEx_[11] = data_[3];

    sendData(dataEx_, pSize_);
}

// move one servo to an angle between -160 and 160
void HerkulexClass::moveOneAngle(int servoID, float angle, int pTime, int iLed) {
    if (angle > 160.0 || angle < -160.0)
        return;
    int position = (int) (angle / 0.325) + 512;
    moveOne(servoID, position, pTime, iLed);
}

// write registry in the RAM: one uint8_t
void HerkulexClass::writeRegistryRAM(int servoID, int address, int writeByte) {
    pSize_ = 0x0A; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID - 253=all servos
    cmd_ = HRAMWRITE; // 5. CMD
    data_[0] = address; // 8. Address
    data_[1] = 0x01; // 9. Length
    data_[2] = writeByte; // 10. Write error=0

    lengthString_ = 3; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address 52
    dataEx_[8] = data_[1]; // Length
    dataEx_[9] = data_[2]; // Value1
    dataEx_[10] = data_[3]; // Value2

    sendData(dataEx_, pSize_);
}

// write registry in the EEP memory (ROM): one uint8_t
void HerkulexClass::writeRegistryEEP(int servoID, int address, int writeByte) {
    pSize_ = 0x0A; // 3.Packet size 7-58
    pID_ = servoID; // 4. Servo ID - 253=all servos
    cmd_ = HEEPWRITE; // 5. CMD
    data_[0] = address; // 8. Address
    data_[1] = 0x01; // 9. Length
    data_[2] = writeByte; // 10. Write error=0

    lengthString_ = 3; // lengthData

    ck1_ = checksum1(data_, lengthString_); //6. Checksum1
    ck2_ = checksum2(ck1_); //7. Checksum2

    dataEx_[0] = 0xFF; // Packet Header
    dataEx_[1] = 0xFF; // Packet Header
    dataEx_[2] = pSize_; // Packet Size
    dataEx_[3] = pID_; // Servo ID
    dataEx_[4] = cmd_; // Command Ram Write
    dataEx_[5] = ck1_; // Checksum 1
    dataEx_[6] = ck2_; // Checksum 2
    dataEx_[7] = data_[0]; // Address 52
    dataEx_[8] = data_[1]; // Length
    dataEx_[9] = data_[2]; // Value1
    dataEx_[10] = data_[3]; // Value2

    sendData(dataEx_, pSize_);
}

// Private Methods //////////////////////////////////////////////////////////////

// checksum1
int HerkulexClass::checksum1(const uint8_t *data, int lengthString) {
    XOR_ = 0;
    XOR_ = XOR_ ^ pSize_;
    XOR_ = XOR_ ^ pID_;
    XOR_ = XOR_ ^ cmd_;
    for (int i = 0; i < lengthString; i++) {
        XOR_ = XOR_ ^ data[i];
    }
    return XOR_ & 0xFE;
}

// checksum2
int HerkulexClass::checksum2(int XOR) {
    return (~XOR) & 0xFE;
}

// add data_ to variable list servo for syncro execution
void HerkulexClass::addData(int GoalLSB, int GoalMSB, int set, int servoID) {
    moveData_[conta_++] = GoalLSB;
    moveData_[conta_++] = GoalMSB;
    moveData_[conta_++] = set;
    moveData_[conta_++] = servoID;
}

// Sending the buffer long length to Serial port
void HerkulexClass::sendData(uint8_t *buffer, int length) const {
    //clear the serial port input buffer - try to do it!
    tcflush(fd_, TCIFLUSH);

    write(fd_, buffer, length);
}

const clock_t TIMEOUT_CLOCK = TIME_OUT * CLOCKS_PER_SEC;

// * Receiving the length of bytes from Serial port
bool HerkulexClass::readData(int size) {
    clock_t start = clock();
    int totalReadBytes = 0;
    int totalBytesToRead = size;
    int bytesToRead;
    while (totalReadBytes < size) {
        bytesToRead = totalBytesToRead <= DATA_MOVE_ALL ? totalBytesToRead : DATA_MOVE_ALL;
        int readBytes = read(fd_, &dataEx_[totalReadBytes], bytesToRead);
        totalReadBytes += readBytes;
        totalBytesToRead -= readBytes;

        clock_t now = clock();
        if (now - start > TIMEOUT_CLOCK)
            return false;
    }
    return true;
}

void HerkulexClass::delay(long millis) {
    usleep(millis * 1000);
}

bool HerkulexClass::setInterfaceAttribs(int fd, int speed, int parity) {
    struct termios tty{};
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
        return false;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN] = 0;            // read doesn't block
    tty.c_cc[VTIME] = 0;            // 0 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

bool HerkulexClass::setBlocking(int fd, int should_block) {
    struct termios tty{};
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 0;            // 0 seconds read timeout

    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

int HerkulexClass::baudRateToBaudRateConst(int baudRate) {
    switch (baudRate) {
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
            return B9600;
    }
}


//HerkulexClass Herkulex;
