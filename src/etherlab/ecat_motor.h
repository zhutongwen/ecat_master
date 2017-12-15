

#ifndef ECAT_MOTOR_H
#define ECAT_MOTOR_H

#include <iostream>
#include "ecrt.h"


class EcatMotor
{
public:


    //mode: 0x01 position mode
    //      0x03 velocity mode
    //      0x04 torque mode
    EcatMotor(uint8_t mode);
    ~EcatMotor(){}

    uint8_t GetMode();
    int32_t GetActualPosition();
    int32_t GetActualVelocity();
    int16_t GetActualCurrent();

    int SetMode(uint8_t mode);
    int SetTargtPosition(int32_t position);
    int SetTargtVelocity(int32_t velocity);
    int SetTargtTorque(int16_t torque);

};

#endif
