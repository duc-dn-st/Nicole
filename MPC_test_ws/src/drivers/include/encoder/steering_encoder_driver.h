/*
 * ===============================================================================
 * steering_encoder_driver.cpp
 * Author: Tran Viet Thanh
 * Date: 04.01.2021
 * -------------------------------------------------------------------------------
 * Description:
 * This file accesses the steering encoder for position feedback.
 * ===============================================================================
 */

#ifndef STEERING_ENCODER_DRIVER_H
#define STEERING_ENCODER_DRIVER_H

#include <SerialStream.h>

class SteeringEncoderDriver
{
private:
    // Initialize Serial Communication 
    SerialStream encoder_communication_;
    
    // Port
    unsigned int 

public:
    SteeringEncoderDriver(){}

    ~SteeringEncoderDriver(){}

};

#endif // STEERING_ENCODER_DRIVER_H
