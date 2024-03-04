/*
 * ===============================================================================
 * steering_motor_driver.cpp
 * Author: Schaefle Tobias
 * Date: 04.06.2019
 * -------------------------------------------------------------------------------
 * Description:
 * This file accesses the steering motors for motion.
 * ===============================================================================
 */

#ifndef STEERING_MOTOR_DRIVER_H
#define STEERING_MOTOR_DRIVER_H

#define HIGH_VAL_STEER 10.0
#define LOW_VAL_STEER 0.0

#include <cmath>
#include <iostream>
#include "utilities/utilities.h"

class SteeringMotorDriver
{
public:
    SteeringMotorDriver(){}

    ~SteeringMotorDriver(){}

protected:

    void steeringToVoltage(const double &steering_vel, double &voltage) const;

    /// check if forward or reverse
    void highCCWLowCW(const double &steering_velocity, double &high_low) const;

};

#endif // STEERING_MOTOR_DRIVER_H
