/*
 * ===============================================================================
 * driving_motor_driver.cpp
 * Author: Schaefle Tobias
 * Date: 04.06.2019
 * -------------------------------------------------------------------------------
 * Description:
 * This file accesses the driving motors for motion.
 * ===============================================================================
 */

#ifndef DRIVING_MOTOR_DRIVER_H
#define DRIVING_MOTOR_DRIVER_H

#define HIGH_VAL 10.0
#define LOW_VAL 0.0

#include <cmath>
#include <iostream>

#include "utilities/utilities.h"

class DrivingMotorDriver
{
public:
    DrivingMotorDriver(){}

    ~DrivingMotorDriver(){}

protected:

    /// convert the absolut velocities to voltage for the DAC and the motors
    void velocityToVoltage(const double &velocity, double &voltage) const;

    /// check if forward or reverse
    void highForwardLowReverse(const double &velocity, double &high_low) const;

};

#endif // DRIVING_MOTOR_DRIVER_H



