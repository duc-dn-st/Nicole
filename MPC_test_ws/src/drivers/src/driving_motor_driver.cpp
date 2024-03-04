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

#include "../include/drivers/driving_motor_driver.h"

/**
 * @brief drivingMotorDriver::velocityToVoltage
 * @param velocity
 * @param voltage
 */
void DrivingMotorDriver::velocityToVoltage(const double &velocity, double &voltage) const
{
    // divide the absolut velocity with the driving gain
    std::cout << "Velocity:\t" << velocity << std::endl;
    voltage = std::abs(velocity) / RobotConstants::DRIVING_GAIN;
    // check if hits limit
    if (voltage > RobotConstants::MAX_DRIVING_VOLTAGE)
    {
        voltage = RobotConstants::MAX_DRIVING_VOLTAGE;
    }
}

void DrivingMotorDriver::highForwardLowReverse(const double &velocity, double &high_low) const
{
    high_low = (velocity >= 0.0) ? HIGH_VAL : LOW_VAL;
}
