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

#include "../include/drivers/steering_motor_driver.h"

/**
 * @brief 
 * 
 * @param steering_vel in rad/s
 * @param voltage 
 */
void SteeringMotorDriver::steeringToVoltage(const double &steering_vel, double &voltage) const
{
    voltage = std::abs(steering_vel) / RobotConstants::STEERING_GAIN;
    if (voltage > RobotConstants::MAX_STEERING_VOLTAGE)
    {
        voltage = RobotConstants::MAX_STEERING_VOLTAGE;
    }
}

void SteeringMotorDriver::highCCWLowCW(const double &steering_velocity, double &high_low) const
{
    high_low = (steering_velocity >= 0.0) ? HIGH_VAL_STEER : LOW_VAL_STEER;
}
