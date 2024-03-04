/*
 * ===============================================================================
 * driving_motor_driver.cpp
 * Author: Schaefle Tobias
 * Date: 04.06.2019
 * -------------------------------------------------------------------------------
 * Description:
 * This file accesses the motors for motion.
 * ===============================================================================
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define MAX_DEV_COUNT  100
#define MAX_STR_LENGTH 64

#define DRIVING_VOLTAGE_CH_FL 0
#define DRIVING_FWD_RWD_CH_FL 1

#define DRIVING_VOLTAGE_CH_FR 2
#define DRIVING_FWD_RWD_CH_FR 3

#define DRIVING_VOLTAGE_CH_BL 4
#define DRIVING_FWD_RWD_CH_BL 5

#define DRIVING_VOLTAGE_CH_BR 6
#define DRIVING_FWD_RWD_CH_BR 7

#define STEERING_CW_CH_FL 8
#define STEERING_CCW_CH_FL 9

#define STEERING_CW_CH_FR 10
#define STEERING_CCW_CH_FR 11

#define STEERING_CW_CH_BL 12
#define STEERING_CCW_CH_BL 13

#define STEERING_CW_CH_BR 14
#define STEERING_CCW_CH_BR 15

#include <ros/ros.h>

#include "uldaq.h"

#include "driving_motor_driver.h"
#include "steering_motor_driver.h"

class MotorDriver : public DrivingMotorDriver, public SteeringMotorDriver
{
public:
    MotorDriver(){}

    ~MotorDriver(){}

    // stop the motors (used for emergency stop)
    void stopMotors();
    
    // function for sending commands to driving and steering motors
    void sendMotorCommands(const double vel_fl, const double vel_fr, const double vel_bl, const double vel_br,
                           const double steering_vel_fl, const double steering_vel_fr, const double steering_vel_bl, const double steering_vel_br);

    /// test the driving motors (forward has positiv velocities and reverse negativ)
    void testDrivingMotor(const double vel_fl, const double vel_fr, const double vel_bl, const double vel_br);

    /// test the steering motors (clockwise is negativ and counterclockwise is positiv)
    void testSteeringMotor(const double steering_vel_fl, const double steering_vel_fr, const double steering_vel_bl, const double steering_vel_br);

    /// open the DAC
    void openDAC();

    /// close the DAC
    void closeDAC();

private:

    // uldaq stuff
    int descriptorIndex_ = 0;
    DaqDeviceDescriptor devDescriptors_[MAX_DEV_COUNT];
    DaqDeviceInterface interfaceType_ = ANY_IFC;
    DaqDeviceHandle daqDeviceHandle_ = 0;
    unsigned int numDevs_ = MAX_DEV_COUNT;

    Range test_ranges_ [8] = {UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS};
    Range ranges_[16] = {UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS,
                         UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS, UNI10VOLTS};

    AOutArrayFlag flag_ = AOUTARRAY_FF_DEFAULT;

    UlError err_ = ERR_NO_ERROR;
};

#endif // MOTOR_DRIVER_H
