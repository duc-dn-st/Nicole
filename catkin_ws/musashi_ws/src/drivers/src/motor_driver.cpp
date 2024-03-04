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

#include "drivers/motor_driver.h"

void MotorDriver::openDAC()
{
    // uldaq setup
    // Get descriptors for all of the available DAQ devices
    err_ = ulGetDaqDeviceInventory(interfaceType_, devDescriptors_, &numDevs_);

    if (err_ != ERR_NO_ERROR)
    {
        ROS_ERROR("Error: Possibly no descriptor received");
    }

    // verify at least one DAQ device is detected
    if (numDevs_ == 0)
    {
        ROS_WARN("No device detected!");
    }

    ROS_INFO("Found %d DAQ device", numDevs_);
    
    for (unsigned int device_index = 0; device_index < numDevs_; device_index++)
    {
        if ( strcmp( devDescriptors_[device_index].productName, "USB-3105" ) == 0 ) 
        {
            descriptorIndex_ = device_index;
            ROS_INFO("%s: (%s)", devDescriptors_[device_index].productName, devDescriptors_[device_index].uniqueId);
        }
    }

    daqDeviceHandle_ = ulCreateDaqDevice(devDescriptors_[descriptorIndex_]);

    if (daqDeviceHandle_ == 0)
    {
        ROS_ERROR("Unable to create a handle to the specified DAQ device\n");
    }

    ROS_INFO("Connecting to device %s - please wait ...", devDescriptors_[descriptorIndex_].devString);

    // establish a connection to the device
    err_ = ulConnectDaqDevice(daqDeviceHandle_);

    if (err_ != ERR_NO_ERROR)
    {
        ROS_ERROR("Could not connect to device.");
    }
    else
    {
        ROS_INFO("Connection succesfull.");
    }
}

void MotorDriver::closeDAC()
{
    stopMotors();
    // disconnect from the DAQ device
    ulDisconnectDaqDevice(daqDeviceHandle_);
    // release the handle to the DAQ device
    ulReleaseDaqDevice(daqDeviceHandle_);
    ROS_INFO("Device is shutdown");
}

void MotorDriver::stopMotors()
{
    double data[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ulAOutArray(daqDeviceHandle_, 0, 15, ranges_, flag_, data);
}

/**
 * @brief 
 * 
 * @param vel_fl in m/s
 * @param vel_fr in m/s
 * @param vel_bl in m/s
 * @param vel_br in m/s
 * @param steering_vel_fl in rad/s
 * @param steering_vel_fr in rad/s
 * @param steering_vel_bl in rad/s
 * @param steering_vel_br in rad/s
 */
void MotorDriver::sendMotorCommands(const double vel_fl, const double vel_fr, const double vel_bl, const double vel_br,
                                    const double steering_vel_fl, const double steering_vel_fr, const double steering_vel_bl, const double steering_vel_br)
{
    double voltage, binary_high_low;
    double data[16];
    // currently the order is as follows and for ackerman
    // CH0-1 FL and BL driving, CH2-3 FR and BR driving, CH4-5 FL steering, CH6-7 FR steering
    // above comment obsolete
    // CH0-1 (vel_fl)
    velocityToVoltage(vel_fl, voltage);
    highForwardLowReverse(vel_fl, binary_high_low);
    data[0] = voltage;
    data[1] = binary_high_low;
    // CH2-3 (vel_fr)
    velocityToVoltage(vel_fr, voltage);
    highForwardLowReverse(vel_fr, binary_high_low);
    data[2] = voltage;
    data[3] = binary_high_low;
    // CH4-5 (vel_bl)
    velocityToVoltage(vel_bl, voltage);
    highForwardLowReverse(vel_bl, binary_high_low);
    data[4] = voltage;
    data[5] = binary_high_low;
    // CH6-7 (vel_br)
    velocityToVoltage(vel_br, voltage);
    highForwardLowReverse(vel_br, binary_high_low);
    data[6] = voltage;
    data[7] = binary_high_low;
    std::cout << "Print steering velocities" << std::endl;
    // CH8-9 (steering_vel_fl)
    steeringToVoltage(steering_vel_fl, voltage);
    highCCWLowCW(steering_vel_fl, binary_high_low);
    data[8] = voltage;
    data[9] = binary_high_low;
    // CH10-11 (steering_vel_fr)
    steeringToVoltage(steering_vel_fr, voltage);
    highCCWLowCW(steering_vel_fr, binary_high_low);
    data[10] = voltage;
    data[11] = binary_high_low;
    // CH12-13 (steering_vel_bl)
    steeringToVoltage(steering_vel_bl, voltage);
    highCCWLowCW(steering_vel_bl, binary_high_low);
    data[12] = voltage;
    data[13] = binary_high_low;
    // CH14-15 (steering_vel_br)
    steeringToVoltage(steering_vel_br, voltage);
    highCCWLowCW(steering_vel_br, binary_high_low);
    data[14] = voltage;
    data[15] = binary_high_low;

    //double data_1[8] = {5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // send to the motors
    std::cout << "Steering voltages:\t ["<<data[8] << ", "<<data[10] << ", "<<data[12] << ", "<<data[14] << "]" <<std::endl;
    std::cout << "Steering directions:\t ["<<data[9] << ", "<<data[11] << ", "<<data[13] << ", "<<data[15] << "]" <<std::endl;
    std::cout << "Driving voltages:\t ["<<data[0] << ", "<<data[2] << ", "<<data[4] << ", "<<data[6] << "]" <<std::endl;
    std::cout << "Driving directions\:\t ["<<data[1] << ", "<<data[3] << ", "<<data[5] << ", "<<data[7] << "]" <<std::endl;
    ulAOutArray(daqDeviceHandle_, 0, 15, ranges_, flag_, data);
    //ulFlashLed(daqDeviceHandle_, 0);
    std::cout << "Commands send" << std::endl;
}

void MotorDriver::testDrivingMotor(const double vel_fl, const double vel_fr, const double vel_bl, const double vel_br)
{
    double voltage;
    double binary_fwd_reverse;
    double data[16];

    // get the data for all four motors
    // fl
    velocityToVoltage(vel_fl, voltage);
    highForwardLowReverse(vel_fl, binary_fwd_reverse);
    data[0] = voltage;
    data[1] = binary_fwd_reverse;
    //std::cout << "Voltage:\t" << data[0] << " HL:\t" << data[1] << std::endl;
    // fr
    velocityToVoltage(vel_fr, voltage);
    highForwardLowReverse(vel_fr, binary_fwd_reverse);
    data[2] = voltage;
    data[3] = binary_fwd_reverse;
    //std::cout << "Voltage:\t" << data[2] << " HL:\t" << data[3] << std::endl;
    // bl
    velocityToVoltage(vel_bl, voltage);
    highForwardLowReverse(vel_bl, binary_fwd_reverse);
    data[4] = voltage;
    data[5] = binary_fwd_reverse;
    //std::cout << "Voltage:\t" << data[4] << " HL:\t" << data[5] << std::endl;
    // br
    velocityToVoltage(vel_br, voltage);
    highForwardLowReverse(vel_br, binary_fwd_reverse);
    data[6] = voltage;
    data[7] = binary_fwd_reverse;
    //std::cout << "Voltage:\t" << data[6] << " HL:\t" << data[7] << "\n" << std::endl;

    for (unsigned int ii = 8; ii < 16; ++ii)
    {
        // data points for steering
        data[ii] = 0.0;
    }

    ulAOutArray(daqDeviceHandle_, 0, 15, ranges_, flag_, data);
}

void MotorDriver::testSteeringMotor(const double steering_vel_fl, const double steering_vel_fr, const double steering_vel_bl, const double steering_vel_br)
{
    double voltage;
    double binary_fwd_reverse;
    double data[16];

    for (unsigned int ii = 0; ii < 16; ++ii)
    {
        // data points for driving
        data[ii] = 0.0;
    }

    // get the data for all four motors
    // fl
    steeringToVoltage(steering_vel_fl, voltage);
    highCCWLowCW(steering_vel_fl, binary_fwd_reverse);
    data[8] = voltage;
    data[9] = binary_fwd_reverse;
    // fr
    steeringToVoltage(steering_vel_fr, voltage);
    highCCWLowCW(steering_vel_fr, binary_fwd_reverse);
    data[10] = voltage;
    data[11] = binary_fwd_reverse;
    // bl
    steeringToVoltage(steering_vel_bl, voltage);
    highCCWLowCW(steering_vel_bl, binary_fwd_reverse);
    data[12] = voltage;
    data[13] = binary_fwd_reverse;
    // br
    steeringToVoltage(steering_vel_br, voltage);
    highCCWLowCW(steering_vel_br, binary_fwd_reverse);
    data[14] = voltage;
    data[15] = binary_fwd_reverse;

    std::cout << "Voltage and Binary Voltage:\t[" << voltage << ", " << binary_fwd_reverse << "]" << std::endl;

    ulAOutArray(daqDeviceHandle_, 0, 15, ranges_, flag_, data);
}
