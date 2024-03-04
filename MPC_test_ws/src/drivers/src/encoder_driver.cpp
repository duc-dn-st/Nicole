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
#include "drivers/encoder_driver.h"

UlError EncoderDriver::_getCtrInfoNumberOfChannels(DaqDeviceHandle daq_device_handle, int* number_of_control_channels)
{
    UlError err = ERR_NO_ERROR;
    long long number_channels;

    err = ulCtrGetInfo(daq_device_handle, CTR_INFO_NUM_CTRS, 0, &number_channels);

    *number_of_control_channels = (int)number_channels;

    return err;
}


UlError EncoderDriver::_getCtrInfoHasPacer(DaqDeviceHandle daq_device_handle, int* hasPacer)
{
    UlError err = ERR_NO_ERROR;
    long long pacer_support;

    err = ulCtrGetInfo(daq_device_handle, CTR_INFO_HAS_PACER, 0, &pacer_support);

    *hasPacer = (int)pacer_support;

    return err;
}


UlError EncoderDriver::_getDevInfoHasCtr(DaqDeviceHandle daq_device_handle, int* hasCtr)
{
    long long control_supported;
    UlError err = ERR_NO_ERROR;

    err = ulDevGetInfo(daq_device_handle, DEV_INFO_HAS_CTR_DEV, 0, &control_supported);

    *hasCtr = (int)control_supported;

    return err;
}

UlError EncoderDriver::_getCtrInfoSupportedEncoderCounters(DaqDeviceHandle daq_device_handle, int* encoders, int* number_of_encoders)
{   
    // Initialize error_code
    UlError err = ERR_NO_ERROR;

    int number_of_counters = 0;
    
    long long measurement_types;

    int number_encoder_controllers = 0;

    // Get the number of counter channels
    err = _getCtrInfoNumberOfChannels(daq_device_handle, &number_of_counters);

    // Fill a descriptor
    for (unsigned int i = 0; i < number_of_counters; i++)
    {
        err = ulCtrGetInfo(daq_device_handle, CTR_INFO_MEASUREMENT_TYPES, i, &measurement_types);

        if (measurement_types & CMT_ENCODER)
        {
            if (number_encoder_controllers < *number_of_encoders)
            {
                *encoders++ = i;
            }

            number_of_encoders++;
        }
    }

    *number_of_encoders = number_encoder_controllers;

    return err;
}



void EncoderDriver::openDAQ()
{  
    // Iterative variables
    int i = 0;
    
    // Get descriptors for all of the available DAQ devices
    err_ = ulGetDaqDeviceInventory(interface_type_, device_descriptors_, &number_of_devices_);

    if (err_!= ERR_NO_ERROR)
    {
        ROS_ERROR("Encoder_Error: Possibly no descriptor received");
    }
    
    // Verify at least one DAQ is detected
    if (number_of_devices_ == 0)
    {
        ROS_WARN("No device detected!");
    }
    
    for (unsigned int device_index = 0; device_index < number_of_devices_; device_index++)
    {
        if ( strcmp( device_descriptors_[device_index].productName, "USB-QUAD08" ) == 0 ) 
        {
            descriptor_index_ = device_index;
            ROS_INFO("%s: (%s)", device_descriptors_[device_index].productName, device_descriptors_[device_index].uniqueId);
        }
    }
    
    // Get a handle to the DAQ device associated with the first descriptor
    daq_device_handle_ = ulCreateDaqDevice(device_descriptors_[descriptor_index_]);

    if (daq_device_handle_ == 0)
    {
        ROS_ERROR("Unable to create a handle to the specified DAQ device\n");
    }
    
    // Verify the specified DAQ device supports counter input
    err_ = _getDevInfoHasCtr(daq_device_handle_, &hasCI_);

    if (!hasCI_)
    {
        ROS_ERROR("\nThe scified DAQ device does not support counter input");
    }
    
    // Verify the specified DAQ device supports hardware pacing for counters
    err_ = _getCtrInfoHasPacer(daq_device_handle_, &hasPacer_);

    if (!hasPacer_)
    {
        ROS_ERROR("\nThe scified DAQ device does not support hardware paced counter input");
    }
    
    ROS_INFO("\nConnecting to device %s - please wait ...\n", device_descriptors_[descriptor_index_].devString);
    
    // Create a connection to the device
    err_ = ulConnectDaqDevice(daq_device_handle_);

    if (err_ != ERR_NO_ERROR)
    {
        ROS_ERROR("Unable to connect to specific device\n");
    }
}


void EncoderDriver::getEncoderCountersConfiguration()
{
    ROS_INFO("Start getEncoderCountersConfiguration"); 
    
    // Iterative variables
    int i = 0;
    
    // Get the counter numbers for the supported encoders
    err_ = _getCtrInfoSupportedEncoderCounters(daq_device_handle_, encoder_counters_, &number_of_encoders_);
    
    ROS_INFO("Number of counters = %d", number_of_encoders_);

    if (number_of_encoders_ == 0)
    {
        ROS_ERROR("\nThe specified DAQ device does not support encoder channels");
    }

    if (number_of_encoders_ > MAX_ENCODER_COUNTERS)
    {
        number_of_encoders_ = MAX_ENCODER_COUNTERS;
    }
    
    number_of_encoders_ = 8;

    // Verify that the low_encoder number is valid
    first_encoder_ = encoder_counters_[0];
    
    // NOTE: if low_control_ is smaller than the smallest or larger than the largest then assign it to first_encoder_
    if (low_control_ < first_encoder_)
    {
        low_control_ = first_encoder_;
    }

    if (low_control_ > first_encoder_ + number_of_encoders_ - 1)
    {
        low_control_ = first_encoder_;
    }

    // Set the high_encoder channel
    high_control_ = low_control_ + number_of_encoders_ - 1;

    if (high_control_ > first_encoder_ + number_of_encoders_ - 1)
    {
        high_control_ = first_encoder_ + number_of_encoders_ - 1;
    }

    // Update the actual number of encoders being used 
    encoder_count_ = high_control_ - low_control_ + 1;

    ROS_INFO("Found counters: %d - %d\n", low_control_, high_control_);

    // Configure the encoders
    for (i = 0; i < encoder_count_; i++)
    {
        err_ = ulCConfigScan(daq_device_handle_, i + low_control_, type_, mode_, edge_detection_, tick_size_, debounce_mode_, debounce_time_, config_flags_);
    }

    if (err_ != ERR_NO_ERROR)
    {
        char err_msg[ERR_MSG_LEN];
        ulGetErrMsg(err_, err_msg);
        ROS_ERROR("Encoder Error Message: %s \n", err_msg);
    }

}

void EncoderDriver::initializeEncoder()
{
    ROS_INFO("Start intializeEncoder");
    // Allocate a buffer to receive the data
    buffer_ = (unsigned long long*)malloc(encoder_count_ * samples_per_counter_ * sizeof(unsigned long long));

    ROS_INFO("Start intializeEncoder");
    if (buffer_ == NULL)
    {
        ROS_ERROR("\nOut of memory, unable to create scan buffer\n");
    }

    // Start the acquisition
    err_ = ulCInScan(daq_device_handle_, low_control_, high_control_, samples_per_counter_, &rate_, scan_options_, flags_, buffer_);
    ROS_INFO("Start intializeEncoder");

    if (err_ == ERR_NO_ERROR)
    {
        // Get the initial status of the acquisition
        ulCInScanStatus(daq_device_handle_, &status_, &transfer_status_);
    }
    ROS_INFO("Start intializeEncoder");
}

std::vector<unsigned long long> EncoderDriver::getEncoderData()
{
    // Output initialization 
    std::vector<unsigned long long> encoder_reading(encoder_count_, 0);

    if (status_ == SS_RUNNING && err_ == ERR_NO_ERROR)
    {
        // Get the current status of the acquisition
        err_ = ulCInScanStatus(daq_device_handle_, &status_, &transfer_status_);
        
        index_ = transfer_status_.currentIndex;

        if (index_ >= 0)
        {
            // Display data
            // NOTE: in the future, publish data instead
            for (unsigned int i = 0; i < encoder_count_; i++)
            {
                if (i < 4)
                {

                    // NOTE: Since the wheel speed is not fast enough
                    if (buffer_[index_ + i] > 50000)
                    {
                        //buffer_[index_ + i] = 65535 - buffer_[index_ + i];
                        encoder_reading[i] = 65535 - buffer_[index_ + i];
                        
                    }
                    else
                    {
                        encoder_reading[i] = buffer_[index_ + i];
                    }

                    //if (buffer_[index_ + i] > encoder_sample_)
                    //{
                    //    buffer_[index_ + i] =  buffer_[index_ + i] - encoder_sample_;
                    //}
                }
                else
                    encoder_reading[i] = buffer_[index_ + i];
            }
        }
    }
    
    // Error
    if (err_ != ERR_NO_ERROR)
    {
        char err_msg[ERR_MSG_LEN];
        ulGetErrMsg(err_, err_msg);
        ROS_ERROR("Encoder Error Message: %s \n", err_msg);
    }

    return encoder_reading;
}



