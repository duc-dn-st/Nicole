#ifndef UTILITY_H
#define UTILITY_H

#include <stdio.h>
#include <unistd.h>
#include <memory>

#include "uldaq.h"

/**
  Counter Info Functions
*/
UlError getCtrInfoNumberOfChannels(DaqDeviceHandle daq_device_handle, int* number_of_control_channels)
{
    UlError err = ERR_NO_ERROR;
    long long number_channels;

    err = ulCtrGetInfo(daq_device_handle, CTR_INFO_NUM_CTRS, 0, &number_channels);

    *number_of_control_channels = (int)number_channels;

    return err;
}

UlError getCtrInfoHasPacer(DaqDeviceHandle daq_device_handle, int* hasPacer)
{
    UlError err = ERR_NO_ERROR;
    long long pacer_support;

    err = ulCtrGetInfo(daq_device_handle, CTR_INFO_HAS_PACER, 0, &pacer_support);

    *hasPacer = (int)pacer_support;

    return err;
}

UlError getCtrInfoSupportedEncoderCounters(DaqDeviceHandle daq_device_handle, int* encoders, int* number_of_encoders)
{
    UlError err = ERR_NO_ERROR;
    int i = 0; 
    int number_of_counters = 0;
    long long measurement_types;

    int number_encoder_controllers = 0;

    // Get the number of counter channels
    err = getCtrInfoNumberOfChannels(daq_device_handle, &number_of_counters);

    // Fill a descriptor
    for (i = 0; i < number_of_counters; i++)
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

/**
  Device Info Functions
*/
UlError getDevInfoHasCtr(DaqDeviceHandle daq_device_handle, int* hasCtr)
{
    long long control_supported;
    UlError err = ERR_NO_ERROR;

    err = ulDevGetInfo(daq_device_handle, DEV_INFO_HAS_CTR_DEV, 0, &control_supported);

    *hasCtr = (int)control_supported;

    return err;
}



#endif //UTILITY_H
