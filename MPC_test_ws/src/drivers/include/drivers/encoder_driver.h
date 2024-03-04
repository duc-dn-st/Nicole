/*
 * ===============================================================================
 * driving_motor_driver.cpp
 * Author:Tran Viet Thanh 
 * Date: 04.06.2019
 * -------------------------------------------------------------------------------
 * Description:
 * This file accesses the motors for motion.
 * ===============================================================================
 */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#define MAX_DEV_COUNT 100
#define MAX_SCAN_OPTIONS_LENGTH 256
#define MAX_ENCODER_COUNTERS 16

#include <ros/ros.h>
#include <chrono>
#include "uldaq.h"

class EncoderDriver
{
public:
    EncoderDriver(){}

    ~EncoderDriver(){}
    
    void openDAQ();
    
    //void closeDAQ();

    void getEncoderCountersConfiguration();
    
    void initializeEncoder();

    std::vector<unsigned long long> getEncoderData();

protected:
    // NOTE: compare counter samples and encoder sample
    const int encoder_sample_ = 8000;
    
    // Data
    unsigned long long* buffer_ = NULL; 

    int low_control_ = 0;

    int encoder_count_ = 0;
    
    int index_ = 0;

private:

    // DAQ Device Description  
    int descriptor_index_ = 0;
    
    DaqDeviceDescriptor device_descriptors_[MAX_DEV_COUNT];
    
    DaqDeviceInterface interface_type_ = ANY_IFC;
    
    DaqDeviceHandle daq_device_handle_ = 0;
    
    unsigned int number_of_devices_ = MAX_DEV_COUNT;
    
    // Device settings
    int hasCI_ = 0;
    
    int hasPacer_ = 0;
    
    int number_of_encoders_ = MAX_ENCODER_COUNTERS;
    
    int encoder_counters_[MAX_ENCODER_COUNTERS];

    // Encoder settings 
    CounterMeasurementType type_ = CMT_ENCODER;
    
    CounterMeasurementMode mode_ = (CounterMeasurementMode) (CMM_ENCODER_X2 | CMM_ENCODER_CLEAR_ON_Z);

    CounterEdgeDetection edge_detection_ = CED_RISING_EDGE;
    
    CounterTickSize tick_size_ = CTS_TICK_20ns;
    
    CounterDebounceMode debounce_mode_ = CDM_NONE;
    
    CounterDebounceTime debounce_time_ = CDT_DEBOUNCE_0ns;
    
    CConfigScanFlag config_flags_ = CF_DEFAULT;
    
    // Error
    UlError err_ = ERR_NO_ERROR;
    
    // Variables for data acquisition
    
    int high_control_ = 0;
    
    int first_encoder_ = 0;
    
    ScanOption scan_options_ = (ScanOption) (SO_DEFAULTIO | SO_CONTINUOUS);

    CInScanFlag flags_ = CINSCAN_FF_DEFAULT;
    
    // Time related variables 
    const int samples_per_counter_ = 8000;
    

    double rate_ = 1000;

    // Scan status 
    ScanStatus status_;
    
    TransferStatus transfer_status_;
        
    UlError _getCtrInfoNumberOfChannels(DaqDeviceHandle daq_device_handle, int* number_of_control_channels);

    UlError _getCtrInfoHasPacer(DaqDeviceHandle daq_device_handle, int* hasPacer);

    UlError _getCtrInfoSupportedEncoderCounters(DaqDeviceHandle daq_device_handle, int* encoders, int* number_of_encoders);

    UlError _getDevInfoHasCtr(DaqDeviceHandle daq_device_handle, int* hasCtr);
};

#endif // ENCODER_DRIVER_H
