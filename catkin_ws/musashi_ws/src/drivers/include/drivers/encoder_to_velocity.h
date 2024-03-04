// Global variables
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

#ifndef ENCODER_TO_VELOCITY_H
#define ENCODER_TO_VELOCITY_H

#define UNDEFINE_RATIO 2.252
#define DRIVE_MOTOR_GEAR_RATIO 40 

#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <csignal>
#include "signal.h"
#include <fstream>
#include <mutex>
#include "drivers/encoder_driver.h"
#include "utilities/utilities.h"
#include "sensor_msgs/JointState.h" 

class EncoderToVelocity : public EncoderDriver
{
private: 
    
    // Node handler 
    ros::NodeHandle private_nh_;
    
    // Publisher 
    ros::Publisher velocity_publisher_;

    // Timer 
    ros::Timer encoder_reading_timer_;
    
    ros::Timer publisher_timer_;

    // Message 
    sensor_msgs::JointState velocity_msgs_;
    
    // Encoder scan rate 
    const float scan_rate_ = 0.0001;
    
    const float publish_rate_ = 0.005;
    
    // Variables to calculate velocity 
    std::vector<unsigned long long> current_encoder_reading_;
    
    std::vector<unsigned long long> previous_encoder_reading_;
    
    std::vector<double> initial_angles_;

    // Mutex
    std::mutex encoder_mutex_;
    
    // Encoder revolution
    std::vector<long> encoder_pulses_;
    
    std::vector<int> encoder_revolution_;
    
    const double maximum_increment_ = ( RobotConstants::STEERING_RATED_MOTOR_SPEED * publish_rate_ * EncoderConstants::STEERING_ENCODER_SAMPLE ) / 60;

    const double maximum_driving_increment_ = ( RobotConstants::DRIVING_RATED_MOTOR_SPEED * publish_rate_ * EncoderConstants::DRIVING_ENCODER_SAMPLE ) / 60;

    typedef std::chrono::high_resolution_clock Time;
    
    typedef std::chrono::duration<float, std::milli> duration;
    
    Time::time_point previous_milli_;

    ros::Time start_time_;
    
    double sum = 0;

    double average = 0;

    unsigned int index = 0;

public: 
    // Construstor
    EncoderToVelocity(ros::NodeHandle private_nh);
    
    ~EncoderToVelocity();

    void readEncoderCallback(const ros::TimerEvent&);

    void publishVelocity(const ros::TimerEvent&);

    void readSteeringAngles();
};
#endif // ENCODER_TO_VELOCITY_H

