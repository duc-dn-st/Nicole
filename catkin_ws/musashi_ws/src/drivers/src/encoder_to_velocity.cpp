/*
 * ===============================================================================
 * encoder_to_velocity.cpp
 * Author: Tran Viet Thanh 
 * Date: 04.01.2021
 * -------------------------------------------------------------------------------
 * Description:
 * This file accesses the driving motors' encoder for position and velocity 
 * feedback.
 * ===============================================================================
 */

#include "drivers/encoder_to_velocity.h"

// Global variables
sensor_msgs::JointState global_velocity_msgs;


void saveSteeringAngles()
{
    // File pointer 
    std::cout << "hello" << std::endl;
    std::fstream fout;

    fout.open("/home/musashi/musashi_ws/angles.csv", std::ios::out | std::ios::app); 
    
    fout << global_velocity_msgs.position[4] << ", "
         << global_velocity_msgs.position[5] << ", "
         << global_velocity_msgs.position[6] << ", "
         << global_velocity_msgs.position[7] << ", "
         << "\n";
    fout.flush();
    fout.close();

    global_velocity_msgs.position.clear();
    global_velocity_msgs.velocity.clear();
    global_velocity_msgs.name.clear();
    delete &global_velocity_msgs;
}

EncoderToVelocity::~EncoderToVelocity(){
    std::cout << "Save encoder angle" << std::endl;
}

void EncoderToVelocity::readSteeringAngles()
{
    std::cout << "Start readSteeringAngles" << std::endl; 
    
    // File pointer 
    std::fstream fin; 
     
    fin.open("/home/musashi/musashi_ws/angles.csv", std::ios::in);

    std::string line;
    
    std::vector<std::vector<std::string>> results;
    
    long column_index = 0;

    while (std::getline(fin, line))
    {
        std::stringstream line_stream(line);
        std::string cell;

        long row_index = 0;
        
        std::vector<std::string> row_container;

        while (std::getline(line_stream, cell, ','))
        {
           row_container.push_back(cell);
           ++row_index;
        }

        results.push_back(row_container);

        ++column_index;
    }

    fin.close();
    
    if (results.size() == 0) 
        return;

    initial_angles_[4] = std::stod(results.back()[0]);
    
    initial_angles_[5] = std::stod(results.back()[1]);

    initial_angles_[6] = std::stod(results.back()[2]);
    
    initial_angles_[7] = std::stod(results.back()[3]);
}


EncoderToVelocity::EncoderToVelocity(ros::NodeHandle private_nh) : current_encoder_reading_(encoder_count_, 0), previous_encoder_reading_(encoder_count_, 0)
{
    // Node handler
    private_nh_ = private_nh; 
    
    // Start up DAQ devices
    openDAQ();

    getEncoderCountersConfiguration();

    initializeEncoder();
    
    // Intializing message
    velocity_msgs_.position.resize(encoder_count_);
    
    velocity_msgs_.velocity.resize(encoder_count_);
    
    velocity_msgs_.name.resize(encoder_count_);
    
    global_velocity_msgs.position.resize(encoder_count_);

    global_velocity_msgs.velocity.resize(encoder_count_);

    global_velocity_msgs.name.resize(encoder_count_);
    
    previous_encoder_reading_.resize(encoder_count_);
    
    initial_angles_.resize(encoder_count_);
    
    readSteeringAngles();
    
    encoder_pulses_.resize(encoder_count_);
    
    encoder_revolution_.resize(encoder_count_);
    
    // Publisher
    velocity_publisher_ = private_nh_.advertise<sensor_msgs::JointState>("/raw_vel", 10);
    
    // Timer
    //encoder_reading_timer_ = private_nh_.createTimer(ros::Duration(scan_rate_), &EncoderToVelocity::readEncoderCallback, this);
    
    publisher_timer_ = private_nh_.createTimer(ros::Duration(publish_rate_), &EncoderToVelocity::publishVelocity, this);
    
    
    start_time_ = ros::Time::now();
}


void EncoderToVelocity::publishVelocity(const ros::TimerEvent&)
{   
    // Lock thread
    std::lock_guard<std::mutex> lock_encoder_reading(encoder_mutex_);
    
    std::vector<long long> encoder_reading_diff;
    
    duration elapsed_duration = Time::now() - previous_milli_;
    
    previous_milli_ = Time::now();

    double encoder_timer = elapsed_duration.count() / 1000;

    // Calculate the encoder reading difference 
    std::vector<unsigned long long> current_encoder_reading = getEncoderData();
    
    for (unsigned int i = 0; i < encoder_count_; i++)
    {
        encoder_reading_diff.push_back( current_encoder_reading[i]  - previous_encoder_reading_[i] );
    }
    previous_encoder_reading_ = current_encoder_reading;

    velocity_msgs_.header.frame_id = "base_link";
    
    velocity_msgs_.name[0] = "front_left_wheel_shaft_to_front_left_wheel";
    velocity_msgs_.name[1] = "front_right_wheel_shaft_to_front_right_wheel";
    velocity_msgs_.name[2] = "back_left_wheel_shaft_to_back_left_wheel";
    velocity_msgs_.name[3] = "back_right_wheel_shaft_to_back_right_wheel";
    velocity_msgs_.name[4] = "base_link_to_back_left_wheel_shaft";
    velocity_msgs_.name[5] = "base_link_to_front_left_wheel_shaft";
    velocity_msgs_.name[6] = "base_link_to_back_right_wheel_shaft";
    velocity_msgs_.name[7] = "base_link_to_front_right_wheel_shaft";

    // Initialize array
    ros::Duration duration = ros::Time::now() - start_time_;

    velocity_msgs_.header.stamp = ros::Time(duration.toSec());

    velocity_msgs_.header.seq++;
//    velocity_msgs_.header.stamp = ros::Time::now().toNSec();

    // Position difference
    for (unsigned int i = 0; i < encoder_count_; i++)
    {

        if (i < 4)
        {
            //velocity_msgs_.position[i] = (double)current_encoder_reading[i] / encoder_sample_ * 2 * MathConstants::PI ; // rad

            if ( encoder_reading_diff[i] < - maximum_driving_increment_ )
            {
                encoder_reading_diff[i] = EncoderConstants::DRIVING_ENCODER_SAMPLE + encoder_reading_diff[i];
            }
            else if ( encoder_reading_diff[i] > maximum_driving_increment_ )
            {
                encoder_reading_diff[i] = EncoderConstants::DRIVING_ENCODER_SAMPLE - encoder_reading_diff[i];
            }
            //velocity_msgs_.velocity[i] = ((double)(encoder_reading_diff[i]) * 2 * PI )/( encoder_timer * encoder_sample_ * DRIVE_MOTOR_GEAR_RATIO);  
            velocity_msgs_.velocity[i] = ((double)(encoder_reading_diff[i]) * 2 * MathConstants::PI)/( encoder_timer * encoder_sample_ * UNDEFINE_RATIO);
        }
        else
        {

            // If encoder difference is larger than in maximum case
            if ( encoder_reading_diff[i] < - maximum_increment_ )
            {
                encoder_revolution_[i] += 1;
            }
            else if ( encoder_reading_diff[i] > maximum_increment_ )
            {
                encoder_revolution_[i] -= 1;
            }

            if (encoder_revolution_[i] < 0)

                encoder_revolution_[i] = std::round(RobotConstants::STEERING_GEAR_RATIO);
            
            if (encoder_revolution_[i] > std::round(RobotConstants::STEERING_GEAR_RATIO) )

                encoder_revolution_[i] = 0;

            encoder_pulses_[i] = encoder_revolution_[i] * EncoderConstants::STEERING_ENCODER_SAMPLE + current_encoder_reading[i];
            
            if ( encoder_pulses_[i] > EncoderConstants::STEERING_MAXIMUM_PULSE )
            {
                encoder_pulses_[i] = encoder_pulses_[i] - EncoderConstants::STEERING_MAXIMUM_PULSE;
            }
            else if ( encoder_pulses_[i] < 0 )
            {
                encoder_pulses_[i] = encoder_pulses_[i] + EncoderConstants::STEERING_MAXIMUM_PULSE;
            }
            
            double relative_angle =  (double)encoder_pulses_[i] / EncoderConstants::STEERING_MAXIMUM_PULSE * MathConstants::TWOPI + initial_angles_[i];
            
            if (relative_angle > MathConstants::TWOPI)

                relative_angle = relative_angle - MathConstants::TWOPI;

            relative_angle = MathConstants::TWOPI - relative_angle;

            velocity_msgs_.position[i] = relative_angle; // rad

            //velocity_msgs_.velocity[i] = ( (double)(encoder_reading_diff[i]) * 2 * MathConstants::PI / RobotConstants::STEERING_GEAR_RATIO )/( encoder_timer * EncoderConstants::STEERING_MAXIMUM_PULSE );  
        }
    }
    
    // Save to global
    global_velocity_msgs = velocity_msgs_;

    // Publish 
    velocity_publisher_.publish(velocity_msgs_);
}

void shutdownHandler(int sig)
{
    saveSteeringAngles();
    
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_to_velocity", ros::init_options::NoSigintHandler);
    
//    signal(SIGINT, shutdownHandler);

    ros::NodeHandle private_nh("~");

    EncoderToVelocity encoder_to_velocity(private_nh);
    
    ros::AsyncSpinner s(2);
    
    s.start();

    ros::waitForShutdown();
    
    delete &encoder_to_velocity;
}
