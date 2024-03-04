/*
 * ===============================================================================
 * test_driving_motor.cpp
 * Author: Schaefle Tobias
 * Date: 14.06.2019
 * -------------------------------------------------------------------------------
 * Description:
 * This node is for testing the driving motors for the sdv.
 * ===============================================================================
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "drivers/encoder_driver.h"

class TestEncoderDriverNode : public EncoderDriver
{

public:

    TestEncoderDriverNode(ros::NodeHandle nh)
    {
        this->nh_ = nh;
        
        ROS_INFO("Test encoder.");
        
        openDAQ();

        getEncoderCountersConfiguration();
    }

private:

    // ROS vars
    ros::NodeHandle nh_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_encoder_driver_node");
    
    ros::NodeHandle nh;
    
    TestEncoderDriverNode encoder_driver(nh);
    
    encoder_driver.initializeEncoder();
    
    ros::Rate rate(5);

    while(ros::ok())
    {
        encoder_driver.getEncoderData();
        rate.sleep();
    }

    // Exit
    return 0;
}
