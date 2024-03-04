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

#include "../include/drivers/motor_driver.h"

class testDrivingMotorNode : public MotorDriver
{

public:

    testDrivingMotorNode(ros::NodeHandle nh)
    {
        this->nh_ = nh;
        ROS_INFO("Test driving motors.");
        openDAC();
    }

    ~testDrivingMotorNode()
    {
        closeDAC();
    }

    void velocityCmdCallback(const std_msgs::Float64 &vel_msg)
    {
        testDrivingMotor(vel_msg.data, vel_msg.data, vel_msg.data, vel_msg.data);
    }

private:

    // ROS vars
    ros::NodeHandle nh_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_driving_motor_node");
    ros::NodeHandle nh;
    testDrivingMotorNode test(nh);
    ros::Subscriber sub = nh.subscribe("test_motor", 1, &testDrivingMotorNode::velocityCmdCallback, &test);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Exit
    return 0;
}
