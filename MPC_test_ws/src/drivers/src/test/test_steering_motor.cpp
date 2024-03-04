/*
 * ===============================================================================
 * test_steering_motor.cpp
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

class testSteeringMotorNode : public MotorDriver
{

public:

    testSteeringMotorNode(ros::NodeHandle nh)
    {
        this->nh_ = nh;
        ROS_INFO("Test steering motors.");
        openDAC();
    }

    ~testSteeringMotorNode()
    {
        closeDAC();
    }

    void steeringCmdCallback(const std_msgs::Float64 &steering_msg)
    {
        testSteeringMotor(steering_msg.data, steering_msg.data, steering_msg.data, steering_msg.data);
        
        //fl
        //testSteeringMotor(steering_msg.data, 0.0, 0.0, 0.0);
        
        //fr
        //testSteeringMotor(0, steering_msg.data, 0, 0);
        
        //bl
        //testSteeringMotor(0, 0, steering_msg.data, 0);
        
        //br
        //testSteeringMotor(0, 0, 0, steering_msg.data);
    }

private:

    // ROS vars
    ros::NodeHandle nh_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_steering_motor_node");
    ros::NodeHandle nh;
    testSteeringMotorNode test(nh);
    ros::Subscriber sub = nh.subscribe("test_motor", 1, &testSteeringMotorNode::steeringCmdCallback, &test);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    test.closeDAC();
    // Exit
    return 0;
}
