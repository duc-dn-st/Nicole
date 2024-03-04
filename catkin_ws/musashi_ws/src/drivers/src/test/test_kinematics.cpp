/*
 * ===============================================================================
 * test_kinematics_cmd.cpp
 * Author: Schaefle Tobias
 * Date: 10.05.2021
 * -------------------------------------------------------------------------------
 * Description:
 * ===============================================================================
 */

#include <iostream>

#include <ros/ros.h>

#include "../include/drivers/motor_driver.h"
#include "../include/utilities/utilities.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_kinematics_node");

    ros::NodeHandle nh("~");

    double vel_fl = 0.0;
    double vel_fr = 0.0;
    double vel_bl = 0.0;
    double vel_br = 0.0;
    double steer_fl = 0.0;
    double steer_fr = 0.0;
    double steer_bl = 0.0;
    double steer_br = 0.0;

    double lin_vel = 0.0;
    double turn_vel = 0.0;

    double lin_vel_cmd;
    double turn_vel_cmd;

    MotorDriver driver;
    driver.openDAC();

    ros::Rate loop_rate(20);

    driver.closeDAC();
    return 0;
}
