/*
 * ===============================================================================
 * keyboard_cmd.cpp
 * Author: Schaefle Tobias
 * Date: 10.05.2021
 * -------------------------------------------------------------------------------
 * Description:
 * This program can be used to move the robot with WASD and QE buttons on the
 * keyboard.
 * ===============================================================================
 */

#include <iostream>

#include <ros/ros.h>

#include "../include/drivers/motor_driver.h"
#include "../include/utilities/utilities.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_node");

    ros::NodeHandle nh("~");
    char entry;
    bool stop_reading = false;
    double lin_vel_increment;
    double turn_vel_increment;

    std::vector<bool> use_motors;
    // load params fl fr bl br
    ros::param::param< std::vector<bool> >("~use_motors", use_motors, {true, true, true, true, true, true, false, false});
    ros::param::param<double>("~lin_vel_increment", lin_vel_increment, 0.05);
    ros::param::param<double>("~turn_vel_increment", turn_vel_increment, 0.05);

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

    std::cout << "The following keyboard entries can be used:\n"
              << "'W': Forward Motion\n"
              << "'A': Left Turn\n"
              << "'S': Reverse Motion\n"
              << "'D': Right Motion\n"
              << "'Q': Linear Velocity Increment (+" << lin_vel_increment << ")\n"
              << "'E': Linear Velocity Decrement (-" << lin_vel_increment << ")\n"
              << "'T': Angular Velocity Increment (+" << turn_vel_increment << ")\n"
              << "'Y': Angular Velocity Decrement (-" << turn_vel_increment << ")\n"
              << "'R': Reset Turn Velocity\n"
              << "'Z': Stop Program\n" << std::endl;

    while (ros::ok())
    {
        std::cout << "Enter value:" << std::endl;

        if (stop_reading)
        {break;}

        std::cin >> entry;

        // check which button was pressed
        if (entry == 'w')
        {
            lin_vel_cmd = lin_vel;
            vel_fl = lin_vel_cmd * use_motors.at(0);
            vel_fr = lin_vel_cmd * use_motors.at(1);
            vel_bl = lin_vel_cmd * use_motors.at(2);
            vel_br = lin_vel_cmd * use_motors.at(3);
        }
        else if (entry == 'a')
        {
            turn_vel_cmd = turn_vel;
            steer_fl = turn_vel_cmd * use_motors.at(4);
            steer_fr = turn_vel_cmd * use_motors.at(5);
            steer_bl = turn_vel_cmd * use_motors.at(6);
            steer_br = turn_vel_cmd * use_motors.at(7);
        }
        else if (entry == 's')
        {
            lin_vel_cmd = -1 * lin_vel;
            vel_fl = lin_vel_cmd * use_motors.at(0);
            vel_fr = lin_vel_cmd * use_motors.at(1);
            vel_bl = lin_vel_cmd * use_motors.at(2);
            vel_br = lin_vel_cmd * use_motors.at(3);
        }
        else if (entry == 'd')
        {
            turn_vel_cmd = -1 * turn_vel;
            steer_fl = turn_vel_cmd * use_motors.at(4);
            steer_fr = turn_vel_cmd * use_motors.at(5);
            steer_bl = turn_vel_cmd * use_motors.at(6);
            steer_br = turn_vel_cmd * use_motors.at(7);
        }
        else if (entry == 'q')
        {
            lin_vel += lin_vel_increment;
            lin_vel_cmd = GeneralFunctions::sgn<double>(lin_vel_cmd) * lin_vel;
            vel_fl = lin_vel_cmd * use_motors.at(0);
            vel_fr = lin_vel_cmd * use_motors.at(1);
            vel_bl = lin_vel_cmd * use_motors.at(2);
            vel_br = lin_vel_cmd * use_motors.at(3);
            std::cout << "Linear velocity is " << lin_vel << "m/s" << std::endl;
        }
        else if (entry == 'e')
        {
            lin_vel -= lin_vel_increment;
            if (lin_vel < 0)
            {lin_vel = 0.0;}
            lin_vel_cmd = GeneralFunctions::sgn<double>(lin_vel_cmd) * lin_vel;
            vel_fl = lin_vel_cmd * use_motors.at(0);
            vel_fr = lin_vel_cmd * use_motors.at(1);
            vel_bl = lin_vel_cmd * use_motors.at(2);
            vel_br = lin_vel_cmd * use_motors.at(3);
            std::cout << "Linear velocity is " << lin_vel << "m/s" << std::endl;
        }
        else if (entry == 't')
        {
            turn_vel += turn_vel_increment;
            turn_vel_cmd = GeneralFunctions::sgn<double>(turn_vel_cmd) * turn_vel;
            steer_fl = turn_vel_cmd * use_motors.at(4);
            steer_fr = turn_vel_cmd * use_motors.at(5);
            steer_bl = turn_vel_cmd * use_motors.at(6);
            steer_br = turn_vel_cmd * use_motors.at(7);
            std::cout << "Turn velocity is " << turn_vel << "rad/s" << std::endl;
        }
        else if (entry == 'y')
        {
            turn_vel -= turn_vel_increment;
            if (turn_vel < 0)
            {turn_vel = 0.0;}
            turn_vel_cmd = GeneralFunctions::sgn<double>(turn_vel_cmd) * turn_vel;
            steer_fl = turn_vel_cmd * use_motors.at(4);
            steer_fr = turn_vel_cmd * use_motors.at(5);
            steer_bl = turn_vel_cmd * use_motors.at(6);
            steer_br = turn_vel_cmd * use_motors.at(7);
            std::cout << "Turn velocity is " << turn_vel << "rad/s" << std::endl;
        }
        else if (entry == 'r')
        {
            double steer_fl = 0.0;
            double steer_fr = 0.0;
            double steer_bl = 0.0;
            double steer_br = 0.0;
        }
        else if (entry == 'z')
        {
            stop_reading = true;
        }
        else
        {
            ROS_WARN("Enter valid keyboard button.");
            std::cout << "The following keyboard entries can be used:\n"
                      << "'W': Forward Motion\n"
                      << "'A': Left Turn\n"
                      << "'S': Reverse Motion\n"
                      << "'D': Right Motion\n"
                      << "'Q': Linear Velocity Increment (+" << lin_vel_increment << ")\n"
                      << "'E': Linear Velocity Decrement (-" << lin_vel_increment << ")" << std::endl;
        }

        // send commands
        driver.sendMotorCommands(vel_fl, vel_fr, vel_bl, vel_br,
                            steer_fl, steer_fr, steer_bl, steer_br);

        loop_rate.sleep();
    }
    driver.closeDAC();
    return 0;
}
