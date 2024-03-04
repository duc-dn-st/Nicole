/*
 * ===============================================================================
 * controller_base.h
 * Author: Schaefle Tobias
 * Date: 30.06.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the parent class for every controller class.
 * ===============================================================================
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#define POS_THRESHOLD 0.4
#define HEADING_THRESHOLD 30 * MathConstants::PI / 180

#include "drivers/motor_driver.h"
#include "utilities/utilities.h"
#include "sdv_msgs/ControlReference.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Core>

#include <cmath>

class ControllerBase
{
public:
    // dummy default constructor
    ControllerBase() {}

    ControllerBase(ros::NodeHandle nh, ros::NodeHandle private_nh,
                    tf::StampedTransform trans_fl, tf::StampedTransform trans_fr, 
                    tf::StampedTransform trans_bl, tf::StampedTransform trans_br,
                    double sampling_time)
    {
        nh_ = nh;
        private_nh_ = private_nh;

        sampling_time_ = sampling_time;

        motor_ = new MotorDriver;

        nh_.param("simulation", is_sim_, true);
        nh_.param("no_encoder", no_encoder_, false);

        trans_fl_ = trans_fl;
        trans_fr_ = trans_fr;
        trans_bl_ = trans_bl;
        trans_br_ = trans_br;

        prev_steering_angle_fl_ = 0.0;
        prev_steering_angle_fr_ = 0.0;
        prev_steering_angle_bl_ = 0.0;
        prev_steering_angle_br_ = 0.0;

        if (is_sim_)
        {
            fl_wheel_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/front_left_wheel_velocity_controller/command", 4);
            fr_wheel_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/front_right_wheel_velocity_controller/command", 4);
            bl_wheel_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/back_left_wheel_velocity_controller/command", 4);
            br_wheel_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/back_right_wheel_velocity_controller/command", 4);
            fl_steer_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/front_left_wheel_shaft_position_controller/command", 4);
            fr_steer_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/front_right_wheel_shaft_position_controller/command", 4);
            bl_steer_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/back_left_wheel_shaft_position_controller/command", 4);
            br_steer_pub_ = private_nh_.advertise<std_msgs::Float64>("/musashi_robot/back_right_wheel_shaft_position_controller/command", 4);
        }
        else
        {
            //motor_->openDAC();
            reference_pub_ = private_nh_.advertise<sdv_msgs::ControlReference>("/reference", 1);
        }
        // if no encoder available publish control commands for kinematic localization
        if (no_encoder_)
        {
            no_encoder_pub_ = nh_.advertise<sensor_msgs::JointState>("musashi_robot/joint_states_no_encoder", 1);
        }
    }

    ~ControllerBase()
    {

    }

    void stopMotion();

    void shutdownDAC()
    {
        if (!is_sim_)
        {
            motor_->closeDAC();
        }
    }

protected:
    tf::StampedTransform trans_fl_, trans_fr_, trans_bl_, trans_br_;

    void sendCommands(const double wheel_cmd_fl, const double wheel_cmd_fr, const double wheel_cmd_bl, const double wheel_cmd_br, 
                      const double steering_cmd_fl, const double steering_cmd_fr, const double steering_cmd_bl, const double steering_cmd_br,
                      double steering_enc_fl, double steering_enc_fr, double steering_enc_bl, double steering_enc_br);

    bool atGoal(const Eigen::Vector3d &error) const;

private:
    // only for offline case
    ros::Publisher fl_wheel_pub_;
    ros::Publisher fr_wheel_pub_;
    ros::Publisher bl_wheel_pub_;
    ros::Publisher br_wheel_pub_;
    ros::Publisher fl_steer_pub_;
    ros::Publisher fr_steer_pub_;
    ros::Publisher bl_steer_pub_;
    ros::Publisher br_steer_pub_;

    ros::Publisher no_encoder_pub_;
    ros::Publisher reference_pub_;

    sdv_msgs::ControlReference reference_;

    double prev_steering_angle_fl_, prev_steering_angle_fr_, prev_steering_angle_bl_, prev_steering_angle_br_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    bool is_sim_;
    bool no_encoder_;

    double sampling_time_;

    MotorDriver *motor_;

    /// if sim is true publish the commands
    void publishCommands(const double wheel_cmd_fl, const double wheel_cmd_fr, const double wheel_cmd_bl, const double wheel_cmd_br, 
                         const double steering_cmd_fl, const double steering_cmd_fr, const double steering_cmd_bl, const double steering_cmd_br) const;

    void stopSim();

};

#endif
