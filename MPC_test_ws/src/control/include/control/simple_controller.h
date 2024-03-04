/*
 * ===============================================================================
 * simple_controller.cpp
 * Author: Schaefle Tobias
 * Date: 29.06.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * First controller for the SDV. It receives the robot pose a reference point and
 * encoder values. Controller commands will be calculated and send to the motors.
 * ===============================================================================
 */

#ifndef SIMPLE_CONTROLLER_H
#define SIMPLE_CONTROLLER_H

// for now controller defines will be in this file (later should be moved to a config file)
#define Kx 0.5
#define Ky 0.5
#define Kt 0.3

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Core>

#include <cmath>

#include "../include/control/controller_base.h"

class SimpleController : public ControllerBase
{
public:

    // dummy default constructor
    SimpleController() {}

    SimpleController(ros::NodeHandle nh, ros::NodeHandle private_nh,
                    tf::StampedTransform trans_fl, tf::StampedTransform trans_fr, 
                    tf::StampedTransform trans_bl, tf::StampedTransform trans_br,
                    double sampling_time) : ControllerBase(nh, private_nh, trans_fl, trans_fr, trans_bl, trans_br, sampling_time)
    {
        sampling_time_ = sampling_time;
    }

    /// main controller call
    void control(const Eigen::Vector3d &robot_pose, const Eigen::Vector3d &pose_ref, const Eigen::Vector3d &twist_ref, 
                 const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br, 
                 const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br);

private:

    double sampling_time_;

    /// get the control parameters
    void getControlParams(tf::StampedTransform transform, const double cos_theta, const double sin_theta, const double enc_steering,
                         const Eigen::Vector3d &error, const  Eigen::Vector3d &twist_ref,
                         double &beta, double &gamma, double &alpha_err) const;

    /// calculate the wheel velocity command
    void getWheelVelCmd(const double alpha_err, const double beta, const double gamma, const double enc_vel, double &wheel_cmd) const;

    /// calculate the steering angle command
    void getSteeringCmd(const double alpha_err, const double beta, const double gamma, const double enc_steering, double &steering_cmd) const;
};

#endif