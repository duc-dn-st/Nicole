/*
 * ===============================================================================
 * open_loop.h
 * Author: Schaefle Tobias
 * Date: 07.09.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * A temporary controller for the ackerman drive setup.
 * ===============================================================================
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <ros/ros.h>

#include <eigen3/Eigen/Core>

#include "../include/control/controller_base.h"

class OpenLoop : public ControllerBase
{
public:
    OpenLoop() {}

    OpenLoop(ros::NodeHandle nh, ros::NodeHandle private_nh,
                tf::StampedTransform trans_fl, tf::StampedTransform trans_fr, 
                tf::StampedTransform trans_bl, tf::StampedTransform trans_br,
                double sampling_time) : ControllerBase(nh, private_nh, trans_fl, trans_fr, trans_bl, trans_br, sampling_time)
    {
        sampling_time_ = sampling_time;
        nh.param("open_loop", open_loop_, true);
    }

    ~OpenLoop() {}

    void control(const Eigen::Vector3d &robot_pose, const Eigen::Vector3d &pose_ref, const double linear_vel_ref, const double angular_vel_ref,
                 const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br,
                 const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br);

private:

    double sampling_time_;

    bool open_loop_;
};

#endif //PURE_PURSUIT_H
