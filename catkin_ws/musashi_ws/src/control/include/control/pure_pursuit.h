/*
 * ===============================================================================
 * pure_pursuit.h
 * Author: Schaefle Tobias
 * Date: 04.03.21
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * A pure pursuit controller for the ackerman drive setup of the SDV (mainly used for
 * collision avoidance with VFH).
 * ===============================================================================
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#endif // PURE_PURSUIT_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <eigen3/Eigen/Core>

#include "../include/control/controller_base.h"
#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"

class PurePursuit : public ControllerBase
{
public:
    PurePursuit() {}

    PurePursuit(ros::NodeHandle nh, ros::NodeHandle private_nh,
                tf::StampedTransform trans_fl, tf::StampedTransform trans_fr,
                tf::StampedTransform trans_bl, tf::StampedTransform trans_br,
                double sampling_time) : ControllerBase(nh, private_nh, trans_fl, trans_fr, trans_bl, trans_br, sampling_time)
    {
        sampling_time_ = sampling_time;

        nh_ = nh;
        private_nh_ = private_nh;

        vfh_path_pub_ = private_nh_.advertise<nav_msgs::Path>("vfh_pure_pursuit_path", 1);
        pp_line_connection_pub_ = private_nh_.advertise<nav_msgs::Path>("line_connection", 1);
        vfh_distance_ = ControlConstants::PURE_PURSUIT_LOOKAHEAD;
        prev_lin_vel_ = 0.0;
        prev_steering_fl_ = 0.0;
        prev_steering_fr_ = 0.0;
        prev_steering_bl_ = 0.0;
        prev_steering_br_ = 0.0;
    }

    // returns the vfh trajectory
    void generateVfhPurePursuitTrajectory(const Eigen::Vector3d &rear_axis_pose, const nav_msgs::PathPtr &vfh_heading_path, sdv_msgs::Trajectory &vfh_trajectory);

    void control(const Eigen::Vector3d &rear_axis_pose, const Eigen::Vector3d &vfh_goal, const nav_msgs::PathPtr &vfh_heading_path, const double vfh_vel,
                 const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br,
                 const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br);

private:

    /// calculate the current velocity with respect to epsilon ("curvature") of the path
    double getLinearVelocity(const Eigen::Vector3d &rear_axis_pose, const Eigen::Vector3d &vfh_goal);

    // for generating a trajectory model of the system needed
    inline void getStateUpdate(const Eigen::Vector3d &prev_state, const Eigen::Vector2d input, Eigen::Vector3d &state);

    // get the angle between robot heading and lookahead line
    inline double getAlpha(const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_position);

    // returns the input from the vfh
    void getVfhControlCmd(Eigen::Vector2d &input, const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br,
                          const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br);

    // check linear acc constraint of ackerman drive for center wheel
    inline double linearAccConstraint(double lin_vel, double alpha)
    {
        double lin_acc = (lin_vel - prev_lin_vel_) / sampling_time_;
        // calculate max acc for center wheel
        double max_acc = RobotConstants::MAX_WHEEL_ACC * vfh_distance_ / (vfh_distance_ + 2 * RobotConstants::AXLE_LENGTH * std::abs(std::sin(alpha)));
        if (std::abs(lin_acc) > max_acc)
        {
            if (lin_acc < 0)
            {
                return prev_lin_vel_ - max_acc * sampling_time_;
            }
            else
            {
                return prev_lin_vel_ + max_acc * sampling_time_;
            }
        }
        return lin_vel;
    }

    // check linear velocity constraint of ackerman drive for center wheel
    inline double linearVelConstraint(double lin_vel, double alpha)
    {
        // calculate max linear velocity for the center wheel
        double max_vel = RobotConstants::MAX_WHEEL_VEL * vfh_distance_ / (vfh_distance_ + 2 * RobotConstants::AXLE_LENGTH * std::abs(std::sin(alpha)));
        if (std::abs(lin_vel) > max_vel)
        {
            if (lin_vel < 0)
            {
                return -1 * max_vel;
            }
            else
            {
                return max_vel;
            }
        }
        return lin_vel;
    }

    // check steering vel constraint for inner and outer steering
    inline void steeringVelConstraint(double &inner_steering, double prev_inner_steering, double &outer_steering, double prev_outer_steering)
    {
        double new_inner_steering, new_outer_steering;
        // check if vel of inner steering is violated
        //double inner_steering_vel = (inner_steering - prev_inner_steering) / sampling_time_;
        double inner_steering_vel = GeneralFunctions::angularVelocity(prev_inner_steering, inner_steering, sampling_time_);
        if (std::abs(inner_steering_vel) > RobotConstants::MAX_STEERING_VEL)
        {
            // calculate new steering angle
            if (inner_steering_vel < 0)
            {
                new_inner_steering = -RobotConstants::MAX_STEERING_VEL * sampling_time_ + prev_inner_steering;
            }
            else
            {
                new_inner_steering = RobotConstants::MAX_STEERING_VEL * sampling_time_ + prev_inner_steering;
            }
            // update outer steering based on inner steeering
            double radius = RobotConstants::FRONT_TO_REAR_WHEEL / (std::tan(new_inner_steering)) - RobotConstants::AXLE_LENGTH / 2;
            new_outer_steering = std::atan(RobotConstants::FRONT_TO_REAR_WHEEL / (radius - RobotConstants::AXLE_LENGTH / 2));
            inner_steering = new_inner_steering;
            outer_steering = new_outer_steering;
        }
        /*
        // check if vel of outer steering is violated
        //double outer_steering_vel = (outer_steering - prev_outer_steering) / sampling_time_;
        double outer_steering_vel = GeneralFunctions::angularVelocity(prev_outer_steering, outer_steering, sampling_time_);
        if (std::abs(outer_steering_vel) > RobotConstants::MAX_STEERING_VEL)
        {
            // calculate new steering angle
            if (outer_steering_vel < 0)
            {
                new_outer_steering = -RobotConstants::MAX_STEERING_VEL * sampling_time_ + prev_outer_steering;
            }
            else
            {
                new_outer_steering = RobotConstants::MAX_STEERING_VEL * sampling_time_ + prev_outer_steering;
            }
            // update outer steering based on inner steeering
            double radius = RobotConstants::ROBOT_LENGTH / (std::tan(new_outer_steering)) - RobotConstants::AXLE_LENGTH / 2;
            new_inner_steering = std::atan(RobotConstants::ROBOT_LENGTH / (radius + RobotConstants::AXLE_LENGTH / 2));
            inner_steering = new_inner_steering;
            outer_steering = new_outer_steering;
        }
        */
    }

    double sampling_time_;

    nav_msgs::Path vfh_pp_path_msg_;
    nav_msgs::Path pp_line_connection_msg_;
    double prev_steering_fl_, prev_steering_fr_, prev_steering_bl_, prev_steering_br_;

    double vfh_distance_;

    double prev_lin_vel_;

    ros::NodeHandle nh_, private_nh_;

    ros::Publisher vfh_path_pub_;
    ros::Publisher pp_line_connection_pub_;

};
